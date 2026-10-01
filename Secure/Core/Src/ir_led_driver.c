/**
  ******************************************************************************
  * @file    ir_led_driver.c
  * @brief   LT3763 IR LED driver control. See ir_led_driver.h for the signal
  *          path, the inverted gate, the CTRL2 ceiling and why faults are
  *          polled rather than edge-latched.
  ******************************************************************************
  */
#include "ir_led_driver.h"
#include "dac.h"

/* ---------------------------------------------------------------------------
   Gate polarity. The single place the proxy board's open-drain inverter is
   expressed: PWM_DIMx LOW turns the LEDs ON.
   --------------------------------------------------------------------------- */
#define IR_GATE_ON    GPIO_PIN_RESET
#define IR_GATE_OFF   GPIO_PIN_SET

typedef struct {
    GPIO_TypeDef *gate_port;
    uint16_t      gate_pin;
    GPIO_TypeDef *fault_port;
    uint16_t      fault_pin;
    uint32_t      dac_channel;

    volatile IR_LedMode_t mode;
    volatile uint32_t     line_start;
    volatile uint32_t     line_end;

    volatile bool     gate_is_on;     /* last commanded electrical gate state  */
    volatile uint32_t gate_on_tick;   /* HAL tick at the last off->on edge     */

    volatile bool     fault_latched;
    volatile uint32_t fault_edges;

    uint16_t raw_code;                /* last DAC code written for this channel */
} IR_ChannelState_t;

static volatile uint32_t s_line_counter  = 0;
static volatile uint32_t s_frame_counter = 0;
static volatile bool     s_master_enable = false;
static IR_FlashSource_t  s_flash_src     = IR_FLASH_SRC_HOST;
static uint32_t          s_limit_ma      = IR_LED_DEFAULT_LIMIT_MA;

/* DAC-PWM state. TIM12 is shared, so the frequency is global while the duty
   is per channel. pwm_on gates the ISR so a channel that is not being
   modulated never has its DAC touched from interrupt context. */
static volatile bool     s_pwm_on[IR_CH_COUNT] = { false, false };
static volatile uint32_t s_pwm_duty[IR_CH_COUNT] = { 0U, 0U };

static volatile bool     s_gate_pwm_on[IR_CH_COUNT] = { false, false };
/* Gate PWM timing, in TIM12 input-clock ticks (IR_TIM_CLK_HZ) rather than
   Hz and percent, so that a period given in time - e.g. 33ms on / 77ms off -
   is reproduced exactly instead of being rounded through an integer Hz.
   The period is shared (one timer); the on-time is per channel. */
static volatile uint32_t s_gate_on_ticks[IR_CH_COUNT] = { 0U, 0U };
static uint32_t          s_gate_period_ticks = 0U;

/* Frame-pulse state. TIM12 runs in one-pulse mode (1us ticks) and is started
   once per frame by fp_fire(), from the VSYNC EXTI or the TIM7 fallback. */
static uint32_t          s_fp_width_us   = 0U;
static volatile uint32_t s_fp_count      = 0U;
static volatile uint32_t s_last_vsync_ms = 0U;
static volatile bool     s_vsync_seen    = false;
static volatile bool     s_fp_synced     = false;

/* Timers count at the APB1 timer clock. AHB and APB1 are both /1 off a 64MHz
   SYSCLK (see SystemClock_Config), and with an APB prescaler of 1 the timer
   clock equals PCLK rather than twice it. */
#define IR_TIM_CLK_HZ            64000000U
#define IR_TICKS_PER_US          (IR_TIM_CLK_HZ / 1000000U)

/* DAC modulation moved off TIM12 when TIM12 was given to the gates: PB14 can
   only reach TIM12_CH1, so the hardware dictates which peripheral owns the
   gate. TIM13 and TIM14 each have a single channel, which is exactly what one
   DAC channel needs (update raises, CC1 drops) - and one timer per channel
   means the two DAC channels now get independent frequencies as well. */
static TIM_TypeDef * const s_dac_tim[IR_CH_COUNT] = { TIM13, TIM14 };
static const IRQn_Type     s_dac_irq[IR_CH_COUNT] = { TIM13_IRQn, TIM14_IRQn };
static uint32_t            s_pwm_freq_ch[IR_CH_COUNT] = { 0U, 0U };

static IR_ChannelState_t s_ch[IR_CH_COUNT] = {
    [IR_CH0] = {
        .gate_port  = PWM_DIM0_GPIO_Port,  .gate_pin  = PWM_DIM0_Pin,
        .fault_port = IR_FAULT0_GPIO_Port, .fault_pin = IR_FAULT0_Pin,
        .dac_channel = IR_DAC1_CH1,
        .mode = IR_MODE_OFF, .line_start = 0, .line_end = 0,
        .gate_is_on = false, .gate_on_tick = 0,
        .fault_latched = false, .fault_edges = 0, .raw_code = 0
    },
    [IR_CH1] = {
        .gate_port  = PWM_DIM1_GPIO_Port,  .gate_pin  = PWM_DIM1_Pin,
        .fault_port = IR_FAULT1_GPIO_Port, .fault_pin = IR_FAULT1_Pin,
        .dac_channel = IR_DAC1_CH2,
        .mode = IR_MODE_OFF, .line_start = 0, .line_end = 0,
        .gate_is_on = false, .gate_on_tick = 0,
        .fault_latched = false, .fault_edges = 0, .raw_code = 0
    },
};

/* ---------------------------------------------------------------------------
   Gate primitives
   --------------------------------------------------------------------------- */

/* Writes the gate pin and tracks how long it has been on, which is what the
   fault blanking in IR_LED_Task() keys off. BSRR rather than
   HAL_GPIO_WritePin() because this runs from the SN0_HS ISR, which fires once
   per sensor line (~65kHz at 2160 lines and 30fps). */
static inline void gate_set(IR_Channel_t ch, bool on)
{
    IR_ChannelState_t *c = &s_ch[ch];

    if (on) {
        c->gate_port->BSRR = (uint32_t)c->gate_pin << 16U;   /* drive low  */
    } else {
        c->gate_port->BSRR = (uint32_t)c->gate_pin;          /* drive high */
    }

    if (on && !c->gate_is_on) {
        c->gate_on_tick = HAL_GetTick();
    }
    c->gate_is_on = on;
}

/* ---------------------------------------------------------------------------
   Gate pin ownership: plain GPIO, or TIM12 alternate function
   --------------------------------------------------------------------------- */

/* Hands PB14/PB15 back to the GPIO peripheral and parks them at the OFF
   level. Writing ODR before switching the mode means the pin drives its safe
   level the instant it becomes an output, with no low-going glitch - the same
   reason MX_GPIO_Init() does it in that order. */
static void gate_pin_to_gpio(IR_Channel_t ch)
{
    GPIO_InitTypeDef init = {0};

    HAL_GPIO_WritePin(s_ch[ch].gate_port, s_ch[ch].gate_pin, IR_GATE_OFF);
    init.Pin   = s_ch[ch].gate_pin;
    init.Mode  = GPIO_MODE_OUTPUT_PP;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(s_ch[ch].gate_port, &init);
    s_ch[ch].gate_is_on = false;
}

static void gate_pin_to_tim12(IR_Channel_t ch)
{
    GPIO_InitTypeDef init = {0};

    init.Pin       = s_ch[ch].gate_pin;
    init.Mode      = GPIO_MODE_AF_PP;
    init.Pull      = GPIO_NOPULL;
    init.Speed     = GPIO_SPEED_FREQ_HIGH;
    init.Alternate = GPIO_AF2_TIM12;      /* PB14 = TIM12_CH1, PB15 = CH2 */
    HAL_GPIO_Init(s_ch[ch].gate_port, &init);
}

/* Re-applies the gate for the channel's current mode, enable and fault state.
   FRAME_STROBE's gate belongs to the sync ISRs, so this only forces it off on
   entry; the next VSYNC re-establishes the window. */
static void gate_pwm_apply(IR_Channel_t ch);   /* fwd decl */
static void fp_apply(IR_Channel_t ch);         /* fwd decl */
static void fp_fire(bool from_vsync);          /* fwd decl */
static void fs_sync_running(void);             /* fwd decl */
static void fs_task(void);                     /* fwd decl */
static bool fs_gate_wanted(void);              /* fwd decl */
static void evict_other_tim12_user(IR_Channel_t ch, IR_LedMode_t conflicting);

static void apply_channel(IR_Channel_t ch)
{
    IR_ChannelState_t *c = &s_ch[ch];

    /* Start or stop the PC3 ADC to match whether any channel needs it. */
    fs_sync_running();

    /* Any state other than "actively chopping" must first take the pin back
       from TIM12, otherwise the timer would keep driving it regardless of
       what the mode says. */
    if ((c->mode != IR_MODE_GATE_PWM && c->mode != IR_MODE_FRAME_PULSE) ||
        !s_master_enable || c->fault_latched) {
        if (s_gate_pwm_on[ch]) {
            s_gate_pwm_on[ch] = false;
            TIM12->CCER &= ~((ch == IR_CH0) ? TIM_CCER_CC1E : TIM_CCER_CC2E);
            if (!s_gate_pwm_on[IR_CH0] && !s_gate_pwm_on[IR_CH1]) {
                TIM12->CR1 &= ~TIM_CR1_CEN;
                s_gate_period_ticks = 0U;
            }
            gate_pin_to_gpio(ch);
        }
    }

    if (!s_master_enable || c->fault_latched) {
        gate_set(ch, false);
        return;
    }

    switch (c->mode) {
        case IR_MODE_GATE_PWM:
            gate_pwm_apply(ch);
            break;

        case IR_MODE_FRAME_PULSE:
            fp_apply(ch);
            break;

        case IR_MODE_FLASH_SYNC:
            gate_set(ch, fs_gate_wanted());
            break;

        case IR_MODE_CONTINUOUS:
            gate_set(ch, true);
            break;

        case IR_MODE_HOST_FLASH:
            /* PWM_DIMx high turns Q on, which hands IR_CTRLx over to the
               FLASH net. Electrically identical to "gate off"; what differs
               is that FLASH is no longer ours to hold low. gate_is_on stays
               false so fault confirmation sits out, since we have no idea
               when the host is actually illuminating. */
            gate_set(ch, false);
            break;

        case IR_MODE_FRAME_STROBE:
        case IR_MODE_OFF:
        default:
            gate_set(ch, false);
            break;
    }
}

/* ---------------------------------------------------------------------------
   FLASH net ownership (PC3)
   --------------------------------------------------------------------------- */

/* PC3 is an INPUT ONLY: the sensor's FLASH signal is wired to it, so the MCU
   must never drive it. It is always analog (ADC1 reads it for FLASH sync).
   IR_FLASH_SRC_MCU is kept in the enum for API compatibility but refused. */
static void flash_release(void)
{
    GPIO_InitTypeDef init = {0};

    init.Pin  = FLASH_Pin;
    init.Mode = GPIO_MODE_ANALOG;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(FLASH_GPIO_Port, &init);
    s_flash_src = IR_FLASH_SRC_HOST;
}

bool IR_LED_SetFlashSource(IR_FlashSource_t src)
{
    /* PC3 is input-only; driving it (IR_FLASH_SRC_MCU) is never allowed. */
    return src == IR_FLASH_SRC_HOST;
}

IR_FlashSource_t IR_LED_GetFlashSource(void)
{
    return s_flash_src;
}

/* ---------------------------------------------------------------------------
   Current setpoint conversions
   --------------------------------------------------------------------------- */

/* Rounded integer divide, so a setpoint written and read back lands on the
   same milliamp rather than drifting down a count each way. */
static inline uint64_t div_round(uint64_t num, uint64_t den)
{
    return (num + den / 2U) / den;
}

/* Highest DAC code worth using: the one that puts CTRL1 at the CTRL2 clamp.
   Anything above it is regulated away by the LT3763 and only costs
   resolution. */
static uint16_t ceiling_code(void)
{
    uint64_t code = div_round((uint64_t)IR_CTRL_CEILING_MV * DAC1_CODE_MAX,
                              IR_DAC_VREF_MV);
    if (code > DAC1_CODE_MAX) {
        code = DAC1_CODE_MAX;
    }
    return (uint16_t)code;
}

static uint32_t code_to_mv(uint16_t code)
{
    return (uint32_t)div_round((uint64_t)code * IR_DAC_VREF_MV, DAC1_CODE_MAX);
}

static uint32_t mv_to_ma(uint32_t ctrl_mv)
{
    /* The lowest of CTRL1/CTRL2 wins, so the board never exceeds the clamp
       however high CTRL1 is driven. */
    if (ctrl_mv > IR_CTRL_CEILING_MV) {
        ctrl_mv = IR_CTRL_CEILING_MV;
    }
    if (ctrl_mv > IR_LT3763_VCTRL_FS_MV) {
        ctrl_mv = IR_LT3763_VCTRL_FS_MV;
    }
    return (uint32_t)div_round((uint64_t)IR_LT3763_VSENSE_FS_UV * ctrl_mv * 1000U,
                               (uint64_t)IR_LT3763_VCTRL_FS_MV * IR_RSENSE_UOHM);
}

static uint16_t ma_to_code(uint32_t milliamps)
{
    uint64_t ctrl_mv;
    uint64_t code;

    if (milliamps > IR_LED_MAX_CURRENT_MA) {
        milliamps = IR_LED_MAX_CURRENT_MA;
    }
    ctrl_mv = div_round((uint64_t)milliamps * IR_LT3763_VCTRL_FS_MV * IR_RSENSE_UOHM,
                        (uint64_t)IR_LT3763_VSENSE_FS_UV * 1000U);
    code = div_round(ctrl_mv * DAC1_CODE_MAX, IR_DAC_VREF_MV);

    if (code > ceiling_code()) {
        code = ceiling_code();
    }
    return (uint16_t)code;
}

/* Highest DAC code whose current does not exceed IR_LED_HARD_LIMIT_MA.
   ma_to_code() rounds to nearest, which can land one code above the limit
   (3000mA -> code 274 -> 3006mA), so step down until the readback fits. */
static uint16_t hard_limit_code(void)
{
    static uint16_t code = 0U;
    static bool     done = false;

    if (!done) {
        code = ma_to_code(IR_LED_HARD_LIMIT_MA);
        while (code > 0U && mv_to_ma(code_to_mv(code)) > IR_LED_HARD_LIMIT_MA) {
            code--;
        }
        done = true;
    }
    return code;
}

void IR_LED_SetCurrentRaw(IR_Channel_t ch, uint16_t code_12bit)
{
    if (ch >= IR_CH_COUNT) return;
    if (code_12bit > DAC1_CODE_MAX) code_12bit = DAC1_CODE_MAX;

    /* Hard cap. Every DAC write - mA, mV, raw code, ramp, DAC PWM - goes
       through raw_code, which is only ever set here. */
    if (code_12bit > hard_limit_code()) code_12bit = hard_limit_code();

    s_ch[ch].raw_code = code_12bit;
    DAC1_SetValue(s_ch[ch].dac_channel, code_12bit);
}

uint16_t IR_LED_GetCurrentRaw(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return 0;
    return s_ch[ch].raw_code;
}

void IR_LED_SetCurrent_mA(IR_Channel_t ch, uint32_t milliamps)
{
    if (ch >= IR_CH_COUNT) return;
    if (milliamps > s_limit_ma) {
        milliamps = s_limit_ma;
    }
    IR_LED_SetCurrentRaw(ch, ma_to_code(milliamps));
}

void IR_LED_SetCurrentLimit_mA(uint32_t milliamps)
{
    if (milliamps > IR_LED_HARD_LIMIT_MA) {
        milliamps = IR_LED_HARD_LIMIT_MA;
    }
    s_limit_ma = milliamps;

    /* Re-apply every setpoint so lowering the cap bites immediately. Reading
       the present value back through GetCurrent_mA() and pushing it through
       SetCurrent_mA() re-runs the clamp; a channel already below the new cap
       lands on the same code it already had. */
    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_LED_SetCurrent_mA((IR_Channel_t)i, IR_LED_GetCurrent_mA((IR_Channel_t)i));
    }
}

uint32_t IR_LED_GetCurrentLimit_mA(void)
{
    return s_limit_ma;
}

bool IR_LED_RampCurrent_mA(IR_Channel_t ch, uint32_t target_ma,
                           uint32_t step_ma, uint32_t dwell_ms)
{
    uint32_t now_ma;

    if (ch >= IR_CH_COUNT) return false;
    if (step_ma  == 0U) step_ma  = IR_LED_RAMP_STEP_MA;
    if (dwell_ms == 0U) dwell_ms = IR_LED_RAMP_DWELL_MS;

    if (target_ma > s_limit_ma) {
        target_ma = s_limit_ma;
    }

    now_ma = IR_LED_GetCurrent_mA(ch);

    while (now_ma != target_ma) {
        if (now_ma < target_ma) {
            now_ma = (target_ma - now_ma < step_ma) ? target_ma : now_ma + step_ma;
        } else {
            now_ma = (now_ma - target_ma < step_ma) ? target_ma : now_ma - step_ma;
        }
        IR_LED_SetCurrent_mA(ch, now_ma);
        HAL_Delay(dwell_ms);

        /* Give the fault logic a chance to run at every step: this loop
           blocks the main loop, so without this a ramp would sail past a
           fault that appeared halfway up. */
        IR_LED_Task();
        if (s_ch[ch].fault_latched || !s_master_enable) {
            return false;
        }
    }
    return true;
}

uint32_t IR_LED_GetCurrent_mA(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return 0;
    return mv_to_ma(code_to_mv(s_ch[ch].raw_code));
}

uint32_t IR_LED_GetCtrlVoltage_mV(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return 0;
    return code_to_mv(s_ch[ch].raw_code);
}

uint32_t IR_LED_SetCtrlVoltage_mV(IR_Channel_t ch, uint32_t millivolts)
{
    uint64_t code;

    if (ch >= IR_CH_COUNT) return 0;
    if (millivolts > IR_DAC_VREF_MV) {
        millivolts = IR_DAC_VREF_MV;      /* the DAC cannot exceed its own reference */
    }
    code = div_round((uint64_t)millivolts * DAC1_CODE_MAX, IR_DAC_VREF_MV);
    if (code > DAC1_CODE_MAX) {
        code = DAC1_CODE_MAX;
    }
    IR_LED_SetCurrentRaw(ch, (uint16_t)code);

    /* Report back what the quantised, hard-capped code actually produces,
       not what was asked for - a caller printing the request would be
       quietly lying about the pin voltage. */
    return code_to_mv(s_ch[ch].raw_code);
}

uint32_t IR_LED_GetCtrlStep_uV(void)
{
    return (uint32_t)div_round((uint64_t)IR_DAC_VREF_MV * 1000U, DAC1_CODE_MAX);
}

/* ---------------------------------------------------------------------------
   Lifecycle
   --------------------------------------------------------------------------- */

void IR_LED_Init(void)
{
    /* Order matters. Gates off first, so nothing can illuminate while the
       rest of this runs; then the rail request down; then the analog
       setpoint to zero. */
    for (int i = 0; i < IR_CH_COUNT; i++) {
        s_ch[i].mode          = IR_MODE_OFF;
        s_ch[i].fault_latched = false;
        s_ch[i].fault_edges   = 0;
        s_ch[i].line_start    = 0;
        s_ch[i].line_end      = 0;
        gate_set((IR_Channel_t)i, false);
    }

    /* Output enable starts ON. IR_ENB_MCU (PA11) is not connected to
       anything on this build - it lands on the sheet 3 board-to-board header
       and nothing downstream consumes it - so gating illumination on it only
       ever produced the trap of "mode and current are set but the channel is
       dark". Illumination is now determined by the per-channel mode alone.
       The board still comes up dark because every mode is IR_MODE_OFF above.
       The pin is still driven in step with this flag: it costs nothing and
       keeps the signal correct if it is ever wired up. */
    s_master_enable = true;
    HAL_GPIO_WritePin(IR_ENB_MCU_GPIO_Port, IR_ENB_MCU_Pin, GPIO_PIN_SET);

    /* PC3 (FLASH) is an input from the sensor - never driven. */
    flash_release();

    MX_DAC1_Init();
    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_LED_SetCurrentRaw((IR_Channel_t)i, 0);
    }

    s_line_counter  = 0;
    s_frame_counter = 0;

    for (int i = 0; i < IR_CH_COUNT; i++) {
        s_pwm_on[i]      = false;
        s_pwm_duty[i]    = 0U;
        s_pwm_freq_ch[i] = 0U;
        s_gate_pwm_on[i] = false;
        s_gate_on_ticks[i] = 0U;
    }
    s_gate_period_ticks = 0U;

    /* Boot default on both channels, at IR_BOOT_CURRENT_MA: gated by the
       sensor's FLASH (IR_BOOT_FLASH_SYNC), else one frame pulse per frame.
       This LIGHTS the illuminator at boot. */
#if (IR_BOOT_FLASH_SYNC != 0)
    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_LED_SetCurrent_mA((IR_Channel_t)i, IR_BOOT_CURRENT_MA);
        (void)IR_LED_SetMode((IR_Channel_t)i, IR_MODE_FLASH_SYNC);
    }
#elif (IR_FRAME_PULSE_DEFAULT_WIDTH_US != 0U)
    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_LED_SetCurrent_mA((IR_Channel_t)i, IR_BOOT_CURRENT_MA);
        IR_LED_SetFramePulse((IR_Channel_t)i, IR_FRAME_PULSE_DEFAULT_WIDTH_US);
    }
#endif
}

void IR_LED_Task(void)
{
    fs_task();

    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_ChannelState_t *c = &s_ch[i];

        if (c->fault_latched || !s_master_enable) {
            continue;
        }
        /* Only a channel we are actively holding illuminated can produce a
           meaningful FAULT reading. A dark channel always reports a fault,
           because the LT3763 asserts it whenever the inductor current is
           under ten percent of maximum -- see note 3 in ir_led_driver.h.

           This does mean IR_MODE_FRAME_STROBE channels are effectively
           unmonitored unless their window is longer than the blanking
           period: the gate keeps going off before confirmation can complete.
           IR_LED_GetFaultEdgeCount() is the signal to watch there instead. */
        if (!c->gate_is_on) {
            continue;
        }
        /* Re-read the tick after gate_on_tick: an ISR (FLASH sync) can turn
           the gate on after a tick sampled earlier, and 'tick - gate_on_tick'
           would then wrap to ~2^32 and skip the blanking entirely. */
        uint32_t on_tick = c->gate_on_tick;
        if ((HAL_GetTick() - on_tick) < IR_FAULT_BLANKING_MS) {
            continue;
        }
        if (HAL_GPIO_ReadPin(c->fault_port, c->fault_pin) != GPIO_PIN_RESET) {
            continue;   /* line high: healthy */
        }

        /* Confirmed: still asserted after the converter has had time to come
           up and deliver current. */
        c->fault_latched = true;
        gate_set((IR_Channel_t)i, false);
        /* Conservative: a fault on either channel drops the shared 18V rail
           request. Relax this to a per-channel shutdown if the two boards are
           meant to fail independently. */
        IR_LED_Enable(false);
    }
}

/* ---------------------------------------------------------------------------
   Master enable
   --------------------------------------------------------------------------- */

void IR_LED_Enable(bool enable)
{
    if (!enable) {
        /* FLASH (PC3) is input-only, so it cannot be reclaimed: while the
           sensor holds FLASH high, PWM_DIMx alone cannot force a channel
           dark. Drop HOST_FLASH channels to OFF so the state stays coherent. */
        for (int i = 0; i < IR_CH_COUNT; i++) {
            if (s_ch[i].mode == IR_MODE_HOST_FLASH) {
                s_ch[i].mode = IR_MODE_OFF;
            }
        }
    }

    s_master_enable = enable;
    HAL_GPIO_WritePin(IR_ENB_MCU_GPIO_Port, IR_ENB_MCU_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

    for (int i = 0; i < IR_CH_COUNT; i++) {
        apply_channel((IR_Channel_t)i);
    }
    /* Re-enabling mid-frame in FRAME_STROBE leaves the gate off until the
       next VSYNC rebuilds the window. Worst case is one dark frame, which is
       preferable to illuminating at an arbitrary point in the exposure. */
}

bool IR_LED_IsEnabled(void)
{
    return s_master_enable;
}

/* ---------------------------------------------------------------------------
   Mode and window
   --------------------------------------------------------------------------- */

bool IR_LED_SetMode(IR_Channel_t ch, IR_LedMode_t mode)
{
    if (ch >= IR_CH_COUNT) return false;

    if (mode == IR_MODE_HOST_FLASH && s_flash_src != IR_FLASH_SRC_HOST) {
        /* The gate would follow our own FLASH output, which is held low, so
           the channel would simply stay dark. Refuse rather than look like
           it worked. */
        return false;
    }
    s_ch[ch].mode = mode;
    apply_channel(ch);
    return true;
}

IR_LedMode_t IR_LED_GetMode(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return IR_MODE_OFF;
    return s_ch[ch].mode;
}

bool IR_LED_SetWindow(IR_Channel_t ch, uint32_t line_start, uint32_t line_end)
{
    if (ch >= IR_CH_COUNT) return false;
    if (line_start >= line_end) return false;

    /* The HSYNC ISR compares the live line count against both fields, so
       updating them one at a time can hand it a mixed old/new pair and give
       one frame with a nonsense window. Two stores is short enough that
       masking interrupts outright is cheaper than any alternative. */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_ch[ch].line_start = line_start;
        s_ch[ch].line_end   = line_end;
        __set_PRIMASK(primask);
    }
    return true;
}

void IR_LED_GetWindow(IR_Channel_t ch, uint32_t *line_start, uint32_t *line_end)
{
    if (ch >= IR_CH_COUNT) return;
    if (line_start) *line_start = s_ch[ch].line_start;
    if (line_end)   *line_end   = s_ch[ch].line_end;
}

uint32_t IR_LED_GetLineCount(void)  { return s_line_counter; }
uint32_t IR_LED_GetFrameCount(void) { return s_frame_counter; }

/* ---------------------------------------------------------------------------
   Faults
   --------------------------------------------------------------------------- */

bool IR_LED_HasFault(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return true;
    return s_ch[ch].fault_latched;
}

bool IR_LED_IsFaultLineAsserted(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return true;
    return HAL_GPIO_ReadPin(s_ch[ch].fault_port, s_ch[ch].fault_pin) == GPIO_PIN_RESET;
}

uint32_t IR_LED_GetFaultEdgeCount(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return 0;
    return s_ch[ch].fault_edges;
}

void IR_LED_ClearFault(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    s_ch[ch].fault_latched = false;
    /* Restart the blanking window so the re-arm gets a full grace period
       rather than latching again on the next Task() call. */
    s_ch[ch].gate_on_tick = HAL_GetTick();
    apply_channel(ch);
}

/* ---------------------------------------------------------------------------
   Sensor sync
   --------------------------------------------------------------------------- */

void IR_LED_OnVSync(void)
{
    s_line_counter = 0;
    s_frame_counter++;
    s_last_vsync_ms = HAL_GetTick();
    s_vsync_seen    = true;
    fp_fire(true);

    if (!s_master_enable) return;

    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode != IR_MODE_FRAME_STROBE || s_ch[i].fault_latched) continue;
        gate_set((IR_Channel_t)i, s_ch[i].line_start == 0);
    }
}

void IR_LED_OnHSync(void)
{
    uint32_t line = ++s_line_counter;

    if (!s_master_enable) return;

    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode != IR_MODE_FRAME_STROBE || s_ch[i].fault_latched) continue;
        if (line == s_ch[i].line_start) gate_set((IR_Channel_t)i, true);
        if (line == s_ch[i].line_end)   gate_set((IR_Channel_t)i, false);
    }
}

void IR_LED_OnFaultEdge(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    /* Deliberately does not gate off or latch. The line legitimately toggles
       on every startup and every strobe off-window; IR_LED_Task() decides
       what is real. This only counts the edge so intermittent LED faults are
       visible in "ir status". */
    s_ch[ch].fault_edges++;
}

/* ---------------------------------------------------------------------------
   Hardware gate PWM (TIM12 -> PB14/PB15 -> LT3763 PWM pin)
   --------------------------------------------------------------------------- */

/* Programs TIM12 for this channel and hands the pin over. Output polarity is
   ACTIVE LOW (CCxP = 1): in PWM mode 1 the output is active while CNT < CCR,
   so an active-low output sits LOW for duty_pct of the period. PWM_DIMx LOW
   is LEDs ON through the Q3/Q2 inverter, so the requested duty is the
   fraction of time the LEDs are lit. The inverter cancels out and the caller
   never has to think in complements. */
static void gate_pwm_apply(IR_Channel_t ch)
{
    uint32_t period, on, psc, arr, ccr;

    period = s_gate_period_ticks;
    on     = s_gate_on_ticks[ch];

    if (period == 0U || on == 0U) {     /* steady off */
        gate_pin_to_gpio(ch);
        gate_set(ch, false);
        return;
    }
    if (on >= period) {                 /* steady on */
        gate_pin_to_gpio(ch);
        gate_set(ch, true);
        return;
    }

    __HAL_RCC_TIM12_CLK_ENABLE();

    /* TIM12 is 16 bit, so a period longer than 65536 ticks needs a
       prescaler. When the period is a whole number of microseconds, round
       the prescaler up to a multiple of IR_TICKS_PER_US so each count is a
       whole number of microseconds too - that keeps millisecond on/off
       times exact (110ms -> PSC 128, ARR 55000, CCR 16500) instead of
       landing a fraction of a count off. */
    psc = (period + 65535U) / 65536U;
    if (psc > 1U && (period % IR_TICKS_PER_US) == 0U) {
        psc = ((psc + IR_TICKS_PER_US - 1U) / IR_TICKS_PER_US) * IR_TICKS_PER_US;
    }
    arr = period / psc;
    if (arr < 2U) arr = 2U;

    ccr = on / psc;
    if (ccr == 0U)  ccr = 1U;
    if (ccr >= arr) ccr = arr - 1U;

    TIM12->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_OPM);   /* free-running, not frame pulse */
    TIM12->PSC  = (uint16_t)(psc - 1U);
    TIM12->ARR  = (uint16_t)(arr - 1U);

    if (ch == IR_CH0) {
        /* PWM mode 1 on CH1, preload enabled */
        TIM12->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S);
        TIM12->CCMR1 |= (6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
        TIM12->CCR1   = (uint16_t)ccr;
        TIM12->CCER  |= TIM_CCER_CC1P | TIM_CCER_CC1E;   /* active low, enabled */
    } else {
        TIM12->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
        TIM12->CCMR1 |= (6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
        TIM12->CCR2   = (uint16_t)ccr;
        TIM12->CCER  |= TIM_CCER_CC2P | TIM_CCER_CC2E;
    }

    TIM12->EGR  = TIM_EGR_UG;
    TIM12->SR   = 0U;
    TIM12->CR1 |= TIM_CR1_CEN;

    /* Record what the hardware is really doing after rounding. */
    s_gate_period_ticks  = psc * arr;
    s_gate_on_ticks[ch]  = psc * ccr;
    s_gate_pwm_on[ch]    = true;
    gate_pin_to_tim12(ch);
    /* The timer owns the pin now; gate_is_on is meaningless while chopping,
       and fault confirmation sits out (see IR_LED_Task). */
    s_ch[ch].gate_is_on = false;
}

bool IR_LED_SetGatePwm(IR_Channel_t ch, uint32_t freq_hz, uint32_t duty_pct)
{
    uint32_t period;

    if (ch >= IR_CH_COUNT) return false;
    evict_other_tim12_user(ch, IR_MODE_FRAME_PULSE);

    if (freq_hz < IR_GATE_PWM_MIN_HZ) freq_hz = IR_GATE_PWM_MIN_HZ;
    if (freq_hz > IR_GATE_PWM_MAX_HZ) freq_hz = IR_GATE_PWM_MAX_HZ;
    if (duty_pct > 100U)              duty_pct = 100U;

    period = IR_TIM_CLK_HZ / freq_hz;
    s_gate_period_ticks = period;
    s_gate_on_ticks[ch] = (uint32_t)(((uint64_t)period * duty_pct) / 100U);
    s_ch[ch].mode       = IR_MODE_GATE_PWM;
    apply_channel(ch);
    return true;
}

bool IR_LED_SetGatePulse(IR_Channel_t ch, uint32_t on_us, uint32_t off_us)
{
    uint64_t period_us;

    if (ch >= IR_CH_COUNT) return false;
    evict_other_tim12_user(ch, IR_MODE_FRAME_PULSE);

    period_us = (uint64_t)on_us + off_us;
    if (period_us < IR_GATE_PERIOD_MIN_US || period_us > IR_GATE_PERIOD_MAX_US) {
        return false;
    }

    s_gate_period_ticks = (uint32_t)(period_us * IR_TICKS_PER_US);
    s_gate_on_ticks[ch] = on_us * IR_TICKS_PER_US;
    s_ch[ch].mode       = IR_MODE_GATE_PWM;
    apply_channel(ch);
    return true;
}

void IR_LED_StopGatePwm(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    s_gate_on_ticks[ch] = 0U;
    s_ch[ch].mode       = IR_MODE_OFF;
    apply_channel(ch);
}

bool IR_LED_GetGatePwm(IR_Channel_t ch, uint32_t *freq_hz, uint32_t *duty_pct)
{
    uint32_t period = s_gate_period_ticks;

    if (ch >= IR_CH_COUNT) return false;
    if (freq_hz)  *freq_hz  = (period != 0U) ? (IR_TIM_CLK_HZ + period / 2U) / period : 0U;
    if (duty_pct) *duty_pct = (period != 0U)
                              ? (uint32_t)(((uint64_t)s_gate_on_ticks[ch] * 100U + period / 2U) / period)
                              : 0U;
    return s_gate_pwm_on[ch] && s_ch[ch].mode == IR_MODE_GATE_PWM;
}

bool IR_LED_GetGatePulse(IR_Channel_t ch, uint32_t *on_us, uint32_t *off_us)
{
    uint32_t period = s_gate_period_ticks;
    uint32_t on;

    if (ch >= IR_CH_COUNT) return false;
    on = s_gate_on_ticks[ch];
    if (on > period) on = period;
    if (on_us)  *on_us  = on / IR_TICKS_PER_US;
    if (off_us) *off_us = (period - on) / IR_TICKS_PER_US;
    return s_gate_pwm_on[ch] && s_ch[ch].mode == IR_MODE_GATE_PWM;
}

/* ---------------------------------------------------------------------------
   Frame pulse (TIM12 one-pulse, started from VSYNC or the TIM7 fallback)
   --------------------------------------------------------------------------- */

#define IR_FP_TIM7_IRQ_PRIO   1U     /* same as SN0_VS: both start a frame */

/* TIM12 has one counter for both channels, so frame pulse and gate PWM
   cannot run side by side. Whichever is being set up switches the other
   channel off if it is using the conflicting mode. */
static void evict_other_tim12_user(IR_Channel_t ch, IR_LedMode_t conflicting)
{
    IR_Channel_t other = (ch == IR_CH0) ? IR_CH1 : IR_CH0;

    if (s_ch[other].mode == conflicting) {
        s_ch[other].mode = IR_MODE_OFF;
        apply_channel(other);
    }
}

/* TIM7: a free-running IR_FRAME_PULSE_FPS tick that stands in for VSYNC.
   Left running once frame pulse has been used; its ISR does nothing while
   real VSYNC edges keep arriving. */
static void fp_tim7_start(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();
    if (TIM7->CR1 & TIM_CR1_CEN) return;

    TIM7->CR1  = 0U;
    TIM7->PSC  = (uint16_t)(IR_TICKS_PER_US - 1U);                 /* 1us */
    TIM7->ARR  = (uint16_t)((1000000U / IR_FRAME_PULSE_FPS) - 1U); /* 33333us */
    TIM7->EGR  = TIM_EGR_UG;
    TIM7->SR   = 0U;
    TIM7->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM7_IRQn, IR_FP_TIM7_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1  = TIM_CR1_CEN;
}

/* Programs TIM12 for one-pulse operation and hands this channel's pin to it.
   PWM mode 2 with CCR = 1 and ACTIVE LOW polarity: the output is inactive
   (pin high, LED off) while CNT < 1, active (pin low, LED on) from CNT = 1
   to ARR, and in one-pulse mode the counter stops at the update event with
   CNT = 0 - back to inactive. So each start of the counter is exactly one
   pulse of ARR us (1us after the trigger), and a stopped timer is dark. */
static void fp_apply(IR_Channel_t ch)
{
    if (s_fp_width_us == 0U) {
        gate_pin_to_gpio(ch);
        gate_set(ch, false);
        return;
    }

    __HAL_RCC_TIM12_CLK_ENABLE();

    TIM12->CR1 &= ~TIM_CR1_CEN;
    TIM12->CR1 |= TIM_CR1_OPM;
    TIM12->PSC  = (uint16_t)(IR_TICKS_PER_US - 1U);
    TIM12->ARR  = (uint16_t)s_fp_width_us;

    if (ch == IR_CH0) {
        TIM12->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S | TIM_CCMR1_OC1PE);
        TIM12->CCMR1 |= (7UL << TIM_CCMR1_OC1M_Pos);
        TIM12->CCR1   = 1U;
        TIM12->CCER  |= TIM_CCER_CC1P | TIM_CCER_CC1E;
    } else {
        TIM12->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S | TIM_CCMR1_OC2PE);
        TIM12->CCMR1 |= (7UL << TIM_CCMR1_OC2M_Pos);
        TIM12->CCR2   = 1U;
        TIM12->CCER  |= TIM_CCER_CC2P | TIM_CCER_CC2E;
    }
    /* Load PSC/ARR without starting: UG clears CNT, and with CEN clear the
       output sits at the inactive (dark) level. */
    TIM12->EGR = TIM_EGR_UG;
    TIM12->SR  = 0U;

    s_gate_pwm_on[ch]   = true;       /* timer owns the pin */
    gate_pin_to_tim12(ch);
    s_ch[ch].gate_is_on = false;      /* fault confirmation sits out */

    fp_tim7_start();
}

/* One frame start: fire one pulse on every channel in frame-pulse mode.
   Called from interrupt context (SN0_VS EXTI or TIM7). */
static void fp_fire(bool from_vsync)
{
    bool any = false;

    if (!s_master_enable) return;
    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode == IR_MODE_FRAME_PULSE && s_gate_pwm_on[i] &&
            !s_ch[i].fault_latched) {
            any = true;
        }
    }
    if (!any) return;

    TIM12->CNT  = 0U;
    TIM12->CR1 |= TIM_CR1_CEN;
    s_fp_count++;
    s_fp_synced = from_vsync;
}

void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR = ~TIM_SR_UIF;
        if (!s_vsync_seen || (HAL_GetTick() - s_last_vsync_ms) > IR_VSYNC_TIMEOUT_MS) {
            fp_fire(false);
        }
    }
}

bool IR_LED_SetFramePulse(IR_Channel_t ch, uint32_t width_us)
{
    if (ch >= IR_CH_COUNT) return false;

    if (width_us == 0U) {
        s_ch[ch].mode = IR_MODE_OFF;
        apply_channel(ch);
        return true;
    }
    if (width_us > IR_FRAME_PULSE_MAX_US) width_us = IR_FRAME_PULSE_MAX_US;

    evict_other_tim12_user(ch, IR_MODE_GATE_PWM);
    s_fp_width_us = width_us;
    s_ch[ch].mode = IR_MODE_FRAME_PULSE;
    apply_channel(ch);

    /* The width is shared: re-apply the other channel if it is also pulsing
       so both outputs follow the new ARR. */
    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (i != (int)ch && s_ch[i].mode == IR_MODE_FRAME_PULSE) {
            apply_channel((IR_Channel_t)i);
        }
    }
    return true;
}

bool IR_LED_GetFramePulse(IR_Channel_t ch, uint32_t *width_us)
{
    if (ch >= IR_CH_COUNT) return false;
    if (width_us) *width_us = s_fp_width_us;
    return s_ch[ch].mode == IR_MODE_FRAME_PULSE;
}

uint32_t IR_LED_GetFramePulseCount(void) { return s_fp_count; }
bool     IR_LED_FramePulseIsSynced(void) { return s_fp_synced; }

/* ---------------------------------------------------------------------------
   FLASH sync (IR_MODE_FLASH_SYNC): ADC1 analog watchdog on PC3
   ---------------------------------------------------------------------------
   The IMX568 drives TOUT0 at 1.8V. PC3 runs from the 3.3V I/O supply, whose
   VIH is 0.7 x VDDIO = 2.31V guaranteed (0.52 x VDDIO + 0.18 = 1.90V
   characterised, DS14258 Table 58) - above TOUT0's 1.6-1.8V high level, so
   the digital input cannot be trusted with it. PC3 is also ADC12_INP13, so
   ADC1 converts it continuously and two analog watchdogs turn the level into
   edges, with hysteresis:

     AWD1 (12 bit, window [0, HIGH]) flags once the level rises above HIGH
     AWD2 ( 8 bit, window [LOW, max]) flags once it falls below LOW

   Only the watchdog for the edge being waited for has its interrupt enabled,
   so a level that sits high or low produces no interrupts at all - just one
   per crossing. Conversions run free at ~0.8us each (32MHz ADC clock, 12.5
   cycle sample + 12.5 cycle conversion), so the gate follows FLASH within
   about 1-2us. */

#define IR_FS_ADC                 ADC1
#define IR_FS_ADC_COMMON          ADC12_COMMON
#define IR_FS_ADC_CHANNEL         13U       /* PC3 = ADC12_INP13 */
#define IR_FS_ADC_IRQ_PRIO        0U        /* gate timing; same level as faults */
#define IR_FS_ADC_FULL_MV         IR_DAC_VREF_MV   /* VREF+ = VDDA = 3.3V */

static volatile bool     s_fs_running     = false;
static volatile bool     s_fs_level_high  = false;  /* last detected level   */
static volatile bool     s_fs_blocked     = false;  /* max-on cut, until idle */
static volatile uint32_t s_fs_edges       = 0U;
static volatile uint32_t s_fs_on_ms       = 0U;     /* tick at the last on   */
static bool              s_fs_active_high = IR_FLASH_ACTIVE_HIGH_DEFAULT;
static uint32_t          s_fs_th_high_mv  = IR_FLASH_TH_HIGH_MV;
static uint32_t          s_fs_th_low_mv   = IR_FLASH_TH_LOW_MV;

static bool fs_active(void)
{
    return s_fs_level_high == s_fs_active_high;
}

static bool fs_gate_wanted(void)
{
    return s_fs_running && s_master_enable && fs_active() && !s_fs_blocked;
}

/* Drives every FLASH_SYNC channel's gate from the current FLASH state. */
static void fs_drive_gates(void)
{
    bool on = fs_gate_wanted();

    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode == IR_MODE_FLASH_SYNC) {
            gate_set((IR_Channel_t)i, on && !s_ch[i].fault_latched);
        }
    }
    if (on) {
        s_fs_on_ms = HAL_GetTick();
    }
}

/* Arms the watchdog for the opposite edge of the level just seen. */
static void fs_arm(void)
{
    IR_FS_ADC->IER = s_fs_level_high ? ADC_IER_AWD2IE : ADC_IER_AWD1IE;
    IR_FS_ADC->ISR = ADC_ISR_AWD1 | ADC_ISR_AWD2;
}

static void fs_load_thresholds(void)
{
    uint32_t hi12 = (s_fs_th_high_mv * 4095U) / IR_FS_ADC_FULL_MV;
    uint32_t lo8  = (s_fs_th_low_mv  * 255U)  / IR_FS_ADC_FULL_MV;

    if (hi12 > 4095U) hi12 = 4095U;
    if (lo8  > 255U)  lo8  = 255U;
    IR_FS_ADC->TR1 = (hi12 << ADC_TR1_HT1_Pos) | (0U << ADC_TR1_LT1_Pos);
    IR_FS_ADC->TR2 = (255U << ADC_TR2_HT2_Pos) | (lo8 << ADC_TR2_LT2_Pos);
}

static bool fs_wait(volatile uint32_t *reg, uint32_t mask, bool set)
{
    uint32_t t0 = HAL_GetTick();

    while (((*reg & mask) != 0U) != set) {
        if ((HAL_GetTick() - t0) > 10U) return false;
    }
    return true;
}

static void fs_start(void)
{
    ADC_TypeDef *adc = IR_FS_ADC;

    if (s_fs_running) return;

    flash_release();   /* already analog; PC3 is never driven */

    __HAL_RCC_ADC_CLK_ENABLE();
    /* Asynchronous clock (adc_ker_ck = HCLK 64MHz, shared with the DAC's
       ADCDACSEL) divided by 2: 32MHz, inside the 75MHz limit. */
    IR_FS_ADC_COMMON->CCR = (IR_FS_ADC_COMMON->CCR & ~(ADC_CCR_CKMODE | ADC_CCR_PRESC))
                          | (1UL << ADC_CCR_PRESC_Pos);

    if (adc->CR & ADC_CR_ADEN) {
        adc->CR |= ADC_CR_ADSTP;
        (void)fs_wait(&adc->CR, ADC_CR_ADSTP, false);
        adc->CR |= ADC_CR_ADDIS;
        (void)fs_wait(&adc->CR, ADC_CR_ADEN, false);
    }
    adc->CR = ADC_CR_ADVREGEN;              /* DEEPPWD = 0, regulator on */
    HAL_Delay(1);                           /* tADCVREG_STUP is 20us     */
    adc->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
    (void)fs_wait(&adc->CR, ADC_CR_ADCAL, false);

    adc->ISR = ADC_ISR_ADRDY;
    adc->CR  = ADC_CR_ADVREGEN | ADC_CR_ADEN;
    (void)fs_wait(&adc->ISR, ADC_ISR_ADRDY, true);

    adc->SMPR2 = (adc->SMPR2 & ~ADC_SMPR2_SMP13) | (2UL << ADC_SMPR2_SMP13_Pos); /* 12.5 */
    adc->SQR1  = IR_FS_ADC_CHANNEL << ADC_SQR1_SQ1_Pos;                          /* L = 1 */
    adc->CFGR  = ADC_CFGR_CONT | ADC_CFGR_OVRMOD
               | ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN
               | (IR_FS_ADC_CHANNEL << ADC_CFGR_AWD1CH_Pos);
    adc->AWD2CR = 1UL << IR_FS_ADC_CHANNEL;
    fs_load_thresholds();

    adc->IER = 0U;
    adc->CR |= ADC_CR_ADSTART;
    (void)fs_wait(&adc->ISR, ADC_ISR_EOC, true);

    /* Seed the state from the first conversion, then wait for its opposite. */
    s_fs_level_high = ((adc->DR * IR_FS_ADC_FULL_MV) / 4095U) > s_fs_th_high_mv;
    s_fs_blocked    = false;
    s_fs_running    = true;
    fs_arm();
    HAL_NVIC_SetPriority(ADC1_IRQn, IR_FS_ADC_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
}

/* Stops the ADC. PC3 stays an analog input - it is never driven. */
static void fs_stop(void)
{
    ADC_TypeDef *adc = IR_FS_ADC;

    if (!s_fs_running) return;

    HAL_NVIC_DisableIRQ(ADC1_IRQn);
    adc->IER = 0U;
    adc->CR |= ADC_CR_ADSTP;
    (void)fs_wait(&adc->CR, ADC_CR_ADSTP, false);
    adc->CR |= ADC_CR_ADDIS;
    (void)fs_wait(&adc->CR, ADC_CR_ADEN, false);
    adc->ISR = 0xFFFFFFFFU;
    s_fs_running = false;
}

/* Runs the ADC only while some channel needs it and output is enabled. */
static void fs_sync_running(void)
{
    bool want = false;

    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode == IR_MODE_FLASH_SYNC) want = true;
    }
    want = want && s_master_enable;

    if (want && !s_fs_running)      fs_start();
    else if (!want && s_fs_running) fs_stop();
}

void ADC1_IRQHandler(void)
{
    uint32_t isr = IR_FS_ADC->ISR & IR_FS_ADC->IER;

    if (isr & ADC_ISR_AWD1) {           /* rose above HIGH */
        s_fs_level_high = true;
    } else if (isr & ADC_ISR_AWD2) {    /* fell below LOW  */
        s_fs_level_high = false;
    } else {
        IR_FS_ADC->ISR = IR_FS_ADC->ISR & (ADC_ISR_AWD1 | ADC_ISR_AWD2);
        return;
    }
    s_fs_edges++;
    if (!fs_active()) {
        s_fs_blocked = false;           /* exposure over: re-arm max-on cut */
    }
    fs_arm();
    fs_drive_gates();
}

/* Called from IR_LED_Task(): ends a pulse that has outlasted
   IR_FLASH_MAX_ON_MS (FLASH stuck active), until FLASH next goes idle. */
static void fs_task(void)
{
    if (!s_fs_running || s_fs_blocked || !fs_active()) return;
    if ((HAL_GetTick() - s_fs_on_ms) > IR_FLASH_MAX_ON_MS) {
        s_fs_blocked = true;
        fs_drive_gates();
    }
}

void IR_LED_SetFlashPolarity(bool active_high)
{
    s_fs_active_high = active_high;
    if (s_fs_running) {
        s_fs_blocked = false;
        fs_drive_gates();
    }
}

bool IR_LED_GetFlashActiveHigh(void) { return s_fs_active_high; }

bool IR_LED_SetFlashThresholds_mV(uint32_t high_mv, uint32_t low_mv)
{
    if (low_mv >= high_mv || high_mv >= IR_FS_ADC_FULL_MV) return false;
    s_fs_th_high_mv = high_mv;
    s_fs_th_low_mv  = low_mv;
    if (s_fs_running) {
        /* TR1/TR2 are only written with conversions stopped. */
        IR_FS_ADC->CR |= ADC_CR_ADSTP;
        (void)fs_wait(&IR_FS_ADC->CR, ADC_CR_ADSTP, false);
        fs_load_thresholds();
        fs_arm();
        IR_FS_ADC->CR |= ADC_CR_ADSTART;
    }
    return true;
}

void IR_LED_GetFlashThresholds_mV(uint32_t *high_mv, uint32_t *low_mv)
{
    if (high_mv) *high_mv = s_fs_th_high_mv;
    if (low_mv)  *low_mv  = s_fs_th_low_mv;
}

bool IR_LED_GetFlashSyncStatus(uint32_t *level_mv, bool *active, uint32_t *edges)
{
    if (level_mv) *level_mv = s_fs_running
                              ? ((IR_FS_ADC->DR & 0xFFFU) * IR_FS_ADC_FULL_MV) / 4095U : 0U;
    if (active)   *active   = s_fs_running && fs_active();
    if (edges)    *edges    = s_fs_edges;
    return s_fs_running;
}

/* ---------------------------------------------------------------------------
   DAC-generated PWM (TIM13 / TIM14)
   --------------------------------------------------------------------------- */

/* Drives one channel's DAC to its setpoint (high phase) or to zero (low
   phase). Deliberately bypasses IR_LED_SetCurrentRaw() so raw_code stays the
   amplitude of record and is not overwritten by the modulation itself. */
static inline void pwm_drive(IR_Channel_t ch, bool high)
{
    DAC1_SetValue(s_ch[ch].dac_channel, high ? s_ch[ch].raw_code : 0U);
}

static void dac_tim_stop(IR_Channel_t ch)
{
    s_dac_tim[ch]->CR1  &= ~TIM_CR1_CEN;
    s_dac_tim[ch]->DIER  = 0U;
    s_pwm_freq_ch[ch]    = 0U;
}

bool IR_LED_SetDacPwm(IR_Channel_t ch, uint32_t freq_hz, uint32_t duty_pct)
{
    uint32_t period, psc, arr, ccr;
    TIM_TypeDef *tim;

    if (ch >= IR_CH_COUNT) return false;

    if (freq_hz < IR_DAC_PWM_MIN_HZ) freq_hz = IR_DAC_PWM_MIN_HZ;
    if (freq_hz > IR_DAC_PWM_MAX_HZ) freq_hz = IR_DAC_PWM_MAX_HZ;
    if (duty_pct > 100U)             duty_pct = 100U;

    /* 0% and 100% are steady states, not waveforms - running the timer for
       them would only add interrupts and jitter for no benefit. */
    if (duty_pct == 0U || duty_pct == 100U) {
        s_pwm_on[ch]   = false;
        s_pwm_duty[ch] = duty_pct;
        pwm_drive(ch, duty_pct == 100U);
        dac_tim_stop(ch);
        return true;
    }

    if (ch == IR_CH0) { __HAL_RCC_TIM13_CLK_ENABLE(); }
    else              { __HAL_RCC_TIM14_CLK_ENABLE(); }
    tim = s_dac_tim[ch];

    /* Pick the smallest prescaler that brings one period inside the 16-bit
       counter, so the duty resolution stays as fine as the timer allows. */
    period = IR_TIM_CLK_HZ / freq_hz;
    psc    = (period + 65535U) / 65536U;
    if (psc == 0U) psc = 1U;
    arr    = (period / psc);
    if (arr == 0U) arr = 1U;
    if (arr > 65536U) arr = 65536U;

    ccr = (arr * duty_pct) / 100U;
    if (ccr == 0U)   ccr = 1U;
    if (ccr >= arr)  ccr = arr - 1U;

    tim->CR1 &= ~TIM_CR1_CEN;
    tim->PSC  = (uint16_t)(psc - 1U);
    tim->ARR  = (uint16_t)(arr - 1U);
    tim->CCR1 = (uint16_t)ccr;
    tim->EGR  = TIM_EGR_UG;        /* load PSC/ARR now */
    tim->SR   = 0U;                /* UG above sets UIF; drop it */

    s_pwm_on[ch]      = true;
    s_pwm_duty[ch]    = duty_pct;
    s_pwm_freq_ch[ch] = IR_TIM_CLK_HZ / (psc * arr);

    /* CCxE stays clear: these timers' outputs are not routed to any pin, the
       compare is used purely as a second interrupt per period. */
    tim->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;

    HAL_NVIC_SetPriority(s_dac_irq[ch], 2, 0);   /* below UART4 so the console
                                                    stays responsive */
    HAL_NVIC_EnableIRQ(s_dac_irq[ch]);
    tim->CR1 |= TIM_CR1_CEN;
    return true;
}

void IR_LED_StopDacPwm(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    s_pwm_on[ch]   = false;
    s_pwm_duty[ch] = 0U;
    dac_tim_stop(ch);
    /* Back to the plain DC setpoint the console last asked for. */
    DAC1_SetValue(s_ch[ch].dac_channel, s_ch[ch].raw_code);
}

bool IR_LED_GetDacPwm(IR_Channel_t ch, uint32_t *freq_hz, uint32_t *duty_pct)
{
    if (ch >= IR_CH_COUNT) return false;
    if (freq_hz)  *freq_hz  = s_pwm_freq_ch[ch];
    if (duty_pct) *duty_pct = s_pwm_duty[ch];
    return s_pwm_on[ch];
}

/* One ISR per DAC channel. Update raises the output to the setpoint, the
   compare match drops it to 0V. Overrides the weak handlers in
   startup_stm32h563rgvx.s. */
static inline void dac_tim_isr(IR_Channel_t ch)
{
    TIM_TypeDef *tim = s_dac_tim[ch];
    uint32_t sr = tim->SR;

    if (sr & TIM_SR_UIF) {
        tim->SR = ~TIM_SR_UIF;
        if (s_pwm_on[ch]) pwm_drive(ch, true);
    }
    if (sr & TIM_SR_CC1IF) {
        tim->SR = ~TIM_SR_CC1IF;
        if (s_pwm_on[ch]) pwm_drive(ch, false);
    }
}

void TIM13_IRQHandler(void) { dac_tim_isr(IR_CH0); }
void TIM14_IRQHandler(void) { dac_tim_isr(IR_CH1); }

/**
  * @brief EXTI dispatcher for SN0_HS, SN0_VS, IR_FAULT0 and IR_FAULT1.
  *
  * The HAL permits only one non-weak definition, so if another module ever
  * needs an EXTI callback, merge its cases in here rather than defining a
  * second copy.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case SN0_HS_Pin:    IR_LED_OnHSync();            break;
        case SN0_VS_Pin:    IR_LED_OnVSync();            break;
        case IR_FAULT0_Pin: IR_LED_OnFaultEdge(IR_CH0);  break;
        case IR_FAULT1_Pin: IR_LED_OnFaultEdge(IR_CH1);  break;
        default: break;
    }
}
