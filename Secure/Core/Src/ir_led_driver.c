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
static IR_FlashSource_t  s_flash_src     = IR_FLASH_SRC_MCU;
static uint32_t          s_limit_ma      = IR_LED_DEFAULT_LIMIT_MA;

/* DAC-PWM state. TIM12 is shared, so the frequency is global while the duty
   is per channel. pwm_on gates the ISR so a channel that is not being
   modulated never has its DAC touched from interrupt context. */
static volatile bool     s_pwm_on[IR_CH_COUNT] = { false, false };
static volatile uint32_t s_pwm_duty[IR_CH_COUNT] = { 0U, 0U };

static volatile bool     s_gate_pwm_on[IR_CH_COUNT] = { false, false };
static volatile uint32_t s_gate_duty[IR_CH_COUNT] = { 0U, 0U };
static uint32_t          s_gate_freq = 0U;

/* Timers count at the APB1 timer clock. AHB and APB1 are both /1 off a 64MHz
   SYSCLK (see SystemClock_Config), and with an APB prescaler of 1 the timer
   clock equals PCLK rather than twice it. */
#define IR_TIM_CLK_HZ            64000000U

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

static void apply_channel(IR_Channel_t ch)
{
    IR_ChannelState_t *c = &s_ch[ch];

    /* Any state other than "actively chopping" must first take the pin back
       from TIM12, otherwise the timer would keep driving it regardless of
       what the mode says. */
    if (c->mode != IR_MODE_GATE_PWM || !s_master_enable || c->fault_latched) {
        if (s_gate_pwm_on[ch]) {
            s_gate_pwm_on[ch] = false;
            TIM12->CCER &= ~((ch == IR_CH0) ? TIM_CCER_CC1E : TIM_CCER_CC2E);
            if (!s_gate_pwm_on[IR_CH0] && !s_gate_pwm_on[IR_CH1]) {
                TIM12->CR1 &= ~TIM_CR1_CEN;
                s_gate_freq = 0U;
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

static void flash_configure(IR_FlashSource_t src)
{
    GPIO_InitTypeDef init = {0};

    init.Pin   = FLASH_Pin;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_HIGH;

    if (src == IR_FLASH_SRC_MCU) {
        /* Park it low before switching to an output so the pin never drives
           a stale 1 into the inverter sources. */
        HAL_GPIO_WritePin(FLASH_GPIO_Port, FLASH_Pin, GPIO_PIN_RESET);
        init.Mode = GPIO_MODE_OUTPUT_PP;
    } else {
        init.Mode = GPIO_MODE_INPUT;
    }
    HAL_GPIO_Init(FLASH_GPIO_Port, &init);
    s_flash_src = src;
}

bool IR_LED_SetFlashSource(IR_FlashSource_t src)
{
    if (src == s_flash_src) {
        return true;
    }
    if (src == IR_FLASH_SRC_HOST) {
        /* Releasing FLASH leaves the source of every inverter floating, so
           only channels that are meant to follow the host may be active. */
        for (int i = 0; i < IR_CH_COUNT; i++) {
            if (s_ch[i].mode != IR_MODE_HOST_FLASH && s_ch[i].mode != IR_MODE_OFF) {
                return false;
            }
        }
    }
    flash_configure(src);
    for (int i = 0; i < IR_CH_COUNT; i++) {
        apply_channel((IR_Channel_t)i);
    }
    return true;
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

void IR_LED_SetCurrentRaw(IR_Channel_t ch, uint16_t code_12bit)
{
    if (ch >= IR_CH_COUNT) return;
    if (code_12bit > DAC1_CODE_MAX) code_12bit = DAC1_CODE_MAX;

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
    if (milliamps > IR_LED_MAX_CURRENT_MA) {
        milliamps = IR_LED_MAX_CURRENT_MA;
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

    /* Report back what the quantised code actually produces, not what was
       asked for - at ~0.8mV per code the two differ, and a caller printing
       the request would be quietly lying about the pin voltage. */
    return code_to_mv((uint16_t)code);
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

    /* Take the FLASH net. Until this runs PC3 is whatever MX_GPIO_Init() left
       it as, which is an input, i.e. the inverter sources are floating. */
    flash_configure(IR_FLASH_SRC_MCU);

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
        s_gate_duty[i]   = 0U;
    }
    s_gate_freq = 0U;
}

void IR_LED_Task(void)
{
    uint32_t now = HAL_GetTick();

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
        if ((now - c->gate_on_tick) < IR_FAULT_BLANKING_MS) {
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
    if (!enable && s_flash_src == IR_FLASH_SRC_HOST) {
        /* With the host owning FLASH, PWM_DIMx alone cannot force a channel
           dark. Reclaim the net so "disabled" really means dark, and drop
           any HOST_FLASH channels to OFF so the state stays coherent. */
        for (int i = 0; i < IR_CH_COUNT; i++) {
            if (s_ch[i].mode == IR_MODE_HOST_FLASH) {
                s_ch[i].mode = IR_MODE_OFF;
            }
        }
        flash_configure(IR_FLASH_SRC_MCU);
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
    uint32_t period, psc, arr, ccr;

    if (s_gate_duty[ch] == 0U) {        /* steady off */
        gate_pin_to_gpio(ch);
        gate_set(ch, false);
        return;
    }
    if (s_gate_duty[ch] >= 100U) {      /* steady on */
        gate_pin_to_gpio(ch);
        gate_set(ch, true);
        return;
    }

    __HAL_RCC_TIM12_CLK_ENABLE();

    period = IR_TIM_CLK_HZ / ((s_gate_freq != 0U) ? s_gate_freq : IR_GATE_PWM_MIN_HZ);
    psc    = (period + 65535U) / 65536U;
    if (psc == 0U) psc = 1U;
    arr    = (period / psc);
    if (arr == 0U) arr = 1U;
    if (arr > 65536U) arr = 65536U;

    ccr = (arr * s_gate_duty[ch]) / 100U;
    if (ccr == 0U)  ccr = 1U;
    if (ccr >= arr) ccr = arr - 1U;

    TIM12->CR1 &= ~TIM_CR1_CEN;
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

    s_gate_freq       = IR_TIM_CLK_HZ / (psc * arr);
    s_gate_pwm_on[ch] = true;
    gate_pin_to_tim12(ch);
    /* The timer owns the pin now; gate_is_on is meaningless while chopping,
       and fault confirmation sits out (see IR_LED_Task). */
    s_ch[ch].gate_is_on = false;
}

bool IR_LED_SetGatePwm(IR_Channel_t ch, uint32_t freq_hz, uint32_t duty_pct)
{
    if (ch >= IR_CH_COUNT) return false;

    if (freq_hz < IR_GATE_PWM_MIN_HZ) freq_hz = IR_GATE_PWM_MIN_HZ;
    if (freq_hz > IR_GATE_PWM_MAX_HZ) freq_hz = IR_GATE_PWM_MAX_HZ;
    if (duty_pct > 100U)              duty_pct = 100U;

    s_gate_freq     = freq_hz;
    s_gate_duty[ch] = duty_pct;
    s_ch[ch].mode   = IR_MODE_GATE_PWM;
    apply_channel(ch);
    return true;
}

void IR_LED_StopGatePwm(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    s_gate_duty[ch] = 0U;
    s_ch[ch].mode   = IR_MODE_OFF;
    apply_channel(ch);
}

bool IR_LED_GetGatePwm(IR_Channel_t ch, uint32_t *freq_hz, uint32_t *duty_pct)
{
    if (ch >= IR_CH_COUNT) return false;
    if (freq_hz)  *freq_hz  = s_gate_freq;
    if (duty_pct) *duty_pct = s_gate_duty[ch];
    return s_gate_pwm_on[ch];
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
