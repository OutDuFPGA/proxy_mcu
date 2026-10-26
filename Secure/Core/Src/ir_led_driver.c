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
#include <math.h>

/* Free-running 32-bit timestamp at HCLK - see IR_LED_Init() (TIM5). */
#define IR_CYCCNT  (TIM5->CNT)

/* The project builds at -O0. The SN0_HS interrupt fires on both edges in an
   hsync window (~134k/s, ~470 cycles each at 64 MHz), so its path is built
   at -O2 or it overruns and loses HSYNC edges (seen 2026-10-14: lines/frame
   2203 instead of 2233 and a TIM12 cut-off every window). */
#define IR_HOT  __attribute__((optimize("O2")))

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
/* High-current pulse state (IR_LED_SetHighPulse). Bit n of s_hp_mask = channel
   n is in hpulse. s_hp_code is the pulse DAC code - deliberately NOT raw_code,
   so the hard cap on raw_code still holds for every other path. s_hp_left:
   pulses still to fire, IR_HP_ENDLESS for continuous. */
#define IR_HP_ENDLESS            0xFFFFFFFFU
static volatile uint32_t s_hp_mask       = 0U;
static volatile uint16_t s_hp_code[IR_CH_COUNT]    = { 0U, 0U };
static volatile uint32_t s_hp_left[IR_CH_COUNT]    = { 0U, 0U };
static volatile uint32_t s_hp_fired[IR_CH_COUNT]   = { 0U, 0U };
static volatile uint32_t s_hp_skipped[IR_CH_COUNT] = { 0U, 0U };
static volatile uint32_t s_hp_last_cyc   = 0U;
static volatile bool     s_hp_any_fired  = false;
/* Strobe: channels whose DAC_DIMx is switched with FLASH (setpoint while
   FLASH is low, 0 V while high). Bit n = channel n. */
static volatile uint32_t s_dac_gate      = 0U;
/* DAC test: DAC_DIMx held at a raw voltage that bypasses the current cap.
   Ended by any setpoint write (SetCurrentRaw) or IR_LED_ClearDacTest(). */
static bool              s_dactest[2]    = {false, false};
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
static void hp_clear(IR_Channel_t ch);         /* fwd decl */
static void hp_task(void);                     /* fwd decl */
static void hsp_timer_cut(void);               /* fwd decl */
static void hsp_flash_pin(bool out);            /* fwd decl */
static void hsp_task(void);                    /* fwd decl */
static bool hsp_active(void);                  /* fwd decl */

static void apply_channel(IR_Channel_t ch)
{
    IR_ChannelState_t *c = &s_ch[ch];

    /* Start or stop the PC3 ADC to match whether any channel needs it. */
    fs_sync_running();

    /* hsync pulse owns TIM12: either TIM12 mode on any channel ends it. */
    if (hsp_active() && (c->mode == IR_MODE_GATE_PWM || c->mode == IR_MODE_FRAME_PULSE)) {
        IR_LED_StopHsyncPulse(IR_CH0);
        IR_LED_StopHsyncPulse(IR_CH1);
    }

    /* hpulse lives inside FRAME_PULSE: any other state, a fault or the
       master enable going off ends it, DAC first. */
    if ((s_hp_mask & (1UL << ch)) != 0U &&
        (c->mode != IR_MODE_FRAME_PULSE || !s_master_enable || c->fault_latched)) {
        hp_clear(ch);
    }

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
    /* PC3 itself is configured by MX_GPIO_Init() (digital input + EXTI3 for
       the FLASH monitor); nothing here touches it. */
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
    s_dactest[ch] = false;                 /* a real setpoint ends any DAC test */

    /* Hard cap. Every DAC write - mA, mV, raw code, ramp, DAC PWM - goes
       through raw_code, which is only ever set here. */
    if (code_12bit > hard_limit_code()) code_12bit = hard_limit_code();

    s_ch[ch].raw_code = code_12bit;
    /* Strobe: outside the exposure (FLASH high) the DAC stays at 0 V; the
       new setpoint is applied at the next FLASH falling edge. */
    if ((s_dac_gate & (1UL << ch)) != 0U &&
        HAL_GPIO_ReadPin(FLASH_GPIO_Port, FLASH_Pin) == GPIO_PIN_SET) {
        DAC1_SetValue(s_ch[ch].dac_channel, 0U);
        return;
    }
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

    /* Output enable starts ON (software flag only). PA11 is NOT touched
       here: IR_ENB_MCU/IR_FBC_MCU (PA11/PA12) drive the IR-cut filter's
       DRV8210 (EN/PH) and must only ever be pulsed - see IR_LED_IrCut(). */
    s_master_enable = true;
    HAL_GPIO_WritePin(IR_ENB_MCU_GPIO_Port, IR_ENB_MCU_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IR_FBC_MCU_GPIO_Port, IR_FBC_MCU_Pin, GPIO_PIN_RESET);

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

    /* Sync/FLASH monitors and the hsync engine timestamp edges with TIM5, a
       free-running 32-bit counter at HCLK (APB1 /1, so 64 MHz = one tick per
       core cycle). Not the DWT cycle counter: that only runs while a debug
       probe keeps the debug domain powered - on 2026-10-14 it froze a few
       seconds after the ST-LINK let go, so every period read 0 ("NOT
       toggling") and hsync refused to start. */
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0U;
    TIM5->PSC = 0U;
    TIM5->ARR = 0xFFFFFFFFU;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;

#if IR_PC3_OUTPUT
    /* IMX678: PC3 is an MCU output for good - push-pull LOW from here on
       (gpio.c brought it up as the IMX568 input; this replaces that). */
    hsp_flash_pin(true);
#endif

    /* Operating mode: both channels stay IR_MODE_OFF, i.e. PWM_DIMx HIGH for
       good - the sensor's FLASH switches the LED through Q2/Q3. Only the CH0
       current is set; CH1 has no driver fitted and stays at 0. */
    IR_LED_SetCurrent_mA(IR_CH0, IR_BOOT_CURRENT_MA);
}

/* Non-blocking current ramp, stepped from IR_LED_Task() once per tick. */
static struct {
    bool     active;
    uint32_t from_ma, to_ma, t0_ms, dur_ms, last_ma;
} s_ramp[IR_CH_COUNT];

bool IR_LED_StartRamp_mA(IR_Channel_t ch, uint32_t target_ma, uint32_t time_ms)
{
    if (ch >= IR_CH_COUNT) return false;
    if (target_ma > s_limit_ma) target_ma = s_limit_ma;
    s_ramp[ch].active  = false;
    if (time_ms == 0U) {
        IR_LED_SetCurrent_mA(ch, target_ma);
        return true;
    }
    s_ramp[ch].from_ma = IR_LED_GetCurrent_mA(ch);
    s_ramp[ch].last_ma = s_ramp[ch].from_ma;
    s_ramp[ch].to_ma   = target_ma;
    s_ramp[ch].dur_ms  = time_ms;
    s_ramp[ch].t0_ms   = HAL_GetTick();
    s_ramp[ch].active  = true;
    return true;
}

void IR_LED_StopRamp(IR_Channel_t ch)
{
    if (ch < IR_CH_COUNT) s_ramp[ch].active = false;
}

bool IR_LED_GetRamp(IR_Channel_t ch, uint32_t *to_ma, uint32_t *left_ms)
{
    if (ch >= IR_CH_COUNT || !s_ramp[ch].active) return false;
    uint32_t el = HAL_GetTick() - s_ramp[ch].t0_ms;
    if (to_ma)   *to_ma   = s_ramp[ch].to_ma;
    if (left_ms) *left_ms = (el < s_ramp[ch].dur_ms) ? s_ramp[ch].dur_ms - el : 0U;
    return true;
}

static void ramp_task(void)
{
    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (!s_ramp[i].active) continue;
        uint32_t el = HAL_GetTick() - s_ramp[i].t0_ms;
        uint32_t ma;
        if (el >= s_ramp[i].dur_ms) {
            ma = s_ramp[i].to_ma;
            s_ramp[i].active = false;
        } else if (s_ramp[i].to_ma >= s_ramp[i].from_ma) {
            ma = s_ramp[i].from_ma + (uint32_t)(((uint64_t)(s_ramp[i].to_ma - s_ramp[i].from_ma) * el) / s_ramp[i].dur_ms);
        } else {
            ma = s_ramp[i].from_ma - (uint32_t)(((uint64_t)(s_ramp[i].from_ma - s_ramp[i].to_ma) * el) / s_ramp[i].dur_ms);
        }
        if (ma != s_ramp[i].last_ma || !s_ramp[i].active) {
            IR_LED_SetCurrent_mA((IR_Channel_t)i, ma);
            s_ramp[i].last_ma = ma;
        }
    }
}

/* ---------------------------------------------------------------------------
   IR-cut filter: DRV8210 in PH/EN mode. IR_FBC_MCU (PA12) = IN1/PH selects
   the direction, IR_ENB_MCU (PA11) = IN2/EN drives the coil while HIGH.
   EN must only be pulsed (the filter latches); held HIGH the coil stays
   powered and heats. Same wiring and defaults as the KL730's ircut.sh.
   --------------------------------------------------------------------------- */
static volatile bool     s_cut_busy     = false;
static volatile uint32_t s_cut_t0       = 0U;
static uint32_t          s_cut_ms       = IR_CUT_PULSE_MS;
static uint8_t           s_cut_night_ph = IR_CUT_NIGHT_PH;
static int8_t            s_cut_last     = -1;      /* -1 unknown, 0 day, 1 night */

bool IR_LED_IrCut(bool night)
{
    uint8_t ph;

    if (s_cut_busy) return false;                 /* one pulse at a time */
    ph = night ? s_cut_night_ph : (uint8_t)(1U - s_cut_night_ph);
    HAL_GPIO_WritePin(IR_FBC_MCU_GPIO_Port, IR_FBC_MCU_Pin, ph ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_cut_t0   = HAL_GetTick();
    s_cut_busy = true;
    HAL_GPIO_WritePin(IR_ENB_MCU_GPIO_Port, IR_ENB_MCU_Pin, GPIO_PIN_SET);   /* drive */
    s_cut_last = night ? 1 : 0;
    return true;
}

static void ircut_task(void)
{
    if (s_cut_busy && (HAL_GetTick() - s_cut_t0) >= s_cut_ms) {
        HAL_GPIO_WritePin(IR_ENB_MCU_GPIO_Port, IR_ENB_MCU_Pin, GPIO_PIN_RESET); /* brake */
        HAL_GPIO_WritePin(IR_FBC_MCU_GPIO_Port, IR_FBC_MCU_Pin, GPIO_PIN_RESET); /* idle */
        s_cut_busy = false;
    }
}

uint32_t IR_LED_IrCutSetPulse_ms(uint32_t ms)
{
    if (ms < IR_CUT_PULSE_MIN_MS) ms = IR_CUT_PULSE_MIN_MS;
    if (ms > IR_CUT_PULSE_MAX_MS) ms = IR_CUT_PULSE_MAX_MS;
    s_cut_ms = ms;
    return ms;
}

void IR_LED_IrCutSwap(void)
{
    s_cut_night_ph ^= 1U;
}

void IR_LED_IrCutStatus(int *last, uint32_t *pulse_ms, uint32_t *night_ph, bool *busy)
{
    if (last)     *last     = s_cut_last;
    if (pulse_ms) *pulse_ms = s_cut_ms;
    if (night_ph) *night_ph = s_cut_night_ph;
    if (busy)     *busy     = s_cut_busy;
}

void IR_LED_Task(void)
{
    ircut_task();
    ramp_task();
    fs_task();
    hp_task();
    hsp_task();

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

    s_master_enable = enable;   /* PA11 is the IR-cut EN now - not touched here */

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

#if (IR_FLASH_SYNC_ENABLE == 0) || IR_PC3_OUTPUT
    if (mode == IR_MODE_FLASH_SYNC) {
        return false;   /* blocked: PC3 must not be used (IR_FLASH_SYNC_ENABLE) */
    }
#endif

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

    if (s_hp_mask != 0U) {
        /* High-current pulse. Rate guard: whatever the trigger (TIM7, VSYNC
           or both), never two pulses closer than IR_HP_MIN_SPACING_US - the
           width limit assumes IR_FRAME_PULSE_FPS. Also never restart a
           pulse that is still running. */
        uint32_t now = IR_CYCCNT;
        uint32_t min_cyc = IR_HP_MIN_SPACING_US * (SystemCoreClock / 1000000U);
        bool fire = false;

        if ((TIM12->CR1 & TIM_CR1_CEN) != 0U ||
            (s_hp_any_fired && (now - s_hp_last_cyc) < min_cyc)) {
            for (int i = 0; i < IR_CH_COUNT; i++) {
                if (s_hp_mask & (1UL << i)) s_hp_skipped[i]++;
            }
            return;
        }
        for (int i = 0; i < IR_CH_COUNT; i++) {
            if ((s_hp_mask & (1UL << i)) == 0U || s_hp_left[i] == 0U) continue;
            if (s_hp_left[i] != IR_HP_ENDLESS) s_hp_left[i]--;
            s_hp_fired[i]++;
            fire = true;
            /* Current up now; the gate opens at CNT = 1, 1us later. */
            if (i == IR_CH0) DAC1->DHR12R1 = s_hp_code[i];
            else             DAC1->DHR12R2 = s_hp_code[i];
        }
        if (!fire) return;          /* all hpulse bursts finished: stay dark */
        s_hp_last_cyc  = now;
        s_hp_any_fired = true;
    }

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

/* ---------------------------------------------------------------------------
   High-current pulse (USHIO 20A test) - see IR_LED_SetHighPulse() in the header
   --------------------------------------------------------------------------- */

#define IR_HP_TIM12_IRQ_PRIO  0U     /* the DAC drop ends the pulse current */

/* TIM12 update = end of the one-pulse: the gate has just closed in hardware;
   drop every hpulse DAC to 0 V so a high FLASH cannot light the LED at the
   pulse current between pulses. */
void TIM12_IRQHandler(void)
{
    if (TIM12->SR & TIM_SR_UIF) {
        TIM12->SR = ~TIM_SR_UIF;
        if (s_hp_mask & 1U) DAC1->DHR12R1 = 0U;
        if (s_hp_mask & 2U) DAC1->DHR12R2 = 0U;
        hsp_timer_cut();
    }
}

uint32_t IR_LED_HighPulseMaxWidth_us(uint32_t milliamps)
{
    const float period_us = 1000000.0f / (float)IR_FRAME_PULSE_FPS;
    float duty;

    if (milliamps == 0U) return IR_FRAME_PULSE_MAX_US;
    if (milliamps > IR_LED_PULSE_LIMIT_MA) milliamps = IR_LED_PULSE_LIMIT_MA;
    if (milliamps <= IR_LED_HARD_LIMIT_MA) return IR_FRAME_PULSE_MAX_US;

    /* duty = 1% * (20A / I) ^ (ln 100 / ln 5): the datasheet's pulse-duration
       curve, a straight log-log line from 20A at 1% to 4A at 100%. */
    duty = 0.01f * powf(20000.0f / (float)milliamps, logf(100.0f) / logf(5.0f));
    duty = duty * (float)IR_HP_DERATE_PCT / 100.0f;

    float w = duty * period_us;
    if (w > (float)IR_FRAME_PULSE_MAX_US) return IR_FRAME_PULSE_MAX_US;
    if (w < 1.0f) return 1U;
    return (uint32_t)w;          /* truncate: never round up past the curve */
}

/* Ends hpulse on ch: interrupt off if no channel needs it, DAC to 0 V. Mode
   handling is the caller's (apply_channel / IR_LED_StopHighPulse). */
static void hp_clear(IR_Channel_t ch)
{
    s_hp_mask &= ~(1UL << ch);
    s_hp_left[ch] = 0U;
    if (s_hp_mask == 0U) {
        TIM12->DIER &= ~TIM_DIER_UIE;
    }
    DAC1_SetValue(s_ch[ch].dac_channel, 0U);
    s_ch[ch].raw_code = 0U;
}

bool IR_LED_SetHighPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t width_us, uint32_t count)
{
    uint32_t max_w;
    uint16_t code;

    if (ch >= IR_CH_COUNT) return false;
    if (milliamps == 0U || width_us == 0U) {
        IR_LED_StopHighPulse(ch);
        return true;
    }
    if (milliamps > IR_LED_PULSE_LIMIT_MA) milliamps = IR_LED_PULSE_LIMIT_MA;
    max_w = IR_LED_HighPulseMaxWidth_us(milliamps);
    if (width_us > max_w) width_us = max_w;

    /* Round the code DOWN so the pulse never exceeds the requested current. */
    code = ma_to_code(milliamps);
    while (code > 0U && mv_to_ma(code_to_mv(code)) > milliamps) code--;

    /* Out of every other gate/DAC mode: FLASH gate, strobe, PWM, DAC PWM,
       ramp, DAC test. PWM_DIMx parks HIGH, DC setpoint 0. */
    IR_LED_StopRamp(ch);
    (void)IR_LED_SetDacPwm(ch, IR_DAC_PWM_MIN_HZ, 0U);
    (void)IR_LED_SetPwmLevel(ch, true);
    IR_LED_SetCurrentRaw(ch, 0U);

    __disable_irq();
    s_hp_code[ch]    = code;
    s_hp_left[ch]    = (count == 0U) ? IR_HP_ENDLESS : count;
    s_hp_fired[ch]   = 0U;
    s_hp_skipped[ch] = 0U;
    s_hp_mask       |= (1UL << ch);
    __enable_irq();

    if (!IR_LED_SetFramePulse(ch, width_us)) {
        IR_LED_StopHighPulse(ch);
        return false;
    }
    /* fp_apply() has just written SR = 0 after its UG, so no stale update
       is pending when the interrupt is enabled. */
    TIM12->DIER |= TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM12_IRQn, IR_HP_TIM12_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(TIM12_IRQn);
    return true;
}

void IR_LED_StopHighPulse(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    if ((s_hp_mask & (1UL << ch)) == 0U) return;
    hp_clear(ch);
    (void)IR_LED_SetFramePulse(ch, 0U);       /* mode OFF, gate back to GPIO */
}

bool IR_LED_GetHighPulse(IR_Channel_t ch, uint32_t *ma, uint32_t *width_us,
                         uint32_t *left, uint32_t *fired, uint32_t *skipped)
{
    if (ch >= IR_CH_COUNT || (s_hp_mask & (1UL << ch)) == 0U) return false;
    if (ma)       *ma       = mv_to_ma(code_to_mv(s_hp_code[ch]));
    if (width_us) *width_us = s_fp_width_us;
    if (left)     *left     = (s_hp_left[ch] == IR_HP_ENDLESS) ? 0U : s_hp_left[ch];
    if (fired)    *fired    = s_hp_fired[ch];
    if (skipped)  *skipped  = s_hp_skipped[ch];
    return true;
}

/* Runs from IR_LED_Task(). Belt and braces for the hpulse DAC: whenever no
   pulse is in flight, a DAC above the 4A hard-cap code is an error (missed
   or late ISR) - force it to 0 V. Also retires finished bursts. */
static void hp_task(void)
{
    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_Channel_t ch = (IR_Channel_t)i;
        if (s_dactest[i]) continue;                    /* explicit bench override */
        /* IRQs off: fp_fire() raises the DAC and starts TIM12 in one ISR, so
           this check cannot land between the two and cut a live pulse. */
        __disable_irq();
        if ((TIM12->CR1 & TIM_CR1_CEN) == 0U &&
            DAC1_GetValue(s_ch[i].dac_channel) > hard_limit_code()) {
            DAC1_SetValue(s_ch[i].dac_channel, 0U);
        }
        __enable_irq();
        if ((s_hp_mask & (1UL << i)) != 0U && s_hp_left[i] == 0U &&
            (TIM12->CR1 & TIM_CR1_CEN) == 0U) {
            IR_LED_StopHighPulse(ch);
        }
    }
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

    {   /* ADC needs PC3 in analog mode (still an input, never driven) */
        GPIO_InitTypeDef init = {0};
        init.Pin  = FLASH_Pin;
        init.Mode = GPIO_MODE_ANALOG;
        init.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(FLASH_GPIO_Port, &init);
    }

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
#if (IR_FLASH_SYNC_ENABLE == 0) || IR_PC3_OUTPUT
    want = false;                   /* blocked: never start ADC1 on PC3 */
#endif

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

/* ---------------------------------------------------------------------------
   FLASH input monitor (PC3: digital input, EXTI3 on both edges - read only)
   --------------------------------------------------------------------------- */

static volatile uint32_t s_fl_falls        = 0U;
static volatile uint32_t s_fl_last_fall    = 0U;   /* DWT cycles */
static volatile uint32_t s_fl_period_cyc   = 0U;   /* fall to fall */
static volatile uint32_t s_fl_low_cyc      = 0U;   /* fall to rise */
static volatile uint32_t s_fl_last_edge_ms = 0U;

/* Sensor sync monitor (rising edges of SN0_VS on PC4 and SN0_HS on PB1, both
   1.8V from the sensor, read as digital inputs). */
static volatile uint32_t s_vs_count = 0U, s_vs_last = 0U, s_vs_period_cyc = 0U;
static volatile uint32_t s_hs_count = 0U, s_hs_last = 0U, s_hs_period_cyc = 0U;
static volatile uint32_t s_hs_at_vs = 0U, s_hs_per_frame = 0U, s_vs_last_ms = 0U, s_hs_count_seen = 0U;

/* FLASH gate: PWM_DIM0 follows FLASH (LOW while FLASH is low, i.e. during
   the exposure), driven from the PC3 edge interrupts. */
/* Bit per channel; both PWM_DIM pins are on GPIOB, so one BSRR write
   switches every gated channel at the same instant. */
#define GATE_PIN(ch)  ((ch) == IR_CH0 ? PWM_DIM0_Pin : PWM_DIM1_Pin)
static volatile uint32_t s_gate_pins      = (IR_FLASH_GATE_AT_BOOT != 0) ? PWM_DIM0_Pin : 0U;
static volatile uint32_t s_gate_count     = 0U;
static volatile uint32_t s_low_pins       = 0U;   /* PWM_DIMx held LOW by "pwm low" */
static volatile uint32_t s_gate_inv_pins  = 0U;   /* gated channels with inverted polarity:
                                                     HIGH while FLASH is low, LOW while high */
static uint32_t          s_pwm_duty_req[IR_CH_COUNT] = {0U, 0U};   /* requested duty % */

/* ---------------------------------------------------------------------------
   HSYNC pulse (USHIO 20A test) - see IR_LED_SetHsyncPulse() in the header
   --------------------------------------------------------------------------- */

/* OFF: not set up. WAIT: between windows, DAC 0 V, waiting for VSYNC.
   ARMED: VSYNC seen, waiting for line start-1. PRE: DAC up, gate still off
   (the pre-line gives the LT3763 the DAC step before the first HIGH).
   ACTIVE: gate follows HSYNC level. */
enum { HSP_OFF = 0, HSP_WAIT, HSP_ARMED, HSP_PRE, HSP_ACTIVE };
#define IR_HSP_MARGIN_US      3U
#define IR_HSP_TIM12_IRQ_PRIO 0U     /* the cut-off ends the pulse current */

static volatile uint32_t s_hsp_state  = HSP_OFF;
static IR_Channel_t      s_hsp_ch     = IR_CH0;
static volatile uint16_t s_hsp_code   = 0U;
static volatile uint32_t s_hsp_start  = 0U;
static volatile uint32_t s_hsp_lines  = 0U;
static volatile uint32_t s_hsp_left   = 0U;      /* IR_HP_ENDLESS = continuous */
static volatile uint32_t s_hsp_fired  = 0U;
static volatile uint32_t s_hsp_cuts   = 0U;
static volatile uint32_t s_hsp_window_us = 0U;
static volatile uint32_t s_hsp_high_cyc  = 0U;
static volatile bool     s_hsp_vs_mode   = false;  /* true: LED on while VSYNC is LOW */
static volatile bool     s_hsp_solid     = false;  /* line window: solid ON, ignore HSYNC LOW */
static volatile uint32_t s_hsp_vs_fall   = 0U;     /* DWT at the last VSYNC fall */
static volatile uint32_t s_hsp_vs_low_cyc = 0U;
static volatile uint32_t s_hsp_last_begin = 0U;    /* DWT, for the rate guard */
static volatile bool     s_hsp_begun     = false;
static volatile uint32_t s_hsp_skipped   = 0U;
static volatile bool     s_hsp_flash     = false;  /* gate on FLASH (PC3 driven) instead of PWM_DIMx */
static volatile bool     s_hsp_inv       = false;  /* inv: gate on during HSYNC LOW (PC3 HIGH / PWM_DIMx LOW) */
static volatile bool     s_fx_inv_on     = false;  /* inv: gate currently on */
static volatile uint32_t s_hsp_end_line  = 0U;     /* start + lines, for IR_LED_HsIrqFast() */
static volatile uint32_t s_hsp_space_cyc = 0U;     /* IR_HP_MIN_SPACING_US in TIM5 ticks */

/* fhsync timing check: TIM5 at EXTI1 (SN0_HS) handler entry, set in
   stm32h5xx_it.c, against TIM5 right after each PC3 write. */
volatile uint32_t        g_hs_irq_t = 0U;
static volatile uint32_t s_fx_rise_min, s_fx_rise_max, s_fx_rise_sum, s_fx_rise_n;
static volatile uint32_t s_fx_fall_min, s_fx_fall_max, s_fx_fall_sum, s_fx_fall_n;
static volatile uint32_t s_fx_t_on, s_fx_high_cyc, s_fx_pulses, s_fx_pulses_last;
static volatile uint32_t s_fx_pmin, s_fx_pmax;
/* Rises the SN0_HS handler found pending on its way out of a fall, i.e.
   that arrived while the fall was still being handled. */
static volatile uint32_t s_fx_late = 0U;

static void fx_reset(void)
{
    s_fx_rise_min = s_fx_fall_min = s_fx_pmin = 0xFFFFFFFFU;
    s_fx_rise_max = s_fx_rise_sum = s_fx_rise_n = 0U;
    s_fx_fall_max = s_fx_fall_sum = s_fx_fall_n = 0U;
    s_fx_t_on = s_fx_high_cyc = s_fx_pulses = s_fx_pulses_last = s_fx_pmax = 0U;
    s_fx_late = 0U;
}

static bool hsp_active(void) { return s_hsp_state != HSP_OFF; }

IR_HOT static inline void hsp_dac(uint16_t code)
{
    if (s_hsp_ch == IR_CH0) DAC1->DHR12R1 = code;
    else                    DAC1->DHR12R2 = code;
}

/* LED gate for the window: PWM_DIMx LOW, or in FLASH mode PC3 HIGH (Q2/Q3
   pass FLASH, so FLASH HIGH lights the LED with PWM_DIMx HIGH). */
static volatile bool s_fx_fast_on = false, s_fx_fast_off = false;   /* PC3 already written */

IR_HOT static inline void hsp_led_on(void)
{
    if (s_hsp_inv) return;                                    /* IR_LED_HsIrqFast() owns the gate */
    if (s_hsp_flash) {
        if (s_hsp_inv) return;                                /* IR_LED_HsIrqFast() owns PC3 */
        if (s_fx_fast_on) { s_fx_fast_on = false; return; }   /* IR_LED_HsIrqFast() did it */
        FLASH_GPIO_Port->BSRR = FLASH_Pin;
        uint32_t now = IR_CYCCNT, d = now - g_hs_irq_t;
        s_fx_t_on = now;
        s_fx_pulses++;
        if (d < s_fx_rise_min) s_fx_rise_min = d;
        if (d > s_fx_rise_max) s_fx_rise_max = d;
        s_fx_rise_sum += d; s_fx_rise_n++;
    } else {
        PWM_DIM0_GPIO_Port->BSRR = (uint32_t)GATE_PIN(s_hsp_ch) << 16U;
    }
}

IR_HOT static inline void hsp_led_off(void)
{
    if (s_hsp_inv) return;                                    /* IR_LED_HsIrqFast() owns the gate */
    if (s_hsp_flash) {
        if (s_hsp_inv) return;                                /* IR_LED_HsIrqFast() owns PC3 */
        if (s_fx_fast_off) { s_fx_fast_off = false; return; }
        FLASH_GPIO_Port->BSRR = (uint32_t)FLASH_Pin << 16U;
        uint32_t now = IR_CYCCNT, d = now - g_hs_irq_t;
        s_fx_high_cyc = now - s_fx_t_on;
        if (d < s_fx_fall_min) s_fx_fall_min = d;
        if (d > s_fx_fall_max) s_fx_fall_max = d;
        s_fx_fall_sum += d; s_fx_fall_n++;
    } else {
        PWM_DIM0_GPIO_Port->BSRR = GATE_PIN(s_hsp_ch);
    }
}

/* First thing in EXTI1_IRQHandler (SN0_HS), fhsync only: writes PC3 (and
   the DAC) for this edge before the HAL dispatch and line bookkeeping, which
   had put PC3 ~5 us behind the HSYNC rise and ~2 us behind the fall. It
   makes the same decision hsp_on_rise()/hsp_on_fall() make just after it -
   in the window, or the first line of a window hsp_begin() will accept -
   and those then skip their own PC3 write. Stats: from handler entry. */
IR_HOT void IR_LED_HsIrqFast(bool rise, bool late)
{
    if (!s_hsp_flash && !s_hsp_inv) return;
    uint32_t st = s_hsp_state;
    if (s_hsp_inv) {
        /* Gate on through the LOW gap after each window line (line
           start..end-1, numbered at its rise), off from the next rise: PC3
           HIGH (fhsync inv) or PWM_DIMx LOW (hsync inv). Falls are only
           enabled while ACTIVE. */
        if (!rise) {
            uint32_t line = s_hs_count - s_hs_at_vs;         /* line this LOW belongs to */
            if (st != HSP_ACTIVE || line >= s_hsp_end_line) return;
            hsp_dac(s_hsp_code);                             /* current up, then */
            if (s_hsp_flash) FLASH_GPIO_Port->BSRR = FLASH_Pin;                       /* PC3 HIGH */
            else             PWM_DIM0_GPIO_Port->BSRR = (uint32_t)GATE_PIN(s_hsp_ch) << 16U;  /* PWM LOW */
            uint32_t now = IR_CYCCNT, d = now - g_hs_irq_t;
            s_fx_inv_on = true;
            s_fx_t_on = now;
            s_fx_pulses++;
            if (d < s_fx_rise_min) s_fx_rise_min = d;
            if (d > s_fx_rise_max) s_fx_rise_max = d;
            s_fx_rise_sum += d; s_fx_rise_n++;
        } else if (s_fx_inv_on) {
            if (s_hsp_flash) FLASH_GPIO_Port->BSRR = (uint32_t)FLASH_Pin << 16U;      /* PC3 LOW */
            else             PWM_DIM0_GPIO_Port->BSRR = GATE_PIN(s_hsp_ch);           /* PWM HIGH */
            hsp_dac(0U);                                     /* then current down */
            s_fx_inv_on = false;
            uint32_t now = IR_CYCCNT, d = now - g_hs_irq_t;
            s_fx_high_cyc = now - s_fx_t_on;
            if (late) s_fx_late++;
            if (d < s_fx_fall_min) s_fx_fall_min = d;
            if (d > s_fx_fall_max) s_fx_fall_max = d;
            s_fx_fall_sum += d; s_fx_fall_n++;
        }
        return;
    }
    if (rise) {
        uint32_t line = s_hs_count + 1U - s_hs_at_vs;       /* count not yet bumped */
        bool on = false;
        if (st == HSP_ACTIVE) {
            on = (line < s_hsp_end_line);
        } else if (st == HSP_ARMED && line == s_hsp_start) {
            on = !s_hsp_begun || (IR_CYCCNT - s_hsp_last_begin) >= s_hsp_space_cyc;
        }
        if (!on) return;
        hsp_dac(s_hsp_code);                                 /* current up, then */
        FLASH_GPIO_Port->BSRR = FLASH_Pin;                   /* PC3 HIGH */
        uint32_t now = IR_CYCCNT, d = now - g_hs_irq_t;
        s_fx_fast_on = true;
        s_fx_t_on = now;
        s_fx_pulses++;
        if (late) s_fx_late++;
        if (d < s_fx_rise_min) s_fx_rise_min = d;
        if (d > s_fx_rise_max) s_fx_rise_max = d;
        s_fx_rise_sum += d; s_fx_rise_n++;
    } else if (st == HSP_ACTIVE) {
        FLASH_GPIO_Port->BSRR = (uint32_t)FLASH_Pin << 16U;  /* PC3 LOW, then */
        hsp_dac(0U);                                         /* current down */
        uint32_t now = IR_CYCCNT, d = now - g_hs_irq_t;
        s_fx_fast_off = true;
        s_fx_high_cyc = now - s_fx_t_on;
        if (d < s_fx_fall_min) s_fx_fall_min = d;
        if (d > s_fx_fall_max) s_fx_fall_max = d;
        s_fx_fall_sum += d; s_fx_fall_n++;
    }
}

/* PC3 for the FLASH-gated hsync mode ONLY - user-approved 2026-10-14 with
   the sensor's FLASH output disabled. out: push-pull LOW, FLASH EXTI masked
   (our own edges must not run the gate/strobe/monitor). !out: LOW, then back
   to the boot input with both-edge interrupt. */
static void hsp_flash_pin(bool out)
{
    GPIO_InitTypeDef init = {0};
    FLASH_GPIO_Port->BSRR = (uint32_t)FLASH_Pin << 16U;
    init.Pin  = FLASH_Pin;
    init.Pull = GPIO_NOPULL;
    if (out) {
        EXTI->IMR1 &= ~FLASH_Pin;
        init.Mode  = GPIO_MODE_OUTPUT_PP;
        init.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(FLASH_GPIO_Port, &init);
    } else {
        init.Mode = GPIO_MODE_IT_RISING_FALLING;
        HAL_GPIO_Init(FLASH_GPIO_Port, &init);
        EXTI->RPR1 = FLASH_Pin;
        EXTI->FPR1 = FLASH_Pin;
    }
}

/* End of a window (HSYNC, TIM12 cut-off, VSYNC overrun or stop): gate off,
   DAC 0 V, falling-edge interrupt and timer off. Interrupt context safe. */
IR_HOT static void hsp_end(void)
{
    if (s_hsp_flash) FLASH_GPIO_Port->BSRR = (uint32_t)FLASH_Pin << 16U;  /* FLASH LOW */
    if (s_hsp_flash || s_hsp_inv) {
        s_fx_fast_on = s_fx_fast_off = false;
        if (s_fx_pulses != 0U) {                              /* gate pulses this window */
            s_fx_pulses_last = s_fx_pulses;
            if (s_fx_pulses < s_fx_pmin) s_fx_pmin = s_fx_pulses;
            if (s_fx_pulses > s_fx_pmax) s_fx_pmax = s_fx_pulses;
            s_fx_pulses = 0U;
        }
    }
    PWM_DIM0_GPIO_Port->BSRR = GATE_PIN(s_hsp_ch);          /* LED off */
    hsp_dac(0U);
    s_fx_inv_on = false;
    EXTI->FTSR1 &= ~SN0_HS_Pin;
    TIM12->CR1  &= ~TIM_CR1_CEN;
    s_hsp_state = HSP_WAIT;
}

/* Starts a pulse unless the last one began less than IR_HP_MIN_SPACING_US
   ago - the derating assumes at most IR_FRAME_PULSE_FPS pulses a second,
   so a glitching sync input must not raise the rate. */
IR_HOT static bool hsp_begin(void)
{
    uint32_t now = IR_CYCCNT;
    if (s_hsp_begun && (now - s_hsp_last_begin) < s_hsp_space_cyc) {
        s_hsp_skipped++;
        return false;
    }
    s_hsp_last_begin = now;
    s_hsp_begun      = true;
    if (s_hsp_left != IR_HP_ENDLESS) s_hsp_left--;
    s_hsp_fired++;
    hsp_dac(s_hsp_code);
    TIM12->CNT  = 0U;
    TIM12->SR   = 0U;
    TIM12->CR1 |= TIM_CR1_CEN;                               /* cut-off running */
    s_hsp_state = HSP_PRE;
    return true;
}

/* SN0_VS rising, s_hs_at_vs already updated. */
static void hsp_on_vsync(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (s_hsp_state == HSP_PRE || s_hsp_state == HSP_ACTIVE) hsp_end();
    if (s_hsp_vs_mode) {                                     /* end of the LOW pulse */
        s_hsp_vs_low_cyc = IR_CYCCNT - s_hsp_vs_fall;
        __set_PRIMASK(pm);
        return;
    }
    if (s_hsp_state == HSP_WAIT && s_hsp_left != 0U) {
        s_hsp_state = HSP_ARMED;
        if (s_hsp_solid && s_hsp_start == 1U) hsp_begin();   /* pre-line = VSYNC */
    }
    __set_PRIMASK(pm);
}

/* SN0_HS rising; line = 1 for the first HSYNC after VSYNC. */
IR_HOT static void hsp_on_rise(uint32_t line)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    uint32_t st = s_hsp_state;
    if (st == HSP_ARMED && !s_hsp_solid) {
        /* hsync: no pre-line - DAC and gate both follow HSYNC from line start. */
        if (line == s_hsp_start && hsp_begin()) {
            EXTI->FPR1   = SN0_HS_Pin;                       /* no stale fall */
            EXTI->FTSR1 |= SN0_HS_Pin;
            if (s_hsp_inv) hsp_dac(0U);                      /* inv: up at the fall */
            hsp_led_on();                                    /* LED on */
            s_hsp_state  = HSP_ACTIVE;
        } else if (s_fx_fast_on) {                           /* fast path raised it: undo */
            FLASH_GPIO_Port->BSRR = (uint32_t)FLASH_Pin << 16U;
            hsp_dac(0U);
            s_fx_fast_on = false;
        }
    } else if (st == HSP_ARMED) {
        if (line + 1U == s_hsp_start) hsp_begin();
    } else if (st == HSP_ACTIVE && !s_hsp_solid) {
        if (line >= s_hsp_start + s_hsp_lines) {
            hsp_end();
        } else if (!s_hsp_inv) {
            hsp_dac(s_hsp_code);                             /* current up, then */
            hsp_led_on();                                    /* LED on */
        }
    } else if (st == HSP_PRE || st == HSP_ACTIVE) {
        if (line >= s_hsp_start + s_hsp_lines) {
            hsp_end();
        } else if (line >= s_hsp_start) {
            if (st == HSP_PRE) {
                if (!s_hsp_solid) {                          /* follow HSYNC level */
                    EXTI->FPR1   = SN0_HS_Pin;               /* no stale fall */
                    EXTI->FTSR1 |= SN0_HS_Pin;
                }
                s_hsp_state  = HSP_ACTIVE;
            }
            hsp_led_on();                                    /* LED on */
        }
    }
    __set_PRIMASK(pm);
}

/* SN0_VS falling - only enabled in VSYNC-LOW mode: current up, LED on. */
static void hsp_on_vs_fall(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_hsp_vs_fall = IR_CYCCNT;
    if (s_hsp_vs_mode && s_hsp_state == HSP_WAIT && s_hsp_left != 0U && hsp_begin()) {
        hsp_led_on();                                    /* LED on */
        s_hsp_state = HSP_ACTIVE;
    }
    __set_PRIMASK(pm);
}

/* SN0_HS falling - only enabled while ACTIVE. */
IR_HOT static void hsp_on_fall(void)
{
    if (s_hsp_state == HSP_ACTIVE) {
        hsp_led_off();                                       /* LED off, then */
        if (!s_hsp_solid) hsp_dac(0U);                       /* current down */
    }
    s_hsp_high_cyc = IR_CYCCNT - s_hs_last;
}

/* TIM12 update: the cut-off expired with the window still open. */
static void hsp_timer_cut(void)
{
    if (s_hsp_state == HSP_PRE || s_hsp_state == HSP_ACTIVE) {
        hsp_end();
        s_hsp_cuts++;
    }
}

/* Shared set-up for hsync and vsync pulses: frees TIM12 and the channel,
   programs the cut-off and arms the engine in WAIT. */
static void hsp_setup(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                      uint32_t lines, uint32_t count, uint32_t window, bool vs_mode, bool solid,
                      bool flash, bool inv)
{
    uint16_t code;

    code = ma_to_code(milliamps);
    while (code > 0U && mv_to_ma(code_to_mv(code)) > milliamps) code--;

    /* Free TIM12 and the channel: hpulse on both, the other channel's TIM12
       mode, and every DAC/gate mode on this one. PWM_DIMx HIGH, DC 0. */
    IR_LED_StopHighPulse(IR_CH0);
    IR_LED_StopHighPulse(IR_CH1);
    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode == IR_MODE_GATE_PWM || s_ch[i].mode == IR_MODE_FRAME_PULSE) {
            (void)IR_LED_SetMode((IR_Channel_t)i, IR_MODE_OFF);
        }
    }
    IR_LED_StopRamp(ch);
    (void)IR_LED_SetDacPwm(ch, IR_DAC_PWM_MIN_HZ, 0U);
    (void)IR_LED_SetPwmLevel(ch, true);
    IR_LED_SetCurrentRaw(ch, 0U);

    /* TIM12 as a bare one-pulse timer: 1 us ticks, no outputs, URS so the
       UG below cannot raise UIF. */
    __HAL_RCC_TIM12_CLK_ENABLE();
    TIM12->CR1   = 0U;
    TIM12->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM12->PSC   = (uint16_t)(IR_TICKS_PER_US - 1U);
    TIM12->ARR   = (uint16_t)window;
    TIM12->CR1   = TIM_CR1_OPM | TIM_CR1_URS;
    TIM12->EGR   = TIM_EGR_UG;
    TIM12->SR    = 0U;
    TIM12->DIER  = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM12_IRQn, IR_HSP_TIM12_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(TIM12_IRQn);
    if (flash) hsp_flash_pin(true);
    fx_reset();

    __disable_irq();
    s_hsp_flash     = flash;
    s_hsp_inv       = inv && !vs_mode && !solid;
    s_fx_inv_on     = false;
    s_hsp_vs_mode   = vs_mode;
    s_hsp_solid     = solid;
    s_hsp_vs_low_cyc = 0U;
    s_hsp_begun     = false;
    s_hsp_skipped   = 0U;
    s_hsp_ch        = ch;
    s_hsp_code      = code;
    s_hsp_start     = start_line;
    s_hsp_lines     = lines;
    s_hsp_end_line  = start_line + lines;
    s_hsp_space_cyc = IR_HP_MIN_SPACING_US * (SystemCoreClock / 1000000U);
    s_hsp_left      = (count == 0U) ? IR_HP_ENDLESS : count;
    s_hsp_fired     = 0U;
    s_hsp_cuts      = 0U;
    s_hsp_window_us = window;
    s_hsp_high_cyc  = 0U;
    s_hsp_state     = HSP_WAIT;                 /* arms on the next VSYNC */
    if (vs_mode) {
        EXTI->FPR1   = SN0_VS_Pin;
        EXTI->FTSR1 |= SN0_VS_Pin;              /* VSYNC fall starts each pulse */
    }
    __enable_irq();
}

static bool line_pulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                       uint32_t lines, uint32_t count, bool solid, bool flash, bool inv)
{
    uint32_t cyc_per_us = SystemCoreClock / 1000000U;
    uint32_t line_us, budget, max_lines, window;

    if (ch >= IR_CH_COUNT) return false;
    IR_LED_StopHsyncPulse(IR_CH0);
    IR_LED_StopHsyncPulse(IR_CH1);
    if (milliamps == 0U || lines == 0U || start_line == 0U) return true;
    if (flash && s_fs_running) return false;            /* ADC owns PC3 (analog) */

    /* HSYNC must be running now: an edge within the last 1 ms. */
    if (s_hs_period_cyc == 0U || (IR_CYCCNT - s_hs_last) > 1000U * cyc_per_us) return false;
    /* Line period averaged over the last frame (VSYNC period / lines); one
       HS-to-HS sample carries the EXTI latency jitter (seen 14.3..18 us). */
    {
        uint32_t per_cyc = s_hs_period_cyc;
        if (s_vs_period_cyc != 0U && s_hs_per_frame != 0U) per_cyc = s_vs_period_cyc / s_hs_per_frame;
        line_us = (per_cyc + cyc_per_us - 1U) / cyc_per_us;              /* round up */
    }
    if (line_us == 0U) return false;

    if (milliamps > IR_LED_PULSE_LIMIT_MA) milliamps = IR_LED_PULSE_LIMIT_MA;
    budget = IR_LED_HighPulseMaxWidth_us(milliamps);
    /* At or below the LED's DC rating (IR_LED_HARD_LIMIT_MA) there is no
       per-frame limit, so a window may cover every line of the frame (user,
       2026-10-14: one pulse per HSYNC, 2233 per frame). The next VSYNC ends
       the window (hsp_on_vsync); TIM12 - 16-bit, 1 us ticks - only backs it
       up if VSYNC is lost. Above it the derating budget applies as before. */
    if (milliamps <= IR_LED_HARD_LIMIT_MA) budget = 0xFFFFU;
    {
        uint32_t pre = solid ? 1U : 0U;                                  /* solid: + pre-line */
        if (budget <= IR_HSP_MARGIN_US + (pre + 1U) * line_us) return false;
        max_lines = (budget - IR_HSP_MARGIN_US) / line_us - pre;
        if (lines > max_lines) lines = max_lines;
        window = (lines + pre) * line_us + IR_HSP_MARGIN_US;             /* <= budget */
    }

    hsp_setup(ch, milliamps, start_line, lines, count, window, false, solid, flash, inv);
    return true;
}

bool IR_LED_SetHsyncPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                          uint32_t lines, uint32_t count)
{
    return line_pulse(ch, milliamps, start_line, lines, count, false, false, false);
}

bool IR_LED_SetHsyncLowPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                             uint32_t lines, uint32_t count)
{
    return line_pulse(ch, milliamps, start_line, lines, count, false, false, true);
}

bool IR_LED_SetHsyncFlashPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                               uint32_t lines, uint32_t count, bool inv)
{
    return line_pulse(ch, milliamps, start_line, lines, count, false, true, inv);
}

bool IR_LED_SetVsyncLinePulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                              uint32_t lines, uint32_t count)
{
    return line_pulse(ch, milliamps, start_line, lines, count, true, false, false);
}

bool IR_LED_SetVsyncPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t count)
{
    uint32_t window;

    if (ch >= IR_CH_COUNT) return false;
    IR_LED_StopHsyncPulse(IR_CH0);
    IR_LED_StopHsyncPulse(IR_CH1);
    if (milliamps == 0U) return true;

    /* VSYNC must be running now: an edge within the last 100 ms. */
    if (s_vs_count == 0U || (HAL_GetTick() - s_vs_last_ms) > 100U) return false;

    if (milliamps > IR_LED_PULSE_LIMIT_MA) milliamps = IR_LED_PULSE_LIMIT_MA;
    window = IR_LED_HighPulseMaxWidth_us(milliamps);
    if (window > 0xFFFFU) window = 0xFFFFU;                              /* 16-bit ARR */
    hsp_setup(ch, milliamps, 0U, 0U, count, window, true, false, false, false);
    return true;
}

void IR_LED_StopHsyncPulse(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT || s_hsp_state == HSP_OFF || ch != s_hsp_ch) return;
    __disable_irq();
    hsp_end();
    EXTI->FTSR1 &= ~SN0_VS_Pin;
    s_hsp_state = HSP_OFF;
    if (s_hp_mask == 0U) TIM12->DIER &= ~TIM_DIER_UIE;
    __enable_irq();
    if (s_hsp_flash) {                          /* PC3 back to input */
        hsp_flash_pin(IR_PC3_OUTPUT);           /* IMX568: input again; IMX678: output LOW */
        s_hsp_flash = false;
    }
    s_ch[ch].raw_code = 0U;
}

bool IR_LED_GetHsyncPulse(IR_Channel_t ch, IR_HsyncPulseInfo_t *info)
{
    if (ch >= IR_CH_COUNT || s_hsp_state == HSP_OFF || ch != s_hsp_ch) return false;
    if (info) {
        uint32_t cyc_per_us = SystemCoreClock / 1000000U;
        info->ma         = mv_to_ma(code_to_mv(s_hsp_code));
        info->start_line = s_hsp_start;
        info->lines      = s_hsp_lines;
        info->window_us  = s_hsp_window_us;
        info->left       = (s_hsp_left == IR_HP_ENDLESS) ? 0U : s_hsp_left;
        info->fired      = s_hsp_fired;
        info->timer_cuts = s_hsp_cuts;
        info->hs_high_ns = (uint32_t)(((uint64_t)s_hsp_high_cyc * 1000U) / cyc_per_us);
        info->vs_mode    = s_hsp_vs_mode;
        info->solid      = s_hsp_solid;
        info->vs_low_ns  = (uint32_t)(((uint64_t)s_hsp_vs_low_cyc * 1000U) / cyc_per_us);
        info->skipped    = s_hsp_skipped;
        info->flash      = s_hsp_flash;
        info->inv        = s_hsp_inv;
    }
    return true;
}

/* Runs from IR_LED_Task(): retires a finished burst. */
static void hsp_task(void)
{
    if (s_hsp_state == HSP_WAIT && s_hsp_left == 0U) IR_LED_StopHsyncPulse(s_hsp_ch);
}

/* The STM32H5 HAL dispatches EXTI through these two callbacks, never through
   HAL_GPIO_EXTI_Callback() below - so only FLASH is handled here. */
IR_HOT void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SN0_HS_Pin) {            /* only enabled inside an hsync window */
        /* fhsync: IR_LED_HsIrqFast() already did PC3 + DAC. HSYNC is LOW only
           ~1.7 us, so nothing else may run here or the next rise is late. */
        if (s_hsp_inv) return;
        if (s_fx_fast_off) { s_fx_fast_off = false; return; }
        hsp_on_fall();
        return;
    }
    if (GPIO_Pin == SN0_VS_Pin) {            /* only enabled in vsync-LOW mode */
        hsp_on_vs_fall();
        return;
    }
    if (GPIO_Pin != FLASH_Pin) return;
    if (s_dac_gate & 1U) DAC1->DHR12R1 = s_ch[IR_CH0].raw_code;   /* strobe: current up */
    if (s_dac_gate & 2U) DAC1->DHR12R2 = s_ch[IR_CH1].raw_code;
    if (s_gate_pins != 0U) {                 /* exposure starts */
        uint32_t inv = s_gate_inv_pins, norm = s_gate_pins & ~inv;
        PWM_DIM0_GPIO_Port->BSRR = (norm << 16U) | inv;   /* normal LOW, inverted HIGH */
        s_gate_count++;
    }
    uint32_t now = IR_CYCCNT;
    if (s_fl_falls != 0U) s_fl_period_cyc = now - s_fl_last_fall;
    s_fl_last_fall    = now;
    s_fl_falls++;
    s_fl_last_edge_ms = HAL_GetTick();
}

IR_HOT void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SN0_HS_Pin) {          /* ~65-130 kHz: keep this short */
        uint32_t now = IR_CYCCNT;
        s_hs_period_cyc = now - s_hs_last;
        s_hs_last = now;
        s_hs_count++;
        if (s_hsp_state != HSP_OFF) hsp_on_rise(s_hs_count - s_hs_at_vs);
        return;
    }
    if (GPIO_Pin == SN0_VS_Pin) {
        uint32_t now = IR_CYCCNT;
        if (s_vs_count != 0U) {
            s_vs_period_cyc = now - s_vs_last;
            s_hs_per_frame  = s_hs_count - s_hs_at_vs;
        }
        s_vs_last  = now;
        s_hs_at_vs = s_hs_count;
        s_vs_count++;
        s_vs_last_ms = HAL_GetTick();
        if (s_hsp_state != HSP_OFF) hsp_on_vsync();
        return;
    }
    if (GPIO_Pin != FLASH_Pin) return;
    if (s_gate_pins != 0U) {                 /* exposure ends */
        uint32_t inv = s_gate_inv_pins, norm = s_gate_pins & ~inv;
        PWM_DIM0_GPIO_Port->BSRR = norm | (inv << 16U);   /* normal HIGH, inverted LOW */
    }
    if (s_dac_gate & 1U) DAC1->DHR12R1 = 0U;                     /* strobe: current to 0 */
    if (s_dac_gate & 2U) DAC1->DHR12R2 = 0U;
    if (s_fl_falls != 0U) s_fl_low_cyc = IR_CYCCNT - s_fl_last_fall;
    s_fl_last_edge_ms = HAL_GetTick();
}

/* True while the other channel can hold its PWM_DIM LOW (fixed LOW or PWM). */
static bool other_can_be_low(IR_Channel_t ch)
{
    IR_Channel_t o = (ch == IR_CH0) ? IR_CH1 : IR_CH0;
    return ((s_low_pins & GATE_PIN(o)) != 0U) || (s_ch[o].mode == IR_MODE_GATE_PWM);
}

/* Ends strobe on a channel: DAC back to its steady setpoint. */
static void strobe_clear(IR_Channel_t ch)
{
    s_gate_inv_pins &= ~GATE_PIN(ch);      /* any mode change ends an inverted gate too */
    if ((s_dac_gate & (1UL << ch)) == 0U) return;
    s_dac_gate &= ~(1UL << ch);
    DAC1_SetValue(s_ch[ch].dac_channel, s_ch[ch].raw_code);
}

/* Takes the pin back from TIM12 (if a duty-cycle PWM was running) so the
   BSRR writes below drive it; leaves it HIGH. Also ends strobe. */
static void pwm_release(IR_Channel_t ch)
{
    strobe_clear(ch);
    if (s_ch[ch].mode != IR_MODE_OFF) (void)IR_LED_SetMode(ch, IR_MODE_OFF);
}

bool IR_LED_SetFlashGate(IR_Channel_t ch, bool on)
{
    uint32_t pin = GATE_PIN(ch);

    if (ch >= IR_CH_COUNT) return false;
#if IR_PC3_OUTPUT
    if (on) return false;                    /* IMX678: PC3 is an output, no FLASH to gate on */
#endif
    if (on && ((s_gate_pins & ~pin) != 0U || other_can_be_low(ch))) {
        return false;                        /* check before touching the pin */
    }
    pwm_release(ch);
    if (!on) {
        s_gate_pins &= ~pin;
        s_low_pins  &= ~pin;
        PWM_DIM0_GPIO_Port->BSRR = pin;                    /* HIGH: LED follows FLASH only */
        return true;
    }
    /* Only one channel may be gated. FLASH's high level comes only from the
       IR_CTRLx pull-ups through Q2/Q3, i.e. from a channel whose PWM_DIMx is
       HIGH. With both gated, nothing pulls FLASH up when the sensor releases
       it after the exposure, so FLASH stays low and the gate never releases
       (seen on the bench 2026-10-02: both LEDs latched on). */
    s_low_pins  &= ~pin;
    s_gate_pins |= pin;
    if (HAL_GPIO_ReadPin(FLASH_GPIO_Port, FLASH_Pin) == GPIO_PIN_RESET) {
        PWM_DIM0_GPIO_Port->BSRR = pin << 16U;             /* in exposure now */
    }
    return true;
}

bool IR_LED_SetPwmLevel(IR_Channel_t ch, bool high)
{
    uint32_t pin = GATE_PIN(ch);

    if (ch >= IR_CH_COUNT) return false;
    if (!high && (s_gate_pins & ~pin) != 0U) return false;
    pwm_release(ch);
    if (high) {
        s_gate_pins &= ~pin;
        s_low_pins  &= ~pin;
        PWM_DIM0_GPIO_Port->BSRR = pin;
        return true;
    }
    /* LOW is refused (above) while the other channel is gated: it would leave
       no PWM_DIMx HIGH to pull FLASH up after the exposure (gate latch-up). */
    s_gate_pins &= ~pin;
    s_low_pins  |= pin;
    PWM_DIM0_GPIO_Port->BSRR = pin << 16U;
    return true;
}

uint32_t IR_LED_SetDacTest_mV(IR_Channel_t ch, uint32_t millivolts)
{
    uint32_t code;

    if (ch >= IR_CH_COUNT) return 0U;
    if (millivolts > IR_DACTEST_MAX_MV) millivolts = IR_DACTEST_MAX_MV;
    IR_LED_StopRamp(ch);
    if (IR_LED_GetStrobe(ch)) (void)IR_LED_SetStrobe(ch, false);   /* ISR would overwrite the DAC */
    (void)IR_LED_SetDacPwm(ch, IR_DAC_PWM_MIN_HZ, 100U);            /* no DAC chop */
    code = (millivolts * DAC1_CODE_MAX + IR_DAC_VREF_MV / 2U) / IR_DAC_VREF_MV;
    if (code > DAC1_CODE_MAX) code = DAC1_CODE_MAX;
    DAC1_SetValue(s_ch[ch].dac_channel, (uint16_t)code);           /* bypasses raw_code / cap */
    s_dactest[ch] = true;
    return (code * IR_DAC_VREF_MV + DAC1_CODE_MAX / 2U) / DAC1_CODE_MAX;
}

void IR_LED_ClearDacTest(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT || !s_dactest[ch]) return;
    IR_LED_SetCurrentRaw(ch, s_ch[ch].raw_code);   /* back to the capped setpoint */
}

bool IR_LED_GetDacTest(IR_Channel_t ch, uint32_t *millivolts)
{
    if (ch >= IR_CH_COUNT || !s_dactest[ch]) return false;
    if (millivolts) {
        uint32_t code = DAC1_GetValue(s_ch[ch].dac_channel);
        *millivolts = (code * IR_DAC_VREF_MV + DAC1_CODE_MAX / 2U) / DAC1_CODE_MAX;
    }
    return true;
}

bool IR_LED_SetStrobe(IR_Channel_t ch, bool on)
{
    if (ch >= IR_CH_COUNT) return false;
    if (!on) {
        return IR_LED_SetFlashGate(ch, false);     /* clears strobe, PWM_DIM HIGH */
    }
    if (!IR_LED_SetFlashGate(ch, true)) return false;
    (void)IR_LED_SetDacPwm(ch, IR_DAC_PWM_MIN_HZ, 100U);   /* no DAC chop */
    s_dac_gate |= (1UL << ch);
    IR_LED_SetCurrentRaw(ch, s_ch[ch].raw_code);   /* 0 V now if outside exposure */
    return true;
}

bool IR_LED_GetStrobe(IR_Channel_t ch)
{
    return (ch < IR_CH_COUNT) && ((s_dac_gate & (1UL << ch)) != 0U);
}

bool IR_LED_SetPwmDuty(IR_Channel_t ch, uint32_t freq_hz, uint32_t duty_pct)
{
    IR_Channel_t o = (ch == IR_CH0) ? IR_CH1 : IR_CH0;
    uint32_t pin = GATE_PIN(ch), ofreq, oduty;

    if (ch >= IR_CH_COUNT) return false;
    if (duty_pct == 0U)   return IR_LED_SetPwmLevel(ch, true);
    if (duty_pct >= 100U) return IR_LED_SetPwmLevel(ch, false);
    if ((s_gate_pins & ~pin) != 0U) return false;      /* other gated: latch-up */

    strobe_clear(ch);
    s_gate_pins &= ~pin;
    s_low_pins  &= ~pin;
    (void)IR_LED_SetGatePwm(ch, freq_hz, duty_pct);
    /* TIM12 is shared: re-apply the other channel at the new period with
       its own duty, or its duty would drift with the new ARR. */
    if (s_ch[o].mode == IR_MODE_GATE_PWM && IR_LED_GetGatePwm(o, &ofreq, &oduty)) {
        (void)IR_LED_SetGatePwm(o, freq_hz, s_pwm_duty_req[o]);
    }
    s_pwm_duty_req[ch] = duty_pct;
    return true;
}

IR_PwmState_t IR_LED_GetPwmState(IR_Channel_t ch)
{
    uint32_t pin = GATE_PIN(ch);
    if (ch >= IR_CH_COUNT)    return IR_PWM_HIGH;
    if (s_ch[ch].mode == IR_MODE_GATE_PWM) return IR_PWM_DUTY;
    if (s_gate_pins & pin)    return IR_PWM_GATED;
    if (s_low_pins & pin)     return IR_PWM_LOW;
    return IR_PWM_HIGH;
}

bool IR_LED_SetFlashGateInverted(IR_Channel_t ch)
{
    uint32_t pin = GATE_PIN(ch);

    if (!IR_LED_SetFlashGate(ch, true)) return false;   /* same one-gate rule */
    s_gate_inv_pins |= pin;
    /* Apply the inverted level for the current FLASH state now. */
    if (HAL_GPIO_ReadPin(FLASH_GPIO_Port, FLASH_Pin) == GPIO_PIN_RESET) {
        PWM_DIM0_GPIO_Port->BSRR = pin;              /* FLASH low  -> HIGH */
    } else {
        PWM_DIM0_GPIO_Port->BSRR = pin << 16U;       /* FLASH high -> LOW  */
    }
    return true;
}

bool IR_LED_GetFlashGateInverted(IR_Channel_t ch)
{
    return (ch < IR_CH_COUNT) && ((s_gate_inv_pins & GATE_PIN(ch)) != 0U);
}

bool IR_LED_GetFlashGate(IR_Channel_t ch, uint32_t *count)
{
    if (count) *count = s_gate_count;
    return (ch < IR_CH_COUNT) && ((s_gate_pins & GATE_PIN(ch)) != 0U);
}

void IR_LED_GetSyncInput(IR_SyncInput_t *in)
{
    uint32_t cyc_per_us = SystemCoreClock / 1000000U;
    uint32_t now_ms = HAL_GetTick();

    in->vs_count       = s_vs_count;
    in->vs_period_us   = s_vs_period_cyc / cyc_per_us;
    in->vs_active      = (s_vs_count != 0U) && ((now_ms - s_vs_last_ms) < IR_FLASH_IDLE_MS);
    in->hs_count       = s_hs_count;
    in->hs_period_ns   = (uint32_t)(((uint64_t)s_hs_period_cyc * 1000U) / cyc_per_us);
    in->hs_per_frame   = s_hs_per_frame;
    /* HS alive if its count moved since the last call */
    in->hs_active      = (s_hs_count != s_hs_count_seen);
    s_hs_count_seen    = s_hs_count;
}

/* True HSYNC (PB1) HIGH/LOW times: polls the pin for ~1 ms with interrupts
   off (no EXTI latency in the numbers), TIM5-timed. Refused while a pulse
   mode runs - its edges must not be missed. ns: high min/avg/max, low
   min/avg/max, number of full HIGH periods. */
IR_HOT bool IR_LED_MeasureHsync(uint32_t d[7])
{
    if (s_hsp_state != HSP_OFF) return false;
    uint32_t c = SystemCoreClock / 1000000U, lim = 1000U * c;
    uint32_t hmin = ~0U, hmax = 0U, hsum = 0U, hn = 0U, lmin = ~0U, lmax = 0U, lsum = 0U, ln = 0U;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    uint32_t t0 = IR_CYCCNT, last = 0U, tedge = 0U;
    bool lvl = (SN0_HS_GPIO_Port->IDR & SN0_HS_Pin) != 0U, seen = false;
    while ((IR_CYCCNT - t0) < lim) {
        bool v = (SN0_HS_GPIO_Port->IDR & SN0_HS_Pin) != 0U;
        if (v == lvl) continue;
        uint32_t now = IR_CYCCNT;
        if (seen) {
            uint32_t w = now - tedge;
            if (lvl) { if (w < hmin) hmin = w; if (w > hmax) hmax = w; hsum += w; hn++; }
            else     { if (w < lmin) lmin = w; if (w > lmax) lmax = w; lsum += w; ln++; }
        }
        seen = true; tedge = now; lvl = v; last = now;
    }
    __set_PRIMASK(pm);
    (void)last;
    d[0] = hn ? hmin * 1000U / c : 0U; d[1] = hn ? (hsum / hn) * 1000U / c : 0U; d[2] = hmax * 1000U / c;
    d[3] = ln ? lmin * 1000U / c : 0U; d[4] = ln ? (lsum / ln) * 1000U / c : 0U; d[5] = lmax * 1000U / c;
    d[6] = hn;
    return true;
}

bool IR_LED_GetFlashSyncStats(uint32_t d[12])
{
    if (s_hsp_state == HSP_OFF || (!s_hsp_flash && !s_hsp_inv)) return false;
    uint32_t c = SystemCoreClock / 1000000U;      /* cycles per us -> ns below */
    d[0] = s_fx_rise_n ? s_fx_rise_min * 1000U / c : 0U;
    d[1] = s_fx_rise_n ? (s_fx_rise_sum / s_fx_rise_n) * 1000U / c : 0U;
    d[2] = s_fx_rise_max * 1000U / c;
    d[3] = s_fx_fall_n ? s_fx_fall_min * 1000U / c : 0U;
    d[4] = s_fx_fall_n ? (s_fx_fall_sum / s_fx_fall_n) * 1000U / c : 0U;
    d[5] = s_fx_fall_max * 1000U / c;
    d[6] = s_fx_high_cyc * 1000U / c;            /* last PC3 HIGH */
    d[7] = s_fx_pulses_last;
    d[8] = (s_fx_pmin == 0xFFFFFFFFU) ? 0U : s_fx_pmin;
    d[9] = s_fx_pmax;
    d[10] = s_fx_rise_n;
    d[11] = s_fx_late;
    return true;
}

void IR_LED_GetSyncDebug(uint32_t d[6])
{
    d[0] = HAL_GetTick();     d[1] = s_vs_last_ms;
    d[2] = s_vs_period_cyc;   d[3] = s_hs_period_cyc;
    d[4] = SystemCoreClock;   d[5] = IR_CYCCNT;
}

void IR_LED_GetFlashInput(IR_FlashInput_t *in)
{
    uint32_t cyc_per_us = SystemCoreClock / 1000000U;

    in->level_high = HAL_GPIO_ReadPin(FLASH_GPIO_Port, FLASH_Pin) == GPIO_PIN_SET;
    in->falls      = s_fl_falls;
    in->period_us  = s_fl_period_cyc / cyc_per_us;
    in->low_us     = s_fl_low_cyc / cyc_per_us;
    in->idle_ms    = HAL_GetTick() - s_fl_last_edge_ms;
    in->toggling   = (s_fl_falls != 0U) && (in->idle_ms < IR_FLASH_IDLE_MS);
}

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
