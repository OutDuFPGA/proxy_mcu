/**
  ******************************************************************************
  * @file    ir_led_driver.c
  * @brief   HSYNC/VSYNC-synchronized IR LED strobe control (see .h for pin map).
  ******************************************************************************
  */
#include "ir_led_driver.h"

/* TODO: set this from your sensor's configured mode (total active lines per
   frame, i.e. VMAX or the active-picture line count depending on how you
   define "end of frame" for the strobe window). Used as the default line_end
   for the simple full-frame gate. */
#define IR_LED_TOTAL_LINES_PER_FRAME   2160U

typedef struct {
    GPIO_TypeDef *gate_port;
    uint16_t       gate_pin;
    volatile uint32_t line_start;
    volatile uint32_t line_end;
    volatile IR_LedMode_t mode;
    volatile bool fault_latched;
    volatile uint16_t current_level;
} IR_ChannelState_t;

static volatile uint32_t line_counter = 0;
static volatile bool master_enable = false;

static IR_ChannelState_t s_ch[IR_CH_COUNT] = {
    [IR_CH0] = {
        .gate_port = PWM_DIM0_GPIO_Port, .gate_pin = PWM_DIM0_Pin,
        .line_start = 0, .line_end = IR_LED_TOTAL_LINES_PER_FRAME,
        .mode = IR_MODE_OFF, .fault_latched = false, .current_level = 0
    },
    [IR_CH1] = {
        .gate_port = PWM_DIM1_GPIO_Port, .gate_pin = PWM_DIM1_Pin,
        .line_start = 0, .line_end = IR_LED_TOTAL_LINES_PER_FRAME,
        .mode = IR_MODE_OFF, .fault_latched = false, .current_level = 0
    },
};

static inline void gate_set(IR_Channel_t ch, GPIO_PinState state)
{
    HAL_GPIO_WritePin(s_ch[ch].gate_port, s_ch[ch].gate_pin, state);
}

/* Re-applies the immediate gate state for modes that aren't sensor-clocked
   (CONTINUOUS/OFF). FRAME_STROBE's gate is driven live by OnHSync/OnVSync
   instead, but this still forces it off here when disabled/faulted so a
   disable or a fault takes effect instantly rather than waiting for the next
   sync edge. */
static void apply_channel_gate(IR_Channel_t ch)
{
    if (!master_enable || s_ch[ch].fault_latched) {
        gate_set(ch, GPIO_PIN_RESET);
        return;
    }
    switch (s_ch[ch].mode) {
        case IR_MODE_CONTINUOUS: gate_set(ch, GPIO_PIN_SET);   break;
        case IR_MODE_OFF:        gate_set(ch, GPIO_PIN_RESET); break;
        case IR_MODE_FRAME_STROBE:
            /* Leave as-is; OnHSync/OnVSync own the gate in this mode. Forcing
               it low here would just be re-asserted wrong at the next edge -
               instead, OnVSync/OnHSync already check master_enable/fault
               themselves (see below), so nothing to do on entry. */
            break;
    }
}

void IR_LED_Init(void)
{
    HAL_GPIO_WritePin(IR_ENB_MCU_GPIO_Port, IR_ENB_MCU_Pin, GPIO_PIN_RESET);
    master_enable = false;

    for (int i = 0; i < IR_CH_COUNT; i++) {
        s_ch[i].mode = IR_MODE_OFF;
        s_ch[i].fault_latched = false;
        s_ch[i].current_level = 0;
        gate_set((IR_Channel_t)i, GPIO_PIN_RESET);
    }
    line_counter = 0;

    /* TODO: enable DAC1 (MX_DAC1_Init(), not present in the provided project
       files) before IR_LED_SetCurrentLevel() actually drives the DAC output
       — see that function. */
}

void IR_LED_Enable(bool enable)
{
    master_enable = enable;
    HAL_GPIO_WritePin(IR_ENB_MCU_GPIO_Port, IR_ENB_MCU_Pin,
                       enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (int i = 0; i < IR_CH_COUNT; i++) {
        apply_channel_gate((IR_Channel_t)i);
    }
    /* If re-enabling straight into FRAME_STROBE mid-frame, the gate stays
       off until the next VSYNC/HSYNC edge re-establishes it - normal and
       safe (worst case, one frame with no illumination), never lights the
       LEDs at the wrong point in the frame. */
}

bool IR_LED_IsEnabled(void)
{
    return master_enable;
}

void IR_LED_SetMode(IR_Channel_t ch, IR_LedMode_t mode)
{
    if (ch >= IR_CH_COUNT) return;
    s_ch[ch].mode = mode;
    apply_channel_gate(ch);
}

IR_LedMode_t IR_LED_GetMode(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return IR_MODE_OFF;
    return s_ch[ch].mode;
}

void IR_LED_SetWindow(IR_Channel_t ch, uint32_t line_start, uint32_t line_end)
{
    if (ch >= IR_CH_COUNT) return;
    s_ch[ch].line_start = line_start;
    s_ch[ch].line_end   = line_end;
}

void IR_LED_GetWindow(IR_Channel_t ch, uint32_t *line_start, uint32_t *line_end)
{
    if (ch >= IR_CH_COUNT) return;
    if (line_start) *line_start = s_ch[ch].line_start;
    if (line_end)   *line_end   = s_ch[ch].line_end;
}

void IR_LED_SetCurrentLevel(IR_Channel_t ch, uint16_t level_12bit)
{
    if (ch >= IR_CH_COUNT) return;
    if (level_12bit > 4095) level_12bit = 4095;
    s_ch[ch].current_level = level_12bit;
    /* TODO: this project's provided files don't include dac.c/dac.h, so DAC1
       isn't initialized yet. Once you add MX_DAC1_Init() via CubeMX (DAC1
       OUT1 = PA4 = DAC_DIM0, OUT2 = PA5 = DAC_DIM1), wire this up as:
       extern DAC_HandleTypeDef hdac1;
       uint32_t chan = (ch == IR_CH0) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
       HAL_DAC_SetValue(&hdac1, chan, DAC_ALIGN_12B_R, level_12bit);
       HAL_DAC_Start(&hdac1, chan);
    */
}

uint16_t IR_LED_GetCurrentLevel(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return 0;
    return s_ch[ch].current_level;
}

bool IR_LED_HasFault(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return true;
    return s_ch[ch].fault_latched;
}

void IR_LED_ClearFault(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    s_ch[ch].fault_latched = false;
    apply_channel_gate(ch);
    /* Master enable (IR_ENB_MCU) is NOT automatically restored here even if
       it was dropped by this channel's fault - re-enable explicitly with
       IR_LED_Enable(true) once you've confirmed it's safe to do so. This
       keeps "a fault happened" and "I've re-armed the system" as two
       distinct, deliberate steps. */
}

void IR_LED_OnVSync(void)
{
    line_counter = 0;
    if (!master_enable) return;
    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode != IR_MODE_FRAME_STROBE || s_ch[i].fault_latched) continue;
        if (s_ch[i].line_start == 0) {
            gate_set((IR_Channel_t)i, GPIO_PIN_SET);
        } else {
            gate_set((IR_Channel_t)i, GPIO_PIN_RESET);
        }
    }
}

void IR_LED_OnHSync(void)
{
    line_counter++;
    if (!master_enable) return;
    for (int i = 0; i < IR_CH_COUNT; i++) {
        if (s_ch[i].mode != IR_MODE_FRAME_STROBE || s_ch[i].fault_latched) continue;
        if (line_counter == s_ch[i].line_start) {
            gate_set((IR_Channel_t)i, GPIO_PIN_SET);
        }
        if (line_counter == s_ch[i].line_end) {
            gate_set((IR_Channel_t)i, GPIO_PIN_RESET);
        }
    }
}

void IR_LED_OnFault(IR_Channel_t ch)
{
    if (ch >= IR_CH_COUNT) return;
    s_ch[ch].fault_latched = true;
    gate_set(ch, GPIO_PIN_RESET);
    /* Conservative default: drop the shared master enable on any single
       channel's fault. Relax to per-channel-only if the driver boards are
       meant to fault independently without affecting each other. */
    IR_LED_Enable(false);
}

/**
  * @brief EXTI callback dispatcher for SN0_HS, SN0_VS, IR_FAULT0, IR_FAULT1.
  *
  * If gpio.c or stm32h5xx_it.c already defines HAL_GPIO_EXTI_Callback()
  * elsewhere in the project, merge this dispatch logic into that single
  * definition instead — the HAL only allows one non-weak implementation.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case SN0_HS_Pin:     IR_LED_OnHSync();       break;
        case SN0_VS_Pin:     IR_LED_OnVSync();       break;
        case IR_FAULT0_Pin:  IR_LED_OnFault(IR_CH0); break;
        case IR_FAULT1_Pin:  IR_LED_OnFault(IR_CH1); break;
        default: break;
    }
}
