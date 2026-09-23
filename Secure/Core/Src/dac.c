/**
  ******************************************************************************
  * @file    dac.c
  * @brief   DAC1 register-level bring-up for the two IR analog dimming pins.
  *          See dac.h for why this does not go through the HAL.
  ******************************************************************************
  */
#include "dac.h"

static bool s_ready = false;

void MX_DAC1_Init(void)
{
    /* DAC1 sits on AHB2 on the STM32H5. The RCC handle used here is the
       secure alias (RCC == RCC_S in a secure build), matching the rest of
       this partition. */
    __HAL_RCC_DAC1_CLK_ENABLE();

    /* MCR is only writable while the channel is disabled, so drop both
       enables first. This also makes the function safe to call twice. */
    DAC1->CR &= ~(DAC_CR_EN1 | DAC_CR_EN2);

    /* MODEx = 000: normal mode, output buffer enabled, connected to the
       external pin only. The buffer matters here - CTRL1 on the LED board is
       reached through a connector and ~10cm of harness, and the LT3763's own
       CTRL1 leakage is only 20nA, so an unbuffered output would be at the
       mercy of any leakage on that net. */
    DAC1->MCR &= ~(DAC_MCR_MODE1 | DAC_MCR_MODE2);

    /* Start both channels at zero so enabling them cannot produce a current
       step on the LED string. */
    DAC1->DHR12R1 = 0U;
    DAC1->DHR12R2 = 0U;

    /* TENx left at 0: no external trigger, so a DHR write goes to the output
       on the next APB cycle rather than waiting for a trigger event. */
    DAC1->CR |= (DAC_CR_EN1 | DAC_CR_EN2);

    /* Buffered channels need tWAKEUP (~8-15us on this part) before the output
       is valid. Nothing downstream reads it that fast, but this keeps the
       "DAC is ready" promise of DAC1_IsReady() honest. */
    for (volatile uint32_t i = 0; i < 2000U; i++) {
        __NOP();
    }

    s_ready = true;
}

void DAC1_SetValue(uint32_t dac_channel, uint16_t code_12bit)
{
    if (code_12bit > DAC1_CODE_MAX) {
        code_12bit = DAC1_CODE_MAX;
    }
    if (dac_channel == IR_DAC1_CH1) {
        DAC1->DHR12R1 = code_12bit;
    } else if (dac_channel == IR_DAC1_CH2) {
        DAC1->DHR12R2 = code_12bit;
    }
}

uint16_t DAC1_GetValue(uint32_t dac_channel)
{
    if (dac_channel == IR_DAC1_CH1) {
        return (uint16_t)(DAC1->DHR12R1 & DAC_DHR12R1_DACC1DHR);
    }
    if (dac_channel == IR_DAC1_CH2) {
        return (uint16_t)(DAC1->DHR12R2 & DAC_DHR12R2_DACC2DHR);
    }
    return 0U;
}

bool DAC1_IsReady(void)
{
    return s_ready;
}
