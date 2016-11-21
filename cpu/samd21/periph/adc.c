/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_samr21
 * @{
 *
 * @file
 * @brief       ADC driver implementation
 *
 * @author      Rane Balslev (SAMR21) <ranebalslev@gmail.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Mark Solters <msolters@driblet.io>
 * @author      Michael Andersen <m.andersen@berkeley.edu>
 * @author      Hyun Sin Kim <hs.kim@berkeley.edu>
 *
 * @}
 */

#include <stdint.h>
#include "cpu.h"
#include "periph/adc.h"
#include "periph_conf.h"
#define ENABLE_DEBUG    (0)
#include "debug.h"

/* guard in case that no ADC device is defined */
#if ADC_NUMOF

int adc_init(adc_t dev) {
#if ADC_0_EN
  	while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* Disable ADC Module before init. */
    ADC_0_DEV->CTRLA.bit.ENABLE = 0;

  	/* Setup generic clock mask for adc */
  	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

    /* GCLK Setup*/
  	GCLK->CLKCTRL.reg = (uint32_t)(GCLK_CLKCTRL_CLKEN     |
                                     GCLK_CLKCTRL_GEN_GCLK0 |
                                     (ADC_GCLK_ID << GCLK_CLKCTRL_ID_Pos));

    /* Set RUN_IN_STANDBY */
    ADC_0_DEV->CTRLA.bit.RUNSTDBY = 0;

    /* Set Voltage Reference */
    ADC_0_DEV->REFCTRL.bit.REFSEL  = ADC_REFCTRL_REFSEL_INT1V_Val;
    ADC_0_DEV->REFCTRL.bit.REFCOMP = 1;

    /* Set the accumulation and divide result */
    ADC_0_DEV->AVGCTRL.bit.SAMPLENUM = 0;
	  ADC_0_DEV->AVGCTRL.bit.ADJRES    = 0;

    /* Set Sample length */
    ADC_0_DEV->SAMPCTRL.bit.SAMPLEN = 0;
	  while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* Configure CTRLB Register */
    ADC_0_DEV->CTRLB.bit.DIFFMODE  = 0;
  	ADC_0_DEV->CTRLB.bit.FREERUN   = 0;
  	ADC_0_DEV->CTRLB.bit.CORREN    = 0;
  	ADC_0_DEV->CTRLB.bit.LEFTADJ   = 1; // Left-adjusted results
  	ADC_0_DEV->CTRLB.bit.RESSEL    = ADC_CTRLB_RESSEL_12BIT_Val;
  	ADC_0_DEV->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4_Val;
  	while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

  	ADC_0_DEV->INPUTCTRL.bit.GAIN        = ADC_INPUTCTRL_GAIN_1X_Val;
  	ADC_0_DEV->INPUTCTRL.bit.INPUTOFFSET = 0;
  	ADC_0_DEV->INPUTCTRL.bit.MUXPOS      = ADC_INPUTCTRL_MUXPOS_PIN16_Val; // PA08 PIN (light sensor of hamilton)
  	ADC_0_DEV->INPUTCTRL.bit.MUXNEG      = ADC_INPUTCTRL_MUXNEG_IOGND_Val;
  	while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* Configure Window Mode Register */
    ADC_0_DEV->WINCTRL.reg = ADC_WINCTRL_WINMODE_DISABLE;
    while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* Enable interrupts and events related to result */
    ADC_0_DEV->EVCTRL.bit.RESRDYEO = 1;
    ADC_0_DEV->INTENSET.bit.RESRDY = 1;

    /* ADC runs during debug mode */
    ADC_0_DEV->DBGCTRL.bit.DBGRUN = 1;

    ADC_0_DEV->CTRLA.bit.SWRST = 1;
    while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    //  Enable ADC Module.
    ADC_0_DEV->CTRLA.bit.ENABLE = 1;
    while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    return 0;
#endif
}



int adc_sample(adc_t dev, adc_res_t res){
#if ADC_0_EN
    int output;

    /*  Enable bandgap */
    SYSCTRL->VREF.reg |= SYSCTRL_VREF_BGOUTEN;

    ADC_0_DEV->CTRLA.bit.SWRST = 1;

    while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    ADC_0_DEV->CTRLA.bit.ENABLE = 1;

    while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

  	switch (res) {
		case ADC_RES_8BIT:
  			ADC_0_DEV->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
  			break;
		case ADC_RES_10BIT:
  			ADC_0_DEV->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
  			break;
		case ADC_RES_12BIT:
  			ADC_0_DEV->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
  			break;
		case ADC_RES_16BIT:
  			ADC_0_DEV->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
  			break;
		default:
  			return -1;
  			break;
  	}

  	/* Wait for sync. */
  	while (ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

  	/* Start the conversion. */
  	ADC_0_DEV->SWTRIG.reg = ADC_SWTRIG_START;

  	/* Wait for the result. */
  	while (!(ADC_0_DEV->INTFLAG.reg & ADC_INTFLAG_RESRDY));

  	/* Read result */
  	output = (int)ADC_0_DEV->RESULT.reg;
  	while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

    ADC_0_DEV->CTRLA.bit.ENABLE = 0;
  	while(ADC_0_DEV->STATUS.reg & ADC_STATUS_SYNCBUSY);

  	/*  Disable bandgap */
  	SYSCTRL->VREF.reg &= ~SYSCTRL_VREF_BGOUTEN;

  	/* Return result. */
  	return output;
#endif
}


#endif /* ADC_NUMOF */
