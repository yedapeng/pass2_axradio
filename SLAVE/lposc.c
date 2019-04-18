/**
*******************************************************************************************************
* @file lposc.c
* @brief Calibration of LPOSC Oscillator
* @internal
* @author   Thomas Sailer, Janani Chellappan, Srinivasan Tamilarasan
* $Rev: $
* $Date: $
********************************************************************************************************
* Copyright 2016 Semiconductor Components Industries LLC (d/b/a “ON Semiconductor”).
* All rights reserved.  This software and/or documentation is licensed by ON Semiconductor
* under limited terms and conditions.  The terms and conditions pertaining to the software
* and/or documentation are available at http://www.onsemi.com/site/pdf/ONSEMI_T&C.pdf
* (“ON Semiconductor Standard Terms and Conditions of Sale, Section 8 Software”) and
* if applicable the software license agreement.  Do not use this software and/or
* documentation unless you have carefully read and you agree to the limited terms and
* conditions.  By using this software and/or documentation, you agree to the limited
* terms and conditions.
*
* THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* ON SEMICONDUCTOR SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
* INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
* @endinternal
*
* @ingroup TEMPLATE_FIRMWARE_5043
*
* @details
*/
#if defined __ARMEL__ || defined __ARMEB__
#ifdef __AXM0
#include <axm0f143.h>
#elif defined __AXM0F2
#include <axm0f243.h>
#endif // __AXM0
#else
#include <ax8052f143.h>
#endif
#include <libmftypes.h>
#include <libmfradio.h>
#include "../COMMON/misc.h"
#include "../AX_Radio_Lab_output/configslave.h"
#include "../COMMON/easyax5043.h"

static void wait_n_lposccycles(uint8_t n)
{
    uint8_t cnt = 0;
    __disable_irq();
    radio_write8(AX5043_REG_IRQMASK1, radio_read8(AX5043_REG_IRQMASK1) | 0x04); // LPOSC irq
    for(;;)
    {
        if( radio_read8(AX5043_REG_IRQREQUEST1) & 0x04 )
        {
            cnt++;
            radio_read8(AX5043_REG_LPOSCSTATUS); // clear irq request
        }

        if(cnt > n)
            break;
        enter_standby();
    }

    radio_write8(AX5043_REG_IRQMASK1, (radio_read8(AX5043_REG_IRQMASK1) & ~0x04)); // disable LPOSC irq
    __enable_irq();
}

void calibrate_lposc(void)
{
        radio_write8(AX5043_REG_LPOSCFREQ1, 0x00);
        radio_write8(AX5043_REG_LPOSCFREQ0, 0x00);
#if !(FXTAL >= 33000000 && FXTAL <= 47000000)
        radio_write8(AX5043_REG_LPOSCREF1, (((FXTAL/640)>>8) & 0xFF));
        radio_write8(AX5043_REG_LPOSCREF0, (((FXTAL/640)>>0) & 0xFF));
        radio_write8(AX5043_REG_PWRMODE, AX5043_PWRSTATE_SYNTH_RX);
        radio_write8(AX5043_REG_LPOSCKFILT1, ((lposckfiltmax >> (8 + 1)) & 0xFF)); // kfiltmax >> 1
        radio_write8(AX5043_REG_LPOSCKFILT0, ((lposckfiltmax >> 1) & 0xFF));
        axradio_wait_for_xtal();

        radio_write8(AX5043_REG_LPOSCCONFIG, 0x25); // LPOSC ENA, slow mode; calibrate on rising edge, irq on rising edge
        wait_n_lposccycles(6);

#if FXTAL > 30000000
        if( radio_read8(AX5043_REG_LPOSCFREQ1) == 0x7F)
        {
            // freq tuning went to max pos. start 25% lower
            radio_write8(AX5043_REG_LPOSCFREQ1, 0xBC);
            wait_n_lposccycles(6);
        }

        if( radio_read8(AX5043_REG_LPOSCFREQ1) == 0x80)
        {
            // freq tuning went to max neg. start 25% higher
            radio_write8(AX5043_REG_LPOSCFREQ1, 0xA8);
            wait_n_lposccycles(6);
        }
#endif
        radio_write8(AX5043_REG_LPOSCKFILT1, ((lposckfiltmax >> (8 + 2)) & 0xFF)); // kfiltmax >> 2
        radio_write8(AX5043_REG_LPOSCKFILT0, ((lposckfiltmax >> 2) & 0xFF));
        wait_n_lposccycles(5);

        radio_write8(AX5043_REG_LPOSCCONFIG, 0x00);
        radio_write8(AX5043_REG_PWRMODE, AX5043_PWRSTATE_POWERDOWN);

        {
            uint8_t x = radio_read8(AX5043_REG_LPOSCFREQ1);
            if( x == 0x7f || x == 0x80 )
            {
                radio_write8(AX5043_REG_LPOSCFREQ1, 0);
                radio_write8(AX5043_REG_LPOSCFREQ0, 0);
            }
        }

#endif
}
