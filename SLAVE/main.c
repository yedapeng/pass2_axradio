/**
*******************************************************************************************************
* @file SLAVE\main.c
* @brief Code skeleton for SLAVE module, illustrating reception of packets.
*        The packet format is determined by AX-RadioLAB_output\config.c, produced by the AX-RadioLab GUI
* @internal
* @author   Thomas Sailer, Janani Chellappan, Srinivasan Tamilarasan, Anand Agrawal
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

#include "../AX_Radio_Lab_output/configslave.h"

#if defined __ARMEL__ || defined __ARMEB__
#ifdef __AXM0
#include <axm0f143.h>
#include "axm0_config.h"
#include "axm0_xbar.h"
#include "axm0.h"
#elif defined __AXM0F2
#include <axm0f243.h>
#include "axm0_config.h"
#include "axm0f2_pin.h"
#include "axm0f2.h"
#endif
#else
#include <ax8052f143.h>
#endif
#include <libmftypes.h>
#include <libmfradio.h>
#include <libmfflash.h>
#include <libmfwtimer.h>
#include "libmfosc.h"

#ifdef USE_COM0
#include <libmfuart0.h>
#endif // USE_COM0

#ifdef USE_LCD
#include <libaxlcd2.h>
#endif // USE_LCD

#ifdef USE_DBGLINK
#include <libmfdbglink.h>
#include <libmfuart.h>
#include <libmfuart0.h>
#include <libmfuart1.h>
#endif // USE_DBGLINK

#define ADJUST_FREQREG
#define FREQOFFS_K 3

#if defined(USE_LCD) || defined(USE_COM0)
#define USE_DISPLAY
#endif // defined(USE_LCD) || defined(USE_COM0)

#include "../COMMON/display_com0.h"

#ifdef MINI_KIT
#include <libminikitleds.h>
#else
#include <libdvk2leds.h>
#endif // MINI_KIT

//#include <stdlib.h>

#include "../COMMON/misc.h"
#include "../COMMON/configcommon.h"

// TODO : Verify the function display_writenum16 in libaxdvk2.a
#if defined __ARMEL__ || defined __ARMEB__
#include <stdbool.h>
#ifdef __AXM0
#define AXM0_REVA
#endif
const bool txondemand_mode = 0;
const bool AxM0_Mini_Kit = 0;
extern void RADIO_IRQ(void);
#endif // defined

#if defined(SDCC)
extern uint8_t _start__stack[];
#endif

uint8_t __data coldstart = 1; // caution: initialization with 1 is necessary! Variables are initialized upon _sdcc_external_startup returning 0 -> the coldstart value returned from _sdcc_external startup does not survive in the coldstart case
uint16_t __data pkts_received = 0, pkts_missing = 0;

// Debug Link Configuration
#if defined __ARMEL__ || defined __ARMEB_
    #if defined USE_DBGLINK
        void debuglink_init_axm0(void)
        {
            #ifdef __AXM0
            axm0_xbar_dbglink_config(UART0_TX, UART0_RX_2, AXM0_XBAR_PIN_NOT_REQUIRED, AXM0_XBAR_PIN_NOT_REQUIRED); // tx_pin, rx_pin, UARTx_CLK pin, TxOUT pin
			#endif /* __AXM0 */
            dbglink_timer0_baud(0, 9600, AXM0XX_HFCLK_CLOCK_FREQ);  // HS OSC, Baud rate, Clock
            dbglink_init(0, 8, 1);                              	// TImer No., Word length, Stop bit
        }
    #endif // USE_DBGLINK
#endif // defined

#if defined __ARMEL__ || defined __ARMEB__
#ifdef __AXM0
void GPIO_Handler (void)
{

    // RIRQ on PR5
    if(GPIO->INT_STS & 0x20000000)
    {
        NVIC_ClearPendingIRQ;
        GPIO->INT_STS = 0x20000000;
        RADIO_IRQ();
    }
}
#endif // __AXM0
#endif // defined

// TODO : If IAR will this condition is valid
#if defined SDCC || defined __ICC8051__
static void pwrmgmt_irq(void) __interrupt(INT_POWERMGMT)
{
    uint8_t pc = PCON;
    if (!(pc & 0x80))
        return;
    GPIOENABLE = 0;
    IE = EIE = E2IE = 0;
    for (;;)
        PCON |= 0x01;
}
#endif

void axradio_statuschange(struct axradio_status __xdata *st)
{
    //    static uint16_t rssi_ratelimit;
    #if defined(USE_DBGLINK) && defined(DEBUGMSG)
        if (DBGLNKSTAT & 0x10 && st->status != AXRADIO_STAT_CHANNELSTATE) {
            dbglink_writestr("ST: 0x");
            dbglink_writehex16(st->status, 2, WRNUM_PADZERO);
            dbglink_writestr(" ERR: 0x");
            dbglink_writehex16(st->error, 2, WRNUM_PADZERO);
            dbglink_tx('\n');
        }
    #endif
    switch (st->status)
    {
        case AXRADIO_STAT_RECEIVE:
            #if RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
            switch (st->error)
            {
                case AXRADIO_ERR_TIMEOUT:
                    led2_on();
                    // fall through

                case AXRADIO_ERR_NOERROR:
                case AXRADIO_ERR_RESYNCTIMEOUT:
                    #ifdef MINI_KIT
                        led0_off();
                    #else
                        led3_off();
                    #endif // MINI_KIT
                    break;

                case AXRADIO_ERR_RESYNC:
                    axradio_set_freqoffset(0);
                    // fall through

                case AXRADIO_ERR_RECEIVESTART:
                    #ifdef MINI_KIT
                        led0_on();
                    #else
                        led3_on();
                    #endif // MINI_KIT
                    break;

                default:
                    break;
            }

            if (st->error == AXRADIO_ERR_NOERROR)
            {
                ++pkts_received;
                led2_off();
                #ifdef USE_DISPLAY
                            display_received_packet(st);
                #endif // USE_DISPLAY
            }
            #ifdef USE_DBGLINK
                switch (st->error)
                {
                    case AXRADIO_ERR_RESYNCTIMEOUT:
                        if (DBGLNKSTAT & 0x10)
                            dbglink_writestr("RESYNC Timeout\n");
                        break;

                    case AXRADIO_ERR_RESYNC:
                        if (DBGLNKSTAT & 0x10)
                            dbglink_writestr("RESYNC\n");
                        break;

                    default:
                        break;
                }
            #endif // USE_DBGLINK
            #else  // RADIO_MODE
            {
                uint8_t retran = (st->error != AXRADIO_ERR_NOERROR);
                ++pkts_received;
                led0_toggle();
                #ifdef USE_DISPLAY
                    if (st->error == AXRADIO_ERR_NOERROR)
                        retran = display_received_packet(st) || retran;
                #endif // USE_DISPLAY

                if (retran)
                    led2_on();
                else
                    led2_off();
            }
            #endif // RADIO_MODE
            #ifdef USE_DBGLINK
                if (st->error == AXRADIO_ERR_NOERROR)
                    dbglink_received_packet(st);
            #endif
            // Frequency Offset Correction
            {
                int32_t foffs = axradio_get_freqoffset();
                #if defined(ADJUST_FREQREG)
                    foffs -= (st->u.rx.phy.offset)>>(FREQOFFS_K); //adjust RX frequency by low-pass filtered frequency offset
                #endif
                // reset if deviation too large
                if (axradio_set_freqoffset(foffs) != AXRADIO_ERR_NOERROR)
                    axradio_set_freqoffset(0);
            }
            break;
#if 0
    case AXRADIO_STAT_CHANNELSTATE:
        if (st->u.cs.busy)
            led3_on();
        else
            led3_off();
        {
            uint16_t t = wtimer0_curtime();
            #if WTIMER0_CLKSRC == CLKSRC_LPXOSC
                #define RSSIRATE (uint16_t)(32768/2)
            #elif WTIMER0_CLKSRC == CLKSRC_LPOSC
                #define RSSIRATE (uint16_t)(640/2)
            #else
                #error "unknown wtimer0 clock source"
            #endif
            if ((uint16_t)(t - rssi_ratelimit) < RSSIRATE)
                break;
            rssi_ratelimit = t;
        }
        display_setpos(0x48);
        display_writestr("R:");
        display_tx(st->u.cs.busy ? '*' : ' ');
        display_writenum16(st->u.cs.rssi, 5, WRNUM_SIGNED);
        break;
#endif

	case AXRADIO_STAT_TRANSMITSTART:
        led3_on();
        break;

    case AXRADIO_STAT_TRANSMITEND:
        led3_off();
        break;

    default:
        break;
    }
}

#if defined __ARMEL__ || defined __ARMEB__
#ifdef __AXM0
/* Clock Calibration   */
static inline void clk_calibrate()
{
    /* Configure HSOSC and LPOSC freq, prescalar, clock source  */
    CMU->HS_OSC_CFG = 0x000001f4;
    CMU->LP_OSC_CFG = 0x3C;

    /* Setup the calibration filter for the Fast RC Oscillator and LPOSC. */
    CMU->HS_OSC_FILT = 0x00004000;
    CMU->LP_OSC_FILT = 0x4000;

    /* select the reference frequency divide. FOR HSOSC and LSOSC  */
    CMU->HS_OSC_REF_DIV = 0x00001300;
    CMU->LP_OSC_REF_DIV = 0x4F80;

    /* select the frequency trim value for 20MHz AND 640Hz operation. */
    CMU->HS_OSC_20M_FREQ_TUNE = 0x00000140;   // Configure the tune value to board specific
    CMU->LP_OSC_FREQ_TUNE = 0x00000220;       // Configure the tune value to board specific
}

/* Enable Peripheral Clocks */
static inline void pclk_En()
{
    CMU->PCLK_CFG_b.GPIO_EN = 1;       /* Enable GPIO Clock */
    CMU->PCLK_CFG_b.WUT_EN = 1;        /* Enable Wakeup Timer Clock */
    CMU->PCLK_CFG_b.CL_SYSCFG_EN = 1;  /* Enable Clock & System config Clock */
    CMU->PCLK_CFG_b.PMU_EN = 1;        /* Enable PMU Clock */
    CMU->PCLK_CFG_b.XBAR_EN = 1;       /* Enable XBAR Clock */
    CMU->PCLK_CFG_b.FLASH_EN = 1;      /* Enable Wakeup Flash Clock */
    CMU->PCLK_CFG_b.TICKER_EN = 1;     /* Enable Ticker Timer Clock */
    CMU->PCLK_CFG_b.SPI0_EN = 1;       /* Enable SPI 0 Clock */
    CMU->PCLK_CFG_b.UART0_EN = 1;      /* Enable UART 0 Clock */
}

/* Initialize Hardware ports */
static inline void portInit()
{
#ifndef MINI_KIT
    /* Configure state of PORTA */
     /* LEDs at PA0(GPIO0), PA2(GPIO2), PA5(GPIO5), Configure as Output and turn off */
     GPIO_OR->OUT_EN = ((1<<0) | (1<<2) | (1<<5));
     GPIO_AND->DATA_OUT = ~((1<<0)|(1<<2)|(1<<5));

     /* LPXOSC at PA3(GPIO3), PA4(GPIO4), Configure as high Impedance
     Unused pins are PA6(GPIO6), PA7(GPIO7). Configure as PullUp to minimize current consumption */
     XBAR_OR->PULL_UP_CFG = ((1<<7)|(1<<6)|(1<<1));
     XBAR_AND->PULL_DOWN_CFG = ~((1<<7)|(1<<6)|(1<<1));

     /* Configure state of PORTB */

     /* LCD pins at PB0(GPIO8), PB1(GPIO9), Configure as Output.
     For LCD give high and low output at PB1(GPIO9) AND PB0(GPIO8) respectively */
     GPIO_OR->OUT_EN = ((1<<8) | (1<<9));
     GPIO_OR->DATA_OUT = 1<<9;
     GPIO_AND->DATA_OUT = ~(1<<8);

     /* Switch(D & R) at PB2(GPIO10), PB3(GPIO11), Configure as Input and PullUp
     Switch(L & U) at PB6(GPIO14), PB7(GPIO15), Configure as Input and PullUp
     Debug pins are PB4(GPIO12), PB5(GPIO13). Configure as PullUp */
     XBAR_OR->PULL_UP_CFG = ((1<<10)|(1<<11)|(1<<12)|(1<<13)|(1<<14)|(1<<15));
     XBAR_AND->PULL_DOWN_CFG = ~((1<<10)|(1<<11)|(1<<12)|(1<<13)|(1<<14)|(1<<15));

     /* Configure state of PORTC */

     /* LCD pins at PC0(GPIO16), PC1(GPIO17), PC2(GPIO18), PC3(GPIO19), Configure as Output
     For LCD give high and low output at (PC0, PC1) AND (PC2, PC3) respectively */
     GPIO_OR->OUT_EN = ((1<<16) | (1<<17) | (1<<18) | (1<<19));
     GPIO_OR->DATA_OUT = 1<<16 | 1<<17;
     GPIO_AND->DATA_OUT = ~((1<<18)|(1<<19));

     /* Unused pins are PC4(GPIO20), PC5(GPIO21), PC6(GPIO22), PC7(GPIO23). Configure as PullUp to minimize current consumption*/
     XBAR_OR->PULL_UP_CFG = ((1<<20)|(1<<21)|(1<<22)|(1<<23));
     XBAR_AND->PULL_DOWN_CFG = ~((1<<20)|(1<<21)|(1<<22)|(1<<23));

     /* Configure state of PORTR */

     /* RF SIGNALS from DVK to RF Module at PR0(GPIO24), PR2(GPIO26), PR4(GPIO28), Configure as Output. */
     GPIO_OR->OUT_EN = ((1<<24) | (1<<26) | (1<<28));
     GPIO_AND->DATA_OUT = ~((1<<24) | (1<<26) | (1<<28));

     /* RF SIGNALS from RF Module to DVK at PR1(GPIO25), PR3(GPIO27), Configure as Input and PullUp.
     Unused pins are PR6(GPIO30), PR7(GPIO31). Configure as PullUp to minimize current consumption */
    // XBAR_OR->PULL_UP_CFG = ((1<<25)|(1<<27)|(1<<30)|(1<<31));
   //  XBAR_AND->PULL_DOWN_CFG = ~((1<<25)|(1<<27)|(1<<30)|(1<<31));

     display_portinit();
#else
    /* Configure state of PORTA */
     XBAR_OR->PULL_UP_CFG = ((1<<7)|(1<<6)|(1<<5)|(1<<2)|(1<<1)|(1<<0));
     XBAR_AND->PULL_DOWN_CFG = ~((1<<7)|(1<<6)|(1<<5)|(1<<2)|(1<<1)|(1<<0));

     /* Configure state of PORTB */
     GPIO_OR->OUT_EN = ((1<<9) | (1<<10));
     GPIO_OR->DATA_OUT = 1<<10;
     GPIO_AND->DATA_OUT = ~(1<<9);

     XBAR_OR->PULL_UP_CFG = ((1<<8)|(1<<11)|(1<<12)|(1<<13)|(1<<14)|(1<<15));
     XBAR_AND->PULL_DOWN_CFG = ~((1<<8)|(1<<11)|(1<<12)|(1<<13)|(1<<14)|(1<<15));

     /* Configure state of PORTC */
     XBAR_OR->PULL_UP_CFG = ((1<<16)|(1<<17)|(1<<18)|(1<<19)|(1<<20)|(1<<21)|(1<<22)|(1<<23));
     XBAR_AND->PULL_DOWN_CFG = ~((1<<16)|(1<<17)|(1<<18)|(1<<19)|(1<<20)|(1<<21)|(1<<22)|(1<<23));

     /* Configure state of PORTR */

     /* RF SIGNALS from DVK to RF Module at PR0(GPIO24), PR2(GPIO26), PR4(GPIO28), Configure as Output. */
     GPIO_OR->OUT_EN = ((1<<24) | (1<<26) | (1<<28));
     /// GPIO_AND->DATA_OUT = ~((1<<24) | (1<<26) | (1<<28));

     GPIO_OR->DATA_OUT = (1<<24);
     GPIO_AND->DATA_OUT = ~((1<<26) | (1<<28));


     /* RF SIGNALS from RF Module to DVK at PR1(GPIO25), PR3(GPIO27), Configure as Input and PullUp.
     Unused pins are PR6(GPIO30), PR7(GPIO31). Configure as PullUp to minimize current consumption */
    // XBAR_OR->PULL_UP_CFG = ((1<<25)|(1<<27)|(1<<30)|(1<<31));
   //  XBAR_AND->PULL_DOWN_CFG = ~((1<<25)|(1<<27)|(1<<30)|(1<<31));

        XBAR_OR->PULL_UP_CFG = ((1<<25)|(1<<27)|(1<<30)|(1<<31));
        XBAR_AND->PULL_DOWN_CFG = ~((1<<25)|(1<<27)|(1<<30)|(1<<31));

     ///lcd2_portinit();
#endif
}

/* Configure clock freq for HSOSC and LPOSC, Also configure System clock */
static inline void clk_config()
{
     /* Configure HSOSC to 20MHz clock frequency */
     CMU->HS_OSC_CFG = AXM0_CONFIG_HSCLK_20M;
     /* Configure LPOSC to 640Hz clock frequency */
     CMU->LP_OSC_CFG = AXM0_CONFIG_LPCLK_640;
     /* Configure System Clock to be HSOSC clock */
     CMU->CFG = AXM0_CONFIG_SYSCLK_HSOSC;
}

/* External startup function for AXM0+ */
uint8_t _axm0_external_startup(void)
{
    __disable_irq();
    /* Calibrate Clocks */
    clk_calibrate();
    /* Configure clocks */
    clk_config();
    /* Enable Peripheral clocks */
    pclk_En();
    /* Set clock source and prescalar for wakeupTimer0 */
    wtimer0_setclksrc(WTIMER0_CLKSRC, WTIMER0_PRESCALER);
    /* Set clock source and prescalar for wakeupTimer1 */
    wtimer1_setclksrc(CLKSRC_FRCOSC, 7);
    /* Configure Hardware Ports */
    portInit();

    /* Calibrate 20MHz Internal oscillator using Radio Sysclock.
    If radio module not connected to MCU then MCU will consider default trim value &
    the user responsibility to update the trim value manually */
    axm0_calib_hs_osc(1);

    if (((PMU->STS & 0x02) && (PMU->MOD == 0x1)))
        return 1;
    return 0;
}
#endif // __AXM0
#ifdef __AXM0F2

uint8_t _axm0f2_external_startup(void)
{
    uint8_t     start_cause = get_startcause();


#ifndef MINI_KIT


     if (start_cause == STARTCAUSE_COLDSTART) {

  //   __disable_irq();
    /* Set clock source and prescalar for wakeupTimer0 */
    wtimer0_setclksrc(WTIMER0_CLKSRC, WTIMER0_PRESCALER);
    /* Set clock source and prescalar for wakeupTimer1 */
    wtimer1_setclksrc(CLKSRC_FRCOSC, 7);

    /** \brief config port0
     *  0.0 --> LCD_SEL --> Strong drive ---> (1)
     *  0.1, 0.2 --> BUTTON ---> Weak pullup -->(1)
     *  0.3, 0.7 --> High Imp Analog
     *  0.4, 0.5, 0.6  ---> high imp digital
     *  PORT0 --> GPIO
     */

    HSIOM->PORT_SEL0 = 0x00000000u;
    GPIO_PRT0->DR = 0x00000007u;
    GPIO_PRT0->PC = 0x00049096u;
	GPIO_PRT0->PC2 = 0x00000000u;

    /** \brief config port1
     *  1.0, 1.2.--> LCD --> Strong drive ---> (1)
     *  1.1, 1.3, 1.5 --> led --> strong drive (0)
     *  1.4, 1.6, 1.7  ---> high imp analog
     *  PORT1 --> LCD - SPI; OTHERS - GPIO
     */

    HSIOM->PORT_SEL1 = 0x00000F0Fu;
    GPIO_PRT1->DR = 0x00000005u;
    GPIO_PRT1->PC = 0x00030DB6u;
	GPIO_PRT1->PC2 = 0x00000000u;

	/// TODO
    // PORT 2 configurations from libmf
    // PORT3 config 3.0 - 3.3 from libmf, 3.4 - 3.7 TP

    /** \brief config port4
     *  4.0 --> led --> weak pullup ---> (0)
     *  4.1 ---> RADIO ---> strong drive (0)
     *  4.2  ---> strong drive (0)
     *  4.3  ---> strong drive (1)
     *  PORT4 --> GPIO
     *
     */

    HSIOM->PORT_SEL4 &= ~0xFFFFu;
    GPIO_PRT4->DR = 0x00000008u;
    GPIO_PRT4->PC = 0x00000DB6u;
	GPIO_PRT4->PC2 = 0x00000000;

	display_portinit();
	}
    if(start_cause == STARTCAUSE_COLDSTART)
    {
        coldstart = 1; 	/* Cold start */
        return 0; 		/* Variables init required */
    }
    else
    {
        coldstart = 0; 	/* Warm start */
        return 1; 		/* Variables init not required */
    }

#else
    if (start_cause == STARTCAUSE_COLDSTART) {

 //   __disable_irq();

    /* Set clock source and prescalar for wakeupTimer0 */
    wtimer0_setclksrc(WTIMER0_CLKSRC, WTIMER0_PRESCALER);
    /* Set clock source and prescalar for wakeupTimer1 */
    wtimer1_setclksrc(CLKSRC_FRCOSC, 7);


    /** \brief config port0
     *  0.0 --> Radio-VTCXO --> High Imp Analog
     *  0.1, 0.3, 0.7 --> High Imp Analog
     *  0.2, 0.4, 0.5, 0.6  ---> high imp digital
     *
     *  PORT0 --> GPIO
     */

    HSIOM->PORT_SEL0 = 0x00000000u;
    GPIO_PRT0->DR = 0x00000000u;
    GPIO_PRT0->PC = 0x00049040u;
	GPIO_PRT0->PC2 = 0x00000000u;

    /** \brief config port1
     *  high imp analog
     *
     *  PORT1 --> GPIO
     *
     */

    HSIOM->PORT_SEL1 = 0x00000000u;
    GPIO_PRT1->DR = 0x00000000u;
    GPIO_PRT1->PC = 0x00000000u;
	GPIO_PRT1->PC2 = 0x00000000u;

	    /** \brief config port3
     *  3_6 --> push button
     *  3_7 --> LED
     *
     */

    HSIOM->PORT_SEL3 &= ~0xFF000000u;
    GPIO_PRT3->DR |= 0x00000040u;
    GPIO_PRT3->PC |= 0x00C40000u;

    /** \brief config port4
     *  high impedance digital
     *  PORT4 --> GPIO
     *
     */
    HSIOM->PORT_SEL4 = 0x00000000u;
    GPIO_PRT4->DR = 0x00000000u;
    GPIO_PRT4->PC = 0x00000249u;
	GPIO_PRT4->PC2 = 0x00000000;
	}

    if(start_cause == STARTCAUSE_COLDSTART)
    {
        coldstart = 1; 	/* Cold start */
        return 0; 		/* Variables init required */
    }
    else
    {
        coldstart = 0; 	/* Warm start */
        return 1; 		/* Variables init not required */
    }

#endif // MINIKIT
}
#endif // __AXM0F2
#else

#if defined(__ICC8051__)
//
// If the code model is banked, low_level_init must be declared
// __near_func elsa a ?BRET is performed
//
#if (__CODE_MODEL__ == 2)
__near_func __root char
#else
__root char
#endif
__low_level_init(void) @ "CSTART"
#else
uint8_t _sdcc_external_startup(void)
#endif
{
    LPXOSCGM = 0x8A;
    wtimer0_setclksrc(WTIMER0_CLKSRC, WTIMER0_PRESCALER);
    wtimer1_setclksrc(CLKSRC_FRCOSC, 7);

     // caution: coldstart has to be initialized with 1 in it's definition! Variables are initialized upon _sdcc_external_startup returning 0 -> the coldstart value returned from _sdcc_external startup does not survive in the coldstart case
    coldstart = !(PCON & 0x40);

    ANALOGA = 0x18; // PA[3,4] LPXOSC, other PA are used as digital pins
#ifndef MINI_KIT
    PORTA = 0xC0 | (PINA & 0x25); // pull-up for PA[6,7] which are not bonded, no pull up for PA[3,4] (LPXOSC). Output 0 in PA[0,1,2,5] to prevent current consumption in all DIP switch states
    PORTB = 0xFE; //PB[0,1]  (LCD RS, LCD RST) are overwritten by lcd2_portinit(), enable pull-ups for PB[2..7]  (PB[2,3] for buttons, PB[4..7] unused)
    PORTC = 0xF3 | (PINC & 0x08); // set PC0 = 1 (LCD SEL), PC1 = 1 (LCD SCK), PC2 = 0 (LCD MOSI), PC3 =0 (LED), enable pull-ups for PC[4..7] which are not bonded Mind: PORTC[0:1] is set to 0x3 by lcd2_portinit()
    PORTR = 0xCB; // overwritten by ax5043_reset, ax5043_comminit()
    DIRA = 0x27; // output 0 on PA[0,1,2,5] to prevent current consumption in all DIP switch states. Other PA are inputs, PA[3,4] (LPXOSC) must have disabled digital output drivers
    DIRB = 0x03; // PB[0,1] are outputs (LCD RS, LCD RST), PB[2..7] are inputs (PB[2,3] for buttons,  PB[4..7]  unused)
    DIRC = 0x0F; // PC[0..3] are outputs (LCD SEL, LCD,SCK, LCD MOSI, LED), PC[4..7] are inputs (not bonded).
    DIRR = 0x15; // overwritten by ax5043_reset, ax5043_comminit()
#else
    PORTA = 0xE7; // pull ups except for LPXOSC pin PA[3,4]
    PORTB = 0xFD | (PINB & 0x02); //
    PORTC = 0xFF; //
    PORTR = 0x0B; //

    DIRA = 0x00; //
    DIRB = 0x0e; //  PB1 = LED; PB2 / PB3 are outputs (in case PWRAMP / ANSTSEL are used)
    DIRC = 0x00; //  PC4 = Switch
    DIRR = 0x15; //

#endif // else MINI_KIT

    axradio_setup_pincfg1();
    DPS = 0;
    IE = 0x40;
    EIE = 0x00;
    E2IE = 0x00;
    display_portinit();
    GPIOENABLE = 1; // unfreeze GPIO
#if defined(__ICC8051__)
    return coldstart;
#else
    return !coldstart; // coldstart -> return 0 -> var initialization; start from sleep -> return 1 -> no var initialization
#endif
}
#endif

int main(void)
{
    uint8_t i;
    #if defined __ARMEL__ || defined __ARMEB__

        #ifdef __AXM0
       if (!((PMU->STS & 0x02) && (PMU->MOD == 0x1)))
            coldstart = 1;  //  Coldstart
       else
            coldstart = 0;  //  Warmstart
        #endif // __AXM0
        __enable_irq();

        #if defined USE_DBGLINK
            debuglink_init_axm0();
        #endif // USE_DBGLINK
    #else
        #if !defined(SDCC) && !defined(__ICC8051__)
            _sdcc_external_startup();
        #endif

        #if defined(SDCC)
            __asm
            G$_start__stack$0$0 = __start__stack
            .globl G$_start__stack$0$0
            __endasm;
        #endif

        #ifdef USE_DBGLINK
            dbglink_init();
        #endif

         __enable_irq();
        flash_apply_calibration();
        CLKCON = 0x00;
    #endif // defined

    wtimer_init();

    if (coldstart)
    {
        #if defined __ARMEL__ || defined __ARMEB__
            #ifdef __AXM0
                delay(4000000);     //  4 second Startup delay to recover the board from hibernate mode
            #endif // __AXM0
            #ifdef __AXM0F2
                #ifdef MINI_KIT
                    axradio_setup_pincfg3();
                #endif // MINI_KIT
            #endif // __AXM0F2
        #endif // defined
        led0_off();
        led1_off();
        led2_off();
        led3_off();
        display_init();
        display_setpos(0);

        #ifdef __AXM0F2  /// TODO
            NVIC_EnableIRQ(GPIOPort2_IRQn);
            NVIC_EnableIRQ(GPIOPort0_IRQn);
            /* ENABLE RIRQ PR5 2.4 INTERRUPT */
            GPIO_PRT2->INTR_CFG |= 1<<8;
        #endif // __AXM0F2

        i = axradio_init();
        if (i != AXRADIO_ERR_NOERROR)
        {
            if (i == AXRADIO_ERR_NOCHIP)
            {
                display_writestr("No AX5043 RF\nchip found");
                #ifdef USE_DBGLINK
                    if(DBGLNKSTAT & 0x10)
                        dbglink_writestr("No AX5043 RF chip found \n");
                #endif // USE_DBGLINK
                goto terminate_error;
            }
            goto terminate_radio_error;
        }
        display_writestr("found AX5043\n");

        #ifdef USE_DBGLINK
            if (DBGLNKSTAT & 0x10)
                dbglink_writestr("found AX5043\n");
        #endif // USE_DBGLINK

        axradio_set_local_address(&localaddr);
        axradio_set_default_remote_address(&remoteaddr);

        #if RADIO_MODE == AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE == AXRADIO_MODE_WOR_ACK_RECEIVE
            // LPOSC Calibrations needs full control of the radio, so it must run while the radio is idle otherwise
            calibrate_lposc();
        #endif

        #if RADIO_MODE == MODE_SYNC_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
            display_writestr("settle LPXOSC");
            #ifdef USE_DBGLINK
                if (DBGLNKSTAT & 0x10)
                    dbglink_writestr("settle LPXOSC\n");
            #endif // USE_DBGLINK
            delay_ms(lpxosc_settlingtime);
            display_clear(0x40, 16);
            display_setpos(0x40);
        #endif  // RADIO_MODE

        #ifdef USE_DISPLAY
            display_writestr("RNG=");
            display_writenum16(axradio_get_pllrange(), 2, 0);
            {
                uint8_t x = axradio_get_pllvcoi();
                if (x & 0x80) {
                    display_writestr(" VCOI=");
                    display_writehex16(x, 2, 0);
                }
            }
            delay_ms(1000); // just to show PLL RNG
            display_clear(0, 16);
            display_clear(0x40, 16);
            display_setpos(0);
        #endif // USE_DISPLAY

        #ifdef USE_DBGLINK
            if (DBGLNKSTAT & 0x10) {
                dbglink_writestr("RNG = ");
                dbglink_writenum16(axradio_get_pllrange(), 2, 0);
                {
                    uint8_t x = axradio_get_pllvcoi();
                    if (x & 0x80) {
                        dbglink_writestr("\nVCOI = ");
                        dbglink_writehex16(x, 2, 0);
                    }
                }
                dbglink_writestr("\nSLAVE\n");
            }
        #endif // USE_DBGLINK

        #if RADIO_MODE == AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE == AXRADIO_MODE_WOR_ACK_RECEIVE
            display_writestr("SLAVE  RX WOR\n               ");
            #ifdef USE_DBGLINK
                if (DBGLNKSTAT & 0x10)
                    dbglink_writestr("SLAVE  RX WOR\n");
            #endif // USE_DBGLINK
        #elif RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
            display_writestr("SLAVE  RX SYNC\n               ");
            #ifdef USE_DBGLINK
                if (DBGLNKSTAT & 0x10)
                    dbglink_writestr("SLAVE  RX SYNC\n");
            #endif // USE_DBGLINK
        #else  // RADIO_MODE
            display_writestr("SLAVE  RX CONT\n");
            #ifdef USE_DBGLINK
                if (DBGLNKSTAT & 0x10)
                    dbglink_writestr("SLAVE  RX CONT\n");
            #endif // USE_DBGLINK
        #endif // else RADIO_MODE

        #ifdef __AXM0F2
        /* IMO calibration initialization and setup code start */
        /* Set Pin 0.0 (VTCXO pin) */
        GPIO_PRT0->DR_SET |= (1 << AXM0F2_VTCXO_PIN);

        /* IMO and ILO calibration setup */
        setup_osc_calibration(AXM0XX_HFCLK_CLOCK_FREQ, CLKSRC_RSYSCLK);
        /* IMO calibration initialization and setup code end */
        #endif /*__AXM0F2 */

        i = axradio_set_mode(RADIO_MODE);
        if (i != AXRADIO_ERR_NOERROR)
            goto terminate_radio_error;

        #ifdef DBGPKT
            AX5043_IRQMASK0 = 0x41;
            AX5043_RADIOEVENTMASK0 =0x0C; // radio state changed | RXPS changed
        #endif

    }
    else
    {
        //  Warm Start
        ax5043_commsleepexit();
        #if defined __ARMEL__ || defined __ARMEB__
        #ifdef __AXM0
            #if defined AXM0_REVA
                display_init();
                display_setpos(0);
            #endif // defined
            NVIC_EnableIRQ(GPIO_IRQn);
            GPIO_OR->INT_EN = 0x20000000;       // Enable PR5 interrupt
        #endif
        #ifdef __AXM0F2  /// TODO
            NVIC_EnableIRQ(GPIOPort2_IRQn);
            NVIC_EnableIRQ(GPIOPort0_IRQn);
            /* ENABLE RIRQ PR5 2.4 INTERRUPT */
            GPIO_PRT2->INTR_CFG |= 1<<8;

            /* IMO and ILO calibration setup */
            setup_osc_calibration(AXM0XX_HFCLK_CLOCK_FREQ, CLKSRC_RSYSCLK);

        #endif // __AXM0F2

        #else
            IE_4 = 1; // enable radio interrupt
        #endif // defined
    }
    axradio_setup_pincfg2();


    for(;;)
    {
        wtimer_runcallbacks();
        __disable_irq();
        {
            uint8_t flg = WTFLAG_CANSTANDBY;
            #ifdef MCU_SLEEP
                if (axradio_cansleep()
                #ifdef USE_DBGLINK
                    && dbglink_txidle()
                #endif // USE_DBGLINK
                    && display_txidle())
                        #if defined __ARMEL__ || defined __ARMEB__
                    		#ifdef __AXM0
                                #ifndef MINI_KIT
                                    flg |= WTFLAG_CANSLEEP;
                                #endif // MINI_KIT
                            #endif // __AXM0

                            #ifdef __AXM0F2
                                flg |= WTFLAG_CANSLEEP;
                            #endif // __AXM0F2
                            #else
                                flg |= WTFLAG_CANSLEEP;
                        #endif
            #endif // MCU_SLEEP

            wtimer_idle(flg);
        }
        __enable_irq();

    }
    terminate_radio_error:
        display_radio_error(i);
        #ifdef USE_DBGLINK
            dbglink_display_radio_error(i);
        #endif // USE_DBGLINK
    terminate_error:
        for (;;)
        {
            wtimer_runcallbacks();
            {
                uint8_t flg = WTFLAG_CANSTANDBY;
                #ifdef MCU_SLEEP
                    if (axradio_cansleep()
                        #ifdef USE_DBGLINK
                            && dbglink_txidle()
                        #endif // USE_DBGLINK
                            && display_txidle())
                        #if defined __ARMEL__ || defined __ARMEB__
                            #ifdef __AXM0
                                #ifndef MINI_KIT
                                    flg |= WTFLAG_CANSLEEP;
                                #endif // MINI_KIT
                            #endif // __AXM0

                            #ifdef __AXM0F2
                                flg |= WTFLAG_CANSLEEP;
                            #endif // __AXM0F2

                            #else
                                flg |= WTFLAG_CANSLEEP;
                        #endif
                #endif // MCU_SLEEP

                wtimer_idle(flg);
            }
        }
}



