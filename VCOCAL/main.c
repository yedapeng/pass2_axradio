/**
*************************************************************************************************************
* @file TESTS\main.c
* @brief Code for VCOCAL module, for calibration of the VCOI.*
* @internal
* @author   Thomas Sailer, Janani Chellappan, Srinivasan Tamilarasan
* $Rev: $
* $Date: $
*************************************************************************************************************
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
#include <axm0f143.h>
#include "axm0_config.h"
#include "axm0_xbar.h"
#include "axm0.h"
#else
#include <ax8052f143.h>
#endif
#include <libmftypes.h>
#include <libmfradio.h>
#include <libmfflash.h>
#include <libmfwtimer.h>
#include <libmfdbglink.h>

#ifdef MINI_KIT
#include "../COMMON/libminidvkled.h"
#else
#include <libdvk2leds.h>
#endif // MINI_KIT

#if !(defined __ARMEL__ || defined __ARMEB__)
#include <stdlib.h>
#endif

#include "../COMMON/misc.h"
#include "../COMMON/configcommon.h"
#include "../COMMON/easyax5043.h"

#if defined(SDCC)
extern uint8_t _start__stack[];
#endif

uint8_t __data coldstart = 1; // caution: initialization with 1 is necessary! Variables are initialized upon _sdcc_external_startup returning 0 -> the coldstart value returned from _sdcc_external startup does not survive in the coldstart case
volatile axradio_trxstate_t __data axradio_trxstate = trxstate_off;
uint16_t __xdata vtune[64];
uint8_t __data range[2];

extern const uint32_t __code freqinc;
extern const uint32_t __code freqincmhz;
extern const uint32_t __code freqstart;
extern const uint32_t __code freqend;
uint32_t __data freq;

#define MEMIO

#if defined(SDCC)
    static __xdata uint8_t __at(0x605) number_lines;
    static __xdata uint8_t __at(0x606) lines[50][133];
#elif defined __CX51__ || defined __C51__
    static uint8_t xdata number_lines _at_ 0x605;
    static uint8_t xdata lines[50][133] _at_ 0x606;
#elif defined __ICC8051__
    static __xdata __no_init uint8_t number_lines @ 0x605;
    static __xdata __no_init uint8_t lines[50][133] @ 0x606;
#elif defined __ARMEL__ || defined __ARMEB_
//    static uint8_t __xdata number_lines __attribute__((section(".ARM.__at_0x20000600")));
//    static uint8_t __xdata lines[50][100] __attribute__((section(".ARM.__at_0x20000604")));
    static uint8_t __xdata number_lines ;
    static uint8_t __xdata lines[50][100];
#endif

// Debug Link Configuration
#if defined __ARMEL__ || defined __ARMEB_
    const bool txondemand_mode = 0;
    #define DBGLINK_UART 0              // Mandatory
    #define DBGLNKSTAT 0x10

    void debuglink_init_axm0(void)
    {
        axm0_xbar_dbglink_config(UART0_TX, UART0_RX_2, AXM0_XBAR_PIN_NOT_REQUIRED, AXM0_XBAR_PIN_NOT_REQUIRED); // tx_pin, rx_pin, UARTx_CLK pin, TxOUT pin
        dbglink_timer0_baud(0, 9600, AXM0_FREQ_XOSC_20M);   // HS OSC, Baud rate, Clock
        dbglink_init(0, 8, 1);                              // TImer No., Word length, Stop bit
    }
#endif // defined

#if defined __ARMEL__ || defined __ARMEB__
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
#endif // defined
static void pwrmgmt_irq(void) __interrupt(INT_POWERMGMT)
{
    #if defined __ARMEL__ || defined __ARMEB__

    #else
        uint8_t pc = PCON;
        if (!(pc & 0x80))
            return;
        GPIOENABLE = 0;
        IE = EIE = E2IE = 0;
        for (;;)
            PCON |= 0x01;
    #endif // defined
}

#if defined SDCC
void axradio_isr(void) __interrupt INT_RADIO
#elif defined __CX51__ || defined __C51__
__reentrantb void axradio_isr(void) interrupt INT_RADIO
#elif defined __ICC8051__
#pragma vector=0x23
__interrupt void axradio_isr(void)
#elif defined __ARMEL__ || defined __ARMEB__
void RADIO_IRQ(void)
#else
#error "Compiler unsupported"
#endif
{
    switch (axradio_trxstate)
    {
        default:
            AX5043_IRQMASK1 = 0x00;
            AX5043_IRQMASK0 = 0x00;
            break;

        case trxstate_wait_xtal:
            AX5043_IRQMASK1 = 0x00; // otherwise crystal ready will fire all over again
            axradio_trxstate = trxstate_xtal_ready;
            break;

        case trxstate_pll_ranging:
            AX5043_IRQMASK1 = 0x00; // otherwise autoranging done will fire all over again
            axradio_trxstate = trxstate_pll_ranging_done;
            break;

        case trxstate_pll_settling:
            AX5043_RADIOEVENTMASK0 = 0x00;
            axradio_trxstate = trxstate_pll_settled;
            break;
    } // end switch(axradio_trxstate)
} //end radio_isr

#if defined __ARMEL__ || defined __ARMEB__
void axradio_wait_for_xtal(void)
{
    uint32_t old_primask = 0;
    old_primask = __get_PRIMASK();  //  Disable all Interrupts & only wakeup allowed
    __set_PRIMASK(1);

    axradio_trxstate = trxstate_wait_xtal;
    AX5043_IRQMASK1 |= 0x01; // enable xtal ready interrupt
    for(;;)
    {
        __disable_irq();
        if (axradio_trxstate == trxstate_xtal_ready)
            break;
        wtimer_idle(WTFLAG_CANSTANDBY);
        __enable_irq();
        wtimer_runcallbacks();
    }
    __set_PRIMASK(old_primask);     //  Restore all Interrupts
}
#else
void axradio_wait_for_xtal(void)
{
    uint8_t __autodata iesave = IE & 0x80;
    EA = 0;
    axradio_trxstate = trxstate_wait_xtal;
    AX5043_IRQMASK1 |= 0x01; // enable xtal ready interrupt
    for(;;)
    {
        EA = 0;
        if (axradio_trxstate == trxstate_xtal_ready)
            break;
        wtimer_idle(WTFLAG_CANSTANDBY);
        EA = 1;
        wtimer_runcallbacks();
    }
    IE |= iesave;
}
#endif

static int16_t axradio_tunevoltage(void)
{
    int16_t __autodata r = 0;
    uint8_t __autodata cnt = 64;
    do {
        AX5043_GPADCCTRL = 0x84;
        do {} while (AX5043_GPADCCTRL & 0x80);
    } while (--cnt);
    cnt = 32;
    do {
        AX5043_GPADCCTRL = 0x84;
        do {} while (AX5043_GPADCCTRL & 0x80);
        {
            int16_t x = AX5043_GPADC13VALUE1 & 0x03;
            x <<= 8;
            x |= AX5043_GPADC13VALUE0;
            r += x;
        }
    } while (--cnt);
    return r;
}

#if defined __ARMEL__ || defined __ARMEB__
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
    CMU->HS_OSC_20M_FREQ_TUNE = 0x00000A40;     // Configure the tune value to board specific
    CMU->LP_OSC_FREQ_TUNE = 0x00000220;         // Configure the tune value to board specific
}


/* Enable Peripheral Clocks */
static inline void pclk_En()
{
     /* Enable XBAR Clock */
     CMU->PCLK_CFG_b.XBAR_EN = 1;
     /* Enable GPIO Clock */
     CMU->PCLK_CFG_b.GPIO_EN = 1;
     /* Enable PMU Clock */
     CMU->PCLK_CFG_b.PMU_EN = 1;
     CMU->PCLK_CFG = 0xFFFFFFFF;     //  Radio functionality need
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

     lcd2_portinit();
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

     GPIO_OR->DATA_OUT = (1<<24);
     GPIO_AND->DATA_OUT = ~((1<<26) | (1<<28));


     /* RF SIGNALS from RF Module to DVK at PR1(GPIO25), PR3(GPIO27), Configure as Input and PullUp.
     Unused pins are PR6(GPIO30), PR7(GPIO31). Configure as PullUp to minimize current consumption */

     XBAR_OR->PULL_UP_CFG = ((1<<25)|(1<<27)|(1<<30)|(1<<31));
     XBAR_AND->PULL_DOWN_CFG = ~((1<<25)|(1<<27)|(1<<30)|(1<<31));

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
     wtimer0_setclksrc(CLKSRC_LPXOSC, 1);
     /* Set clock source and prescalar for wakeupTimer1 */
     wtimer1_setclksrc(CLKSRC_FRCOSC, 7);
    /* Configure Hardware Ports */
 //    portInit();

     if (((PMU->STS & 0x02) && (PMU->MOD == 0x1)))
         return 1;
     return 0;
}

#else
#if defined(__ICC8051__)
// If the code model is banked, low_level_init must be declared
// __near_func elsa a ?BRET is performed
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
    //    PORTA = 0xC0; // pull-up for PA[6,7] which are not bonded, no pull up for PA[3,4] (LPXOSC) and PA[0,1,2,5] (DIP switches to gnd, default on)
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
        PORTA = 0xFF; //
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
        GPIOENABLE = 1; // unfreeze GPIO
    #if defined(__ICC8051__)
        return coldstart;
    #else
        return !coldstart; // coldstart -> return 0 -> var initialization; start from sleep -> return 1 -> no var initialization
    #endif
}
#endif // defined

void main(void)
{
    #if defined __ARMEL__ || defined __ARMEB__
       if (!((PMU->STS & 0x02) && (PMU->MOD == 0x1)))
            coldstart = 1;  //  Coldstart
       else
            coldstart = 0;  //  Warmstart
        __enable_irq();
        debuglink_init_axm0();
    #else // defined
        #if !defined(SDCC) && !defined(__ICC8051__)
            _sdcc_external_startup();
        #endif

        #if defined(SDCC)
            __asm
            G$_start__stack$0$0 = __start__stack
            .globl G$_start__stack$0$0
            __endasm;
        #endif
        dbglink_init();

        EA = 1;
        flash_apply_calibration();
        CLKCON = 0x00;
    #endif // defined

    wtimer_init();
    number_lines = 0;

    if (coldstart)
    {
        #if defined __ARMEL__ || defined __ARMEB__
            delay(4000000);     //  4 second Startup delay to recover the board from hibernate mode
        #endif // defined
        led0_off();
        led1_off();
        led2_off();
        led3_off();

        dbglink_tx('\n');
        #if defined __ARMEL__ || defined __ARMEB__
            GPIO_AND->INT_EN = ~0x20000000;     /* Disable PR5interrupt */
        #else
            IE_4 = 0;
        #endif // defined
        axradio_trxstate = trxstate_off;
        if (ax5043_reset())
        {
            dbglink_writestr("ERR: AX5043 not found\n");
            goto terminate_error;
        }
        ax5043_set_registers();
        AX5043_PINFUNCIRQ = 0x03; // use as IRQ pin
        ax5043_set_registers_tx();
        AX5043_PLLVCODIV &= 0xFB; // disable RFDIV
        AX5043_MODULATION = 0x08;
        AX5043_FSKDEV2 = 0x00;
        AX5043_FSKDEV1 = 0x00;
        AX5043_FSKDEV0 = 0x00;
        {
            uint8_t x = AX5043_0xF35;
            x |= 0x80;
            if (2 & (uint8_t)~x)
                ++x;
            AX5043_0xF35 = x;
        }
        #if defined __ARMEL__ || defined __ARMEB__
            __enable_irq();
            NVIC_EnableIRQ(GPIO_IRQn);          // Enable GPIO IRQ
            GPIO_OR->INT_EN = 0x20000000;       // Enable PR5 interrupt
        #else
            IE_4 = 1;
        #endif // defined
        // range all channels
        AX5043_PWRMODE = AX5043_PWRSTATE_XTAL_ON;
        axradio_wait_for_xtal();
        for (freq = freqstart; ; freq += freqinc)
        {
            uint8_t __autodata iesave;
            uint8_t __autodata rng;
            AX5043_FREQA0 = freq;
            AX5043_FREQA1 = freq >> 8;
            AX5043_FREQA2 = freq >> 16;
            AX5043_FREQA3 = freq >> 24;
            AX5043_PLLVCOI = 0x9B;
            AX5043_PLLLOOP = 0x09; // default 100kHz loop BW for ranging
            AX5043_PLLCPI = 0x08;
            #if defined __ARMEL__ || defined __ARMEB__
                uint32_t old_primask = 0;
                old_primask = __get_PRIMASK();  //  Disable all Interrupts & only wakeup allowed
                __set_PRIMASK(1);
            #else
                iesave = IE & 0x80;
                EA = 0;
            #endif // defined
            axradio_trxstate = trxstate_pll_ranging;
            AX5043_IRQMASK1 = 0x10; // enable pll autoranging done interrupt
            AX5043_PLLRANGINGA = 0x18; // init ranging process
            for (;;)
            {
                #if defined __ARMEL__ || defined __ARMEB__
                    __disable_irq();
                #else
                    EA = 0;
                #endif // defined
                if (axradio_trxstate == trxstate_pll_ranging_done)
                    break;
                wtimer_idle(WTFLAG_CANSTANDBY);
                #if defined __ARMEL__ || defined __ARMEB__
                    __enable_irq();
                    __set_PRIMASK(old_primask);     //  Restore all Interrupts
                #else
                    IE |= iesave;
                #endif // defined
                wtimer_runcallbacks();
            }
            axradio_trxstate = trxstate_off;
            AX5043_IRQMASK1 = 0x00;
            #if defined __ARMEL__ || defined __ARMEB__
                __set_PRIMASK(old_primask);     //  Restore all Interrupts
            #else
                IE |= iesave;
            #endif // defined
            if (AX5043_PLLRANGINGA & 0x20)
                goto nextfreq;
            range[0] = range[1] = AX5043_PLLRANGINGA & 0x0F;
            {
                uint8_t __autodata i;
                for (i = 0; i < 2; ++i)
                {
                    uint8_t r = range[i];
                    if (i)
                    {
                        if (r >= 0x0D)
                            r = 0x0F;
                        else
                            r += 2;
                    }
                    else
                    {
                        if (r <= 0x02)
                            r = 0;
                        else
                            r -= 2;
                    }
                    #if defined __ARMEL__ || defined __ARMEB__
                        uint32_t old_primask = 0;
                        old_primask = __get_PRIMASK();  //  Disable all Interrupts & only wakeup allowed
                        __set_PRIMASK(1);
                    #else
                        iesave = IE & 0x80;
                        EA = 0;
                    #endif // defined
                    axradio_trxstate = trxstate_pll_ranging;
                    AX5043_IRQMASK1 = 0x10; // enable pll autoranging done interrupt
                    AX5043_PLLRANGINGA = 0x10 | r; // init ranging process
                    for (;;)
                    {
                        #if defined __ARMEL__ || defined __ARMEB__
                            __disable_irq();
                        #else
                            EA = 0;
                        #endif // defined
                        if (axradio_trxstate == trxstate_pll_ranging_done)
                            break;
                        wtimer_idle(WTFLAG_CANSTANDBY);
                        #if defined __ARMEL__ || defined __ARMEB__
                            __set_PRIMASK(old_primask);     //  Restore all Interrupts
                        #else
                            IE |= iesave;
                        #endif // defined
                        wtimer_runcallbacks();
                    }
                    axradio_trxstate = trxstate_off;
                    AX5043_IRQMASK1 = 0x00;
                    #if defined __ARMEL__ || defined __ARMEB__
                        __set_PRIMASK(old_primask);     //  Restore all Interrupts
                    #else
                        IE |= iesave;
                    #endif // defined
                    if (AX5043_PLLRANGINGA & 0x20)
                        continue;
                    range[i] = AX5043_PLLRANGINGA & 0x0F;
                }
            }
            AX5043_PLLLOOP |= 0x04;
            for (rng = range[0]; rng <= range[1]; ++rng)
            {
                AX5043_PLLRANGINGA = rng;
                AX5043_PWRMODE = AX5043_PWRSTATE_SYNTH_TX;
                {
                    uint8_t __autodata i;
                    for (i = 0x40; i != 0;) {
                        --i;
                        AX5043_PLLVCOI = 0x80 | i;
                        AX5043_PLLRANGINGA; // clear PLL lock loss
                        vtune[i] = axradio_tunevoltage();
                    }
                }
                AX5043_PWRMODE = AX5043_PWRSTATE_XTAL_ON;
#ifdef MEMIO
                if (number_lines >= 50) {
                    dbglink_writestr("#\n");
                    do {
                        // wait for the host to fetch the results
                    } while (number_lines);
                }
                {
                    uint8_t __autodata i;
                    uint8_t __xdata *__autodata p = lines[number_lines];
                    ++number_lines;
                    *p++ = freq;
                    *p++ = freq >> 8;
                    *p++ = freq >> 16;
                    *p++ = freq >> 24;
                    *p++ = AX5043_PLLRANGINGA;
                    for (i = 0; i != 0x40; ++i) {
                        *p++ = vtune[i];
                        *p++ = vtune[i] >> 8;
                    }
                }
#else
                dbglink_writestr("R: ");
                dbglink_writehex32(freq, 8, WRNUM_PADZERO);
                dbglink_tx(' ');
                dbglink_writehex16(AX5043_PLLRANGINGA, 2, WRNUM_PADZERO);
                {
                    uint8_t __autodata i;
                    for (i = 0; i != 0x40; ++i) {
                        dbglink_tx(' ');
                        dbglink_writehex16(vtune[i], 4, WRNUM_PADZERO);
                    }
                }
                dbglink_tx('\n');
#endif
            }
nextfreq:
            if (freq == freqend)
                break;
        }
#ifdef MEMIO
        if (number_lines)
            dbglink_writestr("#\n");
#endif
        dbglink_writestr("END ");
        dbglink_writehex32(freqincmhz, 8, WRNUM_PADZERO);
        dbglink_tx(' ');
        dbglink_writehex32(freqstart, 8, WRNUM_PADZERO);
        dbglink_tx(' ');
        dbglink_writehex32(freqend, 8, WRNUM_PADZERO);
        dbglink_tx(' ');
        dbglink_writehex32(freqinc, 8, WRNUM_PADZERO);
        dbglink_writestr("\n\n");
#if 0
        // VCOI Calibration
        ax5043_set_registers_tx();
        AX5043_PLLLOOP |= 0x04;
        AX5043_PWRMODE = AX5043_PWRSTATE_SYNTH_TX;
        {
            uint8_t __autodata vcoisave = AX5043_PLLVCOI;
            for (i = 39; i < axradio_phy_nrchannels; ++i) {
                if (axradio_phy_chanpllrng_tx[i] & 0x20)
                    continue;
                AX5043_PLLRANGINGA = axradio_phy_chanpllrng_tx[i] & 0x0F;
                {
                    uint32_t __autodata f = axradio_phy_chanfreq[i];
                    AX5043_FREQA0 = f;
                    AX5043_FREQA1 = f >> 8;
                    AX5043_FREQA2 = f >> 16;
                    AX5043_FREQA3 = f >> 24;
                }
                if (axradio_phy_chanvcoiinit[0]) {
                    uint8_t x = axradio_phy_chanvcoiinit[i];
                    if (!(axradio_phy_chanpllrnginit[0] & 0xF0))
                        x += (axradio_phy_chanpllrng_tx[i] & 0x0F) - (axradio_phy_chanpllrnginit[i] & 0x0F);
                    axradio_phy_chanvcoi_tx[i] = axradio_adjustvcoi(x);
                } else {
                    axradio_phy_chanvcoi_tx[i] = axradio_calvcoi();
                }
    #if 0
                dbglink_writestr("\nVCOI Calibration Channel ");
                dbglink_writenum16(i, 0, 0);
                dbglink_writestr(" result ");
                dbglink_writehex16(axradio_phy_chanvcoi_tx[i], 2, WRNUM_PADZERO);
                dbglink_tx('\n');
                {
                    uint8_t i;
                    for (i = 0; i != 64; ++i) {
                        dbglink_writenum16(i, 2, 0);
                        dbglink_tx(' ');
                        dbglink_writenum16(axradio_rxbuffer[2*i] | (((uint16_t)axradio_rxbuffer[2*i+1])<<8), 6, WRNUM_SIGNED);
                        dbglink_tx(' ');
                        dbglink_writehex16(axradio_rxbuffer[2*i] | (((uint16_t)axradio_rxbuffer[2*i+1])<<8), 6, WRNUM_PADZERO);
                        dbglink_tx('\n');
                    }
                }
#endif
            }
            AX5043_PLLVCOI = vcoisave;
        }
        AX5043_PWRMODE = AX5043_PWRSTATE_POWERDOWN;
#if 1
        for (i = 0; i < axradio_phy_nrchannels; ++i) {
            uint8_t chg = (!(axradio_phy_chanpllrnginit[0] & 0xF0) && axradio_phy_chanpllrnginit[i] == axradio_phy_chanpllrng_tx[i])
                || (axradio_phy_chanvcoiinit[0] && !((axradio_phy_chanvcoiinit[i] ^ axradio_phy_chanvcoi_tx[i]) & 0x7F));
            if (1 && chg)
                continue;
            dbglink_writestr("CH ");
            dbglink_writenum16(i, 0, 0);
            dbglink_writestr(" RNG ");
            if (!(axradio_phy_chanpllrnginit[0] & 0xF0)) {
                dbglink_writenum16(axradio_phy_chanpllrnginit[i], 0, 0);
                dbglink_tx('/');
            }
            dbglink_writenum16(axradio_phy_chanpllrng_tx[i], 0, 0);
            dbglink_writestr(" VCOI ");
            if (axradio_phy_chanvcoiinit[0]) {
                dbglink_writenum16(axradio_phy_chanvcoiinit[i] & 0x7F, 0, 0);
                dbglink_tx('/');
            }
            dbglink_writenum16(axradio_phy_chanvcoi_tx[i] & 0x7F, 0, 0);
            if (chg)
                dbglink_writestr(" *");
            dbglink_tx('\n');
        }
#endif
        AX5043_PLLRANGINGA = axradio_phy_chanpllrng_rx[0] & 0x0F;
        {
            uint32_t __autodata f = axradio_phy_chanfreq[0];
            AX5043_FREQA0 = f;
            AX5043_FREQA1 = f >> 8;
            AX5043_FREQA2 = f >> 16;
            AX5043_FREQA3 = f >> 24;
        }
        AX5043_PLLLOOP = pllloop_save; // restore loop settings (works if they came from the common section, unimportant if the came from the rx / tx section)
        AX5043_PLLCPI = pllcpi_save;

        axradio_mode = AXRADIO_MODE_OFF;
        for (i = 0; i < axradio_phy_nrchannels; ++i)
            if ((axradio_phy_chanpllrng_rx[i] | axradio_phy_chanpllrng_tx[i]) & 0x20)
                return AXRADIO_ERR_RANGING;
        return AXRADIO_ERR_NOERROR;
#endif
    }
    axradio_setup_pincfg2();


terminate_error:
    for(;;) {
        wtimer_runcallbacks();
        #if defined __ARMEL__ || defined __ARMEB__
            __disable_irq();
        #else
            EA = 0;
        #endif // defined
        {
            uint8_t flg = WTFLAG_CANSTANDBY;
        #ifdef MCU_SLEEP
            if (dbglink_txidle())
            #if defined __ARMEL__ || defined __ARMEB__
                #ifndef MINI_KIT
                    flg |= WTFLAG_CANSLEEP;
                #endif // MINI_KIT
            #else
                    flg |= WTFLAG_CANSLEEP;
            #endif
        #endif // MCU_SLEEP
            wtimer_idle(flg);
        }
        #if defined __ARMEL__ || defined __ARMEB__
            __enable_irq();
        #else
            EA = 1;
        #endif // defined
    }
}



