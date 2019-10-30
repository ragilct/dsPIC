/*******************************************************************************
* © 2005 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,  
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF 
* STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR 
* ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, 
* DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE, 
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE
* DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S 
* TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE, SHALL NOT 
* EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO HAVE THIS 
* CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
*
*******************************************************************************/

#include <p33fj16gs504.h>  
#include "offline_ups.h"


/* Define Function prototypes */
extern void delay_ms (unsigned int delay);

void initClock(void)
{
  	/* Configure Oscillator to operate the device at 40 MIPS
	   Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
 	   Fosc= 7.37*(43)/(2*2)=80Mhz for Fosc, Fcy = 40Mhz */

	/* Configure PLL prescaler, PLL postscaler, PLL divisor */
	/* When using FRC or 7.37MHz crystal */
	PLLFBD = 41; 			                /* M = PLLFBD + 2 */
	CLKDIVbits.PLLPOST = 0;                 /* N1 = 2 */
	CLKDIVbits.PLLPRE = 0;                  /* N2 = 2 */

    /* Change oscillator to FRC + PLL */
    __builtin_write_OSCCONH(0x01);			/* New Oscillator FRC w/ PLL */
    __builtin_write_OSCCONL(0x01);  		/* Enable Clock Switch */
    
    while(OSCCONbits.COSC != 0b001);		/* Wait for new Oscillator to
                                               become FRC w/ PLL */    
	while(OSCCONbits.LOCK != 1);			/* Wait for Pll to Lock */

	/* Now setup the ADC and PWM clock for 120MHz
	   ((FRC * 16) / APSTSCLR ) = (7.37 * 16) / 1 = ~ 120MHz*/
	
	ACLKCONbits.FRCSEL = 1;					/* FRC provides input for
	                                           Auxiliary PLL (x16) */
	ACLKCONbits.SELACLK = 1;				/* Auxiliary Ocillator provides
	                                           clock source for PWM & ADC */
	ACLKCONbits.APSTSCLR = 7;				/* Divide Auxiliary clock by 1 */
	ACLKCONbits.ENAPLL = 1;					/* Enable Auxiliary PLL */
	
	while(ACLKCONbits.APLLCK != 1);			/* Wait for Aux. PLL to Lock */
    
}
void initPWM(void)
{
    /* PWM setup for Full-Bridge Inverter section */
    PWMCON1bits.ITB = 1;                    /* Center aligned mode needs
                                               Independent time base to be
                                               used */
    PWMCON2bits.ITB = 1;                    /* Center aligned mode needs
                                               Independent time base to be 
                                               used */
    
    PWMCON1bits.DTC = 0;                    /* Positive dead-time enabled */
    PWMCON2bits.DTC = 0;                    /* Positive dead-time enabled */
    
    PWMCON1bits.CAM = 1;                    /* Center-aligned mode used for
                                               3-level control of inverter */    
    PWMCON2bits.CAM = 1;                    /* Center-aligned mode used for
                                               3-level control of inverter */    
    
    IOCON1bits.PENH = 1;                    /* PWM1H pin is controlled by PWM
                                               module */
    IOCON1bits.PENL = 1;                    /* PWM1L pin is controlled by PWM
                                               module */
    
    IOCON2bits.PENH = 1;                    /* PWM2H pin is controlled by PWM 
                                               module */
    IOCON2bits.PENL = 1;                    /* PWM2L pin is controlled by PWM
                                               module */

    IOCON1bits.PMOD = 0;                    /* Complementary PWM mode */
    IOCON2bits.PMOD = 0;                    /* Complementary PWM mode */
    
    IOCON1bits.OVRDAT = 0;                  /* Inverter IGBTs will be turned OFF
                                               during override */
    IOCON2bits.OVRDAT = 0;                  /* Inverter IGBTs will be turned OFF
                                               during override */
        
    IOCON1bits.OVRENH = 1;                  /* Keep Inverter IGBTs OFF during 
                                               SYSTEM_STARTUP */
    IOCON1bits.OVRENL = 1;                  /* Keep Inverter IGBTs OFF during
                                               SYSTEM_STARTUP */
    
    IOCON2bits.OVRENH = 1;                  /* Keep Inverter IGBTs OFF during
                                               SYSTEM_STARTUP */
    IOCON2bits.OVRENL = 1;                  /* Keep Inverter IGBTs OFF during 
                                               SYSTEM_STARTUP */
    
    ALTDTR1 = INVERTER_ALTDTR_VALUE;
                                            /* Dead-time setup to 416ns (based
                                               on IGBT data sheet). When using
                                               Center-aligned mode, only ALTDTR1
                                               should be used because DTR1 will 
                                               produce a notch at the center of 
                                               the high-pulse.
                                               Dead-time = ALTDTR1<15:0>*1.04ns
                                                         = 416ns */
    ALTDTR2 = INVERTER_ALTDTR_VALUE;
                                            /* Dead-time setup to 416ns (based
                                               on IGBT data sheet). When using
                                               Center-aligned mode, only ALTDTR2
                                               should be used because DTR2 will 
                                               produce a notch at the center of 
                                               the high-pulse.
                                               Dead-time = ALTDTR2<15:0>*1.04ns
                                                         = 416ns */
                                         
    PHASE1 = INVERTER_PERIOD_VALUE;
                                            /* PWM period of about 19.53us for
                                               Inverter section. SineTable has
                                               512 samples, so to generate 50Hz
                                               sine wave, we choose inverter
                                               switching frequency equal to
                                               51.2kHz. This means every 2nd
                                               interrupt (at will be used to
                                               update the sine reference and
                                               run control loop.
                                               When in center aligned mode
                                               PWM period = 2 * PHASE1 * 1.04ns
                                                          = 19.53us */
                                          
    PHASE2 = INVERTER_PERIOD_VALUE;
                                            /* PWM period of about 19.53us for
                                               Inverter section. SineTable has
                                               512 samples, so to generate 50Hz
                                               sine wave, we choose inverter
                                               switching frequency equal to
                                               51.2kHz. This means every 2nd
                                               interrupt (at will be used to
                                               update the sine reference and
                                               run control loop.
                                               When in center aligned mode
                                               PWM period = 2 * PHASE2 * 1.04ns
                                                          = 19.53us */
                       
    PDC1 = INVERTER_PDC_NOMINAL_VALUE;
                                            /* Nominal duty cycle setup to 
                                               equalize PWMxH and PWMxL ON
                                               times. As ALTDTRx = 500, the
                                               effective maximum duty cycle is
                                               PHASEx-ALTDTRx. ALTDTRx
                                               only affects the PWMxL pulse. So
                                               PWMxH pulse should be reduced to
                                               make equal to PWMxL pulse. As 
                                               PDCx directly gives pulse-width 
                                               of PWMxH, make 
                                               PDCx = (PHASEx-ALTDTRx)/2 */
                                    
    PDC2 = INVERTER_PDC_NOMINAL_VALUE;
                                            /* Nominal duty cycle setup to 
                                               equalize PWMxH and PWMxL ON
                                               times. As ALTDTRx = 500, the
                                               effective maximum duty cycle is
                                               PHASEx-ALTDTRx. ALTDTRx
                                               only affects the PWMxL pulse. So
                                               PWMxH pulse should be reduced to
                                               make equal to PWMxL pulse. As 
                                               PDCx directly gives pulse-width 
                                               of PWMxH, make 
                                               PDCx = (PHASEx-ALTDTRx)/2 */
                               
    
    TRIG1 = INVERTER_PDC_NOMINAL_VALUE;
                                            /* PWM1 trigger used to trigger ADC
                                               Pair 0 conversion for Inverter
                                               Control loop. Trigger at middle 
                                               of effective duty cycle applied 
                                               to output filter.
                                               Effective duty cycle is obtained 
                                               from the difference of PWM1H and
                                               PWM2H signals. The mid-point of
                                               this effective duty cycle is 
                                               located at the nominal duty cycle
                                               (50%) of each PWM generator.
                                               So INVERTER_PDC_NOMINAL_VALUE is
                                               selected for the ADC trigger. */
                                            /* PWM 1 trigger also used to 
                                               trigger ADC Pair 5 conversion 
                                               for AC Mains sensing. Mains 
                                               sensing interrupt will be queued
                                               due to natural priority of
                                               ADC pairs */

                               
    STRIG1 = INVERTER_PDC_NOMINAL_VALUE + 8;    
                                            /* Work-around for ADC trigger
                                               erratum on A2 silicon */

    TRGCON1bits.DTM = 1;                    /* Work-around for ADC trigger
                                               erratum on A2 silicon */
                               
    TRIG2 = INVERTER_PDC_NOMINAL_VALUE;
                                            /* PWM 2 trigger used to trigger ADC
                                               Pair 2 conversion for Battery 
                                               charging current and battery 
                                               voltage sense.
                                               Trigger at middle of effective 
                                               duty cycle applied to output 
                                               filter. Effective duty cycle is 
                                               obtained from the difference of 
                                               PWM1H and PWM2H signals. The 
                                               mid-point of this effective duty 
                                               cycle is located at the nominal
                                               duty cycle (50%) of each PWM 
                                               generator. 
                                               So INVERTER_PDC_NOMINAL_VALUE is 
                                               selected for the ADC trigger. */
    
    STRIG2 = INVERTER_PDC_NOMINAL_VALUE + 8;
                                            /* Work-around for ADC trigger
                                               erratum on A2 silicon */

    TRGCON2bits.DTM = 1;                    /* Work-around for ADC trigger
                                               erratum on A2 silicon */
                                            
    TRGCON1bits.TRGDIV = 1;                 /* Interrupt every center-aligned
                                               period. As effective period of 
                                               center-aligned PWM signal is 
                                               twice of individual PWM period, 
                                               configure to trigger once every
                    						   two individual PWM periods. So 
                    						   select TRGDIV = 1. */
                            
    TRGCON2bits.TRGDIV = 15;                /* Interrupt every 8 center-aligned
                                               periods. As effective period of 
                                               center-aligned PWM signal is 
                                               twice of individual PWM period, 
                                               configure to trigger once every
                    						   two individual PWM periods. So 
                    						   select TRGDIV = 15. */
    
    TRGCON1bits.TRGSTRT = 0;                /* Wait 0 PWM cycles before 
                                               generating first PWM trigger */

    TRGCON2bits.TRGSTRT = 0;                /* Wait 0 PWM cycles before
                                               generating first PWM trigger */

    FCLCON1 = 0x0003;                       /* Fault mode disabled.
                                               Due to errata on A0 and A1
                                               silicon word-writes must be
                                               used for FCLCONx */

    FCLCON2 = 0x0003;                       /* Fault mode disabled.
                                               Due to errata on A0 and A1
                                               silicon word-writes must be
                                               used for FCLCONx */
    
    /* End of Inverter PWM setup */

    /* PWM Setup for Push-pull Boost Converter */
    PWMCON3bits.ITB = 1;                    /* Push-Pull boost converter 
                                               operated in Independent
                                               time-base. */

    PWMCON3bits.DTC = 2;                    /* Dead-time disabled */

    IOCON3bits.PENH = 1;                    /* PWM3H pin is controlled by PWM 
                                               module */
    IOCON3bits.PENL = 1;                    /* PWM3L pin is controlled by PWM 
                                               module */
    
    IOCON3bits.PMOD = 2;                    /* Push-pull PWM mode */
    
    IOCON3bits.OVRDAT = 0;                  /* Push-pull MOSFETs will be turned 
                                               OFF during override */
        
    IOCON3bits.OVRENH = 1;                  /* Keep push-pull MOSFETs OFF during
                                               SYSTEM_STARTUP */
    IOCON3bits.OVRENL = 1;                  /* Keep push-pull MOSFETs OFF during
                                               SYSTEM_STARTUP */

    PHASE3 = PUSH_PULL_PERIOD_VALUE;
                                            /* PWM period of about 9.765us for
                                               Push-pull Boost converter
                                               section. When in push-pull PWM
                                               mode PWM period
                                               = 2 * PTPER * 1.04ns
                                               = 9.765us */

    PDC3 = 0;                               /* Initial Duty cycle of 0 */
    
    TRIG3 = 0;                              /* PWM3 trigger used to trigger ADC
                                               Pair 1 conversion for push-pull
                                               Control loop. */
    STRIG3 = 8;                             /* Work-around for ADC trigger
                                               erratum on A2 silicon */
    TRGCON3bits.DTM = 1;                    /* Work-around for ADC trigger
                                               erratum on A2 silicon */

                               
    TRGCON3bits.TRGDIV = 3;                 /* Interrupt once every 2 push-pull
                                               periods. As effective period of
                                               push-pull PWM signal is twice of
                                               individual PWM period, configure
                                               to trigger once every four
                                               individual PWM periods. So select
                    						   TRGDIV = 3. */
                            
    TRGCON3bits.TRGSTRT = 0;                /* Wait 0 PWM cycles before
                                               generating first PWM trigger */

    FCLCON3 = 0x0103;                       /* FLT1 pin is CL source, 
                                               CL mode enabled. 
                                               Fault mode disabled. 
                                               Due to errata on A0 and A1 
                                               silicon word-writes must be used
                                               for FCLCONx */

    /* End of PWM Setup for Push-pull Boost Converter */
    
    /* PWM Setup for Battery Charger */
    RPOR2bits.RP4R = 45;                    /* RP4 configured as PWM4L output */
    
    PWMCON4bits.DTC = 2;	                /* Dead Time disabled because of 
                                               independent output mode */
        
    PWMCON4bits.ITB = 1;	                /* Independent time base */
    
    IOCON4bits.PENL = 1;                    /* PWM4L output controlled by PWM */
    IOCON4bits.PMOD = 3;                    /* Select Independent output PWM
                                               mode */
    
    IOCON4bits.OVRENL = 1;                  /* Override PWM output at 
                                               SYSTEM_STARTUP */

    FCLCON4 = 0x0003;                         /* Faults Disabled.
                                                 Due to errata on A0 and A1
                                                 silicon word-writes must be
                                                 used for FCLCONx */


    SPHASE4 = BATTERY_CHARGER_PERIOD_VALUE;	/* Setup Battery charger control
                                               at 25kHz frequency */
    
    TRIG4 = 0;                              /* Trigger at start of PWM cycle */			

    STRIG4 = 8;                             /* Work-around for ADC trigger
                                               erratum on A2 silicon */
    //TRGCON4bits.DTM = 1;                    /* Work-around for ADC trigger
    //                                           erratum on A2 silicon */
                                               

    TRGCON4bits.TRGSTRT = 0;			    /* Wait 0 PWM cycles before 
                                               generating first trigger */
    TRGCON4bits.TRGDIV = 15;			    /* Battery Charger control does not
                                               need to be fast, so choose 
                                               TRGDIV = 15 to produce triggers
                                               once every 16 PWM cycles. This 
                                               is the slowest triggering option
                                               when using PWM triggers */
                                                                                    
    SDC4 = MIN_CHARGING_CURRENT;            /* Battery Charger needs some
                                               minimum duty cycle to operate
                                               properly */
    /* End of PWM Setup for Battery Charger */
    
    /* PWM Setup for Fan Control */
    RPOR1bits.RP3R = 44;                    /* RP3 configured as PWM4H output */
	
    IOCON4bits.PENH = 1;                    /* PWM4H output controlled by PWM */
    
    PHASE4 = FAN_CONTROL_PERIOD_VALUE;      /* Fan Control set for 25kHz */
    
    PDC4 = 0;								/* Initially Fan is at zero speed */
    /* End of PWM Setup for Fan Control */
}

void initADC(void)
{
    /* Common ADC configuration */
    ADCONbits.SLOWCLK = 1;          /* Requirement from ADC Errata */
   	ADCONbits.FORM = 0;             /* Output in Integer Format */
	ADCONbits.EIE = 0;              /* Disable Early Interrupt */
	ADCONbits.ORDER = 0;            /* Even channel first */
	ADCONbits.SEQSAMP = 0;          /* Simultaneus Sampling Enabled */
	ADCONbits.ADCS = 5;             /* Clock Divider is set up for Fadc/6 */
									/* TAD=41.66ns*/
									/* For simultaneus sampling total
									   conversion time for one pair is
									   0.625us */
	
	ADSTAT = 0;                     /* Clear the ADSTAT register */
	
	ADPCFG = 0x0400;                /* All except AN10 are analog inputs */
   
    /* ADC configuration for Full-Bridge Inverter */
    ADPCFGbits.PCFG0 = 0;           /* AN0 is inverter current sense */
    ADPCFGbits.PCFG1 = 0;           /* AN1 is inverter voltage sense */
    
    ADCPC0bits.TRGSRC0 = 4;		    /* AN0 and AN1 triggered by PWM1 */
	ADCPC0bits.IRQEN0 = 0;			/* Global ADC interrupt not requested */
	
	IPC27bits.ADCP0IP = 6;          /* Set ADC Interrupt Priority */
	IFS6bits.ADCP0IF = 0;	    	/* Clear AD Interrupt Flag */
	IEC6bits.ADCP0IE = 1;           /* Enable the ADC Interrupt at start. */
	/* End of ADC configuration for Full-Bridge Inverter */
	
	/* ADC configuration for Push-Pull Boost Converter */
    ADPCFGbits.PCFG2 = 0;           /* AN2 is push-pull current sense */
    ADPCFGbits.PCFG3 = 0;           /* AN3 is DC Link voltage sense */
    
    ADCPC0bits.TRGSRC1 = 6;		    /* AN0 and AN1 triggered by PWM3 */
	ADCPC0bits.IRQEN1 = 0;			/* Global ADC interrupt not requested */
	
	IPC27bits.ADCP1IP = 6;          /* Set ADC Interrupt Priority */
	IFS6bits.ADCP1IF = 0;	    	/* Clear AD Interrupt Flag */
	IEC6bits.ADCP1IE = 1;           /* Enable the ADC Interrupt at start.
	                                   DC Link sensing is required at all
	                                   times */
	/* End of ADC configuration for Push-Pull Boost Converter */
	
	/* ADC configuration for AC Mains sensing */
    ADPCFGbits.PCFG11 = 0;          /* AN11 is AC Mains voltage sense */
    
    ADCPC2bits.TRGSRC5 = 4;         /* AN11 triggered by PWM1 */
    
    ADCPC2bits.IRQEN5 = 0;			/* Global ADC interrupt not requested */
	
	IPC28bits.ADCP5IP = 5;          /* Set ADC Interrupt Priority */
	IFS7bits.ADCP5IF = 0;	    	/* Clear AD Interrupt Flag */
	IEC7bits.ADCP5IE = 1;		    /* Enable the ADC Interrupt. Mains sensing
	                                   is performed at all times */
	/* End of ADC configuration for AC Mains sensing */
	
	/* ADC configuration for Battery Voltage and Current sensing */
	ADPCFGbits.PCFG4 = 0;           /* AN4 is Battery charging current sense */
    ADPCFGbits.PCFG5 = 0;           /* AN5 is Battery voltage sense */
    
    ADCPC1bits.TRGSRC2 = 17;        /* AN4 and AN5 triggered by PWM4 secondary 
                                       trigger */
    ADCPC1bits.IRQEN2 = 0;			/* Global ADC interrupt not requested */
	
	IPC28bits.ADCP2IP = 4;          /* Set ADC Interrupt Priority */
	IFS7bits.ADCP2IF = 0;	    	/* Clear AD Interrupt Flag */
	IEC7bits.ADCP2IE = 1;		    /* Enable the ADC Interrupt. Battery
	                                   current sensing required only during
	                                   BATTERY_CHARGER_MODE. Battery voltage
	                                   sense is required at all times */
	/* End of ADC configuration for Battery Voltage and Current sensing */	
}

void initComparator(void)
{
    RPOR16bits.RP32R = 40;      /* Comparator 2 output is mapped to RP32
                                   (virtual pin) */
    RPINR29bits.FLT1R = 32;     /* FLT1 pin is mapped to RP32 (virtual pin) */
    
    CMPCON2 = 0x0001;           /* Select CMP2A, Internal reference,
                                   High range (AVdd/2), polarity active-high
                                   Due to errata on A0 and A1 silicon 
                                   word-writes must be used for CMPCONx */

    CMPDAC2 = 0x03FF;           /* 1.65V */
    
    CMPCON2 = 0x8001;           /* Enable comparator 2 
                                   Due to errata on A0 and A1 silicon 
                                   word-writes must be used for CMPCONx */
    
}

void initInverterDriver(void)
{
    TRISBbits.TRISB7 = 0;       /* RB7 is FAULT_CLR output */
    TRISCbits.TRISC8 = 1;       /* RC8 is /SYS_FLT input */
    RPINR0bits.INT1R = 29;      /* RP29 is /FAULT_SD input. This signal
                                   is also used for external interrupt INT1 */
    INTCON2bits.INT1EP = 1;     /* Interrupt on negative edge */
    
    LATBbits.LATB7 = 1;         /* Send FAULT_CLR = 1 to clear all
                                   existing fault conditions before
                                   power up.
                                   (FAULT_CLR is active-high) */
    delay_ms(1);                /* 1ms delay to ensure all faults
                                   are cleared */
    
//    LATBbits.LATB7 = 0;         /* Make FAULT_CLR = 0 so that future
//                                   faults can be detected.
//                                   (FAULT_CLR is active-high) */

    IPC5bits.INT1IP = 6;        /* Set interrupt priority */
    IFS1bits.INT1IF = 0;        /* Clear flag for external interrupt INT1 */
    IEC1bits.INT1IE = 1;        /* Enable external interrupt INT1 */
    
}

void initRelays(void)
{
    TRISCbits.TRISC10 = 0;      /* RC10 controls inverter series
                                   resistor bypass relay */
    TRISCbits.TRISC0 = 0;       /* RC0 controls AC mains relay */
    
    LATCbits.LATC10 = 0;        /* Connect inverter series resistor 
                                   at SYSTEM_STARTUP */
    LATCbits.LATC0 = 0;         /* AC mains connected to UPS output when in
                                   SYSTEM_STARTUP */
}


void initStateMachineTimer(void)
{
    T2CONbits.TCKPS = 0;        /* Prescaler of 1:1 */
    PR2 = 4000;                 /* (100us / 25ns) = 4000 */ 
    IPC1bits.T2IP = 4;          /* Set up Timer interrupt priority */
    IEC0bits.T2IE = 1;          /* Enable Timer2 interrupt */
}


void initSPI(void)
{
  	RPOR11bits.RP22R = 7;       /* RP22 configured as SDO1 output */
	RPOR10bits.RP20R = 9;       /* RP20 configured as SS1 output */
	RPOR10bits.RP21R = 8;       /* RP21 configured as SCK1 output */
		
	RPINR20bits.SDI1R = 19;     /* RP19 configured as SDI input */

    TRISCbits.TRISC4 = 0;       /* RC4 is used for SS1 output */
    LATCbits.LATC4 = 1;         /* Set SS high */
    
    SPI1CON1bits.SPRE = 4;      /* Secondary prescale 4:1 */
    SPI1CON1bits.PPRE = 0;      /* Primary prescale 64:1 */
    SPI1CON1bits.SMP = 0;       /* Data sampled at middle of output time */
    SPI1CON1bits.CKE = 0;       /* Data clocked out on rising edge */
    SPI1CON1bits.CKP = 0;       /* SPI clock idle is low */
    SPI1CON1bits.MSTEN = 1;     /* Enable Master mode */
    
    SPI1STATbits.SPIROV = 0;    /* Reset overflow error bit */
    SPI1STATbits.SPIEN = 1;     /* Enable SPI1 module */    

    LATCbits.LATC4 = 0;         /* Enable SPI reception by controller */    
}

