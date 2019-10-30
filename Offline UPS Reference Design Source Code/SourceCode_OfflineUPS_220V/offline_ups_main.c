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
#include "lcd.h"

/* Configuration bits */
_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & IOL1WAY_OFF)
_FWDT(FWDTEN_OFF)
_FPOR(FPWRT_PWR128 & BOREN_OFF)
_FICD(ICS_PGD3 & JTAGEN_OFF)

/* Define Function prototypes */
void delay_ms (unsigned int delay);
void delay_100us (void);
void refreshDisplay(void);
void fanControl(void);

extern void initClock(void);
extern void initInverterDriver(void);
extern void initRelays(void);
extern void initSPI(void);
extern void initADC(void);
extern void initPWM(void);
extern void initComparator(void);
extern void initStateMachineTimer(void);

/* Define Variables */
int timerInterruptCount = 0;

extern int systemState;
extern int batteryVoltage;
extern int errorState;
extern int inverterVoltageRMS; 
extern int inverterCurrentRMS;
extern int fCrestFactorWarning;

t_sDecimalDisplay DecimalDisplayValue_VoltageRMS;
t_sDecimalDisplay DecimalDisplayValue_CurrentRMS;
    
int main(void)
{
    DecimalDisplayValue_VoltageRMS.HundredsDivider = 0x1130;
    DecimalDisplayValue_VoltageRMS.TensDivider = 0x1B8;
    DecimalDisplayValue_VoltageRMS.OnesDivider = 0x2C;
    DecimalDisplayValue_VoltageRMS.FirstDecimalDivider = 0x04;
    
    DecimalDisplayValue_CurrentRMS.HundredsDivider = 0x280A;
    DecimalDisplayValue_CurrentRMS.TensDivider = 0x401;
    DecimalDisplayValue_CurrentRMS.OnesDivider = 0x66;
    DecimalDisplayValue_CurrentRMS.FirstDecimalDivider = 0x0A;

    DecimalDisplayValue_VoltageRMS.Point = '.';
    DecimalDisplayValue_CurrentRMS.Point = '.';

    initClock();                            /* Initialize Primary and Auxiliary
	                                           oscillators */

    initInverterDriver();                   /* Initialize inverter driver by
                                               claring faults at startup */
                                               
    initRelays();                           /* Initialize relays for system
                                               startup */

    initADC();                              /* Initialize ADC to measure:
                                               1) Inverter voltage and current
                                               2) Mains voltage
                                               3) DC Link voltage and current
                                               4) Battery Voltage and current
                                               5) Battery Temperature */

    TRISBbits.TRISB6 = 0;                   /* RB6 is used as I/O for 
                                               development */

    //TRISCbits.TRISC4 = 0;                   /* RC4 is used for LED */
    //TRISCbits.TRISC5 = 0;                   /* RC5 is used for LED */
    //TRISCbits.TRISC6 = 0;                   /* RC6 is used for LED */
    
    TRISCbits.TRISC7 = 0;                   /* RC7 is a control signal used for
                                               enabling battery charger
                                               circuit */

    ADCONbits.ADON = 1;				        /* Enable the ADC module early
                                               because ADC settling time is
                                               about 2us */

    initPWM();                              /* Initialize PWM module for
                                               inverter, push-pull boost
                                               converter, battery charger and
                                               fan control */
                                               
    //initComparator();                       /* Enable comparators for current
    //                                           limiting */
                                               
    
    initSPI();                              /* Enable SPI module to drive LCD
                                               display via PIC18F2420 */
    LCD_displayLogo(1);						/* Display Microchip logo on LCD */
    
    initStateMachineTimer();				/* Initialize Timer2 for running
    										   UPS State Machine. The State
    										   Machine is executed once every
    										   100us using the Timer2 ISR. Any
    										   state changes are handled in
    										   the timer2 ISR. 
    										   (Charger-to-Inverter and
    										   Inverter-to-Charger) */
    
    T2CONbits.TON = 1;                      /* Enable State machine timer */
    PTCONbits.PTEN = 1;                     /* Enable PWM module */
    
    while(1)
    {
        refreshDisplay();
        fanControl();
    }
    return 0;
}


void delay_ms (unsigned int delay)
{
    timerInterruptCount = 0;    /* Clear Interrupt counter flag */
    
    PR1 = 0x9C40;               /* (1ms / 25ns) = 40,000 = 0x9C40 */ 
    IEC0bits.T1IE = 1;          /* Enable Timer1 interrupts */
    IPC0bits.T1IP = 5;			
    T1CONbits.TON = 1;          /* Enable Timer1 */
    
    while (timerInterruptCount < delay);
                                /* Wait for Interrupt counts to equal delay */
    
    T1CONbits.TON = 0;          /* Disable the Timer */
}

void delay_100us (void)
{
    timerInterruptCount = 0;    /* Clear Interrupt counter flag */
    
    PR1 = 0x0FA0;               /* (100us / 25ns) = 4,000 = 0x0FA0 */ 
    IEC0bits.T1IE = 1;          /* Enable Timer1 interrupts */
    IPC0bits.T1IP = 5;			
    T1CONbits.TON = 1;          /* Enable Timer1 */
    
    while (timerInterruptCount < 1);
                                /* Wait for Interrupt counts to equal delay */
    
    T1CONbits.TON = 0;          /* Disable the Timer */
}

void refreshDisplay(void)
{
	HexToDec(inverterVoltageRMS, &DecimalDisplayValue_VoltageRMS);
	HexToDec(inverterCurrentRMS, &DecimalDisplayValue_CurrentRMS);
	
    LCD_writeRow("Microchip OfflineUPS",0,0,20);

    if (systemState == SYSTEM_STARTUP)
    {
        LCD_writeRow("MODE: SYSTEM STARTUP",1,0,20);
                if (batteryVoltage < BATTERY_BULK_VOLTAGE)
        {
            LCD_writeRow("BATTERY: LOW        ",2,0,20);
        }
        else if (batteryVoltage < BATTERY_FLOAT_VOLTAGE)
        {
            LCD_writeRow("BATTERY: OK         ",2,0,20);
        }
        else
        {
            LCD_writeRow("BATTERY: FULL       ",2,0,20);
        }
        LCD_writeRow("                    ",3,0,20);
    }    
    else if (systemState == INVERTER_MODE)
    {
        LCD_writeRow("MODE: INVERTER      ",1,0,20);
                if (fCrestFactorWarning == CREST_FACTOR_HIGH)
        {
            LCD_writeRow("WARNING: HIGH CF    ",2,0,20);                        
        }
        else
        {
            if (batteryVoltage < BATTERY_BULK_VOLTAGE)
            {
                LCD_writeRow("BATTERY: LOW        ",2,0,20);
            }
            else if (batteryVoltage < BATTERY_FLOAT_VOLTAGE)
            {
                LCD_writeRow("BATTERY: OK         ",2,0,20);
            }
            else
            {
                LCD_writeRow("BATTERY: FULL       ",2,0,20);
            }
        }
        LCD_writeRow("V:      V  I:      A",3,0,20);        
        LCD_writeRow((char *)(&DecimalDisplayValue_VoltageRMS),3,2,6);
        LCD_writeRow((char *)(&DecimalDisplayValue_CurrentRMS),3,13,6);
    }
    else if (systemState == BATTERY_CHARGER_MODE)
    {
	    LCD_writeRow("MODE: BATT CHARGER  ",1,0,20);
        LCD_writeRow("BATTERY:            ",2,0,20);
	    LCD_writeRow("                    ",3,0,20);

        if (batteryVoltage < BATTERY_BULK_VOLTAGE)
        {
            LCD_writeRow(" LOW ",2,9,5);
        }
        else if (batteryVoltage < BATTERY_FLOAT_VOLTAGE)
        {
            LCD_writeRow(" OK  ",2,9,5);
        }
        else
        {
            LCD_writeRow(" FULL",2,9,5);
        }
    }
    else if (systemState == SYSTEM_ERROR)
    {
	    LCD_writeRow("MODE: SYSTEM ERROR  ",1,0,20);
        if (batteryVoltage < BATTERY_BULK_VOLTAGE)
        {
            LCD_writeRow("BATTERY: LOW        ",2,0,20);
        }
        else if (batteryVoltage < BATTERY_FLOAT_VOLTAGE)
        {
            LCD_writeRow("BATTERY: OK         ",2,0,20);
        }
        else
        {
            LCD_writeRow("BATTERY: FULL       ",2,0,20);
        }
        if (errorState == DC_LINK_UNDERVOLTAGE_FAULT)
        {
            LCD_writeRow("FAULT: DC LINK UV   ",3,0,20);
        }
        else if (errorState == DC_LINK_OVERVOLTAGE_FAULT)
        {
            LCD_writeRow("FAULT: DC LINK OV   ",3,0,20);
        }
        else if (errorState == BATTERY_UNDERVOLTAGE_FAULT)
        {
            LCD_writeRow("FAULT: BATTERY UV   ",3,0,20);
        }
        else if (errorState == BATTERY_OVERVOLTAGE_FAULT)
        {
            LCD_writeRow("FAULT: BATTERY OV   ",3,0,20);
        }
        else if (errorState == PUSHPULL_OVERCURRENT_FAULT)
        {
            LCD_writeRow("FAULT: BATTERY OC   ",3,0,20);
        }
        else if (errorState == INVERTER_OVERCURRENT_FAULT)
        {
            LCD_writeRow("FAULT: OUTPUT OC    ",3,0,20);
        }
        else if (errorState == INVERTER_DRIVER_FAULT)
        {
            LCD_writeRow("FAULT: DRIVER FAULT ",3,0,20);
        }
        else if (errorState == MAINS_STARTUP_FAULT)
        {
            LCD_writeRow("FAULT: NO AC @ START",3,0,20);
        }
        else if (errorState == MAINS_FREQUENCY_ERROR)
        {
            LCD_writeRow("FAULT:MAINS FREQ ERR",3,0,20);
        }
        else 
        {
            LCD_writeRow("                    ",3,0,20);
        }
    }

    LCD_pauseMedium();  
}

void fanControl(void)
{
    int fanSpeed;

    if (systemState == INVERTER_MODE)
    {
        if (inverterCurrentRMS >= INVERTER_CURRENT_50PC_LOAD)
        {
            fanSpeed = MAX_FAN_SPEED;
        }
        else if (inverterCurrentRMS < INVERTER_CURRENT_40PC_LOAD)
        {
            fanSpeed = NOM_FAN_SPEED;
        }
    }
    else
    {
        fanSpeed = MIN_FAN_SPEED;
    }
    
    if (PDC4 < fanSpeed)
    {
        PDC4 += 1000;
    }
    else if (PDC4 > fanSpeed)
    {
        PDC4 -= 1000;
    }
}

