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
#include <libq.h>
#include "offline_ups.h"
#include "sineTable_50Hz.h"

/* Define Variables */
int batteryVoltage;
int batteryCurrent;
int mainsVoltage;
int dcLinkVoltage;
int pushpullCurrent;

int cmdInverterOpenLoop = 0;
int sampleCount = 0;
int inverterInterruptCount = 0;
int RMSBufferCount = 0;
int dcLinkUnderVoltageCount = 0;
int dcLinkOverVoltageCount = 0;
int pushpullOverCurrentCount = 0;
int batteryUnderVoltageCount = 0;
int batteryOverVoltageCount = 0;
int mainsInterruptCount = 0;
int mainsPresentCount = 0;
int mainsAbsentCount = 0;
int driverFaultCount = 0;
int fInverterDriverFault = DRIVER_FAULT_CLEARED;
int fInverterCurrentLimit = CURRENT_LIMIT_CLEARED;
int mainsQuadrant = UNKNOWN_QUADRANT;
int prevMainsVoltage = 0;
int prevMainsVoltageRef;
int mainsSampleCount = 0;
int mainsVoltageRef;
int mainsError;

int mainsCycleCount = 0;
int mainsBufferSize = 300;
volatile int mainsVref[300];

extern int timerInterruptCount;
extern int inverterOffsetAdjustment;

extern int systemState;
extern int mainsState;
extern int dcLinkState;
extern int batteryState;
extern int pushPullCurrentState;
extern int inverterDriverState;
extern int errorState;

extern int fPushPullSoftStart;
extern int fSwitchRelay;
extern int fRMSCalculation;

extern signed int voltageRMSBuffer[RMS_BUFFER_SIZE] \
                        __attribute__ ((space(xmemory)));
extern signed int currentRMSBuffer[RMS_BUFFER_SIZE] \
                        __attribute__ ((space(xmemory)));

/* Remove comments to following line if open loop execution is desired */
//#define INVERTER_OPEN_LOOP 1

/* Variables for Push-pull control loop */
int pushpullVoltageRef,pushpullVoltageError, pushpullVoltagePreviousError;
int pushpullVoltageDeltaError;
int pushpullVoltageControlOutput;
int PPtemp3,PPtemp2,PPtemp1;
long PPtemp5,PPtemp4;

/* Variables for Inverter control loop */
int inverterVoltageRef, inverterVoltageError;
long temp5,temp4,inverterCurrentRef,inverterCurrentError,VoltageCmd;
int inverterCurrentControlOutput;
int FilteredVo =0 , prevFilteredVo =0 ,delVo =0 ,CapCurrent=0, LoadCurrent = 0;
int Oneover_fourdclinkvoltage;
int inverterOutputVoltage;
int inverterOutputCurrent;
int temp3,temp1;

void __attribute__((interrupt, no_auto_psv)) _ADCP0Interrupt()
{	
    int inverterFreqAdjustment;

	ADSTATbits.P0RDY = 0;                   /* Clear ADC pair ready bit */
	IFS6bits.ADCP0IF = 0;                   /* Clear ADC Interrupt Flag */
    
    if (driverFaultCount >= 2)
    {
        PTCONbits.PTEN = 0;
        inverterDriverState = DRIVER_FAULT_ACTIVE;
        systemState = SYSTEM_ERROR;
        errorState = INVERTER_DRIVER_FAULT;
    }
    
    #ifdef INVERTER_OPEN_LOOP
        cmdInverterOpenLoop = SineTable_50Hz[sampleCount] >> 2;
    #endif

    /* Update reference for inverter output from sine lookup table */
    if ((sampleCount >= 0) && (sampleCount < 128))
    {
        inverterVoltageRef = (SineTable_50Hz[sampleCount] >> OUTPUT_VOLTAGE_SETTING);
        if ((mainsState == MAINS_OK) && (_Q15abs(mainsBufferSize - SINE_TABLE_SIZE/2) <= 10))
        {
            if (sampleCount == 120 && inverterInterruptCount == 0)
            {
                sampleCount = 136;
            }
        }
    }
    else if ((sampleCount >= 128) && (sampleCount < 256))
    {
        inverterVoltageRef = (SineTable_50Hz[255 - sampleCount] >> OUTPUT_VOLTAGE_SETTING);
    }
    else if ((sampleCount >= 256) && (sampleCount < 384))
    {
        inverterVoltageRef = - (SineTable_50Hz[sampleCount - 256] >> OUTPUT_VOLTAGE_SETTING);
        if ((mainsState == MAINS_OK) && (_Q15abs(mainsBufferSize - SINE_TABLE_SIZE/2) <= 10))
        {
            if (sampleCount == 376 && inverterInterruptCount == 0)
            {
                sampleCount = 392;
            }
        }
    }
    else
    {
        inverterVoltageRef = - (SineTable_50Hz[511 - sampleCount] >> OUTPUT_VOLTAGE_SETTING);
    }

    if (inverterInterruptCount == 0)
    {
        inverterInterruptCount++;
    }
    else
    {
        sampleCount++;
        inverterInterruptCount = 0;
    }
    
    /* Measured Inverter Output Voltage in Q15 format */
	inverterOutputVoltage = (ADCBUF1 << 5) - DC_OFFSET - inverterOffsetAdjustment;
	
	/* Measured Inverter Output Current in Q15 format */
    inverterOutputCurrent = (ADCBUF0 << 5) - DC_OFFSET;
    
    inverterVoltageError = (inverterVoltageRef - inverterOutputVoltage);
	temp4 = temp4 + ((__builtin_mulss((int)InvCMvoltageKi,inverterVoltageError))>>15);               //calculate I term
		
	if(temp4 > 32767)
    {
	    temp4 = 32767;
	}
	else if(temp4 < -32767)
	{
		temp4 = -32767;	
    }   

    if (inverterOutputCurrent > INVERTER_CURRENT_MAX_PEAK)
    {
        fInverterCurrentLimit = CURRENT_LIMIT_ACTIVE;
        temp4 = 0;
    }
    else if (inverterOutputCurrent < -INVERTER_CURRENT_MAX_PEAK)
    {
        fInverterCurrentLimit = CURRENT_LIMIT_ACTIVE;
        temp4 = 0;
    }
    
	temp1 = ((__builtin_mulss((int)InvCMvoltageKp,inverterVoltageError))>>15); 
				
	FilteredVo = ((__builtin_mulss((int)InvCMALPHA,prevFilteredVo))>>15) + ((__builtin_mulss((int)InvCMBETA,inverterOutputVoltage))>>15) ; 

	delVo = FilteredVo - prevFilteredVo;

	CapCurrent = ((__builtin_mulss(delVo,InvCbydt))>>13);					
	
    LoadCurrent = (__builtin_mulss((int)((inverterOutputCurrent - (int)CapCurrent)), InvSafetyfactor) >>15);
    inverterCurrentRef = (long)temp1 + temp4 + LoadCurrent;			

	if(inverterCurrentRef > INVERTER_CURRENT_MAX_PEAK)
	{
		inverterCurrentRef = INVERTER_CURRENT_MAX_PEAK;
	}
	if(inverterCurrentRef < -INVERTER_CURRENT_MAX_PEAK)
	{
		inverterCurrentRef = -INVERTER_CURRENT_MAX_PEAK;
    }   

	inverterCurrentError = (inverterCurrentRef - inverterOutputCurrent);

	VoltageCmd = ((__builtin_mulss((int)InvCMcurrentRa,(int)inverterCurrentError))>>15)+ ((__builtin_mulss((int)InvRp,(int)inverterOutputCurrent))>>15); //current P
	
	temp3 = ((__builtin_mulss((int)InvVMvoltagedecouple,(int)inverterOutputVoltage))>>15) ;     	//for 200 V      			//including scale factor

	temp5 = temp3 + VoltageCmd;

    if (OUTPUT_VOLTAGE_SETTING == 0)
    {
	Oneover_fourdclinkvoltage = 9000;				//choose value based on min bus voltage			
													//for ~ 120 V bus ~ 32000  
 														//for ~ 200 V bus ~ 18000
													//for ~ 400 V bus ~ 9000
    }
    else if (OUTPUT_VOLTAGE_SETTING == 1)
    {
        Oneover_fourdclinkvoltage = 18000;
    }
    else if (OUTPUT_VOLTAGE_SETTING == 2)
    {
        Oneover_fourdclinkvoltage = 32000;
    }
	if(temp5 > 32767)
    {
		temp5 =32767;
	}
	else if(temp5 < -32767)
	{
		temp5 = -32767;	
    }
    
	temp5 = (__builtin_mulss((int)temp5,Oneover_fourdclinkvoltage)>>13);

	if(temp5 > InvMAXSCALED_DUTY)												//1/16 scaling artefacts and clampling between values that will give 
    {
		temp5 = InvMAXSCALED_DUTY;
	}
	else if(temp5 < -InvMAXSCALED_DUTY)
	{
		temp5 = -InvMAXSCALED_DUTY;
    }   
	
	if (fInverterDriverFault == DRIVER_FAULT_ACTIVE)
    {
        // clear driver fault
        LATBbits.LATB7 = 1;
        
        // routine to recover from driver fault
        // if driver fault has occurred configure duty cycles to recover from fault
        if (sampleCount <= 255)
        {
            if (inverterOutputVoltage > inverterVoltageRef)
            {
                fInverterDriverFault = DRIVER_FAULT_CLEARED;
                LATBbits.LATB7 = 0;
                temp4 = 0;
            }
            else
            {
                inverterCurrentControlOutput += 10;
				if(inverterCurrentControlOutput   > INVERTER_MAX_CONTROL_OUTPUT)
 				{	
 					inverterCurrentControlOutput = INVERTER_MAX_CONTROL_OUTPUT;
				}	
            }
        }
        else
        {
            if (inverterOutputVoltage < inverterVoltageRef)
            {
                fInverterDriverFault = DRIVER_FAULT_CLEARED;
                LATBbits.LATB7 = 0;
                temp4 = 0;
            }
            else
            {
	            inverterCurrentControlOutput -= 10;
				if(inverterCurrentControlOutput   < -INVERTER_MAX_CONTROL_OUTPUT)
 				{
 					inverterCurrentControlOutput = -INVERTER_MAX_CONTROL_OUTPUT;
				}
            }
        }
    }
    else if (fInverterCurrentLimit == CURRENT_LIMIT_ACTIVE)
    {
        if (sampleCount <= 255)
        {
            if ((inverterOutputCurrent < INVERTER_CURRENT_MAX_PEAK) ||(inverterOutputVoltage > inverterVoltageRef) )
            {
                fInverterCurrentLimit = CURRENT_LIMIT_CLEARED;
            }
            else
            {
                inverterCurrentControlOutput += 2;
				if(inverterCurrentControlOutput > INVERTER_MAX_CONTROL_OUTPUT)
				{
		 			inverterCurrentControlOutput = INVERTER_MAX_CONTROL_OUTPUT;
				}
            }
        }
        else
        {
           if((inverterOutputCurrent > -INVERTER_CURRENT_MAX_PEAK) ||(inverterOutputVoltage < inverterVoltageRef))
	   {
                fInverterCurrentLimit = CURRENT_LIMIT_CLEARED;
            }
            else
            {
                inverterCurrentControlOutput -= 2;
				if(inverterCurrentControlOutput   <  -INVERTER_MAX_CONTROL_OUTPUT)
				{
		 			inverterCurrentControlOutput =  -INVERTER_MAX_CONTROL_OUTPUT;
				}
            }
        }        
    }
    else
    {
        // if no driver fault present then use output of control loop for duty cycle
        inverterCurrentControlOutput = (int)((__builtin_mulss((int)temp5,(int)InvPERIOD))>>15);
    }
    
    #ifndef INVERTER_OPEN_LOOP	
        /* Update PWM duty cycle for PWM1 and PWM2 */
        PDC1 = InvHALF_PERIOD + inverterCurrentControlOutput;
	    PDC2 = InvHALF_PERIOD - inverterCurrentControlOutput;
    #else
        PDC1 = INVERTER_PDC_NOMINAL_VALUE + cmdInverterOpenLoop;
	    PDC2 = INVERTER_PDC_NOMINAL_VALUE - cmdInverterOpenLoop;
    #endif

    if (sampleCount == SINE_TABLE_SIZE)
    {
        sampleCount = 0;                    /* Start from first data point on
                                               sine lookup table after reaching
                                               the end */
        if (fRMSCalculation == CALCULATION_DONE)
        {
            fRMSCalculation = READY_TO_COLLECT_DATA;
        }
    }

    if ((mainsSampleCount == (mainsBufferSize - 125)))
    {
        if ((mainsState == MAINS_OK) && (_Q15abs(mainsBufferSize - SINE_TABLE_SIZE/2) <= 10))
        {
            inverterFreqAdjustment = 32;
        }
        else
        {
            inverterFreqAdjustment = 0;
        }

        if ((sampleCount >= (255 - inverterFreqAdjustment)) && (sampleCount < (269 - inverterFreqAdjustment)) )
        {
            fSwitchRelay = READY_TO_SWITCH;     
                                            /* sampleCount = 410 means there are
                                               4ms remaining for the end of sine
                                               wave cycle. Begin switching relay
                                               now so that total switch-over 
                                               time will be within specification
                                               of 10ms. */
    	}
    }

    /* Save measured voltage and current values in buffers to calculate RMS
       values. Buffer size is only 128 so save once in 4 inverter interrupts
       to cover entire sine wave cycle */    
    if (fRMSCalculation == READY_TO_COLLECT_DATA)
    {
        if ((sampleCount % 4 == 0) && (inverterInterruptCount == 0))
    	{
        	/* Save Measured Voltage */
    	    voltageRMSBuffer[RMSBufferCount] = inverterOutputVoltage;
    	    /* Save Measured Current */
    	    currentRMSBuffer[RMSBufferCount++] = inverterOutputCurrent;
    	}
    }	
	if (RMSBufferCount >= RMS_BUFFER_SIZE)
	{
    	fRMSCalculation = READY_TO_CALCULATE;
    	RMSBufferCount = 0;
    }   
    
	/* Save previous filtered output voltage */
	prevFilteredVo = FilteredVo;

    /* Save previous output of voltage loop */
    //prevVoltageCmd =  VoltageCmd;

   	/* Save previous measured current */
	//prevInverterCurrent = inverterOutputCurrent;

}

void __attribute__((interrupt, no_auto_psv)) _ADCP1Interrupt()
{	
	ADSTATbits.P1RDY = 0;                   /* Clear ADC pair ready bit */
	IFS6bits.ADCP1IF = 0;                   /* Clear ADC Interrupt Flag */
	
	pushpullCurrent = (ADCBUF2 << 5);       /* Measure push-pull primary
	                                           current */
	dcLinkVoltage = (ADCBUF3 << 5);         /* Measure DC link voltage */

	pushpullVoltageError = (pushpullVoltageRef - dcLinkVoltage);

    PPtemp4 = PPtemp4 + ((__builtin_mulss((int)PPVMvoltageKi,pushpullVoltageError))>>15);               //calculate I term
    
	if(PPtemp4 > 4095)
	{
	    PPtemp4 = 4095;
	}
	else if(PPtemp4 < -4095)
	{
	    PPtemp4 = -4095;
	}

    if (fPushPullSoftStart == PUSHPULL_SOFTSTART_ACTIVE)
    {
    	PPtemp1 = ((__builtin_mulss((int)PPVMSSvoltageKp,pushpullVoltageError))>>13); 									//calculate P term
    }
    else
    {
    	PPtemp1 = ((__builtin_mulss((int)PPVMvoltageKp,pushpullVoltageError))>>15); 									//calculate P term
    }

	pushpullVoltageDeltaError = (pushpullVoltageError - pushpullVoltagePreviousError);		
	PPtemp2 = ((__builtin_mulss((int)PPVMvoltageKd,pushpullVoltageDeltaError))>>15);							//calculate D term
	PPtemp3 = ((__builtin_mulss((int)PPVMvoltagedecouple,(int)dcLinkVoltage))>>15);			//including scale factor
	PPtemp5 = PPtemp1+PPtemp4+PPtemp3;

	if(PPtemp5 > PPMAXSCALED_DUTY)												//1/64 scaling artefacts and clampling between values that will give 
	{
		PPtemp5 = PPMAXSCALED_DUTY;
	}
	else if(PPtemp5 < PPMINSCALED_DUTY)
	{
		PPtemp5 = 0;
	}
	
	pushpullVoltageControlOutput = ((__builtin_mulss((int)PPtemp5,PUSH_PULL_PERIOD_VALUE))>>12); 		//post multiply by 64

	pushpullVoltagePreviousError = pushpullVoltageError;

	PDC3 = pushpullVoltageControlOutput;                 /* Update duty cycle for push-pull
                                                            boost converter */
        
    TRIG3 = pushpullVoltageControlOutput;                /* Trigger ADC at the end of PWM duty
                                      cycle */
    STRIG3 = pushpullVoltageControlOutput + 8;           /* Work-around for ADC trigger
                                                            erratum on A2 silicon */

    /* Check for DC Link under voltage */
    if (dcLinkVoltage < DC_LINK_VOLTAGE_MIN)
    {
        dcLinkUnderVoltageCount++;
        if (dcLinkUnderVoltageCount >= 10)
        {
            dcLinkState = DC_LINK_UNDERVOLTAGE;
        }
    }

    /* Check for DC Link over voltage */                
    else if (dcLinkVoltage > DC_LINK_VOLTAGE_MAX)
    {
        dcLinkOverVoltageCount++;
        if (dcLinkOverVoltageCount >= 50)
        {
            dcLinkState = DC_LINK_OVERVOLTAGE;
        }
    }
    else
    {
        dcLinkUnderVoltageCount = 0;
        dcLinkOverVoltageCount = 0;
        dcLinkState = DC_LINK_OK;
    }

    /* Check for Push-pull over current */    
    if (pushpullCurrent > PUSHPULL_CURRENT_MAX)
    {
        pushpullOverCurrentCount++;
        if(fPushPullSoftStart == PUSHPULL_SOFTSTART_ACTIVE)
        {
            if (pushpullOverCurrentCount >= 1000)
            {
                /* Turn OFF PWMs as system is in error */
                PTCONbits.PTEN = 0;
                pushPullCurrentState = PUSHPULL_OVERCURRENT;
                systemState = SYSTEM_ERROR;
            }
        }    
        else
        {
            if (pushpullOverCurrentCount >= 250)
            {
                /* Turn OFF PWMs as system is in error */
                PTCONbits.PTEN = 0;
                pushPullCurrentState = PUSHPULL_OVERCURRENT;
                systemState = SYSTEM_ERROR;
                errorState = PUSHPULL_OVERCURRENT_FAULT;
            }
        }
    }
    else
    {
        pushpullOverCurrentCount = 0;
    }

}

void __attribute__((interrupt, no_auto_psv)) _ADCP2Interrupt()
{	
	ADSTATbits.P2RDY = 0;                   /* Clear ADC pair ready bit */
	IFS7bits.ADCP2IF = 0;                   /* Clear ADC Interrupt Flag */
	
    batteryCurrent = (ADCBUF4 << 5);        /* Measure charging current */
    batteryVoltage = (ADCBUF5 << 5);        /* Measure battery voltage */
            
    /* Check for Battery under voltage */
    if(batteryVoltage < BATTERY_VOLTAGE_MIN)
    {
        batteryUnderVoltageCount++;
        if (batteryUnderVoltageCount >= 150)
        {
            batteryState = BATTERY_UNDERVOLTAGE;
        }
    }
    /* Check for Battery over voltage */
    else if(batteryVoltage > BATTERY_VOLTAGE_MAX)
    {
        batteryOverVoltageCount++;
        if (batteryOverVoltageCount >= 150)
        {
            batteryState = BATTERY_OVERVOLTAGE;
        }
    }
    else
    {
        batteryUnderVoltageCount = 0;
        batteryOverVoltageCount = 0;    
        batteryState = BATTERY_OK;
    }        
}

void __attribute__((interrupt, no_auto_psv)) _ADCP5Interrupt()
{	
	ADSTATbits.P5RDY = 0;                   /* Clear ADC pair ready bit */
    IFS7bits.ADCP5IF = 0;                   /* Clear ADC Interrupt Flag */
    
    mainsInterruptCount = (mainsInterruptCount + 1) % 4;

    mainsVoltage = -((ADCBUF11 << 5) - DC_OFFSET);
                                            /* Measure AC Mains voltage.
                                               Line and Neutral terminals
                                               are swapped on AC mains
                                               measurement (with respect to
                                               Inverter output sense). */
                                               
    if (mainsState == MAINS_OK)
    {
        if (mainsInterruptCount == 0)
        {
            mainsSampleCount++;

            mainsVoltageRef = mainsVref[mainsSampleCount];

            if (mainsSampleCount <= mainsBufferSize)
            {
            	mainsError = mainsVoltageRef - mainsVoltage;
            }
            else if ((mainsSampleCount > mainsBufferSize) && (mainsSampleCount < 300))
            {
                mainsError = mainsVref[(mainsSampleCount - mainsBufferSize)];
            }
            else
            {
                systemState = SYSTEM_ERROR;
                errorState = MAINS_FREQUENCY_ERROR;
            }

            if ((prevMainsVoltage < 0) && (mainsVoltage >= 0))
            {
                if ((mainsVoltage - prevMainsVoltage) > (100 >> OUTPUT_VOLTAGE_SETTING))
                {
                    mainsSampleCount = 0;
                    mainsQuadrant = FIRST_QUADRANT;
                }
            }

            if (_Q15abs(mainsError) > (1200 >> OUTPUT_VOLTAGE_SETTING))
            {
                mainsAbsentCount++;         /* Increment count to indicate
                                               AC Mains is absent */
                if (mainsAbsentCount >= 20)
                {
                    mainsState = MAINS_NOT_OK;
                                            /* If mains is detected bad for 20
                                               consecutive samples, conclude
                                               that AC Mains is not OK. */
                                            
                    mainsAbsentCount = 0;
                }
            }
            else
            {
                mainsAbsentCount = 0;   /* AC Mains is present */
            }

            if ((mainsQuadrant == FIRST_QUADRANT) && (mainsSampleCount > (mainsBufferSize/4)))
            {
                mainsQuadrant = SECOND_QUADRANT;
                
            }
            else if ((mainsQuadrant == SECOND_QUADRANT) && (mainsSampleCount > (mainsBufferSize/2)))
            {
                mainsQuadrant = THIRD_QUADRANT;
            }
            else if ((mainsQuadrant == THIRD_QUADRANT) && (mainsSampleCount > (mainsBufferSize*3/4)))
            {
                mainsQuadrant = FOURTH_QUADRANT;
            }
        }
    }
    else if ((mainsState == MAINS_NOT_OK) || (mainsState == MAINS_STATE_UNKNOWN))
    {
        if (mainsInterruptCount == 0)
        {
            if (_Q15abs(mainsVoltage) > (11000 >> OUTPUT_VOLTAGE_SETTING))
            {
                mainsPresentCount++;            /* Increment count to indicate
                                                   AC Mains is present */
    
                if (mainsPresentCount >= 20)
                {
                    mainsState = MAINS_HV_DETECTED;
                                                /* If mains is detected OK for 100
                                                   consecutive samples, conclude
                                                   that AC Mains is present. */
                    mainsPresentCount = 0;
                    mainsAbsentCount = 0;
                }
            }
            else
            {
                mainsPresentCount = 0;          /* AC Mains is absent */
            }
        }
    }
    else if (mainsState == MAINS_HV_DETECTED)
    {
        if (mainsInterruptCount == 0)
        {
            /* Wait for first zero-crossing */
            if ((prevMainsVoltage < 0) && (mainsVoltage >= 0))
            {
                if ((mainsVoltage - prevMainsVoltage) > (100 >> OUTPUT_VOLTAGE_SETTING))
                {
                    mainsState = MAINS_ZC_DETECTED;
                    mainsSampleCount = 0;
                    mainsCycleCount = 0;
                    mainsBufferSize = 300;
                    mainsQuadrant = FIRST_QUADRANT;
                }
                else
                {
                    mainsState = MAINS_NOT_OK;
                }
            }
        }
    }
    else if (mainsState == MAINS_ZC_DETECTED)
    {
        if (mainsInterruptCount == 0)
        {
            mainsSampleCount++;
            
            if (mainsSampleCount < 300)
            {
                if (mainsCycleCount == 0)
            {
                    mainsVref[mainsSampleCount] = 0;
            }
                mainsVref[mainsSampleCount] += (mainsVoltage >> 2);

                /* Wait for next positive zero-crossing to determine mains frequency */
                if ((prevMainsVoltage < 0) && (mainsVoltage >= 0))
                {
                    if ((mainsVoltage - prevMainsVoltage) > (100 >> OUTPUT_VOLTAGE_SETTING))
                    {
                        if ((mainsSampleCount > 192) && (mainsSampleCount < 300))
                        {
                            mainsCycleCount++; 
                            if (mainsSampleCount <= mainsBufferSize)
                            {
                                mainsBufferSize = mainsSampleCount;
                            } 
                            mainsSampleCount = 0;
                            mainsQuadrant = FIRST_QUADRANT;
                            if (mainsCycleCount == 4)
                            {
                                mainsState = MAINS_SYNCHRONIZING;
                            }
                        }
                        else
                        {
                            mainsState = MAINS_NOT_OK;
                        }    
                     
                    }
                    else
                    {
                        mainsState = MAINS_NOT_OK;
                    }
                }
            }
            else
            {
                mainsSampleCount = 0;
                mainsState = MAINS_NOT_OK;
                mainsCycleCount = 0;
            }
        }
    }
    else if (mainsState == MAINS_SYNCHRONIZING)
    {
	    if (mainsInterruptCount == 0)
	    {
	        mainsSampleCount++;
	        mainsVoltageRef = mainsVref[mainsSampleCount];
	
	        if (mainsSampleCount <= mainsBufferSize)
	        {
	            mainsError = mainsVoltageRef - mainsVoltage;
	        }
	        else if (mainsSampleCount > (mainsBufferSize + 5))
	        {
    	        mainsState = MAINS_NOT_OK;
	        }
	        
	        if ((prevMainsVoltage < 0) && (mainsVoltage >= 0))
	        {
	            if ((mainsVoltage - prevMainsVoltage) > (100 >> OUTPUT_VOLTAGE_SETTING))
	            {
	                mainsSampleCount = 0;
	                mainsQuadrant = FIRST_QUADRANT;
	            }
	        }
	
	        if (_Q15abs(mainsError) < (400 >> OUTPUT_VOLTAGE_SETTING))
	        {
	            mainsPresentCount++;        /* Increment count to indicate
	                                           mains error is within limits */
	            if (mainsPresentCount >= 1000)
	            {
	                mainsState = MAINS_OK;
	                                        /* If mains error is detected within
	                                           limit for 1000 consecutive 
	                                           samples, conclude that AC Mains 
	                                           is OK. */
	                mainsPresentCount = 0;
	                mainsAbsentCount = 0;
	            }
	        }
	        else
	        {
	            mainsAbsentCount++;
	            if (mainsAbsentCount >= 10)
	            {
	                mainsState = MAINS_NOT_OK;
	                mainsAbsentCount = 0;      /* AC Mains is absent */
	                mainsPresentCount = 0;
	            }
	        }
	    }
	}    

    if (mainsInterruptCount == 0)
    {
        prevMainsVoltage = mainsVoltage;
    }    
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt()
{
    timerInterruptCount ++;                 /* Increment interrupt counter */
    IFS0bits.T1IF = 0;                      /* Clear Interrupt Flag */
}

void __attribute__((__interrupt__, no_auto_psv)) _INT1Interrupt()
{

if (systemState == INVERTER_MODE)
{
    fInverterDriverFault = DRIVER_FAULT_ACTIVE;
    driverFaultCount++;    

    if (sampleCount <= 255)
    {
        inverterCurrentControlOutput = 100;
    }
    else
    {
        inverterCurrentControlOutput = -100;
    }
    
    PDC1 = INVERTER_PDC_NOMINAL_VALUE + inverterCurrentControlOutput;
    PDC2 = INVERTER_PDC_NOMINAL_VALUE - inverterCurrentControlOutput;
}

else
{
	LATBbits.LATB7 = 1;
}
	
    IFS1bits.INT1IF = 0;                    /* Clear Interrupt Flag */
    
}
