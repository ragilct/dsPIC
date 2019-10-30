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
#ifndef _offline_ups_h
#define _offline_ups_h

#include <dsp.h>

#define OUTPUT_VOLTAGE_SETTING 0    /* 220V */
//#define OUTPUT_VOLTAGE_SETTING 1    /* 110V */
//#define OUTPUT_VOLTAGE_SETTING 2    /* 55V */

/* System State definitions */
#define SYSTEM_STARTUP 0
#define BATTERY_CHARGER_MODE 1
#define INVERTER_MODE 2
#define SYSTEM_ERROR 3

/* AC Mains State definitions */
#define MAINS_STATE_UNKNOWN 0
#define MAINS_NOT_OK 1
#define MAINS_HV_DETECTED 2
#define MAINS_ZC_DETECTED 3
#define MAINS_SYNCHRONIZING 4
#define MAINS_OK 5

#define UNKNOWN_QUADRANT 0
#define FIRST_QUADRANT 1
#define SECOND_QUADRANT 2
#define THIRD_QUADRANT 3
#define FOURTH_QUADRANT 4

/* Battery State definitions */
#define BATTERY_UNKNOWN 0
#define BATTERY_OK 1
#define BATTERY_UNDERVOLTAGE 2
#define BATTERY_OVERVOLTAGE 3

/* DC Link Voltage State definitions */
#define DC_LINK_UNKNOWN 0
#define DC_LINK_OK 1
#define DC_LINK_UNDERVOLTAGE 2
#define DC_LINK_OVERVOLTAGE 3

/* Error State Definitions */
#define NO_FAULT 0
#define DC_LINK_UNDERVOLTAGE_FAULT 1
#define DC_LINK_OVERVOLTAGE_FAULT 2
#define BATTERY_UNDERVOLTAGE_FAULT 3
#define BATTERY_OVERVOLTAGE_FAULT 4
#define PUSHPULL_OVERCURRENT_FAULT 5
#define INVERTER_OVERCURRENT_FAULT 6
#define INVERTER_DRIVER_FAULT 7
#define MAINS_STARTUP_FAULT 8
#define MAINS_FREQUENCY_ERROR 9

/* Push-pull Primary Current State definitions */
#define PUSHPULL_CURRENT_OK 0
#define PUSHPULL_OVERCURRENT 1

/* Inverter Current State definitions */
#define INVERTER_CURRENT_OK 0
#define INVERTER_OVERCURRENT 1

/* Inverter Constants */
#define INVERTER_PERIOD_VALUE 9392
#define INVERTER_ALTDTR_VALUE 256
#define INVERTER_MIN_ON_TIME 256
#define INVERTER_PDC_NOMINAL_VALUE \
            (INVERTER_PERIOD_VALUE-INVERTER_ALTDTR_VALUE)/2
#define INVERTER_MAX_CONTROL_OUTPUT INVERTER_PDC_NOMINAL_VALUE
#define DC_OFFSET 0x3FF0
#define SINE_TABLE_SIZE 512

#define INVERTER_CURRENT_MAX 525                /* Corresponds to 1050VA
												   Value determined
												   experimentally */
#define INVERTER_CURRENT_40PC_LOAD 200          /* Corresponds to around 400VA
                                                   Value determined
												   experimentally */
#define INVERTER_CURRENT_50PC_LOAD 250          /* Corresponds to around 500VA
												   Value determined
												   experimentally */
#define INVERTER_CURRENT_MAX_OVERLOAD 675       /* Corresponds to 1350VA
												   Value determined
													experimentally */
#define INVERTER_CURRENT_MAX_PEAK 6000

#define CURRENT_LIMIT_CLEARED 0
#define CURRENT_LIMIT_ACTIVE 1

#define DRIVER_FAULT_CLEARED 0
#define DRIVER_FAULT_ACTIVE 1

#define CREST_FACTOR_OK 0
#define CREST_FACTOR_HIGH 1

#define NOT_READY_TO_SWITCH 0
#define READY_TO_SWITCH 1

/* Push-pull Boost Converter Constants */
#define PUSH_PULL_PERIOD_VALUE INVERTER_PERIOD_VALUE/2
#define PUSHPULL_PDC_MAX_VALUE (PUSH_PULL_PERIOD_VALUE/8)*7
#define PUSHPULL_PDC_MIN_VALUE 500

#define PUSHPULL_SOFTSTART_INCREMENT 100
#define PUSHPULL_SOFTSTART_INACTIVE 0
#define PUSHPULL_SOFTSTART_ACTIVE 1

#define DC_LINK_VOLTAGE_NOM (30067 >> OUTPUT_VOLTAGE_SETTING)           /* 385V */
#define DC_LINK_VOLTAGE_MIN (21476 >> OUTPUT_VOLTAGE_SETTING)           /* 275V */
#define DC_LINK_VOLTAGE_MAX (32411 >> OUTPUT_VOLTAGE_SETTING)           /* 415V */

#define PUSHPULL_CURRENT_MAX 16000          /* Value determined
											   experimentally */

/* Battery Charger Constants */
#define BATTERY_CHARGER_PERIOD_VALUE 38461 /* 25KHz */
#define BATTERY_CHARGER_SDC_MIN 1000
#define BATTERY_CHARGER_SDC_MAX 38200

#define BATTERY_CHARGER_DUTYCYCLE_INCREMENT 20

#define MAX_CHARGING_CURRENT 25500          /* 2.25A */          
#define MIN_CHARGING_CURRENT 1400           /* 100mA */

#define BATTERY_VOLTAGE_MIN 14869           /* 34.0V */
#define BATTERY_TRICKLE_VOLTAGE 15613       /* 35.7V */
#define BATTERY_BULK_VOLTAGE 17712          /* 40.5V */
#define BATTERY_FLOAT_VOLTAGE 18892         /* 43.2V */
#define BATTERY_VOLTAGE_MAX 19680           /* 45.0V */

#define BATTERY_OVERCHARGING_SLOPE -20      /* Slope for battery over-charging
                                               profile */
#define BATTERY_OVERCHARGING_OFFSET -18960  /* Offset for battery over-charging
                                               profile */

/* Fan Control Constants */
#define FAN_CONTROL_PERIOD_VALUE 9616     /* 100KHz */
#define MAX_FAN_SPEED 9000                /* 93% duty cycle */
#define NOM_FAN_SPEED 5000                /* 52% duty cycle */
#define MIN_FAN_SPEED 2000                /* 20% duty cycle */

/* RMS Calculation constants */
#define RMS_BUFFER_SIZE 128
#define READY_TO_COLLECT_DATA 0
#define READY_TO_CALCULATE 1
#define CALCULATION_DONE 2

#define SINE_11_25_DEGREES (2507 >> OUTPUT_VOLTAGE_SETTING)
#define SINE_22_5_DEGREES (5086 >> OUTPUT_VOLTAGE_SETTING)
#define SINE_33_75_DEGREES (7470 >> OUTPUT_VOLTAGE_SETTING)
#define SINE_45_DEGREES (9566 >> OUTPUT_VOLTAGE_SETTING)
#define SINE_56_25_DEGREES (11295 >> OUTPUT_VOLTAGE_SETTING)
#define SINE_67_5_DEGREES (12589 >> OUTPUT_VOLTAGE_SETTING)
#define SINE_78_75_DEGREES (13400 >> OUTPUT_VOLTAGE_SETTING)

//the following are constants used in inverter section
//filter coeffs


//#define InvALPHA Q15(0.8)
//#define InvBETA  32767 - InvALPHA

//base voltage 744 V
//base current 81 A

//coeffs for voltage mode control

//#define InvVMvoltageKp   Q15(0.1279)
//#define InvVMvoltageKi   Q15(0.0060)
//#define InvVMvoltageKd   Q15(0.6285)

//coeffs for current mode control
#define InvCMvoltageKp   Q15(0.51)//Q15(0.51)
#define InvCMvoltageKi   Q15(0.035)//Q15(0.035)
#define InvCMcurrentRa   Q15(0.91)//Q15(0.91)
#define InvVMvoltagedecouple Q15(0.92)
#define InvRp			 Q15(0.17/2.0)	
#define InvCMALPHA		 Q15(0.7)
#define InvCMBETA		 Q15(0.3) 	
#define InvCbydt		 Q15(0.57)	//this is prescaled by 1/4
#define InvSafetyfactor	 Q15(0.99)

#define MAXCURRENTREF	 Q15(0.5)		

//duty related constants
#define InvPERIOD		  	INVERTER_PERIOD_VALUE
#define InvMAXSCALED_DUTY 	15490//15075 //15947 //15490 //(INVERTER_MAX_CONTROL_OUTPUT << 15)/9392 //15947 //14400						//44% to allow deadtime and min 										//on time of bottom switch
#define InvMINSCALED_DUTY 	800
#define InvHALF_PERIOD    	INVERTER_PDC_NOMINAL_VALUE //4446 //4696 //0.5*InvPERIOD

//pushpull section coeffs
#define PPALPHA 			Q15(0.85)
#define PPBETA  			Q15(0.15)

//coeffs for voltage mode control
#define PPVMvoltagedecouple Q15(0.0972)
#define PPVMvoltageKp   	Q15(0.74)
#define PPVMvoltageKi   	Q15(0.0043) //0.0043
#define PPVMvoltageKd   	Q15(0.0)

#define PPVMSSvoltageKp   	Q15(2.22/4)

#define PPCapconstant		Q15(0.96)  //ic = CdV/dt convert dV to ic
										//prescaler of 1/128

#define Rp					Q15(10.0/(420.0/80.0)/2.0) // Rp/RN prescaler of 1/2

#define PPMAXSCALED_DUTY    Q15(0.87/8.0)		//prescaler of 8
#define PPMINSCALED_DUTY    PUSHPULL_PDC_MIN_VALUE

#endif

