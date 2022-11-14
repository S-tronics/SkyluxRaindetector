/*
 * File:   AppSensor.c
 * Author: stronics
 *
 * Created on 30 juni 2018, 13:11
 */

/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <fastmath.h>
#include <stdio.h>

#include "..\..\Drv\DrvAdc.h"
#include "..\..\Drv\DrvFlash.h"
//Include Standard Function Libraries
#include "..\..\Std\I2C\StdMtch112.h"
#include "..\..\Std\Temperature\StdDS18S20.h"
#include "..\..\Std\Timer\StdTask.h"

#include "atmel_start_pins.h"
#include "AppIO.h"
#include "AppSensor.h"
#include "AppCasambi.h"
/**********************************************************************************************************************/

/***********************************************************************************************************************
; V E R I F Y    C O N F I G U R A T I O N
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
//Preprocessor Conditions
#define DEBUG_SENSOR
//Algorithm angled sensor
#define SAMPLE_PERIOD			1000		//Every second a measurement of 10 samples (1000ms)

#define NBR_SMPL_AVG			10
#define LT_SMPL_PERIOD			600			//Every 10 min. a long term sample

//#define LT_SMPL_PERIOD			6			//Every 10 min. a long term sample
#define LT_NBR_SMPL				1440		//10 days * 24 * 60 / 10	==>(every 10 mins. a sample)
//#define LT_NBR_SMPL				5		//10 days * 24 * 60 / 10	==>(every 10 mins. a sample)

#define MOD_TEMP_MOISTY			3
#define MOD_TEMP_FREEZING		0

#define PERIOD_10SEC			10			//10 seconds
#define PERIOD_20SEC			20			//20 seconds (Used to index array with avg capacitive values)
#define PERIOD_1MIN				60         	//60 seconds (1min.)
#define PERIOD_2MIN				120	        //120 seconds (2min.) 	
#define PERIOD_10MIN			600			//600 seconds (10min.)
#define PERIOD_30MIN			1800		//1800 seconds (30min.)
#define PERIOD_60MIN			3600		//3600 seconds (60min.)			

#define PERIOD_10SEC_CALC		2			//Every 2 seconds a rico calculation (10sec.) is done
#define PERIOD_1MIN_CALC		2			//Every 2 seconds a rico calculation (1min.) is done
#define	PERIOD_2MIN_CALC		4			//Every 4 seconds a rico calculation (2min.) is done
#define PERIOD_10MIN_CALC		20			//Every 20 seconds a rico calculation (10min.) is done
#define PERIOD_30MIN_CALC		60			//Every 60 seconds a rico calculation (30min.) is done
#define PERIOD_60MIN_CALC		120			//Every 120 seconds a rico calculation (60min.) is done	

#define SMPL_RICO				60

//Rico = RISING
#define	RICO_10SEC				9.92		//Rico for 10 seconds rainmeasurement
#define	RICO_1MIN_RISING		0.75		//Rico for 1 minute rainmeasurement (Before 56)
#define RICO_2MIN_RISING		0.43			//Rico for 2 minutes rainmeasurement (Before 30)
#define	RICO_10MIN_RISING		0.116		//Rico for 10 minutes rainsmeasurement (Before 7.3)
#define RICO_30MIN_RISING		0.065		//Rico for 30 minutes rainmeasurement (Before 3.5)
#define RICO_60MIN_RISING		0.233		//Rico for 60 minutes rainmeasurement (Before 2.6)

//AVG > 3000 | Rico = RISING
#define RICO_LVL				3000					
#define RICO_1MIN_RISING_LVL	0.7		//When avg > 3000 rico is stricter (Before 43)
#define RICO_2MIN_RISING_LVL	0.383		//Before 23
#define	RICO_10MIN_RISING_LVL	0.083		//Before 5

//Rico = FALLING
//#define RICO_1MIN_FALLING		-0.93		//Before -56
//24-05-2022
#define RICO_1MIN_FALLING		-1.6		//1min rico = -92 (more strict than before)
//#define RICO_2MIN_FALLING		-0.7		//Before -42
//24-05-2022
#define RICO_2MIN_FALLING		-1.16		//2min rico = -70 (more strict than before)
#define RICO_10MIN_FALLING		-100.0		//Before -27

//Calibration
#define CAL_MIN_LVL				2100		//Min calibration level to perform a reliable raindetection from a dry state

//Dry Level 
#define DRY_LVL					2430		//Ultimate dry level (new sensor)
#define WET_LVL					2530		//Dry sensor after long time rain

//Saturation detection
#define SAT_SMP_PERIOD			30			//Every 30 seconds a saturation sample
#define SAT_NBR_SMPL			3			//1.5 min.
#define SAT_LVL					4100
#define SAT_LVL_TOL				100			//Only negative tolerance

//Saturation detection with offsets
#define SAT_LVL_OFFSET_UP		1670		//When cap. value is above this offset, there no changement in cap. value anymore to detect rainintensity
#define SAT_LVL_OFFSET_DWN		1070			//When cap. value is below this offset, the sensor is accepted as being dry
#define SAT_MAX					4300		//Max value the cap. value can reach when the sensor is full of fluid.


#define FLASH_LT_VALUE_ADDRESS	0x2810
#define FLASH_LT_CAL_ADDRESS	0x2805


//Dirt detection
#define DIRT_LVL				3500		//Lower level to stay in short term working zone. Measure in long term algorithm.
#define DIRT_LVL_TOL			50
#define DIRT_LVL_MAX			3700		//Max. level to be in a zone where a accurate trendline could be defined.

//Dirt detection with offsets
#define DIRT_LVL_OFFSET_UP		900			//When cap. value is above this offset after the long term loggin period, the sensor is dirt
#define DIRT_LVL_OFFSET_MAX		1200	

//Calibration	
#define CAL_MIN_LVL				2100		//Min calibration level to perform a reliable raindetection from a dry state

#define FIFOLENGTH				12			//2 hours average 2 * 6 * 10min. (Third period


#define SNOW_BASE				2300
#define SNOW_OFFSET_SET			300
#define SNOW_OFFSET_CLEAR		80

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/
typedef enum
{
	STATEIDLE = 0,
	SATLEVEL = 1,
	STARTUP = 2,
	RUNNING = 3,
	TESTING = 4
}
SNS_STATE;

typedef enum
{
	SNS_INIT = 0,
	SNS_RUNNING = 1
}
SNS_TASK;

typedef enum
{
	TEMP_STATE_IDLE = 0,
	TEMP_STATE_CONFIG = 1,
	TEMP_STATE_RUNNING = 2
}
TEMP_STATE;

typedef enum
{
	WORKING_ZONE,				//Capacitance is inside the working zone where it is possible to perform normal detection
	DIRT,						//Capacitance has reached a level where it is difficult to define the trendline in capacitance value
	SATURATION					//Capacitance value is to high ==> Sensor is always active, independent of temperature
}
CAPACITANCE;

typedef struct  
{
	float current;
	float luxvalue;
}
APPLUXVALUE;

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
static uint32_t AppSensorGetCapSample(void);
static void AppSensorTask(const struct timer_task *const timer_task);
static float AppSensorSortMedian(int16_t* ar, uint8_t length);
static float AppSensorCalcRico(int16_t* ar, uint16_t x_offset, uint8_t length);
static void AppSensorFillFifo(uint32_t avg);
static void AppSensorRainAlgoritm(uint32_t avg);
static void AppSensorSnowAlgoritm(uint32_t avg);
static float AppSensorLinInter(float rx0, float rx1, float rx, float ry0, float ry1);
static uint16_t AppSensorGetLux(uint16_t adcvalue);
/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
TASK_HNDL    trend_task;
static float	applightvalue = 0.0;
static uint16_t	applighthysteresis = 1;

static int16_t temperature = 0;
static APPLUXVALUE ar_luxvalue[6]; 

static uint16_t readedvalue = 0x0000;
static uint16_t basevalue = 0x0000;

//Algorithm
//static TASK_HNDL    trend_task;
static uint32_t		avg;
static int16_t		ar_avg[PERIOD_20SEC];
static uint16_t		tensecondstail = 0;
static uint16_t		oneminutetail = 0;
static uint16_t		twominutetail = 0;
static uint16_t		tenminutetail = 0;
static uint16_t		thirtymintail = 0;
static uint16_t		sixtymintail = 0;

static int16_t		ar_med_calc[PERIOD_60MIN];

static int16_t		ri_avg[60];					//always calculate rico over 60 points (except when 10s.)
static uint8_t		ar_mask = 0x00;
static uint8_t		ar_flag = 0x00;
static uint8_t		rico_mask = 0x00;

static uint8_t		running_flag = 0;			//Flag to enable running proces
static uint8_t		ar_algo_flag = 0x00;		//Flag to enable algoritm calculation

static APP_SENS_MODUS	modus = MOD_IDLE;							//Modus van de rainsensor depends on rico
static APP_TEMP_MODUS	temp_modus = TEMP_IDLE;					//Temperature modus of the rainsensor depends on temperature
static CAPACITANCE		zone = WORKING_ZONE;
static CAPACITANCE		lt_zone = WORKING_ZONE;

static uint16_t		idle_counter = 0;
static uint16_t		idle_time = 0;
static bool			idle_time_define = false;

static uint8_t		fifohead = 0;
static uint8_t		fifotail = 0;
static uint32_t		a_fifo_data[FIFOLENGTH];
static bool			fifook = false;
static uint32_t		snow_avg = 0;
static SNOWSTATE	ar_snowstate[2];

static RAINSTATE	rainstate = IDLE;
static RAINSTATE	extrainstate = IDLE;
static uint8_t		testrainstate = 0;
static uint8_t		raindwn = IDLE;
static uint8_t		rainup = IDLE;

static uint32_t		lt_avg = 0;

static char			str_debug[30];

static uint16_t		ar_lt_calvalue;								//Dry state calibration value defined @ startup.
static uint16_t		ar_lt_leveldrift[LT_NBR_SMPL];				//Array used to define long term leveldrift
static uint32_t		ar_sat_level[SAT_NBR_SMPL];					//Array used to define if the sensor is in saturation mode.

/**********************************************************************************************************************/

/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/

/********************************************************************p**************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
static float AppSensorLinInter(float rx0, float rx1, float rx, float ry0, float ry1)
{
    float ry = 0.0;
    
    ry = ry0 * (1 - ((rx - rx0)/(rx1 - rx0))) + ry1 * ((rx - rx0)/(rx1 - rx0));
    
    return ry;
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void AppSensorFillFifo(uint32_t avg)
{
	uint8_t fifocntr = 0;
	
	if((modus == MOD_SNOW)||(modus == MOD_MOISTY))
	{
		a_fifo_data[fifohead] = avg;
		fifohead++;
		if(fifohead == FIFOLENGTH)
		{
			fifohead = 0;
			for(fifocntr = 0; fifocntr < FIFOLENGTH; fifocntr++)
			{
				snow_avg += a_fifo_data[fifocntr];
			}
			snow_avg = snow_avg / FIFOLENGTH;
			fifook = true;
		}
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
static uint16_t AppSensorGetLux(uint16_t adcvalue)
{
	uint8_t i = 0;

	for(i = 0; i < 5; i++)
	{
		if(adcvalue > 3723)
		{
			applighthysteresis = 1000;
			return 10000;
		}
		else
		{
			if((adcvalue >= ar_luxvalue[i].current) && (adcvalue < ar_luxvalue[i+1].current))
			{
				switch(i)
				{
					case 0: 
						applighthysteresis = 5;
						break;
					case 1:
						applighthysteresis = 100;
						break;
					case 2:
						applighthysteresis = 500;
						break;
					case 3:
						applighthysteresis = 1000;
						break;
					case 4:
						applighthysteresis = 1000;
						break;
					default:
						applighthysteresis = 10;
						break;
				}
				return AppSensorLinInter(ar_luxvalue[i].current, ar_luxvalue[i+1].current, (float)adcvalue, ar_luxvalue[i].luxvalue, ar_luxvalue[i+1].luxvalue);
			}
		}
		
	}
	return 0;
}
/*--------------------------------------------------------------------------------------------------------------------*/
static uint32_t AppSensorGetCapSample(void)
{
	uint8_t		avg_cntr = 0;
	uint32_t	avg = 0;
	uint16_t	ar_sample = 0;
	
	for(avg_cntr = 0; avg_cntr < NBR_SMPL_AVG; avg_cntr++)
	{
		stdMtch112Reading(MTCH_SENSOR0, &ar_sample);
		avg += ar_sample;
	}
	avg = avg / NBR_SMPL_AVG;				//Average value of #=NBR_SMPL_AVG samples
	return avg;
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void AppSensorTask(const struct timer_task *const timer_task)
{
	static uint16_t		ar_head = 0;
	static uint8_t		ar_avg_head = 0;
	static uint8_t		ar_debug[2];
	static bool			ar_head_flag = false;
	static uint16_t		ar_cntr = 0;
	static uint8_t		ar_avg_cntr = 0;
	static uint16_t		cntr = 0;
	uint8_t				i = 0;
	static uint16_t		lt_avg_cntr = 0;
	static uint16_t		sat_avg_cntr = 0;
	uint32_t			avg = 0;
	int16_t				med_avg = 0;
	static uint16_t		lt_avg_index = 0;
	static uint8_t		sat_avg_index = 0;
	
	static CAPACITANCE	zone_history = WORKING_ZONE;

	StdTaskStop(trend_task);
	avg = AppSensorGetCapSample();
	
#ifdef DEBUG_SENSOR
	//gpio_set_pin_level(PA04, false);
	AppCasambiDebug(2, &rico_mask, 1);
	ar_debug[0] = 3;						//Instellen ID
	AppCasambiDebug(7, ar_debug, 1);		
#endif
	
	ar_avg[ar_avg_head] = avg; 
	
	running_flag = 0x01;
	
	ar_head++;
	ar_avg_head++;
	
	if(ar_head > 10)
	{
		ar_head_flag = true;
	}
	if(ar_head_flag)
	{
		if(ar_avg_cntr <= (PERIOD_20SEC - 10))
		{
			*(ar_med_calc + ar_cntr) = AppSensorSortMedian(ar_avg + ar_avg_cntr, 10);	
			med_avg = *(ar_med_calc + ar_cntr) ;	
		}
		else
		{
			for(i = 0; i < 10; i++)
			{
				if((ar_avg_cntr + i) < PERIOD_20SEC)
				{
					*(ri_avg + i) = ar_avg[ar_avg_cntr + i];
				}
				else
				{
					*(ri_avg + i) = ar_avg[ar_avg_cntr + i - PERIOD_20SEC];
				}
			}
			*(ar_med_calc + ar_cntr) = AppSensorSortMedian(ri_avg, 10);
			med_avg = *(ar_med_calc + ar_cntr);	
		}
		ar_algo_flag = 0x01;
		ar_cntr++;
		ar_avg_cntr++;
	}
	sprintf(str_debug, "%u, %u\n\r", ar_head, med_avg);
	//DrvSciWrite(str_debug, strlen(str_debug));
#ifdef DEBUG_SENSOR
	ar_debug[0] = (uint8_t)(med_avg >> 8);
	ar_debug[1] = (uint8_t)(med_avg & 0x00FF);
	AppCasambiDebug(0, ar_debug, 2);
//	debugcntr++;
//	AppCasambiDebug(4, &debugcntr, 2);
#endif
	if(ar_head == PERIOD_60MIN)			ar_head = 0;		//Reset Head
	if(ar_avg_head == PERIOD_20SEC)		ar_avg_head = 0;	//Reset Avg Head (Average Capacitive values)
	if(ar_avg_cntr == PERIOD_20SEC)		ar_avg_cntr = 0;	//Reset Avg counter

	if(ar_cntr == PERIOD_10SEC)					ar_mask |= 1 << 0;				//Start sampling for 10 seconds rico
	if(ar_cntr == PERIOD_1MIN)					ar_mask |= 1 << 1;				//Start sampling for 1min. rico
	if(ar_cntr == PERIOD_2MIN)					ar_mask |= 1 << 2;				//Start sampling for 2min. rico
	if(ar_cntr == PERIOD_10MIN)					ar_mask |= 1 << 3;				//Start sampling for 10min. rico
	if(ar_cntr == PERIOD_30MIN)					ar_mask |= 1 << 4;				//Start sampling for 30min. rico
	if(ar_cntr == PERIOD_60MIN)					
	{
		ar_mask |= 1 << 5;														//Start sampling for 60min. rico	
		ar_cntr = 0;															//Only samples for the past 60min. were hold in array
	}
	if((ar_cntr % 2) == 0)						ar_flag |= 1 << 2;			//Flag for 2 minute algorithm
	if((ar_cntr % 10) == 0)						ar_flag |= 1 << 3;			//Flag for 10 minutes algorithm
	if((ar_cntr % 30) == 0)						ar_flag |= 1 << 4;			//Flag for 30 minutes algorithm			
	if((ar_cntr % 60) == 0)						ar_flag |= 1 << 5;			//Flag for 60 minutes algorithm
		
	if(idle_time_define == true)					//After 10 minutes the idle time definition starts if enabled
	{
		idle_counter++;
		if(idle_counter > idle_time)
		{
			modus = MOD_IDLE;
			extrainstate = IDLE;
			idle_counter = 0;
			idle_time_define = false;
		}
	}
	//Defining long term level drift (Every LT_SMP_PERIOD)
	if(lt_avg_cntr >= LT_SMPL_PERIOD)			//After 10 min
	{
		lt_avg_cntr = 0;
		ar_lt_leveldrift[lt_avg_index] = (uint16_t)avg;
		lt_avg_index++;
		lt_avg = 0;
		for(cntr = 0; cntr < LT_NBR_SMPL; cntr++)
		{
			lt_avg += ar_lt_leveldrift[cntr];
		}
		lt_avg = lt_avg / LT_NBR_SMPL;
		if(lt_avg_index == LT_NBR_SMPL)			//After 10 days
		{
			lt_avg_index = 0;
			if((lt_avg > (ar_lt_calvalue + DIRT_LVL_OFFSET_UP)) && (lt_avg <= (ar_lt_calvalue + DIRT_LVL_OFFSET_MAX)))
			{
				zone = DIRT;
			}
			else if((lt_avg > (ar_lt_calvalue + DIRT_LVL_OFFSET_MAX)) || (lt_avg > SAT_MAX))
			{
				zone = SATURATION;
				lt_zone = SATURATION;
			}
		}
	}
	lt_avg_cntr++;
	//Defining saturation (Every SAT_SMP_PERIOD)
	if(sat_avg_cntr >= SAT_SMP_PERIOD)			//Every 30 seconds
	{
		sat_avg_cntr = 0;
		ar_sat_level[sat_avg_index] = avg;	
		sat_avg_index++;
		if(sat_avg_index == SAT_NBR_SMPL)		//After 1.5 min.
		{
			sat_avg_index = 0;
			while(sat_avg_index < SAT_NBR_SMPL)
			{
				if((ar_sat_level[sat_avg_index] > (ar_lt_calvalue + SAT_LVL_OFFSET_UP)) || (ar_sat_level[sat_avg_index] > SAT_MAX)) sat_avg_index++;
				else break;
			}
			if(sat_avg_index == SAT_NBR_SMPL) 
			{
				modus = MOD_SAT;
				zone = SATURATION;				//By every sample there was a saturation level detected
			}
			sat_avg_index = 0;
			while(sat_avg_index < SAT_NBR_SMPL)
			{
				if((ar_sat_level[sat_avg_index] < (ar_lt_calvalue + SAT_LVL_OFFSET_DWN)) && (ar_sat_level[sat_avg_index] < SAT_MAX)) sat_avg_index++;
				else break;
			}
			if((sat_avg_index == SAT_NBR_SMPL)&&(zone == SATURATION))
			{
				zone = WORKING_ZONE;				//By every sample there was a normal working zone detected.
				modus = MOD_DRYING;
				ar_head = 0;
				ar_head_flag = false;
				ar_mask = 0x00;
				ar_cntr = 0;
				tensecondstail = 0;
				oneminutetail = 0;
				twominutetail = 0;
				tenminutetail = 0;
				thirtymintail = 0;
				sixtymintail = 0;
			}
			sat_avg_index = 0;
		}
	}
	sat_avg_cntr++;
	AppSensorFillFifo(avg);
	StdTaskStart(trend_task);
#ifdef DEBUG_SENSOR
	//gpio_set_pin_level(PA04, true);
#endif
}
/*--------------------------------------------------------------------------------------------------------------------*/
static float AppSensorSortMedian(int16_t* ar, const uint8_t length)
{
	uint8_t i, j;
	uint16_t key;
	int16_t		ar_med[20];

	float median;
	
	for(i = 0; i < length; i++)		//copy array
	{
		ar_med[i] = ar[i];
	}
	
	for(i = 1; i < length; i++)
	{
		key = ar_med[i];
		j = i - 1;
		while(j >= 0 && ar_med[j] > key)
		{
			ar_med[j + 1] = ar_med[j];
			j = j - 1;
		}
		ar_med[j + 1] = key;
	}
	
	if(length % 2 == 0)					//length is even
	{
		median = (ar_med[length >> 1] + ar_med[(length >> 1) + 1]) >> 1;		
	}
	else								//length is odd
	{
		median = ar_med[(length + 1) >> 1];
	}
	return median;
}	
/*--------------------------------------------------------------------------------------------------------------------*/
static float AppSensorCalcRico(int16_t* ar, uint16_t x_offset, uint8_t length)
{
	uint8_t index = 0;
	int64_t		x = 0;
	int64_t		sum_x = 0;
	int64_t		sum_y = 0;
	int64_t		sum_xx = 0;
	int64_t		sum_xy = 0;
	int64_t		temp1 = 0;
	int64_t		temp2 = 0;	
	float 		rico = 0.0;
	
	x = x_offset;
	
	while(index < length)
	{
		sum_xy = sum_xy + (x * *ar);
		sum_x = sum_x + x;
		sum_y = sum_y + *ar;
		sum_xx = sum_xx + (x * x);
		x = x + x_offset;
		ar++;
		index++;
	}
	
	temp1 = (length * sum_xy) - (sum_x * sum_y);
	temp2 = (length * sum_xx) - (sum_x * sum_x);
	rico = (float)temp1 / (float)temp2;
	
	return rico;
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void AppSensorRainAlgoritm(uint32_t avg)
{
	uint16_t			i = 0;
	float 				rico = 0.0;

	static uint8_t		dryingmask = 0x00;
	
#ifdef DEBUG_SENSOR
//gpio_set_pin_level(PA04, false);
#endif
	if(ar_algo_flag)
	{
		ar_algo_flag = 0x00;					//Clear flag 
	
		if(ar_mask & 0x01)						//10 seconds rico calculation
		{
			dryingmask |= 0x01;
			for(i = 0; i < PERIOD_10SEC; i++)
			{
				if((tensecondstail + i) >= PERIOD_60MIN)		ri_avg[i] = ar_med_calc[tensecondstail + i - PERIOD_60MIN];
				else											ri_avg[i] = ar_med_calc[tensecondstail + i];
			}
			tensecondstail++;
			if(tensecondstail == PERIOD_60MIN)	tensecondstail = 0;
			rico = AppSensorCalcRico(ri_avg, 1, PERIOD_10SEC);
			if(rico > RICO_10SEC)	
			{
				modus = MOD_RAIN;
				rico_mask |= 0x01;
				//return;
			}
			else 
			{
				dryingmask &= ~0x01;
				rico_mask &= ~0x01;
			}
			
		}
		if(ar_mask & 0x02)						//1 minute rico calculation (No ar_flag because every second there is a rico calculation needed)
		{	
			dryingmask |= 0x02;
			for(i = 0; i < SMPL_RICO; i++)
			{
				if((oneminutetail + i) >= PERIOD_60MIN)			ri_avg[i] = ar_med_calc[oneminutetail + i - PERIOD_60MIN];
				else											ri_avg[i] = ar_med_calc[oneminutetail + i];
			}
			oneminutetail++;
			if(oneminutetail == PERIOD_60MIN) oneminutetail = 0;
			rico = AppSensorCalcRico(ri_avg, 1, SMPL_RICO);
			if((rico > RICO_1MIN_RISING)||((avg > RICO_LVL) && (rico > RICO_1MIN_RISING_LVL))) 
			{
				modus = MOD_RAIN;
				rico_mask |= 0x02;
				//return;
			}
			else if(rico < RICO_1MIN_FALLING) 
			{
				modus = MOD_RAIN;
				rico_mask |= 0x02;
				//return;
			}
			else 
			{
				dryingmask &= ~0x02;
				rico_mask &= ~0x02;
			}
		}
		if((ar_mask & 0x04) && (ar_flag & 0x04))						//2 minutes rico calculation
		{
			dryingmask |= 0x04;
			ar_flag &= ~0x04;
			for(i = 0; i < SMPL_RICO; i++)
			{
				if((twominutetail + (2 * i)) >= PERIOD_60MIN)	ri_avg[i] = ar_med_calc[twominutetail + (2 * i) - PERIOD_60MIN];
				else											ri_avg[i] = ar_med_calc[twominutetail + (2 * i)];
			}
			twominutetail += 2;
			if(twominutetail >= PERIOD_60MIN) twominutetail -= PERIOD_60MIN;
			rico = AppSensorCalcRico(ri_avg, 2, SMPL_RICO);
			if((rico > RICO_2MIN_RISING)||((avg > RICO_LVL) && (rico > RICO_2MIN_RISING_LVL))) 
			{
				modus = MOD_RAIN;
				rico_mask |= 0x04;
				//return;
			}
			else if(rico < RICO_2MIN_FALLING)
			{
				modus = MOD_RAIN;
				rico_mask |= 0x04;
				//return;
			}
			else 
			{
				dryingmask &= ~0x04;
				rico_mask &= ~0x04;
			}
		}
		if((ar_mask & 0x08)	&& (ar_flag & 0x08))				//10 minutes rico calculation
		{
			dryingmask |= 0x08;
			ar_flag &= ~0x08;
			for(i = 0; i < SMPL_RICO; i++)
			{
				if((tenminutetail + (10 * i)) >= PERIOD_60MIN)		ri_avg[i] = ar_med_calc[tenminutetail + (10 * i) - PERIOD_60MIN];
				else												ri_avg[i] = ar_med_calc[tenminutetail + (10 * i)];
			}
			tenminutetail += 10;
			if(tenminutetail >= PERIOD_60MIN) tenminutetail -= PERIOD_60MIN;
			rico = AppSensorCalcRico(ri_avg, 10, SMPL_RICO);
			if((rico > RICO_10MIN_RISING)||((avg > RICO_LVL) && (rico > RICO_10MIN_RISING_LVL)))
			{
				modus = MOD_RAIN;
				rico_mask |= 0x08;
				//return;
			}
			else if(rico < RICO_10MIN_FALLING) 
			{
				modus = MOD_RAIN;
				rico_mask |= 0x08;
				//return;
			}
			else 
			{
				dryingmask &= ~0x08;
				rico_mask &= ~0x08;
			}
		}
		if((ar_mask & 0x10) && (ar_flag & 0x10))			//30 minutes rico calculation
		{
			dryingmask |= 0x10;
			ar_flag &= ~0x10;
			for(i = 0; i < SMPL_RICO; i++)
			{
				if((thirtymintail + (30 * i)) >= PERIOD_60MIN)		ri_avg[i] = ar_med_calc[thirtymintail + (30 * i) - PERIOD_60MIN];
				else												ri_avg[i] = ar_med_calc[thirtymintail + (30 * i)];
			}
			thirtymintail += 30;
			if(thirtymintail >= PERIOD_60MIN) thirtymintail -= PERIOD_60MIN; 
			rico = AppSensorCalcRico(ri_avg, 30, SMPL_RICO);
			if(rico > RICO_30MIN_RISING) 
			{
				modus = MOD_RAIN;
				rico_mask |= 0x10;
				//return;
			}
			else 
			{
				dryingmask &= ~0x10;
				rico_mask &= ~0x10;
			}
		}
		if((ar_mask & 0x20)	&& (ar_flag & 0x20))						//60 minutes rico calculation
		{
			dryingmask |= 0x20;
			ar_flag &= ~0x20;											//disable ar_flag to disable rico calculation for 60s.
			for(i = 0; i < SMPL_RICO; i++)
			{
				if((sixtymintail + (60 * i)) >= PERIOD_60MIN)		ri_avg[i] = ar_med_calc[sixtymintail + (60 * i) - PERIOD_60MIN];
				else                                                ri_avg[i] = ar_med_calc[sixtymintail + (60 * i)];
			}
			sixtymintail += 60;
			if(sixtymintail >= PERIOD_60MIN) sixtymintail -= PERIOD_60MIN;
			rico = AppSensorCalcRico(ri_avg, 60, SMPL_RICO);
			if(rico > RICO_60MIN_RISING)
			{
				modus = MOD_RAIN;
				rico_mask |= 0x20;
				//return;
			}
			else 
			{
				dryingmask &= ~0x20;
				rico_mask &= ~0x20;
			}
		}
	}
	
	//dryingmask &= ~ar_mask;
	
	switch(modus)
	{
		case MOD_IDLE:
			break;
		case MOD_RAIN:
			if(dryingmask == 0)	modus = MOD_DRYING;
			break;
		default:
			break;
	}
#ifdef DEBUG_SENSOR
//gpio_set_pin_level(PA04, true);
#endif

}
/*--------------------------------------------------------------------------------------------------------------------*/
static void AppSensorSnowAlgoritm(uint32_t avg)
{
	if(!fifook) snow_avg = SNOW_BASE;
	
	if(avg > (snow_avg + SNOW_OFFSET_SET))
	{
		if(temp_modus == TEMP_MOISTY)
		{
			modus = MOD_MOISTY;
			rainstate = extrainstate = ar_snowstate[0];	
		}
		if(temp_modus == TEMP_SNOW)
		{
			modus = MOD_SNOW;
			rainstate = extrainstate = ar_snowstate[1];
		}
	}
	else if((avg < snow_avg + SNOW_OFFSET_CLEAR))
	{
		modus = MOD_DRYING;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void AppSensorInit(void)
{	
	//12-bit adc (not left adjusted)
	ar_luxvalue[0].current = 0;
	ar_luxvalue[0].luxvalue = 60;		//Hysteresis = 5
	ar_luxvalue[1].current = 6;
	ar_luxvalue[1].luxvalue = 100;		//Hysteresis = 10
	ar_luxvalue[2].current = 37;
	ar_luxvalue[2].luxvalue = 700;		//Hysteresis = 100
	ar_luxvalue[3].current = 372;
	ar_luxvalue[3].luxvalue = 7000;		//Hysteresis = 1000
	ar_luxvalue[4].current = 3723;
	ar_luxvalue[4].luxvalue = 10000;

	gpio_set_pin_direction(PA22, GPIO_DIRECTION_OUT);		//uP_Out output initialisation
	gpio_set_pin_level(PA22, false);
	gpio_set_pin_direction(PA23, GPIO_DIRECTION_OUT);		//uP_REL output initialisation
	gpio_set_pin_level(PA23, false);
	
	ar_snowstate[0] = MOISTY;
	ar_snowstate[1] = DRYSNOW;
	
	trend_task = StdTaskRegisterTask(SAMPLE_PERIOD, AppSensorTask);
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint16_t AppSensorGetRainValue(void)
{
    stdMtch112Reading(MTCH_SENSOR0, &readedvalue);		
    return readedvalue;
}
/*--------------------------------------------------------------------------------------------------------------------*/
RAINSTATE AppSensorGetRainState(void)
{
    return extrainstate;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppSensorHandler(void)
{
	uint16_t sns_ldr = 0;
	uint16_t ctr = 0;
	static uint32_t		addr = 0x0000000;
	static SNS_STATE sns_state = STATEIDLE;
	static uint8_t cal_ctrl = 0x00;			//Check if sensor is calibrated
	static uint8_t debug = 0x00;
	static uint8_t vm_cntr = 0;
	
	switch(sns_state)
	{
		case STATEIDLE:
			addr = FLASH_LT_CAL_ADDRESS;
			lt_avg = 2400;
			//DrvFlashRead(addr, &cal_ctrl, 1);
			//if(cal_ctrl != 0x01)
			//{
				//ar_lt_calvalue = lt_avg = ar_lt_leveldrift[0] = AppSensorGetCapSample();
				//if(lt_avg > CAL_MIN_LVL)
				//{
					//for(ctr = 1; ctr < LT_NBR_SMPL; ctr++)
					//{
						//ar_lt_leveldrift[ctr] = ar_lt_leveldrift[0];			//Fill leveldrift array with an initial value
					//}
					//addr = FLASH_LT_VALUE_ADDRESS;
					//DrvFlashWrite(addr, ar_lt_leveldrift, 2);
					//addr = FLASH_LT_CAL_ADDRESS;
					//cal_ctrl = 0x01;
					//DrvFlashWrite(addr, &cal_ctrl, 1);
					//sns_state = SATLEVEL;
				//}
				//else
				//{
					//AppCasambiActivity(false, ACTIVITY_ERRRAIN, true);
					//sns_state = STATEIDLE;
				//}
			//}
			//else
			//{
				//addr = FLASH_LT_VALUE_ADDRESS;
				//DrvFlashRead(addr, ar_lt_leveldrift, 2);
				//ar_lt_calvalue = lt_avg = ar_lt_leveldrift[0];
				//for(ctr = 0; ctr < LT_NBR_SMPL; ctr++)
				//{
				//ar_lt_leveldrift[ctr] = lt_avg;			//Fill leveldrift array with an initial value
				//}
				//sns_state = SATLEVEL;
			//}
			ar_lt_calvalue = 2400;
			sns_state = SATLEVEL;
			break;
		case SATLEVEL:
			ar_sat_level[0] = lt_avg;
			for(ctr = 1; ctr < SAT_NBR_SMPL; ctr++)
			{
				ar_sat_level[ctr] = ar_sat_level[0];					//Fill array for detection of saturation level
			}
			sns_state = STARTUP;
			StdTaskStart(trend_task);
			break;
		case STARTUP:
			sns_state = RUNNING;
			//AppCasambiActivity(false, ACTIVITY_RDY, true);				//Set activity register
			break;
		case RUNNING:
			if(running_flag)												//Every second running_flag is set
			{
				running_flag = 0;
				vm_cntr++;
				if(vm_cntr >= 10)
				{
					AppCasambiSendVM1();
					vm_cntr = 0;
				}
				sns_ldr = DrvAdc_0_read();			//Read LDR value.
				applightvalue = AppSensorGetLux(sns_ldr);
				if(lt_zone == SATURATION)
				{
					rainstate = extrainstate = SATURATED;
					idle_time_define = false;
					idle_counter = 0;
				}
				else
				{
					switch(zone)
					{
					case WORKING_ZONE:
					case DIRT:
						if(zone == DIRT)
						{
						AppCasambiActivity(false, ACTIVITY_DIRT, true);
						}
						//Short term working zone depending on temperature
						switch(temp_modus)
						{
							case TEMP_RAIN:
								AppSensorRainAlgoritm(avg);
								break;
							case TEMP_MOISTY:
								AppSensorRainAlgoritm(avg);
								AppSensorSnowAlgoritm(avg);
								break;
							case TEMP_SNOW:
								AppSensorSnowAlgoritm(avg);
								break;
							default:
								break;
						}
						break;
					case SATURATION:
						idle_time_define = false;
						idle_counter = 0;
					break;
					default:
					break;
					}
				}
				switch(modus)
				{
					case MOD_IDLE:
						rainstate = extrainstate = IDLE;
						AppCasambiRainsensorState(false, IDLE);
						idle_time_define = false;
						idle_counter = 0;
						break;
					case MOD_RAIN:
						rainstate = extrainstate = INTSRAIN;
						AppCasambiRainsensorState(false, extrainstate);
						idle_time_define = false;
						idle_counter = 0;
						break;
					case MOD_MOISTY:
					case MOD_SNOW:
						AppCasambiRainsensorState(false, extrainstate);
						idle_time_define = false;
						idle_counter = 0;
						break;
					case MOD_DRYING:
						if(AppCasambiRainsensorIdleTimeValue(true, 0) > 10)
						{
							idle_time = AppCasambiRainsensorIdleTimeValue(true, 0) * 60;
						}
						else idle_time = 10 * 60;
						idle_time_define = true;
						break;
					case MOD_SAT:
						rainstate = extrainstate = SATURATED;
						AppCasambiRainsensorState(false, extrainstate);
						idle_time_define = false;
						idle_counter = 0;
						//sprintf(str_debug, "MOD_SATURATED\r\n");
						//DrvSciWrite(str_debug, strlen(str_debug));
						break;
					default:
						idle_time_define = false;
						break;
				}
				AppCasambiTemperatureValue(false, AppSensorGetTempValue());
				AppCasambiLightValue(false, applightvalue);
				raindwn = AppCasambiRainsensorTresholdValueDown(true, 0);
				rainup = AppCasambiRainsensorTresholdValueUp(true, 0);
				
				if((extrainstate>= raindwn) && (extrainstate <= rainup))
				{
					AppIORelais(1);
					AppIODetRain(1);
					if(temp_modus == TEMP_RAIN)			
					{
						AppCasambiActivity(false, ACTIVITY_RAIN, true);
#ifdef DEBUG_SENSOR
							debug = 1;
							AppCasambiDebug(3, &debug, 1);
#endif					
					}
					else if(temp_modus == TEMP_MOISTY)	
					{
						AppCasambiActivity(false, ACTIVITY_RAIN, true);
#ifdef DEBUG_SENSOR
						debug = 1;
						AppCasambiDebug(3, &debug, 1);
#endif
					}
					else if(temp_modus == TEMP_SNOW)		
					{
						AppCasambiActivity(false, ACTIVITY_SNOW, true);
#ifdef DEBUG_SENSOR					
						debug = 2;
						AppCasambiDebug(3, &debug, 1);
#endif					
					}
				}
				else if(extrainstate == SATURATED)
				{
					AppIORelais(1);
					AppIODetRain(1);
					AppCasambiActivity(false, ACTIVITY_RAIN, true);
					AppCasambiActivity(false, ACTIVITY_SNOW, true);
#ifdef DEBUG_SENSOR
					debug = 3;
					AppCasambiDebug(3, &debug, 1);
#endif
				}
				else
				{
					AppIORelais(0);
					AppIODetRain(0);
					AppCasambiActivity(false, ACTIVITY_RAIN, false);
					AppCasambiActivity(false, ACTIVITY_SNOW, false);
#ifdef DEBUG_SENSOR
					debug = 0;
					AppCasambiDebug(3, &debug, 1);
#endif
				}
				if(AppCasambiTest(true, &testrainstate))
				{
					sns_state = TESTING;					//Check if testmode is necessary
					StdTaskStop(trend_task);
					if(((RAINSTATE)testrainstate >= MIST) && ((RAINSTATE)testrainstate <= SHOWER))	modus = MOD_RAIN;
					else if(((SNOWSTATE)testrainstate >= MOISTY) && ((SNOWSTATE)testrainstate <= DRYSNOW)) modus = MOD_SNOW;
				}
			}
			break;
		case TESTING:
			raindwn = AppCasambiRainsensorTresholdValueDown(true, 0);
			rainup = AppCasambiRainsensorTresholdValueUp(true, 0);
			if((testrainstate >= raindwn) && (testrainstate <= rainup)){
				AppIORelais(1);
				AppIODetRain(1);
				if(temp_modus == TEMP_RAIN) AppCasambiActivity(false, ACTIVITY_RAIN, true);
				else if(temp_modus == TEMP_SNOW) AppCasambiActivity(false, ACTIVITY_SNOW, true);
				extrainstate = testrainstate;
			}
			else if(testrainstate == SATURATED){
				AppIORelais(1);
				AppIODetRain(1);
				AppCasambiActivity(false, ACTIVITY_RAIN, true);
				AppCasambiActivity(false, ACTIVITY_SNOW, true);
			}
			else{
				AppIORelais(0);
				AppIODetRain(0);
				if(temp_modus == TEMP_RAIN) AppCasambiActivity(false, ACTIVITY_RAIN, false);
				else if(temp_modus == TEMP_SNOW) AppCasambiActivity(false, ACTIVITY_SNOW, false);
				extrainstate = testrainstate;
			}
			if(!AppCasambiTest(true, &testrainstate)){
				sns_state = RUNNING;
				StdTaskStart(trend_task);									//Restart normal sampling
				extrainstate = rainstate = IDLE;
				AppCasambiRainsensorState(false, extrainstate);
				if(temp_modus == TEMP_RAIN) AppCasambiActivity(false, ACTIVITY_RAIN, false);
				else if(temp_modus == TEMP_SNOW) AppCasambiActivity(false, ACTIVITY_SNOW, false);
				temp_modus = TEMP_IDLE;
			}
			AppCasambiRainsensorState(false, testrainstate);
			break;
		default:
			sns_state = STATEIDLE;
			break;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint16_t AppSensorGetLightValue(void)
{
	return (uint16_t)(applightvalue);
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint16_t AppSensorGetLightHysteresis(void)
{
	return applighthysteresis;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint16_t AppSensorGetBaseValue(void)
{
    stdMtch112Baseline(MTCH_SENSOR0, &basevalue);
    return (uint16_t)(basevalue);
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint32_t AppSensorGetTempValue(void)
{	
	static TEMP_STATE tempstate = TEMP_IDLE;
	static int16_t inittemperature = 0;
	if(StdDS18S20Handler())
	{
		switch(tempstate)
		{
			case TEMP_STATE_IDLE:
				tempstate = TEMP_STATE_CONFIG;
				break;
			case TEMP_STATE_CONFIG:
				inittemperature = StdDS18S20GetTemperature();
				tempstate = TEMP_STATE_RUNNING;
				break;
			case TEMP_STATE_RUNNING:
				if((StdDS18S20GetTemperature() < (inittemperature + 5))&&(StdDS18S20GetTemperature() > (inittemperature - 5)))
				{
					temperature = inittemperature = StdDS18S20GetTemperature();
					temperature -= 3;											//Sort of temperature calibration
					if(temperature < 0)
					{
						temp_modus = TEMP_SNOW;
						//sprintf(str_debug, "TEMP_SNOW\r\n");
						//DrvSciWrite(str_debug, strlen(str_debug));
						temperature = abs(temperature);
						temperature >>= 1;
						
						temperature |= 0x80;
						return temperature;
					}
					if((temperature >= MOD_TEMP_FREEZING) && (temperature <= (MOD_TEMP_MOISTY * 2)))
					{
						temp_modus = TEMP_MOISTY;
						//sprintf(str_debug, "TEMP_MOISTY\r\n");
						//DrvSciWrite(str_debug, strlen(str_debug));
						temperature >>=1;
						temperature &= ~0x80;
						return temperature;
					}
					if(temperature > (MOD_TEMP_MOISTY * 2))
					{
						temp_modus = TEMP_RAIN;
						//sprintf(str_debug, "TEMP_RAIN\r\n");
						//DrvSciWrite(str_debug, strlen(str_debug));
						temperature >>=1;
						temperature &= ~0x80;
						return temperature;
					}
				}
				return temperature;
				break;
			default:
				break;
		}

	}
	return temperature;
}
/*--------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/


