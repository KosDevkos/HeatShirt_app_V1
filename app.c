/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/


/* MY PROGRESS
 *
 * Changed DEBUG_LEVEL to 1 in app.h
 *
 * Added I2CSPM.c
 *
 * initialized I2C in app.c
 *
 * Added mlx90632.c, mlx90632.h, mlx90632_depends.h
 *
 * Wrote functions mlx90632_i2c_read(), mlx90632_i2c_write() [adapted from 8bit to 16bit] and a temporary usleep() to mlx90632.c
 *
 * Downloaded constants from the sensor
 *
 * Changed BITS_PER_LONG to 32 in mlx90632.h
 *
 * Made the compiler printLof() floats in the project
 *
 */


/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

// include I2C SPM driver header file
#include "i2cspm.h"
#include "em_gpio.h"

#include "app.h"

#include "mlx90632.h"
#include "mlx90632_depends.h"

#include "math.h"

// include timer library for PWM
#include "em_timer.h"


/////////////// SENSOR ADDRESSES
uint16_t Sensor_GND = 0x3A << 1;
uint16_t Sensor_VDD = 0x3B << 1;

/////////////// PWM DEFINE
// this PWM_FREQ = 65000 creates about 1kHz signal.
#define PWM_FREQ 65000
//
#define MOVING_AVG_BUFFER 5

/// void printTime();

// TimeStamp fuction Prototype
uint32_t GetTimeStamp();

uint8_t GetPID(float set_point, double object_t, float p_scalar, float i_scalar, float d_scalar, float *proportional,float *integral,float *derivative, float *previous_error);



////////// GLOBAL VARIABLES



int32_t ret = 0;

uint8_t front_TR_PWM_Local= 5;
uint8_t back_TR_PWM_Local= 5;
uint8_t front_TR_PWM_In;
uint8_t back_TR_PWM_In;

double ambient_VDD;
double ambient_GND;
double object_VDD;
double object_GND;
uint8_t ambient_hex_VDD[3];
uint8_t ambient_hex_GND[3];
uint8_t object_hex_VDD[3];
uint8_t object_hex_GND[3];

double frontDesiredTemp = 32;
double backDesiredTemp = 32;

// PID Var
float proportional_VDD = 0;
float integral_VDD = 0;
float derivative_VDD = 0;
float previous_error_VDD = 0;
float proportional_GND = 0;
float integral_GND = 0;
float derivative_GND = 0;
float previous_error_GND = 0;
float PforPID = 0;
float IforPID = 0;
float DforPID = 0;
int8_t sendPID_arr[6];




 //////////// MOVING AVARAGE FILTER var
 int pos = 0;
 double newAvg_object_VDD = 32;
 double newAvg_object_GND = 32;
 double sum_VDD = 0;
 double sum_GND = 0;
 double arrNumbers_VDD[MOVING_AVG_BUFFER] = {0};
 double arrNumbers_GND[MOVING_AVG_BUFFER] = {0};


uint32_t deltaTimeStamp;
uint32_t currentTimeStamp;
uint32_t startOfRecordingTimeStamp = 0;
uint8_t timeStampArr[4];

uint8_t isRecording = 0;
uint8_t modeOfOperation = 0;


I2C_TransferReturn_TypeDef transfer_status;

int16_t object_new_raw_GND;
int16_t object_old_raw_GND;
int16_t ambient_new_raw_GND;
int16_t ambient_old_raw_GND;
int16_t object_new_raw_VDD;
int16_t object_old_raw_VDD;
int16_t ambient_new_raw_VDD;
int16_t ambient_old_raw_VDD;




/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();

  /* Initialize stack */
  gecko_init(pconfig);

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending()) {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

        bootMessage(&(evt->data.evt_system_boot));


 ////////////// VARIABLES DEFINITIONS



///////////// PWM TIMERS SETUP


        /* Enable clock for TIMER0 module */
        CMU_ClockEnable(cmuClock_TIMER0, true);

        /* Initialize pins used for PWM */
        GPIO_PinModeSet(gpioPortC, 10, gpioModePushPull, 0);
        ///!!!GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
		GPIO_PinModeSet(gpioPortA, 4, gpioModePushPull, 0);

	    /* Route pins to timer */
	    // $[TIMER0 I/O setup]
	    /* Set up CC0 */
		///!!!TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0 & (~_TIMER_ROUTELOC0_CC0LOC_MASK))
		///!!!        | TIMER_ROUTELOC0_CC0LOC_LOC0;    /// set to location 0 (for P0)
	    TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0 & (~_TIMER_ROUTELOC0_CC0LOC_MASK))
        | TIMER_ROUTELOC0_CC0LOC_LOC15;    /// set to location 0 (for PC10!!!!!!!!!)  P12 on dev board
	    TIMER0->ROUTEPEN = TIMER0->ROUTEPEN | TIMER_ROUTEPEN_CC0PEN;
	    /* Set up CC1 */
	    TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0 & (~_TIMER_ROUTELOC0_CC1LOC_MASK))
	            | TIMER_ROUTELOC0_CC1LOC_LOC3;    /// set to location 0 (for PA4)  P14 on dev board
	    TIMER0->ROUTEPEN = TIMER0->ROUTEPEN | TIMER_ROUTEPEN_CC1PEN;
	    // [TIMER0 I/O setup]$



	    /* Select CC channel parameters */
	      TIMER_InitCC_TypeDef timerCCInit =
	      {
	        .eventCtrl  = timerEventEveryEdge,
	        .edge       = timerEdgeBoth,
	        .prsSel     = timerPRSSELCh0,
	        .cufoa      = timerOutputActionNone,
	        .cofoa      = timerOutputActionNone,
	        .cmoa       = timerOutputActionToggle,
	        .mode       = timerCCModePWM,
	        .filter     = false,
	        .prsInput   = false,
	        .coist      = false,
	        .outInvert  = false,
	      };

	      /* Configure CC channel 0 */
	      TIMER_InitCC(TIMER0, 0, &timerCCInit);
	      /* Configure CC channel 1 */
	      TIMER_InitCC(TIMER0, 1, &timerCCInit);
	      /* Set Top Value */
	      TIMER_TopSet(TIMER0, CMU_ClockFreqGet(cmuClock_HFPER)/PWM_FREQ);
	      /* Set compare value starting at 0 - it will be incremented in the interrupt handler */
	      /////// (DC=591, 99.2%=590)
	      TIMER_CompareBufSet(TIMER0, 0, 1);
	      TIMER_CompareBufSet(TIMER0, 1, 1);


	      /* Select timer parameters */
	      TIMER_Init_TypeDef timerInit =
	      {
	        .enable     = true,
	        .debugRun   = true,
	        .prescale   = timerPrescale64,
	        .clkSel     = timerClkSelHFPerClk,
	        .fallAction = timerInputActionNone,
	        .riseAction = timerInputActionNone,
	        .mode       = timerModeUp,
	        .dmaClrAct  = false,
	        .quadModeX4 = false,
	        .oneShot    = false,
	        .sync       = false,
	      };

	     // /* Enable overflow interrupt */
	     // TIMER_IntEnable(TIMER0, TIMER_IF_OF);
	     // /* Enable TIMER0 interrupt vector in NVIC */
	     /// NVIC_EnableIRQ(TIMER0_IRQn);

	      /* Configure timer */
	      TIMER_Init(TIMER0, &timerInit);







///////////// I2C SETUP
		  CMU_ClockEnable(cmuClock_I2C0, true); /// ???????????????


		  I2CSPM_Init_TypeDef myi2cinit = I2CSPM_INIT_DEFAULT;
		  myi2cinit.sclPort = gpioPortC;
		  myi2cinit.sclPin = 6;
		  myi2cinit.portLocationScl = 10;
		  myi2cinit.sdaPort = gpioPortC;
		  myi2cinit.sdaPin = 8;
		  myi2cinit.portLocationSda = 13;




		   I2CSPM_Init(&myi2cinit);




		usleep(10,10);


///////// I2C SESORS CONSTANTS EXTRACTION

		int32_t P_T;
		uint16_t P_T_MS;
		uint16_t P_T_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_T, &P_T_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_T+1, &P_T_MS);
		P_T = (P_T_MS <<16) | P_T_LS;
		printLog("	P_T is %ld or %X\n\n\r",P_T,P_T);


		int32_t P_R;
		uint16_t P_R_MS;
		uint16_t P_R_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_R, &P_R_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_R +1, &P_R_MS);
		P_R = (P_R_MS <<16) | P_R_LS;
		printLog("	P_R is %ld or %X\n\n\r",P_R,P_R);

		int32_t P_G ;
		uint16_t P_G_MS;
		uint16_t P_G_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_G, &P_G_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_G + 1, &P_G_MS);
		P_G = (P_G_MS <<16) | P_G_LS;
		printLog("	P_G is %ld or %X\n\n\r",P_G,P_G);

		int32_t P_O ;
		uint16_t P_O_MS;
		uint16_t P_O_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_O, &P_O_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_P_O+1, &P_O_MS);
		P_O = (P_O_MS <<16) | P_O_LS;
		printLog("	P_O is %ld or %X\n\n\r",P_O,P_O);

		uint16_t Gb_UINT;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Gb, &Gb_UINT);
		int16_t Gb = Gb_UINT;
		printLog("	Gb is %d or %X\n\n\r",Gb,Gb);

		uint16_t Ka_UINT ;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Ka, &Ka_UINT);
		int16_t Ka = Ka_UINT;
		printLog("	Ka is %d or %X\n\n\r",Ka,Ka);

		int32_t Ea ;
		uint16_t Ea_MS;
		uint16_t Ea_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Ea, &Ea_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Ea+1, &Ea_MS);
		Ea = (Ea_MS <<16) | Ea_LS;
		printLog("	Ea is %ld or %X\n\n\r",Ea,Ea);

		int32_t Eb ;
		uint16_t Eb_MS;
		uint16_t Eb_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Eb, &Eb_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Eb+1, &Eb_MS);
		Eb = (Eb_MS <<16) | Eb_LS;
		printLog("	Eb is %ld or %X\n\n\r",Eb,Eb);

		int32_t Ga ;
		uint16_t Ga_MS;
		uint16_t Ga_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Ga, &Ga_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Ga+1, &Ga_MS);
		Ga = (Ga_MS <<16) | Ga_LS;
		printLog("	Ga is %ld or %X\n\n\r",Ga,Ga);

		int32_t Fa ;
		uint16_t Fa_MS;
		uint16_t Fa_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Fa, &Fa_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Fa+1, &Fa_MS);
		Fa = (Fa_MS <<16) | Fa_LS;
		printLog("	Fa is %ld or %X\n\n\r",Fa,Fa);

		int32_t Fb ;
		uint16_t Fb_MS;
		uint16_t Fb_LS;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Fb, &Fb_LS);
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Fb+1, &Fb_MS);
		Fb = (Fb_MS <<16) | Fb_LS;
		printLog("	Fb is %ld or %X\n\n\r",Fb,Fb);

		uint16_t Ha_UINT ;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Ha, &Ha_UINT);
		uint16_t Ha = Ha_UINT;
		printLog("	Ha is %d or %X\n\n\r",Ha,Ha);

		uint16_t Hb_UINT ;
		mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_EE_Hb, &Hb_UINT);
		int16_t Hb = Hb_UINT;
		printLog("	Hb is %d or %X\n\n\r",Hb,Hb);

		printLog("Im in app.c\n\n\r");

	    uint16_t reg_status;

	    /// ADDING ONE BIT TO A STATUS REGISTER??????????
	    mlx90632_i2c_read(I2C0,Sensor_VDD,MLX90632_REG_STATUS, &reg_status);
			//printLog("Im in mlx90632_start_measurement - REG_STATUS_1 IS: %x\n\n\r",reg_status);
	    mlx90632_i2c_write(I2C0,Sensor_VDD,MLX90632_REG_STATUS, reg_status | 0x0100);






        /* Set advertising parameters. 100ms advertisement interval.
         * The first parameter is advertising set handle
         * The next two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6).
         * The last two parameters are duration and maxevents left as default. */
        gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);


        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        printLog("boot event - starting advertising\r\n");

        break;

      case gecko_evt_le_connection_opened_id:

        printLog("connection opened\r\n");
        gecko_cmd_hardware_set_soft_timer(50000,0,0);

        /* Read sensor EEPROM registers needed for calcualtions */
        /* Now we read current ambient and object temperature */


        break;

      case gecko_evt_hardware_soft_timer_id:

        if (evt->data.evt_hardware_soft_timer.handle == 0) {

	       	printLog("front_TR_PWM is : %d \n\r", front_TR_PWM_Local);
	       	printLog("back_TR_PWM is : %d \n\r", back_TR_PWM_Local);
       //////////////// VDD SENSOR READING (AMBIENT AND OBJECT)
        				ret = mlx90632_read_temp_raw(Sensor_VDD, &ambient_new_raw_VDD, &ambient_old_raw_VDD,
        											 &object_new_raw_VDD, &object_old_raw_VDD);
        				if(ret < 0){
        					/* Something went wrong - abort */
        					printLog("error is : %ld\n\r", ret);
        				}
        				/* Now start calculations (no more i2c accesses) */
        				/* Calculate ambient temperature */
        						//printLog("ambient_new_raw in mlx90632_read_temp_raw() is %d or %x\n\n\r",ambient_new_raw,ambient_new_raw);
        						//printLog("ambient_old_raw in mlx90632_read_temp_raw() is %d or %x\n\n\r",ambient_old_raw,ambient_old_raw);

        				ambient_VDD = mlx90632_calc_temp_ambient(ambient_new_raw_VDD, ambient_old_raw_VDD, P_T, P_R, P_G, P_O, Gb);
        				/* Get preprocessed temperatures needed for object temperature calculation */
        				double pre_ambient_VDD = mlx90632_preprocess_temp_ambient(ambient_new_raw_VDD, ambient_old_raw_VDD, Gb);
        				double pre_object_VDD = mlx90632_preprocess_temp_object(object_new_raw_VDD, object_old_raw_VDD,ambient_new_raw_VDD, ambient_old_raw_VDD,Ka);
        				/* Calculate object temperature */
        				object_VDD = mlx90632_calc_temp_object(pre_object_VDD, pre_ambient_VDD, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
        				printLog("ambient_VDD is : %f - %x    object is : %f - %x \n\r", ambient_VDD,ambient_VDD,object_VDD,object_VDD);
        				///!!!printLog("int ambient_VDD is : %d - %x    int object is : %d - %x \n\r", (int) ambient,(int) ambient,(int) object,(int) object);




       //////////////// GND SENSOR READING (AMBIENT AND OBJECT)
        				ret = mlx90632_read_temp_raw(Sensor_GND, &ambient_new_raw_GND, &ambient_old_raw_GND,
        											 &object_new_raw_GND, &object_old_raw_GND);
        				if(ret < 0){
        					/* Something went wrong - abort */
        					printLog("error is : %ld\n\r", ret);
        				}
        				/* Now start calculations (no more i2c accesses) */
        				/* Calculate ambient temperature */
        						//printLog("ambient_new_raw in mlx90632_read_temp_raw() is %d or %x\n\n\r",ambient_new_raw,ambient_new_raw);
        						//printLog("ambient_old_raw in mlx90632_read_temp_raw() is %d or %x\n\n\r",ambient_old_raw,ambient_old_raw);

        				ambient_GND = mlx90632_calc_temp_ambient(ambient_new_raw_GND, ambient_old_raw_GND, P_T, P_R, P_G, P_O, Gb);
        				/* Get preprocessed temperatures needed for object temperature calculation */
        				double pre_ambient_GND = mlx90632_preprocess_temp_ambient(ambient_new_raw_GND, ambient_old_raw_GND, Gb);
        				double pre_object_GND = mlx90632_preprocess_temp_object(object_new_raw_GND, object_old_raw_GND,ambient_new_raw_GND, ambient_old_raw_GND,Ka);
        				/* Calculate object temperature */
        				object_GND = mlx90632_calc_temp_object(pre_object_GND, pre_ambient_GND, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
        				printLog("ambient_GND is : %f - %x    object_GND is : %f - %x \n\r", ambient_GND,ambient_GND,object_GND,object_GND);
        				///!!!printLog("int ambient GND is : %d - %x    int object is : %d - %x \n\r", (int) ambient,(int) ambient,(int) object,(int) object);




	////////////////////// MOVING AVERAGE FILTER


						  // Upated the sum of a buffer for the moving average filter
						  //Subtract the oldest number from the prev sum, add the new number
						  sum_VDD = sum_VDD - arrNumbers_VDD[pos] + object_VDD;
						  sum_GND = sum_GND - arrNumbers_GND[pos] + object_GND;
						  //Assign the nextNum to the position in the array
						  arrNumbers_VDD[pos] = object_VDD;
						  arrNumbers_GND[pos] = object_GND;
						  //return the average
						  newAvg_object_VDD = sum_VDD / MOVING_AVG_BUFFER;
						  newAvg_object_GND = sum_GND / MOVING_AVG_BUFFER;
						  pos++;
						  if (pos >= MOVING_AVG_BUFFER) pos = 0;




        /////////////////VDD SENSOR AMBIENT CONVERSION FOR TRANSMISSION
        				//////////// The method of storing double might differ from device to device.
        				//////////// Hence,  double is split into integer and decimal points and stored and converted integers

						double ambient_VDD_buffer = ambient_VDD;
        				double ambient_remainder_VDD = modf(ambient_VDD_buffer,&ambient_VDD_buffer);
        				int16_t ambient_int_VDD;
        				uint8_t ambient_remainder_Uint_VDD;


        				ambient_int_VDD = (int16_t) ambient_VDD;
        				 //// multiply remainder by 100 to get 2 significant figures, convert to int,
        				///// then get the absolute value
        				ambient_remainder_Uint_VDD = abs((uint8_t) (ambient_remainder_VDD*100));


        				ambient_hex_VDD[2] = ambient_remainder_Uint_VDD;
        				ambient_hex_VDD[1] = ambient_int_VDD;
        				ambient_hex_VDD[0] = ambient_int_VDD >> 8;


		 /////////////////GND SENSOR OBJECT CONVERSION FOR TRANSMISSION
						double object_GND_buffer = newAvg_object_GND;
						double object_remainder_GND = modf(object_GND_buffer,&object_GND_buffer);
						int16_t object_int_GND;
						uint8_t object_remainder_Uint_GND;
						object_int_GND = (int16_t)newAvg_object_GND;
						object_remainder_Uint_GND = abs((uint8_t) (object_remainder_GND*100));

						object_hex_GND[2] = object_remainder_Uint_GND;
						object_hex_GND[1] = object_int_GND;
						object_hex_GND[0] = object_int_GND >> 8;


         /////////////////VDD SENSOR OBJECT CONVERSION FOR TRANSMISSION
						double object_VDD_buffer = newAvg_object_VDD;
        				double object_remainder_VDD = modf(object_VDD_buffer,&object_VDD_buffer);
        				int16_t object_int_VDD;
        				uint8_t object_remainder_Uint_VDD;
        				object_int_VDD = (int16_t)newAvg_object_VDD;
        				 //// multiply remainder by 100 to get 2 significant figures, convert to int,
        				///// then get the absolute value
        				object_remainder_Uint_VDD = abs((uint8_t) (object_remainder_VDD*100));

        				object_hex_VDD[2] = object_remainder_Uint_VDD;
        				object_hex_VDD[1] = object_int_VDD;
        				object_hex_VDD[0] = object_int_VDD >> 8;


		/////////////////GND SENSOR AMBIENT CONVERSION FOR TRANSMISSION
        				double ambient_GND_buffer = ambient_GND;
						double ambient_remainder_GND = modf(ambient_GND_buffer,&ambient_GND_buffer);
						int16_t ambient_int_GND;
						uint8_t ambient_remainder_Uint_GND;
						ambient_int_GND = (int16_t)ambient_GND;
						ambient_remainder_Uint_GND = abs((uint8_t) (ambient_remainder_GND*100));

						ambient_hex_GND[2] = ambient_remainder_Uint_GND;
						ambient_hex_GND[1] = ambient_int_GND;
						ambient_hex_GND[0] = ambient_int_GND >> 8;



		//////////////// GETTING THE TIMESTAMP
						// Reset the timeStamp. Zero will be sent if isRecording Flag is False!
						deltaTimeStamp = 0;
						// Only assign values to time stamp if isRecording flag is true
							if (isRecording == 1){
							// Get the current time for the time stamp
							currentTimeStamp = GetTimeStamp();
							printLog("currentTimeStamp is : %ld \n\r", currentTimeStamp);
							// Substract the start of the recoirding time from current time stamp
							deltaTimeStamp = currentTimeStamp - startOfRecordingTimeStamp;
							printLog("startOfRecordingTimeStamp is : %ld \n\r", startOfRecordingTimeStamp);
							printLog("deltaTimeStamp is : %ld \n\r", deltaTimeStamp);
						}
						// Split the delta timestamp into 8bit pieces and put them into the array for the future transmission via bluetooth
						for(int i = 0; i < 4; i++) {
							timeStampArr[3-i] = (deltaTimeStamp >> (8*i)) & 0xFF;
						};


          /////////////////  RECALCULATING AND UPDATING THE DUCY CYCLE ON TRANSISTORS


						if (modeOfOperation == 0){ 			// Automatic mode



							proportional_VDD = 0;   derivative_VDD = 0;  	proportional_GND = 0; 		derivative_GND = 0;


							//front_TR_PWM_Local = GetPID(set_point, object_t, p_scalar, i_scalar, d_scalar, &integral, &previous_error);
							front_TR_PWM_Local = GetPID(frontDesiredTemp, newAvg_object_VDD, PforPID, IforPID, DforPID, &proportional_VDD, &integral_VDD, &derivative_VDD, &previous_error_VDD);
							printLog("Integral_VDD is %lf \n\r", integral_VDD);
							back_TR_PWM_Local = GetPID(backDesiredTemp, newAvg_object_GND, PforPID, IforPID, DforPID, &proportional_GND, &integral_GND, &derivative_GND, &previous_error_GND);
							printLog("Integral_GND is %lf \n\r", integral_GND);

							printLog("PforPID is %lf, IforPID is %lf, DforPID is %lf \n\r", PforPID,IforPID,DforPID);

							sendPID_arr[0] = proportional_VDD;
							sendPID_arr[1] = integral_VDD;
							sendPID_arr[2] = derivative_VDD;
							sendPID_arr[3] = proportional_GND;
							sendPID_arr[4] = integral_GND;
							sendPID_arr[5] = derivative_GND;

							for(int i = 0; i < 6; i++) {
								printLog("sendPID_arr%d is: %d;  ",i,sendPID_arr[i]);
							};
							printLog("\n\r");



						}else if (modeOfOperation == 1){	// Manual mode
							front_TR_PWM_Local = front_TR_PWM_In;
							back_TR_PWM_Local = back_TR_PWM_In;
						}else{
							printLog("Error: wrong mode of operation - %d", modeOfOperation);
						}


						// Set calculated Duty Cycle to transistors
						TIMER_CompareBufSet(TIMER0, 1, back_TR_PWM_Local*5.91);
						TIMER_CompareBufSet(TIMER0, 0, front_TR_PWM_Local*5.91);

						// Required, so the gecko_cmd_gatt_server_send_characteristic_notification won't rewrite front_TR_PWM variable
						const uint8_t front_TR_PWM_Out = front_TR_PWM_Local;
						const uint8_t back_TR_PWM_Out = back_TR_PWM_Local;


		 ///////////////// 	SENDING DATA AS NOTIFICATIONS
						 //// doesnt work without soft timer, needs to have a break to send the data
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_TimeStamp, 4, (const uint8*)&timeStampArr);
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_sendPID_Characteristic, 6, (const uint8*)&sendPID_arr);
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_Front_TR_PWM_OUT, 1, (const uint8*)&front_TR_PWM_Out);
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_Back_TR_PWM_OUT, 1, (const uint8*)&back_TR_PWM_Out);
          		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_Object_characteristic_VDD, 3, (const uint8*)&object_hex_VDD);
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_Ambient_characteristic_GND, 3, (const uint8*)&ambient_hex_GND);
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_Object_characteristic_GND, 3, (const uint8*)&object_hex_GND);
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_Ambient_characteristic_VDD, 3, (const uint8*)&ambient_hex_VDD);
        		       	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_Ambient_characteristic_VDD, 3, (const uint8*)&ambient_hex_VDD);





        }
        break;


      case gecko_evt_le_connection_closed_id:

        printLog("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_le_connection_closed.reason);

        /* Check if need to boot to OTA DFU mode */
        if (boot_to_dfu) {
          /* Enter to OTA DFU mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      // If the phone app did update any of characteristics
      case gecko_evt_gatt_server_attribute_value_id:
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_Front_TR_PWM_IN){
                	front_TR_PWM_In = 0;
    		       	printLog("IM IN gecko_evt_gatt_server_attribute_value_id \n\r");
                	front_TR_PWM_In = ((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]);

                }
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_Back_TR_PWM_IN){
                	back_TR_PWM_In = 0;
                	back_TR_PWM_In = ((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]);

                }
                // If the app initiated Recording
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_isRecording){
                	isRecording = 0;
                	isRecording = ((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]);
                	// If Recording is True, update the startOfRecordingTimeStamp
                	if (isRecording == 1){
                		startOfRecordingTimeStamp = GetTimeStamp();
                		}
                	}

                // If Mode or Record value has changed
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_modeOfOperation){
                	modeOfOperation = 0;
                	modeOfOperation = ((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]);
                }
                
                
                
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_front_Desired_Temp_Characteristic){
                	frontDesiredTemp = 0;
                    // REQUIRED TO Divide BY 2 TO BE ABLE TO READ AS INTEGERS. Note Steper step is 0.5.
                	frontDesiredTemp = (((double)((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]))  /2 );
                }
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_back_Desired_Temp_Characteristic){
                	backDesiredTemp = 0;
                    // REQUIRED TO Divide BY 2 TO BE ABLE TO READ AS INTEGERS. Note Steper step is 0.5.
                	backDesiredTemp = (((double)((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]))  /2 );
                	printLog("backDesiredTemp is %lf \n\r", backDesiredTemp);
                }
                



                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_PforPID_Characteristic){
                	PforPID = 0;
                    // NOTE, Scaling down the received range (0-100) to (0-100) by dividing by 1
                	PforPID = (((float)((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]))  /1 );
                	printLog("PforPID is %f \n\r", PforPID);
                }
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_IforPID_Characteristic){
                	IforPID = 0;
                    // NOTE, Scaling down the received range (0-100) to (0-20) by dividing by 5
                	IforPID = (((float)((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]))  /5 );
                	printLog("IforPID is %f \n\r", IforPID);
                }
                if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_DforPID_Characteristic){
                	DforPID = 0;
                    // NOTE, Scaling down the received range (0-100) to (0-100) by dividing by 1
                	DforPID = (((float)((uint16_t) evt->data.evt_gatt_server_user_write_request.value.data[0]))  /1 );
                	printLog("DforPID is %f \n\r", DforPID);
                }


                break;



       /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      /* Add additional event handlers as your application requires */

      default:
        break;
    }
  }
}

uint8_t GetPID(float set_point, double object_t, float p_scalar, float i_scalar, float d_scalar, float *proportional,float *integral,float *derivative, float *previous_error){

	printLog("Im here, set_point %f; object_t %f; p_scalar %f; i_scalar %f; d_scalar %f; integral %f; previous_error %f; \n\r",set_point, object_t, p_scalar, i_scalar, d_scalar, *
			integral, *previous_error);
	float error = set_point - object_t;
	//printLog("error %f \n\r", error);

	*proportional = (error * p_scalar);
	//printLog("proportional in_funct_1 %f \n\r", proportional);
	
	// Boundary if statements, as duty cycle can not be over 100 and below 0.
	// IMPORTANT As uint8_t variable, proportional may overflow. If so, either 0 or 100 is assigned to it
	if (error * p_scalar >  100) *proportional = 100; // limit wind-up
	if (error * p_scalar < 0) *proportional = 0;
	//printLog("proportional in_funct_2 %f \n\r", proportional);

	// calculate the integral component (summation of past errors * i scalar)
	//printLog("Integral in_funct_1 is %lf, ", *integral);
	*integral += error * i_scalar;
	//printLog("Integral in_funct_2 is %lf, ", *integral);
	if(*integral >  100) *integral = 100; // limit wind-up
	if(*integral < 0) *integral = 0;
	//printLog("Integral in_funct_3 is %lf \n\r", *integral);


	// calculate the derivative component (change since previous error * d scalar)
	//printLog("previous_error in_funct_1 is %lf, ", *previous_error);

	*derivative = (error - *previous_error) * d_scalar;
	if(*derivative >  100) *derivative = 100; // limit wind-up
	if(*derivative < -100) *derivative = -100;
	//printLog("derivative in_funct_1 is %lf, ", derivative);
	*previous_error = error;
	//printLog("previous_error in_funct_2 is %lf \n\r", *previous_error);



	double resultingPID = *proportional + *integral + *derivative  ;
	///printLog("resultingPID in_funct_1 is %lf, ", resultingPID);
	// Limit
	if(resultingPID >  100) resultingPID = 100;
	if(resultingPID < 0) resultingPID = 0;

	//printLog("resultingPID in_funct_2 is %lf \n\r", resultingPID);

	printLog(" error is %f; proportional is %f; *integral is %f; derivative is %f; resultingPID is %f; \n\r",error, *proportional, *integral,
			*derivative, resultingPID);
	return (int8_t)resultingPID ;
}





  uint32_t GetTimeStamp(){
    	/// Declare variable to store calculated days,hours,mins,secs and millisecs.
    	uint8_t days,hours,mins,sec,ms;
    	/// Also declare variables to store the total number of seconds and milliseconds
    	/// elapsed from the chip's power-up.
    	/// End goal is to keep the total elapsed time in one variable, so that it is easier
    	/// to send it via Bluetooth Low energy
    	uint32_t total_sec;
    	uint32_t total_MS;

    	/// Get time function and store in a special time variable
    	struct gecko_msg_hardware_get_time_rsp_t* time = gecko_cmd_hardware_get_time();

    	/// Here we will calculate separately number of days,hours,mins,secs elapsed
    	/// So we can print it in an understandable form
    	days = time->seconds/60/60/24;
    	hours = time->seconds/60/60 % 24;
    	mins = time->seconds/60 % 60;
    	sec = time->seconds % 60;
    	/// In order to obtain millisecond passed we have to get it from the number of clock ticks elapsed.
    	/// *100 defines how many significant figures to keep - 2 in our case,
    	/// while division by 32768 is required to convert clock ticks into milliseconds.
    	ms = time->ticks * 100 / 32768;

    	/// Print passed days,hours,mins,secs and millisecs in usual form.
    	printLog("%03d-%02d:%02d:%02d.%02d \n\r",days,hours,mins,sec,ms);


    	/// We will also fill the variable that stores total time in milliseconds.
    	/// total_MS later will be used to send the timestamp over BLE.
    	total_sec = time->seconds ;
    	total_MS = (total_sec * 100) + ms;
    	/// Print the total time in milliseconds in decimal and hex for later comparison and
    	/// debug with the received variable on a PC
    	printLog("total_MS: decimal - %ld,   hex - %x\n\r",total_MS,total_MS);

    	/// Function returns the total time in milliseconds
    	return total_MS;
  }


  /* Print stack version and local Bluetooth address as boot message */
  static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
  {
  #if DEBUG_LEVEL
    bd_addr local_addr;
    int i;

    printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
    local_addr = gecko_cmd_system_get_bt_address()->address;

    printLog("local BT device address: ");
    for (i = 0; i < 5; i++) {
      printLog("%2.2x:", local_addr.addr[5 - i]);
    }
    printLog("%2.2x\r\n", local_addr.addr[0]);
  #endif
  }







