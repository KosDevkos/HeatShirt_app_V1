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

#define SENSOR_1    0x3A << 1

void printTime();



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
        printLog("boot event - starting advertising\r\n");

        /* Set advertising parameters. 100ms advertisement interval.
         * The first parameter is advertising set handle
         * The next two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6).
         * The last two parameters are duration and maxevents left as default. */
        //gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

        /* Start general advertising and enable connections. */
        //gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);

		  CMU_ClockEnable(cmuClock_I2C0, true); /// ???????????????



		  I2CSPM_Init_TypeDef myi2cinit = I2CSPM_INIT_DEFAULT;
		  myi2cinit.sclPort = gpioPortC;
		  myi2cinit.sclPin = 6;
		  myi2cinit.portLocationScl = 10;
		  myi2cinit.sdaPort = gpioPortC;
		  myi2cinit.sdaPin = 7;
		  myi2cinit.portLocationSda = 12;




		   I2CSPM_Init(&myi2cinit);


		I2C_TransferReturn_TypeDef transfer_status;

        int16_t object_new_raw;
		int16_t object_old_raw;
		int16_t ambient_new_raw;
		int16_t ambient_old_raw;

		usleep(10000,1);

		int32_t P_T;
		uint16_t P_T_MS;
		uint16_t P_T_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_T, &P_T_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_T+1, &P_T_MS);
		P_T = (P_T_MS <<16) | P_T_LS;
		printLog("	P_T is %ld or %X\n\n\r",P_T,P_T);


		int32_t P_R;
		uint16_t P_R_MS;
		uint16_t P_R_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_R, &P_R_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_R +1, &P_R_MS);
		P_R = (P_R_MS <<16) | P_R_LS;
		printLog("	P_R is %ld or %X\n\n\r",P_R,P_R);

		int32_t P_G ;
		uint16_t P_G_MS;
		uint16_t P_G_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_G, &P_G_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_G + 1, &P_G_MS);
		P_G = (P_G_MS <<16) | P_G_LS;
		printLog("	P_G is %ld or %X\n\n\r",P_G,P_G);

		int32_t P_O ;
		uint16_t P_O_MS;
		uint16_t P_O_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_O, &P_O_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_P_O+1, &P_O_MS);
		P_O = (P_O_MS <<16) | P_O_LS;
		printLog("	P_O is %ld or %X\n\n\r",P_O,P_O);

		uint16_t Gb_UINT;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Gb, &Gb_UINT);
		int16_t Gb = Gb_UINT;
		printLog("	Gb is %d or %X\n\n\r",Gb,Gb);

		uint16_t Ka_UINT ;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Ka, &Ka_UINT);
		int16_t Ka = Ka_UINT;
		printLog("	Ka is %d or %X\n\n\r",Ka,Ka);

		int32_t Ea ;
		uint16_t Ea_MS;
		uint16_t Ea_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Ea, &Ea_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Ea+1, &Ea_MS);
		Ea = (Ea_MS <<16) | Ea_LS;
		printLog("	Ea is %ld or %X\n\n\r",Ea,Ea);

		int32_t Eb ;
		uint16_t Eb_MS;
		uint16_t Eb_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Eb, &Eb_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Eb+1, &Eb_MS);
		Eb = (Eb_MS <<16) | Eb_LS;
		printLog("	Eb is %ld or %X\n\n\r",Eb,Eb);

		int32_t Ga ;
		uint16_t Ga_MS;
		uint16_t Ga_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Ga, &Ga_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Ga+1, &Ga_MS);
		Ga = (Ga_MS <<16) | Ga_LS;
		printLog("	Ga is %ld or %X\n\n\r",Ga,Ga);

		int32_t Fa ;
		uint16_t Fa_MS;
		uint16_t Fa_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Fa, &Fa_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Fa+1, &Fa_MS);
		Fa = (Fa_MS <<16) | Fa_LS;
		printLog("	Fa is %ld or %X\n\n\r",Fa,Fa);

		int32_t Fb ;
		uint16_t Fb_MS;
		uint16_t Fb_LS;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Fb, &Fb_LS);
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Fb+1, &Fb_MS);
		Fb = (Fb_MS <<16) | Fb_LS;
		printLog("	Fb is %ld or %X\n\n\r",Fb,Fb);

		uint16_t Ha_UINT ;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Ha, &Ha_UINT);
		uint16_t Ha = Ha_UINT;
		printLog("	Ha is %d or %X\n\n\r",Ha,Ha);

		uint16_t Hb_UINT ;
		mlx90632_i2c_read(I2C0,Device_ID,MLX90632_EE_Hb, &Hb_UINT);
		int16_t Hb = Hb_UINT;
		printLog("	Hb is %d or %X\n\n\r",Hb,Hb);

		printLog("Im in app.c\n\n\r");

	    uint16_t reg_status;

	    /// ADDING ONE BIT TO A STATUS REGISTER??????????
	    mlx90632_i2c_read(I2C0,Device_ID,MLX90632_REG_STATUS, &reg_status);
			//printLog("Im in mlx90632_start_measurement - REG_STATUS_1 IS: %x\n\n\r",reg_status);
	    mlx90632_i2c_write(I2C0,Device_ID,MLX90632_REG_STATUS, reg_status | 0x0100);



        int32_t ret = 0;
        double ambient;
        double object;
        /* Read sensor EEPROM registers needed for calcualtions */
        /* Now we read current ambient and object temperature */
        while(1){
			ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw,
										 &object_new_raw, &object_old_raw);
			if(ret < 0){
				/* Something went wrong - abort */
				printLog("error is : %ld\n\r", ret);
			}
			/* Now start calculations (no more i2c accesses) */
			/* Calculate ambient temperature */
					//printLog("ambient_new_raw in mlx90632_read_temp_raw() is %d or %x\n\n\r",ambient_new_raw,ambient_new_raw);
					//printLog("ambient_old_raw in mlx90632_read_temp_raw() is %d or %x\n\n\r",ambient_old_raw,ambient_old_raw);

			ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,P_T, P_R, P_G, P_O, Gb);
			/* Get preprocessed temperatures needed for object temperature calculation */
			double pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
			double pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,ambient_new_raw, ambient_old_raw,Ka);
			/* Calculate object temperature */
			object = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
			printLog("ambient is : %f\    object is : %f\n\r", ambient,object);
        }





        break;

      case gecko_evt_le_connection_opened_id:

        printLog("connection opened\r\n");

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


