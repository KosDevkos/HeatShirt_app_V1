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

///////////////// MY FUCNTIONS START HERE ////////////////
/*
static I2C_TransferReturn_TypeDef i2cReadByte(I2C_TypeDef *i2c, uint16_t addr, uint8_t command, uint8_t *val)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef sta;
  uint8_t                    i2c_write_data[1];
  uint8_t                    i2c_read_data[1];

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  //* Select command to issue
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  //* Select location/length of data to be read
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;

  sta = I2CSPM_Transfer(i2c, &seq);
  if (sta != i2cTransferDone)
  {
    return sta;
  }
  if (NULL != val)
  {
    *val = i2c_read_data[0];
  }
  return sta;
}

static I2C_TransferReturn_TypeDef i2cWriteByte(I2C_TypeDef *i2c, uint16_t addr, uint8_t command, uint8_t message)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef sta;
  uint8_t                    i2c_write_data[1];
  uint8_t                    i2c_message_data[1];

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_WRITE;
  // Select command to issue
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  // Select location/length of data to be read
  i2c_message_data[0] = message;       ///// needs additional input in function decalration
  seq.buf[1].data = i2c_message_data;
  seq.buf[1].len  = 1;

  sta = I2CSPM_Transfer(i2c, &seq);
  return sta;
}



void mcube_i2c_setup()
{
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




	   uint16_t Device_ID = 	SENSOR_1;		  ///  Contains the Device ID

	   uint8_t command_1 = 0x0D;  //1
	   uint8_t  message_1 = 0x40;

	   uint8_t command_2 = 0x0F ; //2
	   uint8_t  message_2 = 0x42;

	   uint8_t command_3 = 0x20;   //3
	   uint8_t  message_3 = 0x01;

	   uint8_t command_4 = 0x21 ;  ///4
	   uint8_t  message_4 = 0x80;

	   uint8_t command_5 = 0x28;   ///5
	   uint8_t  message_5 = 0x00;

	   uint8_t command_6 = 0x1A;    ///6
	   uint8_t  message_6 = 0x00;

	   uint8_t command_7 = 0x1C ;  ///7
	   uint8_t message_7 = 0x03;

	   uint8_t command_8 = 0x15;   ///8
	   uint8_t  message_8 = 0x15;

	   uint8_t command_9 = 0x11;   ///9
	   uint8_t  message_9 = 0x08;

	   uint8_t command_10 = 0x10;  ///10
	    uint8_t  message_10 = 0x05;

	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_1, message_1);  //1
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_2, message_2);  //2
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_3, message_3);  //3
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_4, message_4);  //4
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_5, message_5);  //5
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_6, message_6);  //6
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_7, message_7);  //7
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_8, message_8);  //8
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_9, message_9);  //9
	  transfer_status = i2cWriteByte(I2C0,Device_ID, command_10, message_10);  //10

	  for(volatile long i=0; i<100000; i++);  ///delay to allow registers to update properly


}

*/

///////////////// MY FUCNTIONS END HERE ////////////////

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


		int32_t P_T =MLX90632_EE_P_T;
		int32_t P_R =MLX90632_EE_P_R;
		int32_t P_G =MLX90632_EE_P_G;
		int32_t P_O =MLX90632_EE_P_O;
		int16_t Gb = MLX90632_EE_Ga;

		int16_t Ka =MLX90632_EE_Ka;

		int32_t Ea =MLX90632_EE_Ea;
		int32_t Eb =MLX90632_EE_Eb;
		int32_t Ga =MLX90632_EE_Ga;
		int32_t Fa =MLX90632_EE_Fa;
		int32_t Fb =MLX90632_EE_Fb;
		int16_t Ha =MLX90632_EE_Ha;
		int16_t Hb =MLX90632_EE_Hb;


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
			ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,P_T, P_R, P_G, P_O, Gb);
			/* Get preprocessed temperatures needed for object temperature calculation */
			double pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
			double pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,ambient_new_raw, ambient_old_raw,Ka);
			/* Calculate object temperature */
			object = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
			printLog("ambient is : %f\n\r", ambient);
			printLog("object is : %f\n\r", object);
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


