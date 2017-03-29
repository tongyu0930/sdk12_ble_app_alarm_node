/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * release
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "relay.h"
#include "nrf_error.h"
#include "nrf.h"
#include "app_mpu.h"


#define CENTRAL_LINK_COUNT       		0   		/**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT    		1   		/**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0   		/**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0   		/**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4   		/**< Size of timer operation queues. */
APP_TIMER_DEF(alarm_timer_id1);
APP_TIMER_DEF(alarm_timer_id2);


#define     						SELF_NUMBER 		  	5
#define     						ALARM_SENDING_TIME_MIN	5			// 2500 ms
#define     						ALARM_SENDING_TIME_MAX	30			// 15000 ms
#define     						ADV_COUNT_INTERVAL		500			// timer1 interval 500 ms
#define     						IMU_GET_INTERVAL		5			// timer2 5 ms // 200 Hz
#define     						V_THRESHOLD				7 			//0.7 // m/s
#define     						RSS_THRESHOLD			28			//2.8 // 2.8*g
#define								Y_AXIS_OFFSET		   -112
#define								Z_AXIS_OFFSET			560

volatile static bool 				fall_detected 			= false;
volatile static bool 				ACK_received 			= false;
volatile static bool                get_acceleration		= true;
		 static accel_values_t 		acc_values;
		 static uint8_t				advertising_time_count	= 0;
		 static uint16_t 			g_relative 				= 0;	// the gravity according to IMU value
		 static uint32_t 			v_relative_threshold 	= 0; 	// m/s
		 static uint32_t 			rss_relative_threshold 	= 0; 	// 2.8*g
		 static int32_t 			v 						= 0;	// Speed
		 static bool 				speed 					= false;
		 static bool 				impact 					= false;
		 static bool 				angle 					= false;
		 static bool 				cali 					= true;
		 static int32_t 			acc_relative 			= 0;
		 static uint16_t 			angle_time_count 			= 0;
		 static uint16_t 			cosine_time_count 			= 0;
		 static int32_t 			cali_x 					= 0;
		 static int32_t 			cali_y 					= 0;
		 static int32_t 			cali_z 					= 0;
		 static int16_t 			x 						= 0;
		 static int16_t 			y 						= 0;
		 static int16_t 			z 						= 0;
		 static uint8_t 			speed_sample_count 					= 0;
		 static uint8_t 			cali_count 				= 0;



static enum
{
    RELAY_NODE,
	CENTER_NODE,
	NONE
} node_type;

const ble_gap_adv_params_t m_adv_params =
  {
	.type        					= BLE_GAP_ADV_TYPE_ADV_IND,					// Undirected advertisement.
	.p_peer_addr					= NULL,												// 我觉得null是不是默认就是static的address？
	.fp          					= BLE_GAP_ADV_FP_ANY,
	.interval    					= 0x0020, // 20 ms+ random delay(0-10ms)						// 虽然这个最小值时100ms，但是你可以通过timer以更快的频率启动关闭广播。
	.timeout     					= 0
  };

const ble_gap_scan_params_t m_scan_params =
  {
    .active      					= 0,
    .use_whitelist   				= 0,
    .adv_dir_report 				= 0,
    .interval    					= 0x0040,	//1 second
    .window      					= 0x0040, //0x0030 is 30ms
    .timeout     					= 0
  };


static void advertising_start(void);
static void scanning_start(void);
static void mpu_setup(void);
static void try_stop_advertising(void);
static void start_loop(void);



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze how your product is supposed to react in case of Assert.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


static void app_timer_handler1(void * p_context)
{
	uint32_t err_code;
	NRF_GPIO->OUT ^= (1 << 20);
	advertising_time_count++;
	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);
	uint8_t	alarm[10] = {0x09, 0xff,'T','O','N','G',0,0,advertising_time_count,SELF_NUMBER};
	sd_ble_gap_adv_data_set(alarm, sizeof(alarm), NULL, 0);
	advertising_start();
	try_stop_advertising();
}


static void app_timer_handler2(void * p_context) // 网上说，因为timer handler interrupt和spi interrupt冲突了，建议还是把spi放到main里
{
	get_acceleration = true;
}


static void advertising_start(void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}


static void scanning_start(void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}


static void get_adv_data(ble_evt_t * p_ble_evt)
{
	uint32_t index = 0;
	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *) p_adv_report->data;

	while (index < p_adv_report->dlen)
	{
		//uint32_t err_code;
		uint8_t  field_length = p_data[index];
		uint8_t  field_type = p_data[index + 1];

		if ((field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA) || (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME))
		{
			uint8_t a = index+2;
			/************************************************ check origin *********************************************************************/
			if(!((p_data[a]== 'T') && (p_data[a+1]== 'O') && (p_data[a+2]== 'N') && (p_data[a+3]== 'G')))
			{
				return;
			}
			/************************************************ check node type *********************************************************************/
			node_type = NONE;

			if(field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME)
			{
				node_type = CENTER_NODE;
			}else
			{
				if(p_data[a+4]== 0) // ALARM_NODE;
				{
					return;
				}else
				{
					node_type = RELAY_NODE;
				}
			}
			/************************************************ switch node type *********************************************************************/
			switch(node_type)
			{
			case CENTER_NODE:
				if((p_data[a+10] - 48) * 16 + (p_data[a+11] - 48) == SELF_NUMBER) // "TONG0100" + "alarm node number"
				{
					ACK_received = true;
				}
				break;

			case RELAY_NODE:
				if(p_data[a+7] == SELF_NUMBER)
				{
					ACK_received = true;
				}
				break;

			case NONE:
				break;
			}
			return;
		}
		index += field_length + 1;
	}
}

static void try_stop_advertising(void)
{
	uint32_t err_code;

	if(((advertising_time_count >= ALARM_SENDING_TIME_MIN) && (ACK_received == true)) || (advertising_time_count >= ALARM_SENDING_TIME_MAX))
	{

		err_code = sd_ble_gap_adv_stop();
		APP_ERROR_CHECK(err_code);

		err_code = sd_ble_gap_scan_stop();
		APP_ERROR_CHECK(err_code);

		err_code = app_timer_stop(alarm_timer_id1);
		APP_ERROR_CHECK(err_code);
		ACK_received = false;
		fall_detected = false;
		advertising_time_count = 0;
		NRF_GPIO->OUT |= (1 << 20);
	}
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	get_adv_data(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL); 													// Initialize the SoftDevice handler module.

    ble_enable_params_t ble_enable_params;

    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);									//Check the ram settings against the used number of links

    err_code = softdevice_enable(&ble_enable_params); 												// Enable BLE stack.
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch); 									// Register with the SoftDevice handler module for BLE events.
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void start_loop(void)
{
	uint32_t err_code;
	uint8_t	alarm[10] = {0x09, 0xff,'T','O','N','G',0,0,advertising_time_count,SELF_NUMBER};
	sd_ble_gap_adv_data_set(alarm, sizeof(alarm), NULL, 0);
	advertising_start();
	scanning_start();
	err_code = app_timer_start(alarm_timer_id1, APP_TIMER_TICKS(ADV_COUNT_INTERVAL, APP_TIMER_PRESCALER), NULL); // timer1 interval = 500 ms
	APP_ERROR_CHECK(err_code);
}

static void gpio_configure(void)
{
	NRF_GPIO->DIRSET = LEDS_MASK; // set register
	NRF_GPIO->OUTSET = LEDS_MASK; // clear register
}


static void mpu_setup(void)
{
    ret_code_t ret_code;
    ret_code = mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value

    // Setup and configure the MPU with intial values
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 4;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_4G; // Set accelerometer full scale range to 2G
    ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    int16_t cosine 				= 0;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("###################### System Started ####################\r\n");
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false); //(0,4,false)
    err_code = app_timer_create(&alarm_timer_id1, APP_TIMER_MODE_REPEATED, app_timer_handler1);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&alarm_timer_id2, APP_TIMER_MODE_REPEATED, app_timer_handler2);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(alarm_timer_id2, APP_TIMER_TICKS(IMU_GET_INTERVAL, APP_TIMER_PRESCALER), NULL); // timer2 interval 5 ms
    APP_ERROR_CHECK(err_code);
    ble_stack_init();
	err_code = sd_ble_gap_tx_power_set(-20);
	APP_ERROR_CHECK(err_code);
    gpio_configure();
    mpu_setup();


//    uint8_t out_data[13]					   = {0x0c,0xff,'T','O','N','G',0,0,4,9,0,0,0};
//    uint8_t out_data[13]					   = {0x0c,0xff,'T','O','N','G',0,0,4,8,0,0,0};
//    uint8_t out_data[13]					   = {0x0c,0xff,'T','O','N','G',2,6,7,0,8,2,7};
//    uint8_t out_data[13]					   = {0x0c,0xff,'T','O','N','G',2,5,3,0,3,2,8};
//    uint8_t out_data[13]					   = {0x0c,0xff,'T','O','N','G',0,0,12,8,0,0,0};
//    sd_ble_gap_adv_data_set(out_data, sizeof(out_data), NULL, 0); // 用这句话来躲避掉flag
//    advertising_start();
//    scanning_start();

    for (;; )
    {
    	if(get_acceleration && !fall_detected) //the program will get here at 200 Hz
    	{
			err_code = mpu_read_accel(&acc_values);
			APP_ERROR_CHECK(err_code);

//			NRF_LOG_INFO("X: %06d   Y: %06d   Z: %06d\r\n", acc_values.x, acc_values.y, acc_values.z);
//			if(acc_values.x != 0)
//			{
//				NRF_GPIO->OUT ^= (1 << 17);
//			}

			if(cali)	// at the beginning of the program, the device will measure the gravity vector, do not shake the device.
			{
				cali_count++;
				cali_x += acc_values.x;
				cali_y += acc_values.y + Y_AXIS_OFFSET;
				cali_z += acc_values.z + Z_AXIS_OFFSET;
				if(cali_count == 200)
				{
					cali = false;
					x = cali_x / 200;
					y = cali_y / 200;
					z = cali_z / 200;
					g_relative = sqrt(x * x + y * y + z * z);

//					g_relative = 8300;
					v_relative_threshold = V_THRESHOLD * g_relative;
					rss_relative_threshold = RSS_THRESHOLD * g_relative;
					NRF_LOG_INFO("g: %06d\r\n", g_relative);
				}
			}else
			{
				uint16_t value = sqrt((acc_values.x) * (acc_values.x) + (acc_values.y + Y_AXIS_OFFSET)*(acc_values.y + Y_AXIS_OFFSET) + (acc_values.z + Z_AXIS_OFFSET) * (acc_values.z + Z_AXIS_OFFSET));

				speed_sample_count++;
				if(speed_sample_count == 4) // reduce to 50Hz sample rate for speed monitor
				{
					speed_sample_count = 0;

					acc_relative += (g_relative - value);

					if((acc_relative > 0) && ((unsigned)acc_relative > 100)) //"(unsigned)acc_relative > 100" is for the jittering value
					{
						v += acc_relative * 2; // "2" represents 0.020 s, to make sure all the math operations are interger operations.

						if((unsigned)v > v_relative_threshold * 1000 / 981)
						{
							speed = true;
							NRF_LOG_INFO("velocity: %09d\r\n", value);
						}
					}else
					{
						v = 0;
					}
					acc_relative = 0;
				}

				if(value*10 > rss_relative_threshold)	// impact monitor
				{
					impact = true;
					NRF_LOG_INFO("impact\r\n");
				}

				if(impact || speed)
				{
					angle_time_count++;

					if(angle_time_count >= 599)
					{
						if(cosine_time_count >= 299) // 400*0.75 = 300
						{
							angle = true;
						}else
						{
							impact = false;
							speed = false;
						}
						cosine_time_count = 0;
						angle_time_count = 0;
					}

					if(angle_time_count >= 199)
					{
						cosine = (10 * (x*acc_values.x + y*(acc_values.y + Y_AXIS_OFFSET) + z*(acc_values.z + Z_AXIS_OFFSET))) / (g_relative * value);
						if(((unsigned)cosine < 5) || (cosine < 0))
						{
							cosine_time_count++;
						}
					}
				}

				if(speed && impact && angle)
				{
					if(fall_detected == false)
					{
						fall_detected 	= true;
						impact 			= false;
						speed 			= false;
						angle 			= false;
						start_loop();
					}
				}
			}
			get_acceleration = false;
    	}

        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
