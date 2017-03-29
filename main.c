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
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */
// 最后在删减makefile
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "relay.h"
#include "nrf_error.h"
#include "nrf.h"
#include "app_mpu.h"


#define CENTRAL_LINK_COUNT       		0  /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT    		1  /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0 /**< 不懂Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4   /**< Size of timer operation queues. */
APP_TIMER_DEF(alarm_timer_id1);
APP_TIMER_DEF(alarm_timer_id2);


#define     						SELF_NUMBER 		  	5
#define     						ALARM_SENDING_TIME		5
#define     						ALARM_SENDING_TIME_MAX	30
#define     						ADV_INTERVAL			500
#define     						IMU_GET_INTERVAL		5
#define     						V_THRESHOLD				7 //0.7 // m/s
#define     						RSS_THRESHOLD			28//2.8 // 2.8*g
volatile static bool 				want_scan 			= false;
volatile static bool 				alarm_event 		= false;
volatile static bool 				begin 			= true;
		 static accel_values_t 		acc_values;
		 static bool                get_imu				= true;
		 static uint8_t				advertising_count	= 0;
		 static bool 				ACK 				= false;
		 static uint8_t				alarm_count			= 0;

static enum
{
    RELAY_NODE,
	CENTER_NODE,
	NONE
} node_type;

const ble_gap_adv_params_t m_adv_params =
  {
	.type        					= BLE_GAP_ADV_TYPE_ADV_IND,					// Undirected advertisement.
	//.p_peer_addr->addr_type 		= BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
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



void HardFault_Handler(void)  //重写HardFault_Handler
{
    uint32_t *sp = (uint32_t *) __get_MSP();
    uint32_t ia = sp[24/4];
    NRF_LOG_INFO("Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    while(1)
    	;
}



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


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)  //重写app_error_fault_handler
{
    // static error variables - in order to prevent removal by optimizers
    static volatile struct
    {
        uint32_t        fault_id;
        uint32_t        pc;
        uint32_t        error_info;
        assert_info_t * p_assert_info;
        error_info_t  * p_error_info;
        ret_code_t      err_code;
        uint32_t        line_num;
        const uint8_t * p_file_name;
    } m_error_data = {0};

    // The following variable helps Keil keep the call stack visible, in addition, it can be set to
    // 0 in the debugger to continue executing code after the error check.
    volatile bool loop = true;
    UNUSED_VARIABLE(loop);

    m_error_data.fault_id   = id;
    m_error_data.pc         = pc;
    m_error_data.error_info = info;

    switch (id)
    {
        case NRF_FAULT_ID_SDK_ASSERT:
            m_error_data.p_assert_info = (assert_info_t *)info;
            m_error_data.line_num      = m_error_data.p_assert_info->line_num;
            m_error_data.p_file_name   = m_error_data.p_assert_info->p_file_name;
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            m_error_data.p_error_info = (error_info_t *)info;
            m_error_data.err_code     = m_error_data.p_error_info->err_code;
            m_error_data.line_num     = m_error_data.p_error_info->line_num;
            m_error_data.p_file_name  = m_error_data.p_error_info->p_file_name;
            break;
    }

    UNUSED_VARIABLE(m_error_data);

    //NRF_LOG_INFO("ASSERT\r\n\tError: 0x%08x\r\n\tLine: %d\r\n\tFile: %s\r\n", m_error_data.err_code, m_error_data.line_num, m_error_data.p_file_name);
    NRF_LOG_INFO("error!!!");

    // If printing is disrupted, remove the irq calls, or set the loop variable to 0 in the debugger.
    __disable_irq();

    while(loop);

    __enable_irq();
}


static void app_timer_handler1(void * p_context)
{
	NRF_GPIO->OUT ^= (1 << 20);
	uint32_t      err_code;

	if(begin)
	{
		advertising_start();

		begin 	= false;
		want_scan 	= true;
	}else
	{
		if(want_scan)
		{
			err_code = sd_ble_gap_adv_stop();
			APP_ERROR_CHECK(err_code);

			scanning_start();

			NRF_LOG_INFO("scanning\r\n");
			want_scan = false;

		}else
		{
			err_code = sd_ble_gap_scan_stop();
			APP_ERROR_CHECK(err_code);

			alarm_count++;
			advertising_count++;
			uint8_t	alarm[10] = {0x09, 0xff,'T','O','N','G',0,0,alarm_count,SELF_NUMBER};
			sd_ble_gap_adv_data_set(alarm, sizeof(alarm), NULL, 0);
			advertising_start();

			NRF_LOG_INFO("broadcasting\r\n");
			want_scan = true;
		}
	}

	try_stop_advertising();
}


static void app_timer_handler2(void * p_context) // 网上说，因为timer handler interrupt和spi interrupt冲突了，建议还是把spi放到main里
{
	get_imu = true;
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
//	NRF_GPIO->OUT ^= (1 << 18);
//	NRF_LOG_INFO("TTT\r\n");
//	return;

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
					ACK = true;
				}
				break;

			case RELAY_NODE:
				if(p_data[a+7] == SELF_NUMBER)
				{
					ACK = true;
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

	if(((alarm_count >= ALARM_SENDING_TIME) && (ACK == true)) || (advertising_count >= ALARM_SENDING_TIME_MAX))
	{
		if(want_scan)
		{
			err_code = sd_ble_gap_adv_stop();
			APP_ERROR_CHECK(err_code);

		}else
		{
			err_code = sd_ble_gap_scan_stop();
			APP_ERROR_CHECK(err_code);
		}

		err_code = app_timer_stop(alarm_timer_id1);
		APP_ERROR_CHECK(err_code);
		begin = true;
		ACK = false;
		alarm_event = false;
		advertising_count = 0;
		alarm_count = 0;
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


void GPIOTE_IRQHandler(void)
{
	uint32_t err_code;

    if (NRF_GPIOTE->EVENTS_IN[1] != 0) // button1 开启关闭 广播
    {
    	nrf_delay_us(200000);
        NRF_GPIOTE->EVENTS_IN[1] = 0;

        if(begin)
		{
			err_code = app_timer_start(alarm_timer_id1, APP_TIMER_TICKS(200, APP_TIMER_PRESCALER), NULL); // 每100ms就触发一次handler
			APP_ERROR_CHECK(err_code);
		}else
		{
			if(want_scan)
			{
				err_code = sd_ble_gap_adv_stop();
				APP_ERROR_CHECK(err_code);

			}else
			{
				err_code = sd_ble_gap_scan_stop();
				APP_ERROR_CHECK(err_code);
			}

			err_code = app_timer_stop(alarm_timer_id1);
			APP_ERROR_CHECK(err_code);
			begin = true;
		}
    }
}

static void start_loop(void)
{
	uint32_t err_code;

	if(begin)
	{
		err_code = app_timer_start(alarm_timer_id1, APP_TIMER_TICKS(ADV_INTERVAL, APP_TIMER_PRESCALER), NULL); // 每100ms就触发一次handler
		APP_ERROR_CHECK(err_code);
	}
}

static void gpio_configure(void)
{
	NRF_GPIO->DIRSET = LEDS_MASK; // set register
	NRF_GPIO->OUTSET = LEDS_MASK; // clear register

	NRF_GPIO->PIN_CNF[BUTTON_1] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	nrf_delay_us(5000);																			// Do I have to delay?

	NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_1                      << GPIOTE_CONFIG_PSEL_Pos);


	// Interrupt
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Msk;
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOWEST);
	NVIC_EnableIRQ(GPIOTE_IRQn); 											 					//declaration值得一看！！！有关IRQ
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
    int32_t acc_relative = 0;
    bool speed = false;
    bool impact = false;
    bool angle = false;
    uint16_t angle_count = 0;
    uint16_t cosine_count = 0;
    int32_t cali_x = 0;
    int32_t cali_y = 0;
    int32_t cali_z = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    uint8_t count = 0;
    uint16_t g_relative = 0;
    int32_t v = 0;
    uint8_t cali_count = 0;
    bool 	cali = true;
    uint32_t v_relative_threshold = 0; // m/s
    uint32_t rss_relative_threshold = 0; // 2.8*g
    int16_t cosine = 0;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("###################### System Started ####################\r\n");
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false); //(0,4,false)
    err_code = app_timer_create(&alarm_timer_id1, APP_TIMER_MODE_REPEATED, app_timer_handler1);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&alarm_timer_id2, APP_TIMER_MODE_REPEATED, app_timer_handler2);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(alarm_timer_id2, APP_TIMER_TICKS(IMU_GET_INTERVAL, APP_TIMER_PRESCALER), NULL); // 每200ms就触发一次handler
    APP_ERROR_CHECK(err_code);
    ble_stack_init();
	err_code = sd_ble_gap_tx_power_set(-20);
	APP_ERROR_CHECK(err_code);
    gpio_configure(); // 注意gpio和timesync是相对独立的，同步时钟本质上不需要gpio
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
    	if(get_imu && !alarm_event) //200 Hz
    	{
			err_code = mpu_read_accel(&acc_values);
			APP_ERROR_CHECK(err_code);
//			NRF_LOG_INFO("X: %06d   Y: %06d   Z: %06d\r\n", acc_values.x, acc_values.y, acc_values.z);
//			if(acc_values.x != 0)
//			{
//				NRF_GPIO->OUT ^= (1 << 17);
//			}



			if(cali)
			{
				cali_count++;
				cali_x += acc_values.x;
				cali_y += acc_values.y - 112;
				cali_z += acc_values.z + 560;
				if(cali_count == 200)
				{
					cali = false;
					x = cali_x/200;
					y = cali_y/200;
					z = cali_z/200;
					g_relative = sqrt(x*x+y*y+z*z);

//					g_relative = 8300;
					v_relative_threshold = V_THRESHOLD * g_relative;
					rss_relative_threshold = RSS_THRESHOLD * g_relative;
					NRF_LOG_INFO("g: %06d\r\n", g_relative);
				}
			}else
			{
				uint16_t value = sqrt((acc_values.x)*(acc_values.x) + (acc_values.y-112)*(acc_values.y-112) + (acc_values.z+560)*(acc_values.z+560));

				count++;
				if(count == 4) // 50Hz
				{
					count = 0;

					acc_relative += (g_relative - value);

					if((acc_relative > 0) && ((unsigned)acc_relative > 100))
					{
						v += acc_relative*2; //0.020 s

						if((unsigned)v > v_relative_threshold*1000/981)
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


				if(value*10 > rss_relative_threshold)
				{
					impact = true;
					NRF_LOG_INFO("impact\r\n");
				}


				if(impact || speed)
				{
					angle_count++;

					if(angle_count >= 599)
					{
						if(cosine_count >= 299) // 400*0.75
						{
							angle = true;
						}else
						{
							impact = false;
							speed = false;
						}
						cosine_count = 0;
						angle_count = 0;
					}

					if(angle_count >= 199)
					{
						cosine = (10 * (x*acc_values.x + y*(acc_values.y - 112) + z*(acc_values.z + 560))) / (g_relative * value);
						if(((unsigned)cosine < 5) || (cosine < 0))
						{
							cosine_count++;
						}
					}
				}

				if(speed && impact && angle)
				{
					if(alarm_event == false)
					{
						alarm_event = true;
						impact = false;
						speed = false;
						angle = false;
						start_loop();
					}
				}
			}
			get_imu = false;
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
