/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
//#include <asf.h>

/* InvenSense drivers and utils */
#include "Devices/Drivers/Icm20789-DMP/Icm20789.h"
#include "Devices/Drivers/Icm20789-DMP/Icm20789Defs.h"
#include "Devices/Drivers/Icm20789-DMP/Icm20789MPUFifoControl.h"
#include "Invpres.h"
#include "Devices/SensorTypes.h"
#include "Devices/SensorConfig.h"
#include "EmbUtils/InvScheduler.h"
#include "EmbUtils/RingByteBuffer.h"
#include "EmbUtils/Message.h"
#include "EmbUtils/ErrorHelper.h"
#include "EmbUtils/DataConverter.h"
#include "EmbUtils/RingBuffer.h"
#include "DynamicProtocol/DynProtocol.h"
#include "DynamicProtocol/DynProtocolTransportUart.h"

/*******************************************************************************/

/*******************************************************************************/
#include "esp32-hal.h"
//#include "ASF/sam/drivers/pio/pio.h"
//#include "ASF/sam/drivers/pio/pio_handler.h"
//#include "ASF/sam/drivers/twi/twi.h"
//#include "ASF/sam/drivers/tc/tc.h"

/*****************************************************************/

//#include "main.h"
#include "system.h"
#include "sensor.h"
//#include "fw_version.h"

/*****************************************************************/

static const uint8_t dmp3_image[] = {
	#include "icm20789_img.dmp3.h"
};

#if USE_20789_NOT_20689
static const uint8_t EXPECTED_WHOAMI[] = {0x03};  /* WHOAMI value for ICM20789 */
#else
static const uint8_t EXPECTED_WHOAMI[] = {0x98};  /* WHOAMI value for ICM20689 */
#endif

/* FSR configurations */
int32_t cfg_acc_fsr = 4000; // Accel FSR must be set as +/- 4g and cannot be changed. 
int32_t cfg_gyr_fsr = 2000; // Gyro FSR must be set at +/- 2000dps and cannot be changed.

inv_icm20789_t icm_device;
inv_invpres_t invpres_device;

/*
 * Mask to keep track of enabled sensors
 */
uint32_t user_enabled_sensor_mask = 0;

/* Forward declaration */
static void icm20789_apply_mounting_matrix(void);
int icm20789_run_selftest(void);
static void convert_sensor_event_to_dyn_prot_data(const inv_sensor_event_t * event, VSensorDataAny * vsensor_data);
static void check_rc(int rc, const char * context_str);
//static void store_offsets(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3]);
static int handle_command(enum DynProtocolEid eid, const DynProtocolEdata_t * edata, DynProtocolEdata_t * respdata);
//static void notify_event(void * context, uint8_t sensortype, uint64_t timestamp, const void * data, const void *arg);
static void notify_event(void * context, enum inv_icm20789_sensor sensor, uint64_t timestamp, const void * data, const void *arg);
static void handle_data_function(void* context, enum inv_icm20789_sensor sensor, uint64_t timestamp, const void* data, const void* arg);

void sensor_event(const inv_sensor_event_t * event, void * arg);

/*
 * Variable to keep track of the expected period common for all sensors
 */
uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;

#if USE_20789_NOT_20689
static uint32_t pressure_ms = DEFAULT_PRESSURE_ODR_MS; /* 40Hz by default */
#endif

/*
 * Variable to drop the first timestamp(s) after a sensor start cached by the interrupt line.
 * This is needed to be inline with the driver. The first data polled from the FIFO is always discard.
 */
//static uint8_t timestamp_to_drop = 0;

/*
 * Variable keeping track of chip information
 */
static uint8_t chip_info[3];

#if 0
/* Keep a copy of the estimated biases loaded from FLASH in RAM */
static int32_t last_loaded_acc_bias[3] = {0};
static int32_t last_loaded_gyr_bias[3] = {0};
static int32_t last_loaded_mag_bias[3] = {0};
#endif 

/* 
 * Mounting matrix configuration applied for both Accel and Gyro 
 * The coefficient values are coded in integer q30
 */
static float cfg_mounting_matrix[9]= {1.f,   0,   0,
										0, 1.f,   0,
										0,   0, 1.f };

static uint8_t convert_to_generic_ids[INV_ICM20789_SENSOR_MAX] = {
	/*INV_SENSOR_TYPE_ACCELEROMETER,*/
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_CUSTOM_PRESSURE };

/*
 * Dynamic protocol and transport handles
 */
DynProtocol_t protocol;
DynProTransportUart_t transport;

/************************ EMD Hooks *********************************************/
uint64_t InvEMDFrontEnd_getTimestampUs(void)
{
	return ul_ticks * SYSTIMER_PERIOD_US;
}

uint64_t inv_icm20789_get_time_us(void) {
	return InvEMDFrontEnd_getTimestampUs();
}

void InvEMDFrontEnd_busyWaitUsHook(uint32_t us)
{
	delayMicroseconds(us);
	//delay_us(us);
}

int InvEMDFrontEnd_isHwFlowCtrlSupportedHook(void)
{
	return 0;
}

int InvEMDFrontEnd_putcharHook(int c)
{
	//if(usart_serial_putchar(CONF_UART, (uint8_t)c))
	//	return c;
	//else
	//	return -1;
	Serial.write(c);
	return c;
}

/******************************************************************************************/
/*
 * Sleep implementation for ICM20789
 */
void inv_icm20789_sleep(int ms)
{
	delay(ms);
	//delay_ms(ms);
}

void inv_icm20789_sleep_us(int us)
{
	//delay_us(us);
	delayMicroseconds(us);
}

/******************************************************************************************/

int icm20789_sensor_setup(void)
{
	int rc;
	uint8_t i, whoami = 0xff;

	/*
	 * Just get the whoami
	 */
	rc = inv_icm20789_get_whoami(&icm_device, &whoami);

	INV_MSG(INV_MSG_LEVEL_INFO, "Device WHOAMI=0x%02x", whoami);
	check_rc(rc, "Error reading WHOAMI");

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x.", whoami);
		check_rc(-1, "");
	}
	rc = inv_icm20789_get_chip_info(&icm_device, chip_info);
	check_rc(rc, "Could not obtain chip info");

	/*
	 * Configure and initialize the ICM20789 for normal use
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up device...");

	/* set default power mode */
//	if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE) &&
//		!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER)) {
	if(inv_icm20789_all_sensors_off(&icm_device)) {			
		INV_MSG(INV_MSG_LEVEL_INFO, "Putting device in sleep mode...");
		inv_icm20789_init_matrix(&icm_device);
		INV_MSG(INV_MSG_LEVEL_INFO, "inv_icm20789_init_matrix done...");

		icm20789_apply_mounting_matrix();
		INV_MSG(INV_MSG_LEVEL_INFO, "iicm20789_apply_mounting_matrix done...");
		rc = inv_icm20789_initialize(&icm_device, dmp3_image);
		check_rc(rc, "Error while initializing device");
	}


	/* set default ODR = 50Hz */
	rc = inv_icm20789_set_sensor_period(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error while setting ODR for racc");

	rc = inv_icm20789_set_sensor_period(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error while setting ODR for rgyr");

	period_us = DEFAULT_ODR_US;

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	return 0;
}

int icm20789_sensor_configuration(void)
{
	int rc;

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring accelerometer FSR");
	rc = inv_icm20789_set_accel_fullscale(&icm_device, inv_icm20789_accel_fsr_2_reg(cfg_acc_fsr));
	check_rc(rc, "Error configuring ACC sensor");

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring gyroscope FSR");
	rc = inv_icm20789_set_gyro_fullscale(&icm_device, inv_icm20789_gyro_fsr_2_reg(cfg_gyr_fsr));
	check_rc(rc, "Error configuring GYR sensor");

	return rc;
}

/*
 * Helper function to check RC value and block program execution
 */
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}

/*
 * Send response event to host to signal setup is finished
 */
int InvEMDFrontEnd_acknowledgeReset(void)
{
	static DynProtocolEdata_t respEdata; /* static to take on .bss */
	static uint8_t respBuffer[64]; /* static to take on .bss */
	uint16_t respLen;

	respEdata.d.response.rc = 0;
	INV_MSG(INV_MSG_LEVEL_DEBUG, "InvEMDFrontEnd: Acknowledge reset now");

	DynProtocol_encodeResponse(&protocol, DYN_PROTOCOL_EID_RESET, &respEdata,
			respBuffer, sizeof(respBuffer), &respLen);
			
	DynProTransportUart_tx(&transport, respBuffer, respLen);
	
	return INV_ERROR_SUCCESS;
}

/*
 * IddWrapper protocol handler function
 *
 * Will dispatch command and send response back
 */
void iddwrapper_protocol_event_cb(
	enum DynProtocolEtype etype,
	enum DynProtocolEid eid,
	const DynProtocolEdata_t * edata,
	void * cookie
)
{
	(void)cookie;

	static DynProtocolEdata_t resp_edata; /* static to take on .bss */
	static uint8_t respBuffer[256]; /* static to take on .bss */
	uint16_t respLen;
	
	switch(etype) {
		case DYN_PROTOCOL_ETYPE_CMD:
			resp_edata.d.response.rc = handle_command(eid, edata, &resp_edata);

			/* send back response */		
			if(DynProtocol_encodeResponse(&protocol, eid, &resp_edata,
				respBuffer, sizeof(respBuffer), &respLen) != 0) {
				goto error_dma_buffer;
			}
					
			DynProTransportUart_tx(&transport, respBuffer, respLen);
			break;

		default:
			INV_MSG(INV_MSG_LEVEL_WARNING, "DeviceEmdWrapper: unexpected packet received. Ignored.");
			break; /* no suppose to happen */
	}
	return;
	
error_dma_buffer:
	INV_MSG(INV_MSG_LEVEL_WARNING, "iddwrapper_protocol_event_cb: encode error, response dropped");

	return;
}

/*
 * IddWrapper transport handler function
 *
 * This function will:
 *  - feed the Dynamic protocol layer to analyse for incomming CMD packet
 *  - forward byte coming from transport layer to be send over uart to the host
 */
void iddwrapper_transport_event_cb(enum DynProTransportEvent e,
	union DynProTransportEventData data, void * cookie)
{
	(void)cookie;

	int rc;
	int timeout = 5000; /* us */

	switch(e) {
		case DYN_PRO_TRANSPORT_EVENT_ERROR:
			INV_MSG(INV_MSG_LEVEL_ERROR, "ERROR event with value %d received from IddWrapper transport", data.error);
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_PKT_SIZE:
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_PKT_BYTE:
			/* Feed IddWrapperProtocol to look for packet */
			rc = DynProtocol_processPktByte(&protocol, data.pkt_byte);
			if(rc < 0) {
				INV_MSG(INV_MSG_LEVEL_DEBUG, "DynProtocol_processPktByte(%02x) returned %d", data.pkt_byte, rc);
			}
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_PKT_END:
			break;

		/* forward buffer from EMD Transport, to the SERIAL */
		case DYN_PRO_TRANSPORT_EVENT_TX_START:
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_TX_BYTE:
			while ((InvEMDFrontEnd_putcharHook(data.tx_byte) == EOF) && (timeout > 0)) {
				InvEMDFrontEnd_busyWaitUsHook(10);
				timeout -= 10;
			}
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_TX_END:
			break;
			
		case DYN_PRO_TRANSPORT_EVENT_TX_START_DMA:
			break;	
	}
}

int sensor_configure_odr(uint32_t odr_us)
{
	int rc = 0;

	/* All sensors running at the same rate */

	/* Do not reconfigure the rate if it's already applied */
	if(odr_us == period_us)
		return rc;

	if(odr_us < MIN_ODR_US)
		odr_us = MIN_ODR_US;

	if(odr_us > MAX_ODR_US)
		odr_us = MAX_ODR_US;

	/* FIFO has been reset by ODR change */
	if (rc == 0) {
		//pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
	
		/* Clear any remaining interrupts */
		//__disable_irq();
		irq_from_device = 0;
		//__enable_irq();
	}
	
	/* Keep track in static variable of the odr value for further algorihtm use */
	period_us = odr_us;

	return rc;
}

int sensor_control(int enable)
{
	int rc = 0;
	static uint8_t sensors_on = 0;

	/* Keep track of the sensors state */
	if(enable && sensors_on)
		return rc;

	if(enable)
		sensors_on = 1;
	else 
		sensors_on = 0;

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	if (enable) {
		/* Clock is more accurate when gyro is enabled, so let's enable it first to prevent side effect at startup */
		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE, 1);

		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE, 1);
		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE_UNCALIBRATED))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE_UNCALIBRATED, 1);
			
		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER, 1);
        //		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER))
        //			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER, 1);

		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GAME_ROTATION_VECTOR))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GAME_ROTATION_VECTOR, 1);
			
		/*
		 * There is a situation where two samples need to be dropped: if
		 * accelerometer is enable before gyroscope first interrupt triggers,
		 * both interrupts are raised causing the odr to be wrong if only one
		 * sample is dropped.
		 * We are in this exact situation since both sensors are enabled one after
		 * the other.
		 */
		//timestamp_to_drop = 2; //todo: is this needed? -- Qing
	} else {
		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER, 0);
//		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER, 0);
		
		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE, 0);		
		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE, 0);
		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE_UNCALIBRATED, 0);

		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GAME_ROTATION_VECTOR, 0);		
	}

	/* Clear the remaining items in the IRQ timestamp buffer when stopping all sensors */
	if(inv_icm20789_all_sensors_off(&icm_device)) 
		//pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
	

	return rc;
}

static float angle[3];

/*
* Poll devices for data
*/
float* Icm20789_data_poll(void) {
	int rc = 0;
	int16_t int_read_back = 0;
	//INV_MSG(INV_MSG_LEVEL_INFO, "pulling data ...");
	/*
	 *  Ensure data ready status
	 */
	inv_icm20789_identify_interrupt(&icm_device, &int_read_back);

	if (int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_2 | BIT_MSG_DMP_INT_3)) {
		//inv_icm20789_poll_sensor(&icm_device, (void *)0, notify_event);
		//INV_MSG(INV_MSG_LEVEL_INFO, "int_read_back  ...");
		inv_icm20789_poll_sensor(&icm_device, (void*)0, handle_data_function);

		return angle;
	}
	else
	{
		return NULL;
	}

	//__disable_irq();
	irq_from_device = 0;
	//__enable_irq();
	
	//return rc;
}

static void icm20789_apply_mounting_matrix(void)
{
	int ii;
	
	for (ii = 0; ii < INV_ICM20789_SENSOR_MAX; ii++) {
		inv_icm20789_set_matrix(&icm_device, cfg_mounting_matrix, (inv_icm20789_sensor)ii);
	}
}

float pitch_save, yaw_save, roll_save;
void handle_data_function(void* context, inv_icm20789_sensor sensortype, uint64_t timestamp, const void* data, const void* arg)
{
	float raw_bias_data[6];
	float quad[4] = { 0 };
	uint8_t sensor_id = convert_to_generic_ids[sensortype];
	//INV_MSG(INV_MSG_LEVEL_INFO, "sensor_id %d\r\n", sensor_id);

	if (sensor_id == INV_SENSOR_TYPE_GAME_ROTATION_VECTOR)
	{
		memcpy(quad, data, sizeof(quad));
		angle[1] = asin(-2 * quad[1] * quad[3] + 2 * quad[0] * quad[2]) * 57.3;	// pitch
		angle[2] = atan2(2 * quad[2] * quad[3] + 2 * quad[0] * quad[1], -2 * quad[1] * quad[1] - 2 * quad[2] * quad[2] + 1) * 57.3;	// roll
		angle[0] = atan2(2 * (quad[1] * quad[2] + quad[0] * quad[3]), quad[0] * quad[0] + quad[1] * quad[1] - quad[2] * quad[2] - quad[3] * quad[3]) * 57.3;	//yaw
	}
	//INV_MSG(INV_MSG_LEVEL_INFO, "%f %f %f\r\n",angle[0], angle[1], angle[2]);
}



void notify_event(void * context, enum inv_icm20789_sensor sensor, uint64_t timestamp, const void * data, const void *arg)
{
	float raw_bias_data[6];
	inv_sensor_event_t event;
	uint8_t sensor_id = convert_to_generic_ids[sensor];
	
	memset(&event, 0, sizeof(event));

	event.sensor = sensor_id;
	event.timestamp = timestamp;
	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;

	switch(sensor_id) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
			INV_MSG(INV_MSG_LEVEL_DEBUG, "%d %d %d\r\n", /*(long)data[0], (long)data[1], (long)data[2],*/ 
			event.data.raw3d.vect[0],event.data.raw3d.vect[1],event.data.raw3d.vect[2]);
			break;
		case INV_SENSOR_TYPE_ACCELEROMETER:
			memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
			memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			memcpy(raw_bias_data, data, sizeof(raw_bias_data));
			memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
			memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
			memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
			memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
			event.data.quaternion.accuracy = 0;
			memcpy(&(event.data.quaternion.accuracy_flag), arg, sizeof(event.data.quaternion.accuracy_flag));
			break;
			
		default:
			return;
	}
	sensor_event(&event, NULL);
}

void sensor_event(const inv_sensor_event_t * event, void * arg)
{
	/* arg will contained the value provided at init time */
	(void)arg;

	/*
	 * Encode sensor event and sent to host over UART through IddWrapper protocol
	 */
	static DynProtocolEdata_t async_edata; /* static to take on .bss */
	static uint8_t async_buffer[256]; /* static to take on .bss */
	uint16_t async_bufferLen;
	
	async_edata.sensor_id = event->sensor;
	async_edata.d.async.sensorEvent.status = DYN_PRO_SENSOR_STATUS_DATA_UPDATED;
	convert_sensor_event_to_dyn_prot_data(event, &async_edata.d.async.sensorEvent.vdata);

	if(DynProtocol_encodeAsync(&protocol,
			DYN_PROTOCOL_EID_NEW_SENSOR_DATA, &async_edata,
			async_buffer, sizeof(async_buffer), &async_bufferLen) != 0) {
		goto error_dma_buf;
	}
	
	DynProTransportUart_tx(&transport, async_buffer, async_bufferLen);
	return;

error_dma_buf:
	INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: encode error, frame dropped");
	
	return;
}

/*
 * Convert sensor_event to VSensorData because dynamic protocol transports VSensorData
 */
static void convert_sensor_event_to_dyn_prot_data(const inv_sensor_event_t * event, VSensorDataAny * vsensor_data)
{
	vsensor_data->base.timestamp = event->timestamp;

	switch(event->sensor) {
	case DYN_PRO_SENSOR_TYPE_RESERVED:
		break;
	case DYN_PRO_SENSOR_TYPE_GRAVITY:
	case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:
	case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
		inv_dc_float_to_sfix32(&event->data.acc.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.acc.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
		inv_dc_float_to_sfix32(&event->data.gyr.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.gyr.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:
		inv_dc_float_to_sfix32(&event->data.gyr.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		inv_dc_float_to_sfix32(&event->data.gyr.bias[0], 3, 16, (int32_t *)&vsensor_data->data.u32[3]);
		vsensor_data->base.meta_data = event->data.gyr.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_0:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_1:
	case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		inv_dc_float_to_sfix32(&event->data.quaternion.quat[0], 4, 30, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.quaternion.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE:
		vsensor_data->data.u32[0] = event->data.raw3d.vect[0];
		vsensor_data->data.u32[1] = event->data.raw3d.vect[1];
		vsensor_data->data.u32[2] = event->data.raw3d.vect[2];
		break;
	case DYN_PRO_SENSOR_TYPE_CUSTOM_PRESSURE:
		// raw pressure
		vsensor_data->data.u32[0] = event->data.custom_pressure.raw_pressure;
		// pressure
		inv_dc_float_to_sfix32(&event->data.custom_pressure.pressure, 1, 8, (int32_t *)&vsensor_data->data.u32[1]);
		// raw temperature
		vsensor_data->data.u32[2] = event->data.custom_pressure.raw_temperature;
		// temperature
		inv_dc_float_to_sfix32(&event->data.custom_pressure.temperature, 1, 8, (int32_t *)&vsensor_data->data.u32[3]);
		break;
	default:
		break;
	}
}

enum inv_icm20789_sensor idd_sensortype_conversion(int sensor)
{
	switch(sensor) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:      return INV_ICM20789_SENSOR_RAW_ACCELEROMETER;
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:          return INV_ICM20789_SENSOR_RAW_GYROSCOPE;
//		case INV_SENSOR_TYPE_ACCELEROMETER:          return INV_ICM20789_SENSOR_ACCELEROMETER;
		case INV_SENSOR_TYPE_GYROSCOPE:              return INV_ICM20789_SENSOR_GYROSCOPE;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:        return INV_ICM20789_SENSOR_GYROSCOPE_UNCALIBRATED;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:   return INV_ICM20789_SENSOR_GAME_ROTATION_VECTOR;
#if USE_20789_NOT_20689
		case INV_SENSOR_TYPE_CUSTOM_PRESSURE:		 return INV_ICM20789_SENSOR_CUSTOM_PRESSURE;
#endif
		
		default:                                     return INV_ICM20789_SENSOR_MAX;
	}
}

static int handle_command(enum DynProtocolEid eid, const DynProtocolEdata_t * edata, DynProtocolEdata_t * respdata)
{
	int rc = 0;
	uint8_t whoami;
	const int sensor = edata->sensor_id;

	switch(eid) {

		case DYN_PROTOCOL_EID_GET_SW_REG:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command GET_SW_REG");
			if(edata->d.command.regAddr == DYN_PROTOCOL_EREG_HANDSHAKE_SUPPORT)
				return InvEMDFrontEnd_isHwFlowCtrlSupportedHook();
			return 0;

		case DYN_PROTOCOL_EID_SETUP:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command SETUP");
			/* Re-init the device */
			rc = icm20789_sensor_setup();
			rc += icm20789_sensor_configuration();

			//sensor_configure_odr(period_us);
			/* Enable all sensors but.. */
			//rc += sensor_control(1);
			/* .. no sensors are reporting on setup */
			user_enabled_sensor_mask = 0;
			return rc;

		case DYN_PROTOCOL_EID_WHO_AM_I:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command WHO_AM_I");
			rc = inv_icm20789_get_whoami(&icm_device, &whoami);
			return (rc == 0) ? whoami : rc;

		case DYN_PROTOCOL_EID_RESET:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command RESET");
			/* --- Cleanup --- */
			rc = sensor_control(0);
			/* Soft reset */
			rc += inv_icm20789_soft_reset(&icm_device);
			/* --- Setup --- */
			/* Re-init the device */
			rc += icm20789_sensor_setup();
			rc += icm20789_sensor_configuration();
			//sensor_configure_odr(period_us);

			/* Enable all sensor but.. */
			//rc += sensor_control(1);
			/* All sensors stop reporting on reset */
			user_enabled_sensor_mask = 0;
			return rc;

		case DYN_PROTOCOL_EID_PING_SENSOR:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command PING_SENSOR(%s)", inv_sensor_2str(sensor));
			if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER)			
			|| (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE)
			/*|| (sensor == INV_SENSOR_TYPE_ACCELEROMETER)*/
			|| (sensor == INV_SENSOR_TYPE_GYROSCOPE)
			|| (sensor == INV_SENSOR_TYPE_UNCAL_GYROSCOPE)
			|| (sensor == INV_SENSOR_TYPE_GAME_ROTATION_VECTOR)
#if USE_20789_NOT_20689
			|| (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE)
#endif
			) {
				return 0;
			} else
				return INV_ERROR_BAD_ARG;

		case DYN_PROTOCOL_EID_SELF_TEST:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command SELF_TEST(%s)", inv_sensor_2str(sensor));
			if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER) ||
			   (sensor == INV_SENSOR_TYPE_ACCELEROMETER) ||
			   (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE) ||
			   (sensor == INV_SENSOR_TYPE_GYROSCOPE))
				return icm20789_run_selftest();
			else
				return INV_ERROR_NIMPL;

		case DYN_PROTOCOL_EID_START_SENSOR:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command START_SENSOR(%s)", inv_sensor_2str(sensor));
			if (sensor > 0 && idd_sensortype_conversion(sensor) < INV_ICM20789_SENSOR_MAX) {
				/* Sensor data will be notified */
#if USE_20789_NOT_20689
				if (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE) { 
					set_pressure_timer((MICROSECONDS_PER_SECOND / SYSTIMER_PERIOD_US) * pressure_ms / 1000);
					inv_invpres_enable_sensor(&invpres_device, 1);
				}
				else 
#endif
				{
					inv_icm20789_enable_sensor(&icm_device, idd_sensortype_conversion(sensor), 1);
				}
				user_enabled_sensor_mask |= (1 << idd_sensortype_conversion(sensor));
				return 0;
			} else
				return INV_ERROR_NIMPL; /*this sensor is not supported*/

		case DYN_PROTOCOL_EID_STOP_SENSOR:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command STOP_SENSOR(%s)", inv_sensor_2str(sensor));
			if (sensor > 0 && idd_sensortype_conversion(sensor) < INV_ICM20789_SENSOR_MAX) {
				/* Sensor data will not be notified anymore */
#if USE_20789_NOT_20689
				if (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE && (user_enabled_sensor_mask & (1 << INV_ICM20789_SENSOR_CUSTOM_PRESSURE))) {
					inv_invpres_enable_sensor(&invpres_device, 0);
				}
				else 
#endif				
				{
					inv_icm20789_enable_sensor(&icm_device, idd_sensortype_conversion(sensor), 0);
				}
				user_enabled_sensor_mask &= ~(1 << idd_sensortype_conversion(sensor));
				return 0;
			} else
				return INV_ERROR_NIMPL; /*this sensor is not supported*/

		case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:
		{	
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command SET_SENSOR_PERIOD(%d us)",edata->d.command.period);
#if USE_20789_NOT_20689
			if (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE) {
				pressure_ms = edata->d.command.period / 1000;
				if ((pressure_ms) > MAX_PRESSURE_ODR_MS)	// Min 1000 ms (or 1 Hz)
					pressure_ms = MAX_PRESSURE_ODR_MS;
				else if ((pressure_ms) < MIN_PRESSURE_ODR_MS) // Max 25ms (or 40 Hz)
					pressure_ms = MIN_PRESSURE_ODR_MS;
				set_pressure_timer((MICROSECONDS_PER_SECOND / SYSTIMER_PERIOD_US) * pressure_ms / 1000);
			}
			else 
#endif
			{
				rc = sensor_configure_odr(edata->d.command.period);
				inv_icm20789_set_sensor_period(&icm_device, idd_sensortype_conversion(sensor), period_us / 1000);
			}
			return rc;
		}

		case DYN_PROTOCOL_EID_SET_SENSOR_CFG:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command SET_SENSOR_CFG(%s)", inv_sensor_2str(sensor));
			switch(edata->d.command.cfg.base.type) {			
				case VSENSOR_CONFIG_TYPE_REFERENCE_FRAME:
				{
					intq30_t q30_mmatrix[9];
				
					/* Ensure any float manipulation is word-aligned because of unsupported unaligned float access depending on CPU target */
					memcpy(q30_mmatrix, edata->d.command.cfg.buffer, edata->d.command.cfg.base.size);
				
					if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER)
					|| (sensor == INV_SENSOR_TYPE_ACCELEROMETER)
					|| (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE)
					|| (sensor == INV_SENSOR_TYPE_GYROSCOPE)
					|| (sensor == INV_SENSOR_TYPE_UNCAL_GYROSCOPE)) {
						inv_dc_sfix32_to_float(q30_mmatrix, 9, 30, cfg_mounting_matrix);
						icm20789_apply_mounting_matrix();
						return 0;
					}

					else return INV_ERROR_BAD_ARG;
				}
			
				default:
					return INV_ERROR_NIMPL;
			}
		
		case DYN_PROTOCOL_EID_CLEANUP:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command CLEANUP");
			rc = sensor_control(0);
			/* Soft reset */
			rc += inv_icm20789_soft_reset(&icm_device);

			/* All sensors stop reporting on cleanup */
			user_enabled_sensor_mask = 0;
			return rc;

		default:
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command UNKNOWN");
			return INV_ERROR_NIMPL;
	}
}

#if 0
void store_offsets(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3])
{
	uint8_t i, idx = 0;
	int raw_bias[12] = {0};
	uint8_t sensor_bias[84] = {0};
	
	/* Store offsets in NV memory */
	inv_icm20789_get_st_bias(&icm_device, raw_bias);
	/* Store ST biases: 3(axis) * 4(AccLP, AccLN, GyrLP, GyrLN) * 4(uint32_t) = 48 B [total=48 B] */
	for(i = 0; i < 12; i++)
		inv_dc_int32_to_little8(raw_bias[i], &sensor_bias[i * sizeof(uint32_t)]);
	idx += sizeof(raw_bias);
	
	/* Store Calib Accel biases: 3(axis) * 4(uint32_t) = 12 B [total=60 B] */
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(acc_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += (sizeof(acc_bias_q16[0]) * 3);
	
	/* Store Calib Gyro biases: 3(axis) * 4(uint32_t) = 12 B [total=72 B] */
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(gyr_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += (sizeof(gyr_bias_q16[0]) * 3);
}
#endif

int icm20789_run_selftest(void)
{
	int raw_bias[12];
	int rc = 0;

	if (icm_device.selftest_done == 1) {
		INV_MSG(INV_MSG_LEVEL_INFO, "Self-test has already run. Skipping.");
	}
	else {
		/* 
		 * Perform self-test
		 * For ICM20789 self-test is performed for both RAW_ACC/RAW_GYR
		 */
		INV_MSG(INV_MSG_LEVEL_INFO, "Running self-test...");

		/* Run the self-test */
		rc = inv_icm20789_run_selftest(&icm_device);
		/* Check transport errors */
		check_rc(rc, "Self-test failure");
		if (rc != 0x3) {
			/*
			 * Check for GYR success (1 << 0) and ACC success (1 << 1),
			 * but don't block as these are 'usage' failures.
			 */
			INV_MSG(INV_MSG_LEVEL_ERROR, "Self-test failure");
			/* 0 would be considered OK, we want KO */
			return INV_ERROR;
		} else {
			/* On success, offset will be kept until reset */
			icm_device.selftest_done = 1;
			rc = 0;
		}
	}

	/* 
	 * Get Low Noise / Low Power bias computed by self-tests scaled by 2^16
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Getting LP/LN bias");
	inv_icm20789_get_st_bias(&icm_device, raw_bias);
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
			(float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LP bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
			(float)(raw_bias[3] / (float)(1 << 16)), (float)(raw_bias[4] / (float)(1 << 16)), (float)(raw_bias[5] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (FS=2g) (g): x=%f, y=%f, z=%f", 
			(float)(raw_bias[0 + 6] / (float)(1 << 16)), (float)(raw_bias[1 + 6] / (float)(1 << 16)), (float)(raw_bias[2 + 6] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LP bias (FS=2g) (g): x=%f, y=%f, z=%f", 
			(float)(raw_bias[3 + 6] / (float)(1 << 16)), (float)(raw_bias[4 + 6] / (float)(1 << 16)), (float)(raw_bias[5 + 6] / (float)(1 << 16)));

	return rc;
}

