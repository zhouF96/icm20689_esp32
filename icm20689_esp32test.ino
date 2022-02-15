/*
    Name:       icm20689_esp32test.ino
    Created:	2022/02/10 17:44:54
    Author:     ZEPHYRUS-CHOU\jncho
*/
#define INV_MSG_ENABLE INV_MSG_LEVEL_VERBOSE
#define ICM_EXT_IRQ 1
volatile int irq = 0;
/* InvenSense drivers and utils */
#include "Invn/Devices/Drivers/Icm20789-DMP/Icm20789.h"
#include "Invn/Invpres.h"
#include "Invn/Devices/SensorTypes.h"
#include "Invn/Devices/SensorConfig.h"
#include "Invn/EmbUtils/InvScheduler.h"
#include "Invn/EmbUtils/RingByteBuffer.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/EmbUtils/RingBuffer.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"
/* InvenSense drivers and utils */

/* InvenSense user configs */
//#include "Invn/icm2689_usercfg.h"
/* InvenSense user configs */


//#include<Serial>

#include "Invn/system.h"
#include "Invn/sensor.h"
#include <Wire.h>

static InvSchedulerTask commandHandlerTask;
static InvSchedulerTask blinkerLedTask;

void setup()
{
    Serial.begin(115200);
    pinMode(7, OUTPUT);
    pinMode(chipSelectPin, OUTPUT);
    pinMode(intPin, INPUT_PULLUP);
    //pinMode(0, INPUT);

    //attachInterrupt(0, irqhandler, RISING);
    //pinMode()
#ifdef INV_MSG_ENABLE
	INV_MSG_SETUP(INV_MSG_ENABLE, msg_printer);
#endif // INV_MSG_ENABLE

    //pinMode(35, PULLUP);
    //pinMode(36, PULLUP);
    //digitalWrite(36, HIGH);
    //digitalWrite(35, HIGH);
    //digitalWrite(7, LOW);
    //digitalWrite(36, LOW);
    //digitalWrite(35, LOW);
    //digitalWrite(7, HIGH);
    //delay(1000);
    //digitalWrite(2, LOW);
    digitalWrite(7, LOW);
    delay(1000);
    digitalWrite(chipSelectPin,1);
    //delay(500);

    Serial.println("configure_console ...");
    configure_console();
    Serial.println("done ...");
    Serial.println("interface_initialize ...");
    interface_initialize();
    Serial.println("done ...");
#ifdef ICM_EXT_IRQ	
    /* Initialize External Sensor Interrupt */
    //ext_int_initialize(ext_interrupt_handler);
#endif
	/*
	* Initialize icm20789 serif structure
	*/
    struct inv_icm20789_serif icm20789_serif;
    icm20789_serif.context = 0; /* no need */
    icm20789_serif.read_reg = idd_io_hal_read_reg;
    icm20789_serif.write_reg = idd_io_hal_write_reg;
    icm20789_serif.max_read = 1024 * 16; /* maximum number of bytes allowed per serial read */
    icm20789_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */
     /*
     * Init SPI/I2C communication
     */
    if (USE_SPI_NOT_I2C) {
        icm20789_serif.is_spi = 1;
        INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through SPI");
    }
	else {
        Wire.begin(MOSIPIN,SCLKPIN);
		//Wire.begin(33,25);
		Wire.setClock(400000);
        //Wire.setTimeOut(5000);
		icm20789_serif.is_spi = 0;
		INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through I2C");


		byte error, address;
		int nDevices = 0;
		Serial.println("Scanning for I2C devices ...");
		for (address = 0x01; address < 0x7f; address++) {
			Wire.beginTransmission(address);
			error = Wire.endTransmission();
			if (error == 0) {
				Serial.printf("I2C device found at address 0x%02X\n", address);
				nDevices++;
			}
			else if (error != 2) {
				Serial.printf("Error %d at address 0x%02X\n", error, address);
			}
		}
		if (nDevices == 0) {
			Serial.println("No I2C devices found");
		}
	}
    /*
    * Reset icm20789 driver states
    */
    inv_icm20789_reset_states(&icm_device, &icm20789_serif);

    //Serial.println("Setup the icm20789 device ...");
    /*
     * Setup the icm20789 device
     */
    icm20789_sensor_setup();
    //Serial.println("done ...");
    //Serial.println("config   icm20789...");
    icm20789_sensor_configuration();
    icm20789_run_selftest();
#if defined (ICM_EXT_IRQ)
    attachInterrupt(intPin, irqhandler, RISING);
#endif // defined (ICM_EXT_IRQ)
    sensor_control(1);
    //Serial.println("done ...");
#if USE_20789_NOT_20689

    /*
    * Initialize pressure sensor serif structure
    */
    struct inv_invpres_serif invpres_serif;

    invpres_serif.context = 0; /* no need */
    invpres_serif.read_reg = invpres_hal_read_reg;
    invpres_serif.write_reg = invpres_hal_write_reg;
    invpres_serif.max_read = 64; /* maximum number of bytes allowed per serial read */
    invpres_serif.max_write = 64; /* maximum number of bytes allowed per serial write */
    invpres_serif.is_spi = 0;

    /*
     * Reset pressure sensor driver states
     */
    inv_invpres_reset_states(&invpres_device, &invpres_serif);

    inv_invpres_setup(&invpres_device);
#endif
    //Serial.println("Initialize Dynamic protocol stuff ...");
    /*
     * Initialize Dynamic protocol stuff
     */
    //DynProTransportUart_init(&transport, iddwrapper_transport_event_cb, 0);
    //DynProtocol_init(&protocol, iddwrapper_protocol_event_cb, 0);
    //Serial.println("done ...");

    //InvScheduler_init(&scheduler);
    //InvScheduler_initTask(&scheduler, &commandHandlerTask, "commandHandlerTask", CommandHandlerTaskMain, 0, INVSCHEDULER_TASK_PRIO_MIN, INVSCHEDULER_PERIOD_US / SYSTIMER_PERIOD_US);
    //InvScheduler_initTask(&scheduler, &blinkerLedTask, "blinkerLedTask", BlinkerLedTaskMain, 0, INVSCHEDULER_TASK_PRIO_MIN + 1, 1000000 / SYSTIMER_PERIOD_US);
    //InvScheduler_startTask(&blinkerLedTask, 0);
    //InvScheduler_startTask(&commandHandlerTask, 0);
    InvEMDFrontEnd_acknowledgeReset();

}
float yaw, pitch, roll = 0;
void loop()
{
    //if (Serial.available())
    //{
    //    uint8_t rxbyte;
    //    Serial.readBytes(&rxbyte, 1);
    //    //Serial.print(rxbyte);
    //    if (!RingByteBuffer_isFull(&uart_rx_rb))
    //        RingByteBuffer_pushByte(&uart_rx_rb, rxbyte);
    //}
    //INV_MSG(INV_MSG_LEVEL_INFO, "pin 0 from_device: %d ", digitalRead(0));
    //INV_MSG(INV_MSG_LEVEL_INFO, "irq pin1 from_device: %d ", digitalRead(1));
    //INV_MSG(INV_MSG_LEVEL_INFO, "irq : %d ", irq);
    //delay(1000);
    //Serial.println("loop ...");
    //InvScheduler_dispatchTasks(&scheduler);
    //CommandHandlerTaskMain(0);
    //delay(5000);
#ifdef ICM_EXT_IRQ
    if (irq == 1)
    {
        INV_MSG(INV_MSG_LEVEL_INFO, "IRQ got, pulling data ...");
#endif
        
        get_dmp_data(&yaw,&pitch, &roll);
        INV_MSG(INV_MSG_LEVEL_INFO, "yaw: %f pitch: %f roll: %f ", yaw, pitch, roll);
        //delay(1000);
        //Icm20789_data_poll();
#ifdef ICM_EXT_IRQ
        //__disable_irq();
        //detachInterrupt(1);
        irq = 0;
        //__enable_irq();
        //attachInterrupt(1, ext_interrupt_handler, RISING);
    }
#endif
    

}

static void BlinkerLedTaskMain(void* arg)
{
    (void)arg;
    static uint8_t value = 1;
    digitalWrite(13, value);
    value = !value;
    //ioport_toggle_pin_level(LED_0_PIN);
}
/*
 * CommandHandlerTaskMain - Task that monitors the UART
 */
static void CommandHandlerTaskMain(void* arg)
{
    (void)arg;
    //INV_MSG(INV_MSG_LEVEL_INFO, "CommandHandlerTaskMain ...");
    int byte;
    do {
        byte = EOF;
        //__disable_irq();

        if (!RingByteBuffer_isEmpty(&uart_rx_rb)) {
            byte = RingByteBuffer_popByte(&uart_rx_rb);
        }
        //__enable_irq();

        if (byte != EOF) {
            DynProTransportUart_rxProcessByte(&transport, (uint8_t)byte);
        }
    } while (byte != EOF);
}

static void msg_printer(int level, const char* str, va_list ap)
{
#ifdef INV_MSG_ENABLE
    static char out_str[256]; /* static to limit stack usage */
    unsigned idx = 0;
    const char* ptr = out_str;
    const char* s[INV_MSG_LEVEL_MAX] = {
        "",    // INV_MSG_LEVEL_OFF
        "[E] ", // INV_MSG_LEVEL_ERROR
        "[W] ", // INV_MSG_LEVEL_WARNING
        "[I] ", // INV_MSG_LEVEL_INFO
        "[V] ", // INV_MSG_LEVEL_VERBOSE
        "[D] ", // INV_MSG_LEVEL_DEBUG
    };
    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
    if (idx >= (sizeof(out_str)))
        return;
    idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
    if (idx >= (sizeof(out_str)))
        return;
    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
    if (idx >= (sizeof(out_str)))
        return;

    while (*ptr != '\0') {
        Serial.print(*ptr);
        ++ptr;
    }
#else

    (void)level, (void)str, (void)ap;

#endif
}


unsigned char get_dmp_data(float* yaw, float* pitch, float* roll)
{
    float* point = Icm20789_data_poll();

    if (point)
    {
        *yaw = *point;
        *pitch = *(point + 1);
        *roll = *(point + 2);
        return 0;
    }
    return 1;
    irq = 0;
}


void ARDUINO_ISR_ATTR irqhandler(void)
{
    irq = 1;
    //get_dmp_data(&yaw, &pitch, &roll);
}