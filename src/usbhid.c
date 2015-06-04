/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

/* Define this to include the DFU APP interface. */

#if 0
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>
#endif

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5710,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	},
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 0, /* not  boot */
	.bInterfaceProtocol = 0, /* not mouse */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"HID Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];
#define USB_HID_GET_REPORT   0x01
#define USB_HID_GET_IDLE     0x02
#define USB_HID_GET_PROTOCOL 0x03
#define USB_HID_SET_REPORT   0x09
#define USB_HID_SET_IDLE     0x0A
#define USB_HID_SET_PROTOCOL 0x0B
#define USB_HID_REPORT_INPUT 0x01
#define USB_HID_REPORT_OUTPUT 0x02 

volatile uint8_t report_buffer[128]= {0,10,'T','e','s','t',0};
volatile uint8_t tried=0;


static int hid_class_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)usbd_dev;

    gpio_toggle(GPIOA,GPIO9);
    uint8_t wValueH =(uint8_t)(req->wValue >>8);
    uint8_t wValueL = (uint8_t)(req->wValue & 0xff);
      switch(req->bRequest){
        case USB_HID_GET_REPORT:
            if(wValueL==0 && (wValueH == USB_HID_REPORT_INPUT || wValueH == 0x03 )){
              *buf = (uint8_t*)&report_buffer;
              *len = sizeof(report_buffer);
              return 1;
            }else{
              return 2;
            }
            break;
        case USB_HID_SET_REPORT:
            return 2;
            break;
        default: 
              return 2;
      }
}

static int hid_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)usbd_dev;


	if ((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

	return 1;
}

static void hid_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	(void)usbd_dev;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

    usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_CLASS, USB_REQ_TYPE_TYPE,hid_class_request);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT | USB_REQ_TYPE_DIRECTION,
				hid_control_request);

    gpio_toggle(GPIOB,GPIO1);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(99999);
	systick_interrupt_enable();
	systick_counter_enable();
}
#define MY_ADC_CHANNEL ADC_CHANNEL0

inline void adc_set_single_channel(uint32_t adc,uint8_t channel){
  uint8_t channels[16];
  channels[0]=channel;
  adc_set_regular_sequence(adc,1,channels);
}

static void setup_timer(void)
{
	/* Set up the timer TIM2 for injected sampling */
	uint32_t timer;

	timer   = TIM3;
	rcc_periph_clock_enable(RCC_TIM3);

	/* Time Base configuration */
    timer_reset(timer);
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
	    TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(timer, 0xFF);
    timer_set_prescaler(timer, 0x8);
    timer_set_clock_division(timer, 0x0);
    /* Generate TRGO on every update. */
    timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);
    timer_enable_counter(timer);
}

static void setup_adc(void){
    rcc_periph_clock_enable(RCC_ADC1);
    adc_off(ADC1);


    nvic_set_priority(NVIC_ADC1_2_IRQ, 0);
    nvic_enable_irq(NVIC_ADC1_2_IRQ);

    adc_disable_scan_mode(ADC1);
    //adc_set_continuous_conversion_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_enable_external_trigger_regular(ADC1,ADC_CR2_EXTSEL_TIM3_TRGO);
 //   adc_enable_eoc_interrupt(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time(ADC1, MY_ADC_CHANNEL, ADC_SMPR_SMP_55DOT5CYC);
    adc_set_single_channel(ADC1,MY_ADC_CHANNEL);

    adc_power_on(ADC1);
	/* Wait for ADC starting up. */
    int i;
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");
    adc_reset_calibration(ADC1);
    adc_calibration(ADC1);
    adc_enable_eoc_interrupt(ADC1);
//	adc_start_conversion_regular(ADC1);
}

static usbd_device *usbd_dev;

int main(void)
{
	int i;


	rcc_clock_setup_in_hse_8mhz_out_72mhz();

    


    /*External test led (not on the board*/
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);

    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO0);
    gpio_set(GPIOA,GPIO9);
    gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,
                      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
    gpio_clear(GPIOB,GPIO1);

    /*Enable transistor switch to make usb autodetect work*/
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_OPENDRAIN, GPIO9);
    gpio_set(GPIOB, GPIO9);

    setup_adc();
    setup_timer();
    /*
    uint16_t value = adc_read_regular(ADC1);
    report_buffer[0]=value && 0xff;
    report_buffer[1]=(value>>8) && 0xff;
    */
	for (i = 0; i < 0x80000; i++)
		__asm__("nop");
	gpio_clear(GPIOB, GPIO9);

	usbd_dev = usbd_init(&stm32f103_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

	for (i = 0; i < 0x80000; i++)
		__asm__("nop");

	//gpio_clear(GPIOA, GPIO9);

	while (1)
		usbd_poll(usbd_dev);
}

void adc1_2_isr(void){
  uint16_t value = adc_read_regular(ADC1);
  report_buffer[2] = 5;
  report_buffer[0]=value &0xff;
  report_buffer[1]=(value>>8) &0xff; 
  ADC_SR(ADC1) &= ~ADC_SR_EOC; // Clear EOC flag
}

volatile uint32_t counter=0;
void sys_tick_handler(void)
{
  /*
  if(tried){
    gpio_set(GPIOA,GPIO9);
  }
  else{
    gpio_clear(GPIOA,GPIO9);
  }
  counter++;
  if(counter>200){
    counter=0;
    gpio_toggle(GPIOA,GPIO9);
  }
  */
}
