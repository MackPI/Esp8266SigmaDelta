/******************************************************************************
 * MIT License 2016 Prototype Iteration(Steve McNeil)
 *
 * FileName: user_main.c
 *
 * Description: Hello World example for Sigma Delta testing
 *
 * Modification history:
 *     2016/06/23, v00A create this file.
*******************************************************************************/

#include "osapi.h"
#include "user_interface.h"
#include "gpio.h"

#define GPIO_SIGMA_DELTA	0x68  //defined in gpio register.xls
#define GPIO_PIN4			0x38

#define HOST_INF_SEL 0x3ff00028
#define reg_cspi_overlap  (BIT7)
#define SPI 			0
#define HSPI			1

#define REG_SPI_BASE(i)  (0x60000200-i*0x100)
#define SPI_USER(i)                           (REG_SPI_BASE(i)  + 0x1C)
#define SPI_EXT3(i)                           (REG_SPI_BASE(i)  + 0xFC)

uint8 prescaler = 0;
uint8 duty = 0;
os_timer_t timer0;
bool firstTime = true;
void one_second(){
	uint32 analog;
//	duty = prescaler;
	analog = system_adc_read();
	os_printf("%03d-",duty);
	os_printf("%d.%02d ",(analog*21)/1024,(analog*20)%1024);
	duty++;
	GPIO_REG_WRITE(GPIO_SIGMA_DELTA, 0x10000 | (prescaler<<8) | duty); // set Target Prescaler and duty cycle
//	if (firstTime)  // only request connect once
//		wifi_station_connect();
//	firstTime = false;

}

hspi_overlap_init(void)
{
	//hspi overlap to spi, two spi masters on cspi
	SET_PERI_REG_MASK(HOST_INF_SEL, reg_cspi_overlap);

	//set higher priority for spi than hspi
	SET_PERI_REG_MASK(SPI_EXT3(SPI),0x1);
	SET_PERI_REG_MASK(SPI_EXT3(HSPI),0x3);
	SET_PERI_REG_MASK(SPI_USER(HSPI), BIT(5));
}
void ICACHE_FLASH_ATTR
gpio16_output_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC to output rtc_gpio0

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   (READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe) | (uint32)0x1);	//out enable
}

void ICACHE_FLASH_ATTR
gpio16_output_set(uint8 value)
{
    WRITE_PERI_REG(RTC_GPIO_OUT,
                   (READ_PERI_REG(RTC_GPIO_OUT) & (uint32)0xfffffffe) | (uint32)(value & 1));
}


void user_rf_pre_init(void)
{
}

void user_init(void)
{
	char ssid[32] = "";
	char password[64] = "";

	system_update_cpu_freq(80);
	struct station_config stationConf;

//	wifi_set_opmode(STATION_MODE);
  	wifi_set_opmode(0);
	uart_div_modify(0, UART_CLK_FREQ / (115200));//SET BAUDRATE
	os_timer_setfn(&timer0,&one_second,NULL);
	os_timer_arm(&timer0,5000,true);

//	hspi_overlap_init(); // Turn on overlap mode

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,FUNC_GPIO4);
    GPIO_OUTPUT_SET(4, 1);
    GPIO_REG_WRITE(GPIO_PIN4,1); // connect GPIO 4 to Sigmadelta Channel
	GPIO_REG_WRITE(GPIO_SIGMA_DELTA, 0x10000 | (prescaler<<8) | duty); // set Target Prescaler and duty cycle
	gpio16_output_conf();
	gpio16_output_set(0); // enable voltage boost circuit
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U,FUNC_GPIO0);
    GPIO_OUTPUT_SET(0, 0); // select Vpp as Analog input
//    GPIO_OUTPUT_SET(0, 1); // select Vt as Analog input


//    stationConf.bssid_set = 0; //This sets to ignore Mac address
//    os_memcpy(&stationConf.ssid, ssid, 32); // set Connection parameters
//    os_memcpy(&stationConf.password, password, 64);
//    wifi_station_set_config_current(&stationConf);

}
