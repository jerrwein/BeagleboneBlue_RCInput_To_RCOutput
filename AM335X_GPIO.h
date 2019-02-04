#ifndef AM335X_GPIO_H_
#define AM335X_GPIO_H_

/*****************************************************************************
* Fast GPIO Declarations                                             *
*****************************************************************************/
#ifndef _BEAGLEBONE_FAST_GPIO_H_
#define _BEAGLEBONE_FAST_GPIO_H_

/* Clock Module Peripherals */
#define CM_PER_START_ADDR 0x44E00000
#define CM_PER_SIZE 0x400
#define CM_WKUP_START_ADDR 0x44E00400
#define CM_WKUP_GPIO0_CLKCTRL (CM_WKUP_START_ADDR + 0x8)
#define CM_PER_GPIO1_CLKCTRL_OFFSET 0xAC
#define CM_PER_GPIO2_CLKCTRL_OFFSET 0xB0
#define CM_PER_GPIO3_CLKCTRL_OFFSET 0xB4

#define IDLE_STATE_MASK (0x03 << 16)

#define GPIO0_START_ADDR 0x44E07000
#define GPIO0_END_ADDR 0x44E08FFF
#define GPIO0_SIZE (GPIO0_END_ADDR - GPIO0_START_ADDR)

#define GPIO1_START_ADDR 0x4804C000
#define GPIO1_END_ADDR 0x4804DFFF
#define GPIO1_SIZE (GPIO1_END_ADDR - GPIO1_START_ADDR)

#define GPIO2_START_ADDR 0x481AC000
#define GPIO2_END_ADDR 0x481ADFFF
#define GPIO2_SIZE (GPIO2_END_ADDR - GPIO2_START_ADDR)

#define GPIO3_START_ADDR 0x481ae000
#define GPIO3_END_ADDR 0x481AFFFF
#define GPIO3_SIZE (GPIO3_END_ADDR - GPIO3_START_ADDR)

#define GPIO_OE (0x134)
#define GPIO_DATAIN (0x138)
#define GPIO_DATAOUT (0x13C)
#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT (0x194)

#define GPIO_OUTPUT 1
#define GPIO_INPUT 0

// #define GPIO2_16_PIN (1<<16)
#define GPIO0_09_PIN 0x00000200
#define GPIO0_26_PIN 0x04000000		/* P8.14 LED_2  */
#define GPIO1_12_PIN 0x00001000		/* P8.12 LED_3	*/
#define GPIO1_16_PIN 0x00010000		/* P9.15 Unused	*/
#define GPIO1_17_PIN 0x00020000		/* P9.23 LED_4 	*/
#define GPIO1_29_PIN 0x20000000
#define GPIO2_03_PIN 0x00000008		/* P8.8 LED_1 */
#define GPIO2_04_PIN 0x00000010		/* P8.10 LED_4 */
#define GPIO2_15_PIN 0x00008000		/* P8.38 Opto_2	*/
#define GPIO2_16_PIN 0x00010000
#define GPIO3_20_PIN 0x00100000

#endif /* _BEAGLEBONE_FAST_GPIO_H_ */

/*****************************************************************************
* Global Function Declarations                                               *
*****************************************************************************/
int gpio_fast_init (int nVal1);

int gpio_export (unsigned int gpio);
int gpio_unexport (unsigned int gpio);
int gpio_set_dir (unsigned int gpio, unsigned int out_flag);
int gpio_set_value (unsigned int gpio, unsigned int value);

#endif /* AM335X_GPIO_H_ */
