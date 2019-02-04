#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include <sys/io.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "AM335X_GPIO.h"

/*****************************************************************************
* Explicit External Declarations                                             *
*****************************************************************************/

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64
#define OUTPUT 1
#define INPUT 0


/*****************************************************************************
* Global Declarations                                                        *
*****************************************************************************/
volatile unsigned int   *gpio0_datain_addr = NULL;
volatile unsigned int   *gpio1_datain_addr = NULL;
volatile unsigned int   *gpio2_datain_addr = NULL;
volatile unsigned int   *gpio3_datain_addr = NULL;

volatile unsigned int   *gpio0_setdataout_addr = NULL;
volatile unsigned int   *gpio0_cleardataout_addr = NULL;
volatile unsigned int   *gpio2_setdataout_addr = NULL;
volatile unsigned int   *gpio2_cleardataout_addr = NULL;
volatile unsigned int   *gpio1_setdataout_addr = NULL;
volatile unsigned int   *gpio1_cleardataout_addr = NULL;
volatile unsigned int   *gpio3_setdataout_addr = NULL;
volatile unsigned int   *gpio3_cleardataout_addr = NULL;


/*****************************************************************************
* Global Function Definitions                                                *
*****************************************************************************/
int gpio_fast_init (int nVal)
{
	int fast_fd = -1;

	fast_fd = open("/dev/mem", O_RDWR);
	if (fast_fd < 0)
	{
		printf("gpio_fast_init(), error: failed to open /dev/mem device\n");
		return -1;
	}

	volatile void			*pClkModuleAddr = NULL;
	volatile uint32_t 		*pRegU32 = 0;
	volatile uint32_t 		u32_reg;
	int						nAttempts;

	printf("\nMapping %X - %X (size: %X)\n", CM_PER_START_ADDR, CM_PER_START_ADDR+CM_PER_SIZE-1, CM_PER_SIZE);
	pClkModuleAddr = (void *) mmap (NULL, CM_PER_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fast_fd, CM_PER_START_ADDR);
	printf("GPIO-CM_PER mapped to %p\n", pClkModuleAddr);

	pRegU32 = (uint32_t *)(pClkModuleAddr + CM_PER_GPIO1_CLKCTRL_OFFSET);
	printf("GPIO-CM_PER_GPIO1_CLKCTRL mapped to %p\n", pRegU32);
	u32_reg = *pRegU32;
	printf("GPIO-CM_PER_GPIO1_CLKCTRL : %08X\n", u32_reg);
	if (u32_reg & IDLE_STATE_MASK)
	{
		printf("IDLEST detected for GPIO-CM_PER_GPIO1_CLKCTRL : %08X\n", u32_reg);
		// Enable peripheral clock
		*pRegU32 = u32_reg | 0x2;
		// Wait for the clock enable to complete.
		nAttempts = 0;
		while (u32_reg & IDLE_STATE_MASK)
		{
			if (100 < ++nAttempts)
			{
				printf("Excessive attempts (100) waiting for CM_PER_GPIO1_CLKCTRL\n");
				munmap ((void *)pClkModuleAddr, CM_PER_SIZE);
				close (fast_fd);
				return -1;
			}
			usleep(250);
			u32_reg = *pRegU32;
		};
		printf("%d attempts required for GPIO-CM_PER_GPIO1_CLKCTRL to enable\n", nAttempts);
	}

	pRegU32 = (uint32_t *)(pClkModuleAddr + CM_PER_GPIO2_CLKCTRL_OFFSET);
	printf("GPIO-CM_PER_GPIO2_CLKCTRL mapped to %p\n", pRegU32);
	u32_reg = *pRegU32;
	printf("GPIO-CM_PER_GPIO2_CLKCTRL : %08X\n", u32_reg);
	if (u32_reg & IDLE_STATE_MASK)
	{
		printf("IDLEST detected for GPIO-CM_PER_GPIO2_CLKCTRL : %08X\n", u32_reg);
		// Enable peripheral clock
		*pRegU32 = u32_reg | 0x2;
		// Wait for the clock enable to complete.
		nAttempts = 0;
		while (u32_reg & IDLE_STATE_MASK)
		{
			if (100 < ++nAttempts)
			{
				printf("Excessive attempts (100) waiting for CM_PER_GPIO2_CLKCTRL\n");
				munmap ((void *)pClkModuleAddr, CM_PER_SIZE);
				close (fast_fd);
				return -1;
			}
			usleep(250);
			u32_reg = *pRegU32;
		};
		printf("%d attempts required for GPIO-CM_PER_GPIO2_CLKCTRL to enable\n", nAttempts);
	}

	pRegU32 = (uint32_t *)(pClkModuleAddr + CM_PER_GPIO3_CLKCTRL_OFFSET);
	printf("GPIO-CM_PER_GPIO3_CLKCTRL mapped to %p\n", pRegU32);
	u32_reg = *pRegU32;
	printf("GPIO-CM_PER_GPIO3_CLKCTRL : %08X\n", u32_reg);
	if (u32_reg & IDLE_STATE_MASK)
	{
		printf("IDLEST detected for GPIO-CM_PER_GPIO3_CLKCTRL : %08X\n", u32_reg);
		// Enable peripheral clock
		*pRegU32 = u32_reg | 0x2;
		// Wait for the clock enable to complete.
		nAttempts = 0;
		while (u32_reg & IDLE_STATE_MASK)
		{
			if (100 < ++nAttempts)
			{
				printf("Excessive attempts (100) waiting for CM_PER_GPIO3_CLKCTRL\n");
				munmap ((void *)pClkModuleAddr, CM_PER_SIZE);
				close (fast_fd);
				return -1;
			}
			usleep(250);
			u32_reg = *pRegU32;
		};
		printf("%d attempts required for GPIO-CM_PER_GPIO3_CLKCTRL to enable\n", nAttempts);
	}

	/* Un-map clock module region */
	munmap ((void *)pClkModuleAddr, CM_PER_SIZE);

    volatile void           *gpio_addr = NULL;
    volatile unsigned int   *gpio_oe_addr = NULL;
    unsigned int            oe_reg;

#if 1
	/* The following maps GPIO0 input/outputs */
	printf("\nMapping %X - %X (size: %X)\n", GPIO0_START_ADDR, GPIO0_END_ADDR, GPIO0_SIZE);
	gpio_addr = mmap (0, GPIO0_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fast_fd, GPIO0_START_ADDR);
	gpio_oe_addr = gpio_addr + GPIO_OE;
	gpio0_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
	gpio0_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;
	gpio0_datain_addr = gpio_addr + GPIO_DATAIN;
	if (gpio_addr == MAP_FAILED)
	{
		printf("Unable to map GPIO-0\n");
		return -1;
	}
	printf("GPIO-0 mapped to %p\n", gpio_addr);
	printf("GPIO-0 OE mapped to %p\n", gpio_oe_addr);
	printf("GPIO-0 SETDATAOUTADDR mapped to %p\n", gpio0_setdataout_addr);
	printf("GPIO-0 CLEARDATAOUT mapped to %p\n", gpio0_cleardataout_addr);
	printf("GPIO-0 DATAIN mapped to %p\n", gpio0_datain_addr);

	oe_reg = *gpio_oe_addr;
	printf("GPIO-0 OE : %X\n", oe_reg);
	/* The following OEs GPIO0.9 */
	oe_reg = oe_reg & (0xFFFFFFFF - GPIO0_09_PIN);
	*gpio_oe_addr = oe_reg;
	printf("GPIO-0 OE : %X\n", oe_reg);
//	munmap ((void *)gpio_addr, GPIO0_SIZE);
#endif

#if 1
	/* The following maps GPIO1 */
	printf("\nMapping %X - %X (size: %X)\n", GPIO1_START_ADDR, GPIO1_END_ADDR, GPIO1_SIZE);
	gpio_addr = mmap (0, GPIO1_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fast_fd, GPIO1_START_ADDR);
	gpio_oe_addr = gpio_addr + GPIO_OE;
	gpio1_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
	gpio1_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;
	gpio1_datain_addr = gpio_addr + GPIO_DATAIN;

	if (gpio_addr == MAP_FAILED)
	{
		printf("Unable to map GPIO-1\n");
		return -1;
	}
	printf("GPIO-1 mapped to %p\n", gpio_addr);
	printf("GPIO-1 OE mapped to %p\n", gpio_oe_addr);
	printf("GPIO-1 SETDATAOUTADDR mapped to %p\n", gpio1_setdataout_addr);
	printf("GPIO-1 CLEARDATAOUT mapped to %p\n", gpio1_cleardataout_addr);
	printf("GPIO-1 DATAIN mapped to %p\n", gpio1_datain_addr);

	oe_reg = *gpio_oe_addr;
	printf("GPIO-1 OE : %X\n", oe_reg);
	/* The following OEs GPIO1.16 */
	oe_reg = oe_reg & (0xFFFFFFFF - GPIO1_16_PIN);
	*gpio_oe_addr = oe_reg;
	printf("GPIO-1 OE : %X\n", oe_reg);
//	munmap ((void *)gpio_addr, GPIO1_SIZE);
#endif

#if 1
	/* The following maps GPIO2 */
	printf("\nMapping %X - %X (size: %X)\n", GPIO2_START_ADDR, GPIO2_END_ADDR, GPIO2_SIZE);
	gpio_addr = mmap (0, GPIO2_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fast_fd, GPIO2_START_ADDR);
	gpio_oe_addr = gpio_addr + GPIO_OE;
	gpio2_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
	gpio2_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;
	gpio2_datain_addr = gpio_addr + GPIO_DATAIN;

	if (gpio_addr == MAP_FAILED)
	{
		printf("Unable to map GPIO-2\n");
		return -1;
	}
	printf("GPIO-2 mapped to %p\n", gpio_addr);
	printf("GPIO-2 OE mapped to %p\n", gpio_oe_addr);
	printf("GPIO-2 SETDATAOUTADDR mapped to %p\n", gpio2_setdataout_addr);
	printf("GPIO-2 CLEARDATAOUT mapped to %p\n", gpio2_cleardataout_addr);
	printf("GPIO-2 DATAIN mapped to %p\n", gpio2_datain_addr);

	oe_reg = *gpio_oe_addr;
	printf("GPIO-2 OE: %X\n", oe_reg);
	/* The following enables GPIO2.16 (CONN2.1) */
	oe_reg = oe_reg & (0xFFFFFFFF - GPIO2_16_PIN);
	*gpio_oe_addr = oe_reg;
	printf("GPIO-2 OE: %X\n", oe_reg);
//	munmap ((void *)gpio_addr, GPIO2_SIZE);
#endif

#if 1
	/* The following maps GPIO3.20 */
	printf("\nMapping %X - %X (size: %X)\n", GPIO3_START_ADDR, GPIO3_END_ADDR, GPIO3_SIZE);
	gpio_addr = mmap (0, GPIO3_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fast_fd, GPIO3_START_ADDR);
	gpio_oe_addr = gpio_addr + GPIO_OE;
	gpio3_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
	gpio3_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;
	gpio3_datain_addr = gpio_addr + GPIO_DATAIN;

	if (gpio_addr == MAP_FAILED)
	{
		printf("Unable to map GPIO-3\n");
		return -1;
	}
	printf("GPIO-3 mapped to %p\n", gpio_addr);
	printf("GPIO-3 OE mapped to %p\n", gpio_oe_addr);
	printf("GPIO-3 SETDATAOUTADDR mapped to %p\n", gpio3_setdataout_addr);
	printf("GPIO-3 CLEARDATAOUT mapped to %p\n", gpio3_cleardataout_addr);
	printf("GPIO-3 DATAIN mapped to %p\n", gpio3_datain_addr);

	oe_reg = *gpio_oe_addr;
	printf("GPIO-3 OE: %X\n", oe_reg);
	/* The following enables GPIO3.30 (CONN2.1) */
	oe_reg = oe_reg & (0xFFFFFFFF - GPIO3_20_PIN);
	*gpio_oe_addr = oe_reg;
	printf("GPIO-3 OE: %X\n", oe_reg);
//	munmap ((void *)gpio_addr, GPIO2_SIZE);
#endif

//	close (fast_fd);
	return 0;
}

int gpio_export(unsigned int gpio)
{
        int     fd, len;
        char    buf[MAX_BUF];

        fd = open (SYSFS_GPIO_DIR "/export", O_WRONLY);
        if (fd < 0) {
                perror("gpio/export");
                return fd;
        }

        len = snprintf (buf, sizeof(buf), "%d", gpio);
        write (fd, buf, len);
        close (fd);
        return 0;
}

int gpio_unexport(unsigned int gpio)
{
        int fd, len;
        char buf[MAX_BUF];

        fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
        if (fd < 0) {
                perror("gpio/export");
                return fd;
        }

        len = snprintf(buf, sizeof(buf), "%d", gpio);
        write(fd, buf, len);
        close(fd);
        return 0;
}

int gpio_set_dir (unsigned int gpio, unsigned int out_flag)
{
        int     fd;
        char    buf[MAX_BUF];

        snprintf (buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

        fd = open (buf, O_WRONLY);
        if (fd < 0) {
                perror("gpio/direction");
                return fd;
        }

        if (out_flag)
                write(fd, "out", 4);
        else
                write(fd, "in", 3);

        close(fd);
        return 0;
}

int gpio_set_value (unsigned int gpio, unsigned int value)
{
	int	fd;
	char	buf[MAX_BUF];

	snprintf (buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open (buf, O_WRONLY);
	if (fd < 0)
	{
		perror("gpio/set-value");
		return fd;
	}

	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);

	close(fd);
	return 0;
}
