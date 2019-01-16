/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
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
#include <asf.h>

/* InvenSense drivers and utils */
#include "Invn/Devices/Drivers/Icm20789-DMP/Icm20789.h"
#include "Invpres.h"
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

/*******************************************************************************/

/*******************************************************************************/

#include "ASF/sam/drivers/pio/pio.h"
#include "ASF/sam/drivers/pio/pio_handler.h"
#include "ASF/sam/drivers/twi/twi.h"
#include "ASF/sam/drivers/tc/tc.h"

/*****************************************************************/

#include "system.h"

#if SERIF_TYPE_SPI
static void spi_master_initialize(void);
static int spi_master_transfer_rx(void * context, uint8_t register_addr, uint8_t * value, uint32_t len);
static int spi_master_transfer_tx(void * context, uint8_t register_addr, const uint8_t * value, uint32_t len);
#endif
#if SERIF_TYPE_I2C
static void i2c_master_initialize(void);
#endif

/* Flag set from device irq handler */
volatile int irq_from_device = 0;
volatile int irq_start_pressure_capture = 0;

static uint32_t pressure_timer_count = 0;

InvScheduler 	scheduler;

#if SERIF_TYPE_I2C
/*
 * Variable for storing I2C Address
 */
uint8_t I2C_Address = ICM_I2C_ADDR;
#endif


/************************ PIO ************************************************/
/*!
* @brief	Sensor general interrupt handler, calls specific handlers.
*
* This function is called when an external interrupt is triggered by the sensor,
* checks interrupt registers of InvenSense Sensor to determine the source and type of interrupt
* and calls the specific interrupt handler accordingly.
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void ext_interrupt_handler(void)
{
	irq_from_device = 1;
}	

void ext_int_initialize(void (*handler_function)(void))
{
	/* Enable the peripheral clock for the MAG extension board interrupt pin. */
	pmc_enable_periph_clk(ID_TC0);

	/* Configure PIOs as input pins. */
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_INTERRUPT_MASK, PIN_EXT_INTERRUPT_ATTR);

	/* Initialize PIO interrupt handler, interrupt on rising edge. */
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_INTERRUPT_MASK, 
						PIN_EXT_INTERRUPT_ATTR, (void (*) (uint32_t, uint32_t))handler_function);

	/* Initialize and enable push button (PIO) interrupt. */
	pio_handler_set_priority(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_IRQn, 0);
	pio_enable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
}

/************************ UART ************************************************/
/* Configuration for console uart IRQ */
#define CONSOLE_UART_IRQn           FLEXCOM0_IRQn
/* Configuration for console uart IRQ handler */
#define console_uart_irq_handler    FLEXCOM0_Handler

static uint8_t uart_rx_rb_buffer[512];
RingByteBuffer uart_rx_rb;

void configure_console(void)
{
	const usart_serial_options_t uart_serial_options_debug = {
		.baudrate = DEBUG_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure debug UART. */
	sysclk_enable_peripheral_clock(DEBUG_UART_ID);
	usart_serial_init(DEBUG_UART, (usart_serial_options_t *)&uart_serial_options_debug);

	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	usart_serial_init(CONF_UART, (usart_serial_options_t *)&uart_serial_options);

	RingByteBuffer_init(&uart_rx_rb, uart_rx_rb_buffer, sizeof(uart_rx_rb_buffer));

	/* Enable UART IRQ */
	usart_enable_interrupt(CONSOLE_UART, US_IER_RXRDY);

	/* Enable UART interrupt */
	NVIC_SetPriority(CONSOLE_UART_IRQn, 0);

	/* Enable UART interrupt */
	NVIC_EnableIRQ(CONSOLE_UART_IRQn);
}

/**
 * \brief Interrupt handler for USART interrupt.
 */
void console_uart_irq_handler(void)
{
	uint32_t ul_status;

	/* Read USART Status. */
	ul_status = usart_get_status(CONSOLE_UART);
	
	if((ul_status &  US_CSR_RXRDY ))
	{
		uint8_t rxbyte;
		usart_serial_getchar(CONSOLE_UART, &rxbyte);
		if(!RingByteBuffer_isFull(&uart_rx_rb))
			RingByteBuffer_pushByte(&uart_rx_rb, rxbyte);
	}
}

/************************ I2C *************************************************/
twi_options_t opt;
twi_packet_t packet_tx, packet_rx;

static void i2c_master_initialize(void)
{
	/* Insert application code here, after the board has been initialized. */
	/* Enable the peripheral and set TWI mode. */
	flexcom_enable(BOARD_FLEXCOM_TWI);
	flexcom_set_opmode(BOARD_FLEXCOM_TWI, FLEXCOM_TWI);
	
	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed = TWI_CLK;

	twi_master_init(BOARD_BASE_TWI, &opt);
}


unsigned long i2c_pres_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	twi_packet_t packet_read;

	if(Address == 0){
		Address = PRES_I2C_ADDR;	/* On Atmel platform, Slave Address should be set to 0x68 as pin SA0 is logic low */
	}

	packet_read.chip = Address;
	packet_read.addr[0] = 0;
	packet_read.addr_length = 0;		// For the InvPres we don't send a register address
	packet_read.buffer = &RegisterValue[0];
	packet_read.length = RegisterLen;

	if(twi_master_read((Twi*)BOARD_BASE_TWI, &packet_read) == TWI_SUCCESS){
		return TWI_SUCCESS;
	}
	return TWI_BUSY;
}

unsigned long i2c_pres_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	twi_packet_t packet_write;

	if(Address == 0){
		Address = PRES_I2C_ADDR; 
	}
	packet_write.chip = Address;
	packet_write.addr[0] = RegisterValue[0];
	packet_write.addr_length = 1;
	packet_write.buffer = (void *)&RegisterValue[1];
	packet_write.length = RegisterLen-1;

	return twi_master_write((Twi*)BOARD_BASE_TWI, &packet_write);
}

#if SERIF_TYPE_I2C
unsigned long i2c_master_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	twi_packet_t packet_read;

	if(Address == 0){
		Address = I2C_Address;	// Slave Address is 0x69 for on-board sensors, 0x68 for external sensors
	}
	
	packet_read.chip = Address;
	packet_read.addr[0] = RegisterAddr;
	packet_read.addr_length = 1;
	packet_read.buffer = RegisterValue;
	packet_read.length = RegisterLen;
	
	if(twi_master_read((Twi*)BOARD_BASE_TWI, &packet_read) == TWI_SUCCESS){
		return TWI_SUCCESS;
	}
	return TWI_BUSY;
}

unsigned long i2c_master_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	twi_packet_t packet_write;

	if(Address == 0){
		Address = I2C_Address; // Slave Address is 0x69 for on-board sensors, 0x68 for external sensors
	}
	packet_write.chip = Address;
	packet_write.addr[0] = RegisterAddr;
	packet_write.addr_length = 1;
	packet_write.buffer = (void *)RegisterValue;
	packet_write.length = RegisterLen;

	return twi_master_write((Twi*)BOARD_BASE_TWI, &packet_write);	
}
#endif

/************************ SPI *************************************************/
#if SERIF_TYPE_SPI
#define SPI_Handler				FLEXCOM5_Handler
#define SPI_IRQn				FLEXCOM5_IRQn
#define SPI_CHIP_SEL			CHIP_SELECT /* Chip select. */
#define SPI_CHIP_PCS			spi_get_pcs(SPI_CHIP_SEL)
#define SPI_CLK_POLARITY		1 /* Clock polarity. */
#define SPI_CLK_PHASE			1 /* Clock phase. */
#define SPI_DLYBS				0x40 /* Delay before SPCK. */
#define SPI_DLYBCT				0x10 /* Delay between consecutive transfers. */
#define SPI_CLK_SPEED			6000000 /* SPI clock setting (Hz). */

#define READ_BIT_MASK			0x80
#define WRITE_BIT_MASK			0x7F

/* Pointer to UART PDC register base */
Pdc *g_p_spim_pdc, *g_p_spis_pdc;

/* Function prototype declaration */
/**
 * \brief Initialize SPI as master.
 * PDC and Interrupts Initialized but not used
 */
static void spi_master_initialize(void)
{
	/* Enable the peripheral and set SPI mode. */
	
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);

	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_SEL);
	
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL, (sysclk_get_peripheral_hz() / SPI_CLK_SPEED));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS, SPI_DLYBCT);

	spi_enable(SPI_MASTER_BASE);
}

static int spi_master_transfer_tx(void * context, uint8_t register_addr, const uint8_t * value, uint32_t len)
{
	uint8_t reg	= register_addr; 
	const uint8_t *p_rbuf = value; 
	uint32_t rsize = len;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;
	
	reg &= WRITE_BIT_MASK;
	
	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */

	for (i = 0; i < rsize; i++) {
		spi_write(SPI_MASTER_BASE, p_rbuf[i], 0, 0); /* dummy write to generate clock */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
	}
	
	return 0;
}

static int spi_master_transfer_rx(void * context, uint8_t register_addr, uint8_t * value, uint32_t len)
{
	uint8_t reg	= register_addr; 
	uint8_t *p_rbuf	= value; 
	uint32_t rsize	= len;
	uint32_t i;
	uint8_t uc_pcs = 0;//SPI_CHIP_PCS;
	uint16_t data = 0;

	reg |= READ_BIT_MASK;
	
	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */

	for (i = 0; i < rsize; i++) {
		spi_write(SPI_MASTER_BASE, 0x0, 0, 0); /* dummy write to generate clock */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
		p_rbuf[i] = (uint8_t)(data & 0xFF);
	}
	
	return 0;
}
#endif // SERIF_TYPE_SPI

int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	(void)context;

#if SERIF_TYPE_SPI
return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
#else /* SERIF_TYPE_I2C */
return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
#endif

}

int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	(void)context;

#if SERIF_TYPE_SPI
return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);
#else /* SERIF_TYPE_I2C */
return i2c_master_write_register(I2C_Address, reg, wlen, wbuffer);
#endif
}

int invpres_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	(void)context;

	return i2c_pres_read_register(PRES_I2C_ADDR, reg, rlen, rbuffer);
}

int invpres_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	(void)context;

	return i2c_pres_write_register(PRES_I2C_ADDR, reg, wlen, wbuffer);
}

void interface_initialize(void)
{
#if SERIF_TYPE_SPI
	spi_master_initialize();
#endif
	i2c_master_initialize();	
}

void set_pressure_timer(uint32_t new_timer_val)
{
	pressure_timer_count = new_timer_val;
}

/************************ Systick *********************************************/

volatile uint32_t ul_ticks = 0;

void SysTick_Handler(void)
{
	static uint32_t timer_count = 0;
	
	if (++timer_count >= pressure_timer_count) {
		irq_start_pressure_capture = 1;
		timer_count = 0;
	}
	ul_ticks++;
	InvScheduler_updateTime(&scheduler);
}