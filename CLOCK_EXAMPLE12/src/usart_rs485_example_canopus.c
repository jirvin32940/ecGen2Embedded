/**
 * \file
 *
 * \brief USART RS485 example for SAM.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 *  \mainpage USART RS485 Example with PDC
 *
 *  \par Purpose
 *
 *  The USART RS485 Example demonstrates how to use USART in RS485
 *  mode.
 *
 *  \par Requirements
 *
 *  This package can be used with same70_xplained_pro/samv71_xplained_ulta boards. Before running, make sure
 *  to connect two boards with RS485 lines. Match each paired pins of two
 *  boards respectively with A to A, B to B and FGND to FGND.
 *
 *  Please refer to the board user guide for the details of RS485 jumper
 *  settings.
 *
 *  \par Description
 *
 *  This example connects two boards through RS485 interface. One board acts
 *  as the transmitter and the other one as the receiver. It is determined by
 *  the sequence that the two applications started.
 *
 *  In any case, the application sends a sync character at running to seek a
 *  receiver. If the acknowledgement is received in a short time, it will act
 *  as the transmitter and then send a full frame text to the receiver.
 *
 *  The earlier started board will act automatically as the receiver due to no
 *  acknowledgement received. The receiver will wait until sync character is
 *  received. Then it sends the acknowledgement and waits for the full frame
 *  sent by the transmitter. At the end of reception, it prints out message
 *  through UART interface to assert that the whole process succeeds.
 *
 *  \par Usage
 *
 *  -# Build the program and download it into the two evaluation boards.
 *  -# Connect a serial cable to the UART port for each evaluation kit.
 *  -# On the computer, open and configure a terminal application for each board
 *     (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start application from two boards in sequence. Make sure the second board
 *     should NOT be started unless the first board has run to wait for the
 *     synchronizing character. The output message in later section would
 *     describe this.
 *
 *  -# In the terminal window, the following text should appear:
 *     \code
	-- USART RS485 Example --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *  -# The consequent messages will indicate the boards' behavior.
 *
 *     -  The earlier started board will output the message below to indicate it
 *     is waiting for a synchronization character:
 *     \code
	-I- Receiving sync character.
\endcode
 *     -  If it receives a sync character and prepare to receive a frame, it
 *     will print out the message below:
 *     \code
	-I- Start receiving!
\endcode
 *     -  After successfully receives a frame, the board will output the
 *     following message to indicate that the whole process succeeds.
 *     \code
	-I- Received successfully!
\endcode
 *     -  The later started one will act as transmitter, and if it receives an
 *     acknowledgement character successfully, it will output the following
 *     message and start transmitting:
 *     \code
	-I- Start transmitting!
\endcode
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <string.h>
#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_example.h"
#include "pca9952.h"
#include "serial_id_ds2411.h"
#include "afec.h"

/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE         2000

/** Size of the buffer. */
#define PDC_BUF_SIZE        BUFFER_SIZE

/** Acknowledge time out. */
#define TIMEOUT             (1000)

/** Character to synchronize with the other end. */
#define SYNC_CHAR            0x11

/** Character to acknowledge receipt of the sync char. */
#define ACK_CHAR             0x13

/** All interrupt mask. */
#define ALL_INTERRUPT_MASK   0xffffffff

/** System tick frequency in Hz. */
#define SYS_TICK_FREQ        1000

#define STRING_EOL    "\r"
#define STRING_HEADER "-- USART RS485 Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** State of the USART. */
typedef enum st_usart_state {
	INITIALIZED,
	TRANSMITTING,
	RECEIVING,
	RECEIVED,
	TRANSMITTED
} usart_state_t;

/** Global usart state. */
volatile usart_state_t g_state = INITIALIZED;

/** Tick Counter in unit of ms. */
volatile uint32_t g_ul_tick_count;

/** Transmit buffer. Pure ASCII text. */
uint8_t g_uc_transmit_buffer[BUFFER_SIZE] = "DESCRIPTION of this example: \r\n \
 **************************************************************************\r\n\
 *  This application gives an example of how to use USART in RS485 mode.\r\n\
 *  RS-485 is a standard defining the electrical characteristics of drivers \r\n\
 *  and receivers for use in balanced digital multipoint systems. The standard \r\n\
 *  is published by the ANSI TIA/EIA. \r\n\
 *  \r\n\
 *  This example connects two boards through RS485 interface. One board acts \r\n\
 *  as the transmitter and the other one as the receiver. It is determined by \r\n\
 *  the sequence the two applications started. The earlier started board will \r\n\
 *  act automatically as the receiver due to no acknowledgement received. The \r\n\
 *  receiver will wait until sync character is received. Then it sends the \r\n\
 *  acknowledgement and waits for the full frame sent by the transmitter. \r\n\
 *  At the end of reception, it prints out message through UART interface to \r\n\
 *  assert that the whole process succeeds.\r\n\
 **************************************************************************\r\n\
 END of DESCRIPTION \r\n\
 ";

/** Receive buffer. */
uint8_t g_uc_receive_buffer[BUFFER_SIZE];

/** Pointer to receive buffer base. */
uint8_t *p_revdata = &g_uc_receive_buffer[0];
/** count number for received data. */
uint32_t g_ulcount = 0;


/* Global ul_ms_ticks in milliseconds since start of application */
volatile uint32_t ul_ms_ticks = 0;

/**
 * \brief Wait for the given number of microseconds (using the ul_ms_ticks generated
 * by the SAM microcontroller system tick).
 *
 * \param ul_dly_ticks  Delay to wait for, in milliseconds.
 */
void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = ul_ms_ticks;
	while ((ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks) {
	}
}


/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event.
 *  Increment the ul_ms_ticks counter.
 */
void SysTick_Handler(void)
{
	g_ul_tick_count++;
	ul_ms_ticks++; //jsi 6feb16
}

/**
 *  \brief Get the tick count value.
 *
 */
static uint32_t get_tick_count(void)
{
	return g_ul_tick_count;
}

/**
 *  \brief Wait for some time in ms.
 *
 */
static void wait(volatile uint32_t ul_ms)
{
	uint32_t ul_start;
	uint32_t ul_current;

	ul_start = g_ul_tick_count;
	do {
		ul_current = g_ul_tick_count;
	} while (ul_current - ul_start < ul_ms);
}

/**
 *  \brief Handler for USART interrupt.
 *
 */
void USART_Handler(void)
{
	uint32_t ul_status;
	uint8_t uc_char;

	/* Read USART status. */
	ul_status = usart_get_status(BOARD_USART);

	/*transmit interrupt rises*/
	if(ul_status & (US_IER_TXRDY | US_IER_TXEMPTY)) {
		usart_disable_interrupt(BOARD_USART, (US_IER_TXRDY | US_IER_TXEMPTY));
	}

	/*receive interrupt rise, store character to receiver buffer*/
	if((g_state == RECEIVING) && (usart_read(BOARD_USART, (uint32_t *)&uc_char) == 0)) {
		*p_revdata++ = uc_char;
		g_ulcount++;
		if(g_ulcount >= BUFFER_SIZE) {
			g_state = RECEIVED;
			usart_disable_interrupt(BOARD_USART, US_IER_RXRDY);
		}
	}
}

/**
 *  \brief USART RS485 mode configuration.
 *
 *  Configure USART in RS485 mode, asynchronous, 8 bits, 1 stop bit,
 *  no parity, 256000 bauds and enable its transmitter and receiver.
 */
static void configure_usart(void)
{
	const sam_usart_opt_t usart_console_settings = {
		BOARD_USART_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		/* This field is only used in IrDA mode. */
		0
	};

	/* Enable the peripheral clock in the PMC. */
	sysclk_enable_peripheral_clock(BOARD_ID_USART);

	/* Configure USART in RS485 mode. */
//jsi 7feb16 we want rs232 not rs485 for our application	usart_init_rs485(BOARD_USART, &usart_console_settings,
//jsi 7feb16 we want rs232 not rs485 for our application			sysclk_get_cpu_hz());
			
	usart_init_rs232(BOARD_USART, &usart_console_settings, sysclk_get_cpu_hz());

	/* enable transmitter timeguard, 4 bit period delay. */
	usart_set_tx_timeguard(BOARD_USART, 4);

	/* Disable all the interrupts. */
	usart_disable_interrupt(BOARD_USART, ALL_INTERRUPT_MASK);

	/* Enable TX & RX function. */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);

	/* Configure and enable interrupt of USART. */
	NVIC_EnableIRQ(USART_IRQn);
}

/**
 *  Configure system tick to generate an interrupt every 1us. Note that this was 1ms in the example code. jsi 11feb16
 */
static void configure_systick(void)
{
	uint32_t ul_flag;

	ul_flag = SysTick_Config(sysclk_get_cpu_hz()/SYS_TICK_FREQ);
	if (ul_flag) {
		puts("-F- Systick configuration error\r");
		while (1) {
		}
	}
}

/**
 *  Configure UART for debug message output.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
* \brief transmit data.
*
* \param *p_buff  data to be transmitted
* \param ulsize size of all data.
*
*/
uint8_t func_transmit(const uint8_t *p_buff, uint32_t ulsize)
{
	Assert(p_buff);

	while(ulsize > 0) {
		if(0 == usart_write(BOARD_USART, *p_buff)){
			usart_enable_interrupt(BOARD_USART, US_IER_TXRDY | US_IER_TXEMPTY);
			ulsize--;
			p_buff++;
		}
	}

	while(!usart_is_tx_empty(BOARD_USART)) {
		;  /*waiting for transmit over*/
	}

	return 0;
}

/**
 * \brief Dump buffer to uart.
 *
 */
static void dump_info(char *p_buf, uint32_t ul_size)
{
	uint32_t ul_i = 0;

	while ((ul_i < ul_size) && (p_buf[ul_i] != 0)) {
		printf("%c", p_buf[ul_i++]);
	}
}


/*
 * BELOW: Carry over from EC GEN I code 31jan16 Modified for EC Gen II
 */

/*
 * Commands for LED display: we can only display the strings provided for by the display
 */

unsigned char CMD_READY[7] =	{0x55, 0xAA, 0x91, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEAN[7] =	{0x55, 0xAA, 0x92, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEANING[7] = {0x55, 0xAA, 0x93, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_DIRTY[7] =	{0x55, 0xAA, 0x94, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_ERROR[7] =	{0x55, 0xAA, 0x95, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF1[7] =	{0x55, 0xAA, 0x96, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF2[7] =	{0x55, 0xAA, 0x97, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF3[7] =	{0x55, 0xAA, 0x98, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF4[7] =	{0x55, 0xAA, 0x99, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEAR[7] =	{0x55, 0xAA, 0xCE, 0x00, 0x00, 0x00, 0x00}; //experiment 11apr15
	
unsigned char* cmdPtrArray[10] = {
	&CMD_READY[0],
	&CMD_CLEAN[0],
	&CMD_CLEANING[0],
	&CMD_DIRTY[0],
	&CMD_ERROR[0],
	&CMD_SHELF1[0],
	&CMD_SHELF2[0],
	&CMD_SHELF3[0],
	&CMD_SHELF4[0],
	&CMD_CLEAR[0]
};

enum {
	IDX_READY,
	IDX_CLEAN,
	IDX_CLEANING,
	IDX_DIRTY,
	IDX_ERROR,
	IDX_SHELF1,
	IDX_SHELF2,
	IDX_SHELF3,
	IDX_SHELF4,
	IDX_CLEAR
	
};


volatile U16 adc_current_conversion;

#define ECLAVE_DOOR_LATCH	//TBD different scheme now
#define ECLAVE_ACTION_PB	//TBD different scheme now
#define ECLAVE_DEBUG_LED	PIO_PC16_IDX
#define ECLAVE_PSUPPLY_ONn	PIO_PA2_IDX
#define ECLAVE_LED_OEn		PIO_PA1_IDX
#define ECLAVE_MFP			PIO_PA0_IDX	//set to 1 for 1X, set to 0 for 4X

//TBD different scheme now #define EC_DOOR_LATCHED (!gpio_get_pin_value(ECLAVE_DOOR_LATCH)) //12apr15 this is the correct sense for the equipment going to the show
//TBD different scheme now #define EC_ACTION_PB	(!gpio_get_pin_value(ECLAVE_ACTION_PB)) //12apr15 this is the correct sense for the equipment going to the show


enum {
	SHELF_INACTIVE,
	SHELF_ACTIVE
};


void display_text(unsigned char idx);
void display_text(unsigned char idx)
{
	for (int i = 0; i<7; i++)
	{
		//jsi 3feb16 doesn't compile but we don't need it yet usart_putchar(DISPLAY_USART, ((unsigned char) ((*(cmdPtrArray[idx]+i)))));
	}
}

static void twi_init(void);
static void twi_init(void)
{
	twihs_options_t opt;

	/* Enable the peripheral clock for TWI */
	pmc_enable_periph_clk(ID_TWIHS0);

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_cpu_hz();
	opt.speed      = TWIHS_CLK; //400KHz

	if (twihs_master_init(TWIHS0, &opt) != TWIHS_SUCCESS) {
		while (1) {
			/* Capture error */
		}
	}
}




/*
 * ABOVE: Carry over from EC GEN I code 31jan16 Modified for EC Gen II
 */


/*
 * Analog conversion...Bluesense0..3
 */

/** The conversion data is done flag */

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
#define MAX_DIGITAL     (4095UL)



uint32_t ul_vol;

volatile bool is_conversion_done[4] = {false, false, false, false};

/** The conversion data value */
volatile uint32_t g_ul_value[4] = {0, 0, 0, 0};


uint8_t afecSel[4] = {AFEC1, AFEC0, AFEC0, AFEC0};
uint8_t adcCh[4] = 	{AFEC_CHANNEL_9, AFEC_CHANNEL_0, AFEC_CHANNEL_4, AFEC_CHANNEL_5};

/**
 * \brief AFEC interrupt callback function.
 */
static void afec_end_conversion(uint8_t bluesenseCh)
{
	g_ul_value[bluesenseCh] = afec_channel_get_value(afecSel[bluesenseCh], adcCh[bluesenseCh]);
	is_conversion_done[bluesenseCh] = true;
}

void afec_end_conversion_bluesense0(void)
{
	afec_end_conversion(0);
}
void afec_end_conversion_bluesense1(void)
{
	afec_end_conversion(1);
}
void afec_end_conversion_bluesense2(void)
{
	afec_end_conversion(2);
}
void afec_end_conversion_bluesense3(void)
{
	afec_end_conversion(3);
}

void init_adc(void)
{
	struct afec_config afec_cfg;

	afec_get_config_defaults(&afec_cfg);

	afec_init(AFEC0, &afec_cfg);
	afec_init(AFEC1, &afec_cfg);

	afec_set_trigger(AFEC0, AFEC_TRIG_SW);
	afec_set_trigger(AFEC1, AFEC_TRIG_SW);

	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);

	afec_ch_cfg.gain = AFEC_GAINVALUE_0;

	afec_ch_set_config(AFEC1, AFEC_CHANNEL_9, &afec_ch_cfg); //bluesense0 for now
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_0, &afec_ch_cfg); //bluesense1 for now
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_4, &afec_ch_cfg); //bluesense2
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_5, &afec_ch_cfg); //bluesense3

	/*
	 * Because the internal ADC offset is 0x200, it should cancel it and shift
	 * down to 0.
	 */
	afec_channel_set_analog_offset(AFEC1, AFEC_CHANNEL_9, 0x200);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_0, 0x200);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_4, 0x200);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_5, 0x200);

#if 0 //not sure if we have to do something else here for these non temp sensor channels or not 13feb16
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_cfg.rctc = true;
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);
#endif

	afec_set_callback(AFEC1, AFEC_CHANNEL_9,
		afec_end_conversion_bluesense0, 1);
	afec_set_callback(AFEC0, AFEC_CHANNEL_0,
		afec_end_conversion_bluesense1, 1);
	afec_set_callback(AFEC0, AFEC_CHANNEL_4,
		afec_end_conversion_bluesense2, 1);
	afec_set_callback(AFEC0, AFEC_CHANNEL_5,
		afec_end_conversion_bluesense3, 1);

}

void read_adc(uint8_t bluesenseCh)
{
	char printStr[32];
	
	ul_vol = g_ul_value[bluesenseCh] * VOLT_REF / MAX_DIGITAL;

	sprintf(printStr, "adc ch %d: %4x\r\n", bluesenseCh, ul_vol);

	func_transmit(printStr, strlen(printStr));

	is_conversion_done[bluesenseCh] = false;
}


/**
 *  \brief usart_rs485 Application entry point.
 *
 *  Configure USART in RS485 mode. If the application starts earlier, it acts
 *  as a receiver. Otherwise, it should be a transmitter.
 *
 *  \return Unused (ANSI-C compatibility).
 */

#  define EXAMPLE_LED_GPIO    LED0_GPIO

int main(void) //6feb16 this version of main has been hacked up for only exactly what we need
{
	char		txBuf[11] = {0,0,0,0,0,0,0,0,0,0,0}, rxByte;
	uint32_t	i;	
	uint32_t	time_elapsed = 0;
	uint32_t	ul_i;
	uint8_t		displayState = 0;
	uint8_t		charCount = 0;
	
	char		printStr[64];
	
	unsigned char idFamily, acc, id[6], idcsum; //10feb16 temp serial ID code, make more formal later

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Configure UART for blue scrolling display */
	configure_console();

	/* Configure USART. */
	configure_usart();

	/* 1ms tick. */
	configure_systick();


	/*
	 * Put into some kind of "init_io()" function at some point
	 */
	
	ioport_set_pin_dir(ECLAVE_SERIAL_ID0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID0, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID1, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID2, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID3, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID3, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID4, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID4, IOPORT_PIN_LEVEL_HIGH);

	SetSpeed(1); //1==standard speed, not overdrive 
	
	for (int i=0; i<5; i++)
	{
		if(!OWTouchReset(i)) //we think a board is present, try to read the serial ID
		{
			OWWriteByte(i, 0x33); //Read ID command
			
			idFamily = OWReadByte(i);
			
			acc = crc8_add(0x00, idFamily);
			
			for (int j=0; j<6; j++)
			{
				id[j] = OWReadByte(i);
				acc = crc8_add(acc, id[j]);
			}
			
			idcsum = OWReadByte(i);
			
			if (acc != idcsum)
			{
				func_transmit("Invalid serial ID checksum.\r\n", strlen("Invalid serial ID checksum.\r\n"));
			}
			else
			{
				sprintf(printStr,"LED board %d serial ID: %x%x%x%x%x%x\r\n", i, id[0], id[1], id[2], id[3], id[4], id[5]);
				func_transmit(printStr,strlen(printStr));
			}
		}
		else
		{
			func_transmit("no board this slot\r\n", strlen("no board this slot\r\n"));
			
		}
	}

	/*
	 * End of minimalist serial ID chip stuff
	 */

	twi_init();

//make this ecII jsi 7feb16	gpio_set_pin_high(ECLAVE_LED_OEn); //make sure outputs are disabled at the chip level

	PCA9952_init();

	init_adc();
	
	/*
	 * Enable transmitter here, and disable receiver first, to avoid receiving
	 * characters sent by itself. It's necessary for half duplex RS485.
	 */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);

	while (1) {

		/* Test the debug LED */
		ioport_toggle_pin_level(EXAMPLE_LED_GPIO);


		/* Test the debug usart rx and tx */
		for (i=0; i<70; i++) //7 seconds
		{
			mdelay(100);
		
			if (usart_is_rx_ready(BOARD_USART)) {
				usart_read(BOARD_USART, (uint32_t *)&rxByte);
				func_transmit(&rxByte, 1);
			}
		}

		/* Test the scrolling blue LED display */
		for (charCount = 0; charCount < 7; charCount++)
		{
			unsigned char temp;
			temp = (*(cmdPtrArray[displayState]+charCount));
			putchar(temp);
		}
		
		/* Test the debug port */

		switch (displayState)
		{
			case IDX_READY:
				sprintf(txBuf, "Ready\r\n");
				break;
			case IDX_CLEAN:
				sprintf(txBuf, "Clean\r\n");
				break;
			case IDX_CLEANING:
				sprintf(txBuf, "Cleaning\r\n");
				break;
			case IDX_DIRTY:
				sprintf(txBuf, "Dirty\r\n");
				break;
			case IDX_ERROR:
				sprintf(txBuf, "Error\r\n");
				break;
			case IDX_SHELF1:
				sprintf(txBuf, "Shelf1\r\n");
				break;
			case IDX_SHELF2:
				sprintf(txBuf, "Shelf2\r\n");
				break;
			case IDX_SHELF3:
				sprintf(txBuf, "Shelf3\r\n");
				break;
			case IDX_SHELF4:
				sprintf(txBuf, "Shelf4\r\n");
				break;
		}
		
		func_transmit(txBuf, strlen(txBuf));


		if ((++displayState) > 8)
		{
			displayState = 0;
		}
		
		/*
		 * Read Bluesense lines
		 */
		
		if ((is_conversion_done[0] == true) &&
		    (is_conversion_done[1] == true) &&
		    (is_conversion_done[2] == true) &&
		    (is_conversion_done[3] == true))
		{
			read_adc(0);
			read_adc(1);
			read_adc(2);
			read_adc(3);
		}
		
	}//while
}//main


int pristine_main(void) //jsi 6feb16 keep this one as it was from the example code
{
	static uint8_t uc_sync = SYNC_CHAR;
	uint32_t time_elapsed = 0;
	uint32_t ul_i;

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Configure UART for debug message output. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);

	/* Configure USART. */
	configure_usart();


	/* 1ms tick. */
	configure_systick();

	/* Initialize receiving buffer to distinguish with the sent frame. */
	memset(g_uc_receive_buffer, 0x0, BUFFER_SIZE);

	/*
	 * Enable transmitter here, and disable receiver first, to avoid receiving
	 * characters sent by itself. It's necessary for half duplex RS485.
	 */
	usart_enable_tx(BOARD_USART);
	usart_disable_rx(BOARD_USART);

	/* Send a sync character XON (0x11). */
	func_transmit(&uc_sync, 1);
	/* Delay until the line is cleared, an estimated time used. */
	wait(50);

	/* Then enable receiver. */
	usart_enable_rx(BOARD_USART);

	/* Wait until time out or acknowledgement is received. */
	time_elapsed = get_tick_count();
	while (!usart_is_rx_ready(BOARD_USART)) {
		if (get_tick_count() - time_elapsed > TIMEOUT) {
			break;
		}
	}

	/* If acknowledgment received in a short time. */
	if (usart_is_rx_ready(BOARD_USART)) {
		wait(50);
		usart_read(BOARD_USART, (uint32_t *)&uc_sync);
		/* Acknowledgment. */
		if (uc_sync == ACK_CHAR) {
			/* Act as transmitter, start transmitting. */
			g_state = TRANSMITTING;
			puts("-I- Start transmitting!\r");

			func_transmit(&g_uc_transmit_buffer[0], BUFFER_SIZE);
		}
	} else {
		/* Start receiving, act as receiver. */
		puts("-I- Receiving sync character.\r");
		while (!usart_is_rx_ready(BOARD_USART)) {
		}
		wait(50);
		/* Sync character is received. */
		usart_read(BOARD_USART, (uint32_t *)&uc_sync);
		if (uc_sync == SYNC_CHAR) {
			/* SEND XOff as acknowledgement. */
			uc_sync = ACK_CHAR;

			/*
			 * Delay to prevent the character from being discarded by
			 * transmitter due to responding too soon.
			 */
			wait(50);

			/* Send a ack character XOff . */
			func_transmit(&uc_sync, 1);

			g_state = RECEIVING;
			puts("-I- Start receiving!\r");

			/* Enable receiving interrupt. */
			usart_enable_interrupt(BOARD_USART, US_IER_RXRDY);
		}
	}
	while (g_state != RECEIVED) {
	}

	ul_i = 0;
	/* Print received frame out. */
	while ((ul_i < BUFFER_SIZE) && (g_uc_receive_buffer[ul_i] != '\0')) {
		if (g_uc_transmit_buffer[ul_i] != g_uc_receive_buffer[ul_i]) {
			puts("-E- Error occurred while receiving!\r");
			/* Infinite loop here. */
			while (1) {
			}
		}
		ul_i++;
	}
	puts("-I- Received successfully!\r");
	dump_info((char *)g_uc_receive_buffer, BUFFER_SIZE);
	while (1) {
	}
}
