

#include "avr_compiler.h"
#include "spi_driver.h"
#include "USART.h"

/*! \brief The number of test data bytes. */
#define NUM_BYTES     4

/* Global variables */

/*! \brief SPI master module on PORT C. */
SPI_Master_t spiMasterC;

/*! \brief SPI slave module on PORT D. */
SPI_Slave_t spiSlaveD;

/*! \brief SPI Data packet */
SPI_DataPacket_t dataPacket;

/*! \brief Test data to send from master. */
uint8_t masterSendData[NUM_BYTES] = {0x11, 0x22, 0x33, 0x44};

/*! \brief Data received from slave. */
uint8_t masterReceivedData[NUM_BYTES];

/*! \brief Result of the test. */
bool success = true;

/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! USART data struct used in example. */
USART_data_t USART_data;



/*! \brief Test function.
 *
 *  This function tests the SPI master and slave drivers in polled operation,
 *  with a master (on port C) communicating with a slave (on port D).
 *
 *  Hardware setup:
 *
 *    - Connect PC4 to PD4 (SS)
 *    - Connect PC5 to PD5 (MOSI)
 *    - Connect PC6 to PD6 (MISO)
 *    - Connect PC7 to PD7 (SCK)
 *
 *  The drivers are tested in two phases:
 *
 *  1: Data is transmitted on byte at a time from the master to the slave.
 *     The slave  received data and sends it back to computer via UART. 
 
 *  The variable, 'success', will be non-zero when the function reaches the
 *  infinite for-loop if the test was successful.
 */
int main(void)
{
	/* Init SS pin as output with wired AND and pull-up. */
	PORTC.DIRSET = PIN4_bm;
	PORTC.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	PORTC.OUTSET = PIN4_bm;

	/* Instantiate pointer to ssPort. */
	PORT_t *ssPort = &PORTC;

/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PC3 (TXD0) as output. */
	PORTC.DIRSET   = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR   = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock frequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	/* Enable global interrupts. */
	sei();
	
	/* Initialize SPI slave on port D. */
	SPI_SlaveInit(&spiSlaveD,
	              &SPID,
	              &PORTD,
	              false,
				  SPI_MODE_0_gc,
				  SPI_INTLVL_OFF_gc);

	/* PHASE 1: Transceive individual bytes. */

	/* MASTER: Pull SS line low. This has to be done since
	 *         SPI_MasterTransceiveByte() does not control the SS line(s). */
	SPI_MasterSSLow(ssPort, PIN4_bm);

	
		/* SLAVE: Wait for data to be available. */
		while (SPI_SlaveDataAvailable(&spiSlaveD) == false) {

		}

		/* SLAVE: Get the byte received. */
		uint8_t slaveByte = SPI_SlaveReadByte(&spiSlaveD);
       UART_TXBuffer_PutByte(&USART_data, slaveByte);	                     // send data 
		/* SLAVE: Increment received byte and send back. */
		//SPI_SlaveWriteByte(&spiSlaveD, slaveByte);

		
		
	

	while(true) {
		/* SLAVE: Wait for data to be available. */
		while (SPI_SlaveDataAvailable(&spiSlaveD) == false) {

		}

		/* SLAVE: Get the byte received. */
		uint8_t slaveByte = SPI_SlaveReadByte(&spiSlaveD);
          UART_TXBuffer_PutByte(&USART_data, slaveByte);	                     // send data 
	}
}

ISR(USARTC0_RXC_vect)
{ int receive=0;
	USART_RXComplete(&USART_data);
	if (USART_RXBufferData_Available(&USART_data)) {                                               // modified by  me
		receive = USART_RXBuffer_GetByte(&USART_data);}                  // receive the data      // modified
	UART_TXBuffer_PutByte(&USART_data, receive);	                     // send data 
	
}
/*ISR(USARTC0_RXC_vect)                                                                    // befor modified
{ 
	USART_RXComplete(&USART_data);
	
}*/

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}


