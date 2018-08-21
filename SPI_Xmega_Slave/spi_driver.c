/*
 * spi_driver.c
 *
 * Created: 12-05-2018 23:01:56
 *  Author: PRASHANT KURREY
 */ 
/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      XMEGA SPI driver source file.
 *
 *      This file contains the function implementations the XMEGA SPI driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the XMEGA SPI module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *      Several functions use the following construct:
 *          "some_register = ... | (some_parameter ? SOME_BIT_bm : 0) | ..."
 *      Although the use of the ternary operator ( if ? then : else ) is discouraged,
 *      in some occasions the operator makes it possible to write pretty clean and
 *      neat code. In this driver, the construct is used to set or not set a
 *      configuration bit based on a boolean input parameter, such as
 *      the "some_parameter" in the example above.
 *
 * \par Application note:
 *      AVR1309: Using the XMEGA SPI
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 5407 $
 * $Date: 2011-10-12 14:53:14 +0200 (on, 12 okt 2011) $  \n
 *
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.

 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.

 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "spi_driver.h"



/*! \brief Initialize SPI module as master.
 *
 *  This function initializes a SPI module as master. The CTRL and INTCTRL
 *  registers for the SPI module is set according to the inputs to the function.
 *  In addition, data direction for the MOSI and SCK pins is set to output.
 *
 *  \param spi            The SPI_Master_t struct instance.
 *  \param module         The SPI module.
 *  \param port           The I/O port where the SPI module is connected.
 *  \param lsbFirst       Data order will be LSB first if this is set to a
 *                        non-zero value.
 *  \param mode           SPI mode (Clock polarity and phase).
 *  \param intLevel       SPI interrupt level.
 *  \param clk2x	      SPI double speed mode
 *  \param clockDivision  SPI clock prescaler divison factor.
 */
void SPI_MasterInit(SPI_Master_t *spi,
                    SPI_t *module,
                    PORT_t *port,
                    bool lsbFirst,
                    SPI_MODE_t mode,
                    SPI_INTLVL_t intLevel,
                    bool clk2x,
                    SPI_PRESCALER_t clockDivision)
{
	spi->module         = module;
	spi->port           = port;
	spi->interrupted    = false;

	spi->module->CTRL   = clockDivision |                  /* SPI prescaler. */
	                      (clk2x ? SPI_CLK2X_bm : 0) |     /* SPI Clock double. */
	                      SPI_ENABLE_bm |                  /* Enable SPI module. */
	                      (lsbFirst ? SPI_DORD_bm  : 0) |  /* Data order. */
	                      SPI_MASTER_bm |                  /* SPI master. */
	                      mode;                            /* SPI mode. */

	/* Interrupt level. */
	spi->module->INTCTRL = intLevel;

	/* No assigned data packet. */
	spi->dataPacket = NULL;

 	/* MOSI and SCK as output. */
	spi->port->DIRSET  = SPI_MOSI_bm | SPI_SCK_bm;
}



/*! \brief Initialize SPI module as slave.
 *
 *  This function initializes a SPI module as slave. The CTRL and INTCTRL
 *  registers for the SPI module is set according to the inputs to the function.
 *  In addition, data direction for the MISO pin is set to output.
 *
 *  \param spi                  The SPI_Slave_t instance.
 *  \param module               Pointer to the SPI module.
 *  \param port                 The I/O port where the SPI module is connected.
 *  \param lsbFirst             Data order will be LSB first if this is set to true.
 *  \param mode                 SPI mode (Clock polarity and phase).
 *  \param intLevel             SPI interrupt level.
 */
void SPI_SlaveInit(SPI_Slave_t *spi,
                   SPI_t *module,
                   PORT_t *port,
                   bool lsbFirst,
                   SPI_MODE_t mode,
                   SPI_INTLVL_t intLevel)
{
	/* SPI module. */
	spi->module       = module;
	spi->port         = port;

	spi->module->CTRL = SPI_ENABLE_bm |                /* Enable SPI module. */
	                    (lsbFirst ? SPI_DORD_bm : 0) | /* Data order. */
	                    mode;                          /* SPI mode. */

	/* Interrupt level. */
	spi->module->INTCTRL = intLevel;

	/* MISO as output. */
	spi->port->DIRSET = SPI_MISO_bm;
}





/*! \brief SPI mastertransceive byte
 *
 *  This function clocks data in the DATA register to the slave, while data
 *  from the slave is clocked into the DATA register. The function does not
 *  check for ongoing access from other masters before initiating a transfer.
 *  For multimaster systems, checkers should be added to avoid bus contention.
 *
 *  SS line(s) must be pulled low before calling this function and released
 *  when finished.
 *
 *  \note This function is blocking and will not finish unless a successful
 *        transfer has been completed. It is recommended to use the
 *        interrupt-driven driver for applications where blocking
 *        functionality is not wanted.
 *
 *  \param spi        The SPI_Master_t struct instance.
 *  \param TXdata     Data to transmit to slave.
 *
 *  \return           Data received from slave.
 */
uint8_t SPI_MasterTransceiveByte(SPI_Master_t *spi, uint8_t TXdata)
{
	/* Send pattern. */
	spi->module->DATA = TXdata;

	/* Wait for transmission complete. */
	while(!(spi->module->STATUS & SPI_IF_bm)) {

	}
	/* Read received data. */
	uint8_t result = spi->module->DATA;

	return(result);
}



