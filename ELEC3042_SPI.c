/*
 * File:   ELEC3042_SPI.c
 * Author: Alan
 *
 * Created on 28 March 2023
 * 
 * This file talks SPI to an MCP23S17 dual port expander chip
 * Slave select for the SPI chip is on PB2
 */

#include <xc.h>
#include <stdint.h>
#include "ELEC3042_SPI.h"

/**
 * Transfer a byte of data across the SPI bus.
 * We return the byte of data returned (as SPI is synchronous)
 * @param data to transmit
 * @return data returned by slave
 */
uint8_t SPI_transfer(uint8_t data) {
    
    SPDR = data;
    while ((SPSR & _BV(SPIF)) == 0) {
        ;   // wait until transfer completed
    }
    return SPDR;
}

/**
 * Send a command/data byte pair to the MCP23S17
 * 
 * @param reg command register to which we will be writing.
 * @param data value to write to command register
 */
void SPI_Send_Command(uint8_t reg, uint8_t data) {
    // Send a command + byte to SPI interface
    PORTB &= ~_BV(2);    // SS enabled (low))
    SPI_transfer(0x40);  // Send command for SPI data transfer
    SPI_transfer(reg);   // MCP23S17 register address
    SPI_transfer(data);  // data to write to MCP23S17 register
    PORTB |= _BV(2);    // SS disabled (high)
}

/**
 * Read the value of a register on the MCP23S17
 * 
 * @param reg data register we wish to read
 * @return value of the register we read
 */
uint8_t SPI_Read_Command(uint8_t reg) {
    uint8_t data;
    
    // TODO: Read a command output from SPI interface
PORTB &= ~_BV(2);    // SS enabled (low))
    SPI_transfer(0x41);  // Send command for SPI data transfer
    SPI_transfer(reg);   // MCP23S17 register address
    SPI_transfer(data);  // data to write to MCP23S17 register
    PORTB |= _BV(2);    // SS disabled (high)


    return data;
}

/**
 * Set up the SPI bus.
 * We assume a 16MHz IOclk rate, and that Port B 
 * Pin 2 is the SS output which should be set high
 */
void setup_SPI() {
	// TODO: setup PORTB for SPI
    DDRB|=0b00101100;
     PORTB |= _BV(2);//setup SS as high
            
}

/**
 * Set up the Port Expander.
 *
 * We will configure port A as all outputs
 * Port B 0-3 are outputs and 4-7 are inputs
 * Turn on pull-up resistors on Port B
 */
void setup_PortExpander() {
    // Setup SPI operations (See pp176-177 of the data sheet)
	SPCR = _BV(SPE)|_BV(MSTR);   // set master SPI, SPI mode 0 operation
	SPSR = 0;                    // clear interrupt flags and oscillator mode.

    //Now that the SPI interface is configured we need to send SPI commands to
    //configure the MCP23S17 port expander IC
    SPI_Send_Command(0x00, 0x11);   // register IODIRA (port A data direction)
    SPI_Send_Command(0x01, 0x11);   // register IODIRB (port B data direction)
    SPI_Send_Command(0x0d, 0x11);   // register GPPUA (port A GPIO Pullups)
    SPI_Send_Command(0x0d, 0xff);   // register GPPUB (port B GPIO Pullups)
}