/*
 * Header file for SPI interface to MCP23S17 interface chip
 */ 
#ifndef ELEC3042_SPI_H
#define ELEC3042_SPI_H

/**
 * Transfer a byte of data across the SPI bus.
 * We return the byte of data returned (as SPI is synchronous)
 * @param data to transmit
 * @return data returned by slave
 */
uint8_t SPI_transfer(uint8_t data);

/**
 * Send a command/data byte pair to the MCP23S17
 * 
 * @param reg command register to which we will be writing.
 * @param data value to write to command register
 */
void SPI_Send_Command(uint8_t reg, uint8_t data);

/**
 * Read the value of a register on the MCP23S17
 * 
 * @param reg data register we wish to read
 * @return value of the register we read
 */
uint8_t SPI_Read_Command(uint8_t reg);

/**
 * Set up the SPI bus.
 * We assume a 16MHz IOclk rate, and that Port B Pin 2 is the SS output
 */
void setup_SPI();

/**
 * Set up the Port Expander.
 */
void setup_PortExpander();


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* ELEC3042_SPI_H */

