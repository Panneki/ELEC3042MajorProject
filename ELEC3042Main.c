/*
 * File:   ELEC3042_L7P2_simonSays.c
 *
 * Created by Rex on 10th April 2025
 * 
 * Simon Says game
 */

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ELEC3042_SPI.h"

volatile uint32_t   clock_count = 0;
volatile uint8_t    button_int = 0;
uint32_t tone_on = 0;       // time when tone started playing
const uint32_t tone_len = 1000;     // length of time a tone plays
const uint32_t tone_err = 200;      // length of time during an error sequence
uint8_t button = 0;         // last button pressed

// current set of states (Machine 1)
enum PHASE_STATE {
    INIT,               // initialising the system
    HZRD,               // Hazard State
    PRT,
    RST,                // Add a new step to the chain
    DST,
    PRWR,               //park road west right
} state = INIT;
// current set of states (Machine 2)
enum LIGHT_STATE {
    GREEN,
    YELLOW,
    RED,
} Light = INIT;


uint32_t millis() {
    /*
     * Return the current clock_count value.
     * We temporarily disable interrupts to ensure the clock_count value
     * doesn't change while we are reading them.
     * 
     * We then restore the original SREG, which contains the I flag.
     */
    register uint32_t count;
    register char cSREG;
    
    cSREG = SREG;
    cli();
    count = clock_count;
    SREG = cSREG;
    return count;
}

/*
 * We interrupt 1000 times a second,
 * so clock_count increments once every millisecond.
 */
ISR(TIMER2_COMPA_vect) {
    clock_count++;
}
//switch (state) { } use this for states

void write_LEDs(uint16_t leds) {
    SPI_Send_Command(0x14, (leds & 0xff));
    SPI_Send_Command(0x15, (leds >> 8) & 0xff);
}

void setupTimer1() {
    /*
     * we want to send a square wave out OC1A (Port B Pin 1)
     * We have a 16MHz crystal, so the main clock is 16MHz.
     * We want to generate tones about the 1kHz range.
     * 
     * We will initially divide the clock by 8 (CS12/CS11/CS10 = 010)
     * which gives us a 2MHz clock. The timer divides this signal by
     * up to 65536 which gives us the final range of frequencies: 15Hz to 1MHz
     * which covers the range we desire.
     */
    DDRB |= _BV(1);         // set PORT B Pin 1 as an output
    PORTB &= ~_BV(1);       // set output to low
    /*
     * We will toggle Port B Pin 1 on timer expiry, clock /8, WGM=4 (CTC mode)
     */
    TCCR1A = 0b01000000;    // COM1A0 = 1
    TCCR1B = 0b00001000;    // noise = 0, clock is currently stopped.
    TCCR1C = 0b00000000;    // no force output
    /*
     * We set the frequency on OCR1A. When the counter matches this value the
     * output pin will toggle (1->0, 0->1). In this example we want the note
     * frequency to be 440Hz ('A'). This means the output has to toggle at
     * 880Hz. The clock is 2000000Hz, so 2000000/880 = 2273.
     * 
     * We can do the same calculation for whatever frequency we want.
     */
    OCR1A = 2273;           // 440Hz = Middle A
}

void setupTimer2() {
    /*
     * Timer 2 is setup as a millisecond interrupting timer.
     * This generates a regular millisecond interrupt,
     * which we can use internally to create a counter.
     * 
     * This counter is the basis of all internal timing.
     * 
     * The main clock is 16MHz, we need a 1kHz interrupt, so
     * the counter needs to count 16000 clock pulses and then interrupt.
     * 
     * The counter can count to 256, with prescalers of 8, 32, 64, 128, 256 or 1024
     * 
     * Using a prescaler of 128 we need a count of 125.
     * Resulting in 124 being the OCR2A number
     * We want the prescaler to be as large as possible,
     * as it uses less power then the main counter.
     */
    TCCR2B = 0;             // turn off counter to configure
    TCNT2 = 0;              // set current count to zero
    OCR2A = 124;            // 125 * 128 * 1000 = 16000000
    TIFR2 = 0b00000111;     // clear all existing interrupts
    TIMSK2 = 0b00000010;    // enable interrupt on OCRA
    ASSR = 0;               // no async counting
    TCCR2A = 0b00000010;    // No I/O, mode = 2 (CTC)
    TCCR2B = 0b00000101;    // clock/128, mode = 2 (CTC), start counter
}

/*
 * Play the 4 tones and the error tone
 */
void startTone(uint8_t num) {
    static uint16_t tones[] = {1136, 2272, 4545, 6067, 9090, 568};
    
    if (num < 1 || num > 6) {
        write_LEDs(0);      // turn off LEDs
        OCR1A = 0;
        TCCR1B = 0b00001000;
        tone_on = 0;
    } else if (num != 6) {
        write_LEDs(0x000e << ((num - 1) << 2)); // turn on the LEDs
        OCR1A = tones[num - 1];
        TCCR1B = 0b00001010;
        tone_on = millis();
    } else {
        // win tones and ALL leds on
        write_LEDs(0xeeee);
        OCR1A = tones[num - 1];
        TCCR1B = 0b00001010;
        tone_on = millis();
    }
}

/*
 * We set the Data direction register to specify which pins are inputs and
 * which are outputs. A 0 bit indicates the corresponding pin is an input,
 * a 1 indicates the corresponding pin is an output
 * 
 * For the speaker we use PB1.
 * PC1 is the push button input
 * PD4-7 are available as debug LEDs
 * PD2 is available as an input for the interrupt
 */
void setup() {
    DDRB  |=  0b00000010;   // speaker output
    PORTB |=  0b00000000;
    DDRC  |=  0b00001110;   // LEDS on PC1-3
    PORTC &= ~0b00001110;   // start off
    DDRD  |=  0b11110000;   // set LEDs as outputs
    PORTD &= ~0b11110000;   // turn off LEDs
    
    // setup the interrupt for when the PORT expander send an interrupt
    EICRA = _BV(ISC01);     // falling edge of INT 0 causes an interrupt
    EIMSK = _BV(INT0);
    EIFR = _BV(INTF0);
}

ISR(INT0_vect) {
    button_int = 1;
}

// Initialising Buttons
void buttonPressed() {
    uint8_t bp;
    
    button_int = 0;
    bp = SPI_Read_Command(0x12);
    if ((bp & 0x01) == 0) {
        button = 1;
        return;
    }
    if ((bp & 0x10) == 0) {
        button = 2;
        return;
    }
    bp = SPI_Read_Command(0x13);
    if ((bp & 0x01) == 0) {
        button = 3;
        return;
    }
    if ((bp & 0x10) == 0) {
        button = 4;
        return;
    }
}

void debug(uint8_t dbg) {
    PORTC &= ~0b00001110;   // turn off LEDs
    PORTC |= (dbg << 1);    // turn on status
}
     
void debug1(uint8_t dbg) {
    PORTD &= ~0b11110000;   // turn off LEDs
    PORTD |= (dbg << 4);    // turn on LEDs
}

int main(void) {
    uint32_t    now;                    // current time
    
    uint32_t    step_start = 0;         // when this step started
    const uint32_t    step_len = 1100;  // length until the next step
    
    uint8_t     steps[16];              // steps in the pattern
    uint8_t     len = 0;                // current length
    uint8_t     step = 0;               // current step
    
    setup();    // set up the physical hardware
    setupTimer1();
    setupTimer2();
    setup_SPI();
	setup_PortExpander();
    
    sei();

	// main loop
    while (1) {
        if (button_int) {
            // a button interrupt has occurred - set buttons
            buttonPressed();
            button_int = 0;
        }
        
        now = millis();     // current time
        
        // State Machine 2 (tones): check if it needs to do something
        switch(tone) {
            case OFF:
                startTone(0);
                break;
                
            case NORMAL:
                if ((now - tone_on) >= tone_len) {
                    startTone(0);
                }
                break;
        }
        
        // State Machine 1: 
        switch(state) {
            case INIT:
                startTone(0);
                len = 0;
                step = 0;
                state = ADD;   // We start by adding another element to the chain
                step_start = now;
                tone = NORMAL;
                button = 0;     // clear pending buttons
                
//                for (len = 0; len < 4; len++) {
//                    steps[len] = (LFSR() & 0x03) + 1; // a next random state
//                }
//                state = PLAY;
                break;
                
            case PLAY_WAIT:
                /*
                 * wait for the current tone to finish, then play
                 */
                debug(5);
                if ((now - step_start) < step_len) {
                    // still repeating the last tone
                    break;
                }
                state = PLAY;
                step = 0;
                step_start = now;
                break;
                
            case PLAY:
                /*
                 * We play a set of tones, each for tone_len then move onto the next at step_len
                 */
                if (step < len) {
                    if ((now - step_start) < step_len) {
                        // wait for sounds to finish
                        break;
                    }
                    step_start = now;
                    debug(steps[step]);
                    startTone(steps[step]); // play the current sound
                    step++;
                    debug1(step);
                    break;
                }
                
                state = ADD;    // all played - please add another
                break;
                
            case ADD:
                if ((now - step_start) < step_len) {
                    // wait for tone to finish
                    break;
                }
                // we add a tone and then play it
                step_start = now;
                steps[len] = (LFSR() & 0x03) + 1; // a next random state
                startTone(steps[len]);      // play the tone we just added
                len++;
                debug1(len);
                state = ADD_PLAY;           // wait for the tone to finish
                break;
                
            case ADD_PLAY:
                if ((now - step_start) < step_len) {
                    // wait for tone to finish
                    break;
                }
                // tone has finished playing
                startTone(0);
                state = LISTEN;
                step_start = now;   // when we start the listen
                step = 0;           // we listen from the beginning
                break;
                
            case LISTEN:
                // wait for a button press or timeout
                // then check it matches the one in the list
                debug(4);
                debug1(step);

                if ((now - step_start) < step_len) {
                    // still repeating the last tone
                    break;
                }
                if (button != 0) {
                    step_start = now;   // when we start the listen
                    startTone(button);
                    if (button != steps[step]) {
                        startTone(5);
                        // wrong button pressed
                        state = WRONG;
                        button = 0;
                        break;
                    }
                    button = 0;         // handled this button
                    step++;
                    if (step > 15) {          // number needed to win!
                        // The player has won
                        state = WIN;
                        debug(6);
                        break;
                    }
                    if (step == len) {
                        state = PLAY_WAIT;
                        break;
                    }
                }
                break;
                
            case WRONG:
                // the player has made a mistake.
                // Wait until the error message has played.

                if ((now - step_start) < step_len) {
                    // still repeating the last tone
                    break;
                }
                state = INIT;
                break;
                
            case WIN:
                if ((now - step_start) < step_len) {
                    // still repeating the last tone
                    break;
                }
                startTone(6); // play the current sound
                step_start = now;
                state = WIN2;
                break;
                
            case WIN2:
                if ((now - step_start) < step_len) {
                    // still repeating the last tone
                    break;
                }
                startTone(0); // play the current sound
                state = INIT;
                break;
                
        }
    }
}
