/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two different nodes,
 * connect the role_pin_sender to ground on one.  The ping node sends the current time to the pong node,
 * which responds by sending the value back.  The ping node can then see how long the whole cycle
 * took.
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
// (MV:) Adapted to work with the configuration of the shield. Original: RF24 radio(9,10);

RF24 radio(3, 9);

// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
const int role_pin_sender = 7;
const int role_repeat = 6;

//
// Topology
//

// Radio pipe addresses for the 3 nodes to communicate.
const uint64_t pipes[3] = { 0x123456789aLL, 0x987654321bLL, 0x384654f2cbLL };

const int sendValue = 170; // binary; 10101010
const int numberOfPackets = 1000;
const int RESETVAL = 42;

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  The hardware itself specifies
// which node it is.
//
// This is done through the role_pin_sender
//

// The various roles supported by this sketch
typedef enum { role_sender = 1, role_receiver = 2, role_repeater = 3 } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Sender", "Receiver", "Repeater"};

const rf24_pa_dbm_e outputPowerLevel[] = {RF24_PA_MAX, RF24_PA_HIGH, RF24_PA_LOW, RF24_PA_MIN};
const rf24_datarate_e datarateLevel[] = {RF24_250KBPS, RF24_1MBPS, RF24_2MBPS};


// The role of the current running sketch
role_e role;

void setup(void) {
    // set up the role pin
    pinMode(role_pin_sender, INPUT);
    digitalWrite(role_pin_sender,HIGH);
    delay(20); // Just to get a solid reading on the role pin
    
    // read the address pin, establish our role
    if (!digitalRead(role_pin_sender)) {
        role = role_sender; // sender        
    }
    else {
            // set up the role pin
        pinMode(role_repeat, INPUT);
        digitalWrite(role_repeat,HIGH);
        delay(20); // Just to get a solid reading on the role pin

        if (!digitalRead(role_repeat)) {
            role = role_repeater; // repeater
        }
        else {
            role = role_receiver; // receiver
        }
    }

    //
    // Print preamble
    //

    Serial.begin(57600);
    printf_begin();
    printf("\n\rRF24/examples/pingpair/\n\r");
    printf("ROLE: %s\n\r",role_friendly_name[role]);

    //
    // Setup and configure rf radio
    radio.begin();
    radio.setRetries(0,0);

    radio.setDataRate(datarateLevel[0]);
    radio.setPALevel(outputPowerLevel[0]);
    radio.setChannel(0);

    // optionally, reduce the payload size.  seems to
    // improve reliability
    radio.setPayloadSize(8);

    if ( role == role_sender ) { // Sender
        radio.openWritingPipe(pipes[1]); // Write to repeater
        radio.openReadingPipe(1,pipes[0]); // Read from repeater
    }
    else if (role == role_repeater){ // Repeater
        radio.openReadingPipe(1,pipes[1]);
    }
    else { // Receiver
        radio.openWritingPipe(pipes[1]);
        radio.openReadingPipe(1,pipes[2]);
    }

    radio.startListening();
    radio.printDetails();
}

// Bevat het nummer dat nu gestuurd moet worden
int currentNumber = 1;

void resetRadioReads(void) {
    radio.stopListening();
    radio.openWritingPipe(0x111111110aLL);
}

void loop(void) {
    if (role == role_sender) {
        radio.stopListening();
        radio.openWritingPipe(pipes[1]);
        bool ok = radio.write( &currentNumber, sizeof(int) );
        resetRadioReads();
        radio.startListening();

        // Wait here until we get a response, or timeout (250ms)
        unsigned long started_waiting_at = millis();
        bool timeout = false;
        while ( ! radio.available() && ! timeout )
            if (millis() - started_waiting_at > 250 )
                timeout = true;

        // Describe the results
        if ( timeout ) {
          // Zend opnieuw!
          // currentNumber wordt opnieuw verzonden in de volgende iteratie van loop()
          printf("Timeout occurred at sender; no ACK received. Packet #: %i\n", currentNumber);
        }
        else {
            int receivedValue;
            radio.read( &receivedValue, sizeof(int) );

            // ACK our value! Increase our success counter.
            if(receivedValue == - currentNumber) {
                // Success!
                printf("ACK succesfully received for packet #%i\n", currentNumber);
                currentNumber++;
            }
            else {
              // Received double ACK
              printf("Received double ACK. Packet #: %i\n", currentNumber);
            }
        }

        delay(50);
    }



    if ( role == role_receiver ) {
        // if there is data ready
        if ( radio.available() ) {
            
            // Dump the payloads until we've gotten everything
            int v; bool done = false;
            while (!done) {
                done = radio.read( &v, sizeof(int) );
                delay(10);
            }

            printf("Received packet # %i\n", v);

            v = -v; // ACK waarde is negatief

            radio.stopListening();
            radio.openWritingPipe(pipes[1]);
            radio.write( &v, sizeof(int) );
            resetRadioReads();
            radio.startListening();
        }
    }
    
    if (role==role_repeater) {

        if ( radio.available() ) {
            int v; bool done = false;
            while (!done) {
                done = radio.read( &v, sizeof(int) );
                delay(10);
            }

            radio.stopListening();

            if (v > 0) { // Dit is het originele bericht; stuur naar receiver
                radio.openWritingPipe(pipes[2]);
            }
            else { // Dit is de ACK; stuur naar sender
                radio.openWritingPipe(pipes[0]);
            }

            radio.write( &v, sizeof(int) );
            resetRadioReads();
            radio.startListening();
        }
    }
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
