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
 * connect the role_pin to ground on one.  The ping node sends the current time to the pong node,
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
const int role_pin = 7;

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0x123456789aLL, 0x987654321bLL };

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
// This is done through the role_pin
//

// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

const rf24_pa_dbm_e outputPowerLevel[] = {RF24_PA_MAX, RF24_PA_HIGH, RF24_PA_LOW, RF24_PA_MIN};
const rf24_datarate_e datarateLevel[] = {RF24_250KBPS, RF24_1MBPS, RF24_2MBPS};


// The role of the current running sketch
role_e role;

void setup(void) {
    // set up the role pin
    pinMode(role_pin, INPUT);
    digitalWrite(role_pin,HIGH);
    delay(20); // Just to get a solid reading on the role pin

    // read the address pin, establish our role
    if ( ! digitalRead(role_pin) )
        role = role_ping_out;
    else
        role = role_pong_back;

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

    if ( role == role_ping_out ) {
        radio.openWritingPipe(pipes[0]);
        radio.openReadingPipe(1,pipes[1]);
    }
    else {
        radio.openWritingPipe(pipes[1]);
        radio.openReadingPipe(1,pipes[0]);
    }

    radio.startListening();
    radio.printDetails();
}


// Number of succesfully received packages; 0 <= success <= rounds
int success = 0;
// Rounds of communication so far; 0 <= rounds <= numberOfPackets
int rounds = 0;
// Testnumber
int test = 0;
int test2 = 0;
int testChannel = 0;

void loop(void) {
    //
    // Ping out role.  Repeatedly send the current time
    //

    if (role == role_ping_out) {
        rounds++;

        radio.stopListening();
        bool ok = radio.write( &sendValue, sizeof(int) );
        radio.startListening();

        // Wait here until we get a response, or timeout (250ms)
        unsigned long started_waiting_at = millis();
        bool timeout = false;
        while ( ! radio.available() && ! timeout )
            if (millis() - started_waiting_at > 250 )
                timeout = true;

        // Describe the results
        if ( timeout ) {
            // printf(".");
        }
        else {
            int receivedValue;
            radio.read( &receivedValue, sizeof(int) );

            // Successfull round trip of our value! Increase our success counter.
            if(receivedValue == sendValue) {
                success++;
            }
        }

        if (rounds == numberOfPackets) {
            printf("\n--------\n");
           // printf("Power level: %i (0=MAX, 3=MIN)\n", test);
           // printf("Data rate: %i (0=250kbps, 1=1mbps 2=2mbps)\n", test);
            printf("Channel: %i\n", testChannel);
            printf("# packets sent:               %i\n", numberOfPackets);
            printf("# packets correctly received: %i\n", success);
            printf("--------\n");
            // Reset counters
            success = 0;
            rounds = 0;

            radio.stopListening();
            radio.setPALevel(outputPowerLevel[0]); // Max power; increase chance of succesfully receiving it
            bool ok = radio.write( &RESETVAL, sizeof(int) ); // Pray this will be received
            radio.startListening();

            test = (test+1)%3;
            test2 = (test2+1)%8; 
            //radio.setDataRate(datarateLevel[test]);
            // radio.setPALevel(outputPowerLevel[test]);
            
            testChannel = (15*test2);
            radio.setChannel(testChannel);
        }

        delay(10);
    }

    //
    // Pong back role.  Receive each packet, dump it out, and send it back
    //

    if ( role == role_pong_back ) {
        // if there is data ready
        if ( radio.available() ) {
            // Dump the payloads until we've gotten everything
            int v;
            bool done = false;
            while (!done) {
                // Fetch the payload, and see if this was the last one.
                done = radio.read( &v, sizeof(int) );

                delay(10);
            }

            if (v == RESETVAL) {
                test = (test+1)%3;
                test2 = (test2+1)%8;
                //radio.setDataRate(datarateLevel[test]);
                // radio.setPALevel(outputPowerLevel[test]);
                radio.setChannel(test2);
                printf("Finished test; moving to next!\n");
            }

            radio.stopListening();
            radio.write( &v, sizeof(int) );
            radio.startListening();
        }
    }
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
