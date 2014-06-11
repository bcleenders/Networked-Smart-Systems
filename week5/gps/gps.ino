/*
    GPS algorithm borrowed from: 
        http://link.springer.com/chapter/10.1007/978-3-642-25486-4_59#page-1
*/
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "MatrixMath.h"

#define N (3)

RF24 radio(3, 9);
unsigned long radiotime;
unsigned long audiotime;
unsigned long timelimit = 50000LL;
uint8_t activeBeacon;

float pos[4][2] = { // Positions van de beacons; pos[1][1] is de y positie van beacon 1
    {0.0, 75.0},
    {72.0, 0.0},
    {294.0, 0.0},
    {372.0, 136.0}
};

float A[N][N] = { // Relatieve afstanden tussen de nodes; gebruikt node 3 nog niet!
    {2*pos[1][0] - 2*pos[0][0], 2*pos[1][1] - 2*pos[0][1], 0.000001},
    {2*pos[2][0] - 2*pos[1][0], 2*pos[2][1] - 2*pos[1][1], 0.000001},
    {2*pos[0][0] - 2*pos[2][0], 2*pos[0][1] - 2*pos[2][1], 0.000001}
};

// Static; just add d_0 and d_1
float B_static[N] = {
    /*d_0^2 - d_1^2 */ - (pos[0][0]*pos[0][0]) + (pos[1][0]*pos[1][0]) - (pos[0][1]*pos[0][1]) + (pos[1][1]*pos[1][1]) /* dz = 0 */,
    /*d_1^2 - d_2^2 */ - (pos[1][0]*pos[1][0]) + (pos[2][0]*pos[2][0]) - (pos[1][1]*pos[1][1]) + (pos[2][1]*pos[2][1]) /* dz = 0 */,
    /*d_2^2 - d_0^2 */ - (pos[2][0]*pos[2][0]) + (pos[0][0]*pos[0][0]) - (pos[2][1]*pos[2][1]) + (pos[0][1]*pos[0][1]) /* dz = 0 */
};

float D[4];


void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
  printf_begin();

  // Setup and configure rf radio
  radio.begin();
  radio.setRetries(0,0);

  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(76);
  radio.setPayloadSize(1);
  radio.openReadingPipe(1, 0xdeadbeefa1LL);
  radio.openWritingPipe(0xdeadbeefa1LL);
  radio.startListening();
  radio.setAutoAck(false);
  //radio.printDetails();

  Matrix.Invert((float*)A,N);
  Serial.println("\nInverted A:");
  Matrix.Print((float*)A,N,N,"A");
}

void loop() {
  while(radio.available()) { 
    radio.read(&activeBeacon, sizeof(uint8_t)); 
  }
  while (! radio.available());

  radiotime = micros();
  radio.read( &activeBeacon, sizeof(uint8_t));

  if(activeBeacon > 3) { return; }

  while(analogRead(A0) < 50) {
    audiotime = micros();
    if(audiotime - radiotime > timelimit) {
      //Serial.println("timeout");
      return; 
    }
  }
  
  float diff = audiotime - radiotime;
  diff = diff * 0.03432; // Afstand tot beacon in cm

//  D[activeBeacon] = D[activeBeacon]*0.8 + diff*0.2; // Weer schuivend gemiddelde
  D[activeBeacon] = diff;

  if(activeBeacon == 3) {
    calcPosition();
  }
}

void calcPosition() {
    float B[N] = {
        (D[0]*D[0]) - (D[1]*D[1]) + B_static[0],
        (D[1]*D[1]) - (D[2]*D[2]) + B_static[1],
        (D[2]*D[2]) - (D[0]*D[0]) + B_static[2]
    };

    float P[N];
    Matrix.Multiply((float*)A,(float*)B,N,N,1,(float*)P);
    Serial.println("\nPosition (P):");
    Matrix.Print((float*)P,N,1,"P");
}
