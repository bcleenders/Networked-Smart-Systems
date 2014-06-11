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
  {
    0.0, 30.0        }
  ,
  {
    80.0, 0.0        }
  ,
  {
    320.0, 0.0        }
  ,
  {
    400.0, 30.0        }
};

float A[4][N][N] = {
  { // Relatieve afstanden tussen de nodes; gebruikt node 0 nog niet!
    {
      2*pos[2][0] - 2*pos[1][0], 2*pos[2][1] - 2*pos[1][1], 0.000001                }
    ,
    {
      2*pos[3][0] - 2*pos[2][0], 2*pos[3][1] - 2*pos[2][1], 0.000001                }
    ,
    {
      2*pos[1][0] - 2*pos[3][0], 2*pos[1][1] - 2*pos[3][1], 0.000001                }
  }
  ,
  { // Relatieve afstanden tussen de nodes; gebruikt node 1 nog niet!
    {
      2*pos[2][0] - 2*pos[0][0], 2*pos[2][1] - 2*pos[0][1], 0.000001                }
    ,
    {
      2*pos[3][0] - 2*pos[2][0], 2*pos[3][1] - 2*pos[2][1], 0.000001                }
    ,
    {
      2*pos[0][0] - 2*pos[3][0], 2*pos[0][1] - 2*pos[3][1], 0.000001                }
  }
  ,
  { // Relatieve afstanden tussen de nodes; gebruikt node 2 nog niet!
    {
      2*pos[1][0] - 2*pos[0][0], 2*pos[1][1] - 2*pos[0][1], 0.000001                }
    ,
    {
      2*pos[3][0] - 2*pos[1][0], 2*pos[3][1] - 2*pos[1][1], 0.000001                }
    ,
    {
      2*pos[0][0] - 2*pos[3][0], 2*pos[0][1] - 2*pos[3][1], 0.000001                }
  }
  ,
  { // Relatieve afstanden tussen de nodes; gebruikt node 3 nog niet!
    {
      2*pos[1][0] - 2*pos[0][0], 2*pos[1][1] - 2*pos[0][1], 0.000001                }
    ,
    {
      2*pos[2][0] - 2*pos[1][0], 2*pos[2][1] - 2*pos[1][1], 0.000001                }
    ,
    {
      2*pos[0][0] - 2*pos[2][0], 2*pos[0][1] - 2*pos[2][1], 0.000001                }
  }
};

// Static; just add d_0 and d_1
float B_static[4][N] = {
  {
    /*d_0^2 - d_1^2 */    - (pos[1][0]*pos[1][0]) + (pos[2][0]*pos[2][0]) - (pos[1][1]*pos[1][1]) + (pos[2][1]*pos[2][1]) /* dz = 0 */,
    /*d_1^2 - d_2^2 */    - (pos[2][0]*pos[2][0]) + (pos[3][0]*pos[3][0]) - (pos[2][1]*pos[2][1]) + (pos[3][1]*pos[3][1]) /* dz = 0 */,
    /*d_2^2 - d_0^2 */    - (pos[3][0]*pos[3][0]) + (pos[1][0]*pos[1][0]) - (pos[3][1]*pos[3][1]) + (pos[1][1]*pos[1][1]) /* dz = 0 */
    }
    ,
  {
    /*d_0^2 - d_1^2 */    - (pos[0][0]*pos[0][0]) + (pos[2][0]*pos[2][0]) - (pos[0][1]*pos[0][1]) + (pos[2][1]*pos[2][1]) /* dz = 0 */,
    /*d_1^2 - d_2^2 */    - (pos[2][0]*pos[2][0]) + (pos[3][0]*pos[3][0]) - (pos[2][1]*pos[2][1]) + (pos[3][1]*pos[3][1]) /* dz = 0 */,
    /*d_2^2 - d_0^2 */    - (pos[3][0]*pos[3][0]) + (pos[0][0]*pos[0][0]) - (pos[3][1]*pos[3][1]) + (pos[0][1]*pos[0][1]) /* dz = 0 */
    }
    ,
  { // slaat 2 over
    /*d_0^2 - d_1^2 */    - (pos[0][0]*pos[0][0]) + (pos[1][0]*pos[1][0]) - (pos[0][1]*pos[0][1]) + (pos[1][1]*pos[1][1]) /* dz = 0 */,
    /*d_1^2 - d_2^2 */    - (pos[1][0]*pos[1][0]) + (pos[3][0]*pos[3][0]) - (pos[1][1]*pos[1][1]) + (pos[3][1]*pos[3][1]) /* dz = 0 */,
    /*d_2^2 - d_0^2 */    - (pos[3][0]*pos[3][0]) + (pos[0][0]*pos[0][0]) - (pos[3][1]*pos[3][1]) + (pos[0][1]*pos[0][1]) /* dz = 0 */
    }
    ,
  { // slaat 3 over
    /*d_0^2 - d_1^2 */    - (pos[0][0]*pos[0][0]) + (pos[1][0]*pos[1][0]) - (pos[0][1]*pos[0][1]) + (pos[1][1]*pos[1][1]) /* dz = 0 */,
    /*d_1^2 - d_2^2 */    - (pos[1][0]*pos[1][0]) + (pos[2][0]*pos[2][0]) - (pos[1][1]*pos[1][1]) + (pos[2][1]*pos[2][1]) /* dz = 0 */,
    /*d_2^2 - d_0^2 */    - (pos[2][0]*pos[2][0]) + (pos[0][0]*pos[0][0]) - (pos[2][1]*pos[2][1]) + (pos[0][1]*pos[0][1]) /* dz = 0 */
    }
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

  Matrix.Invert((float*)A[0],N);
  Matrix.Invert((float*)A[1],N);
  Matrix.Invert((float*)A[2],N);
  Matrix.Invert((float*)A[3],N);

  Serial.println("\nInverted A[0]:");
  Matrix.Print((float*)A[0],N,N,"A[0]");

  Serial.println("\nInverted A[1]:");
  Matrix.Print((float*)A[1],N,N,"A[1]");

  Serial.println("\nInverted A[2]:");
  Matrix.Print((float*)A[2],N,N,"A[2]");

  Serial.println("\nInverted A[3]:");
  Matrix.Print((float*)A[3],N,N,"A[3]");
}

void loop() {
  while(radio.available()) { 
    radio.read(&activeBeacon, sizeof(uint8_t)); 
  }
  while (! radio.available());

  radiotime = micros();
  radio.read( &activeBeacon, sizeof(uint8_t));

  if(activeBeacon > 3) { 
    return; 
  }

  while(analogRead(A0) < 50) {
    audiotime = micros();
    if(audiotime - radiotime > timelimit) {
      Serial.println("timeout");
      return; 
    }
  }

  float diff = audiotime - radiotime;
  diff = diff * 0.03432; // Afstand tot beacon in cm

  D[activeBeacon] = D[activeBeacon]*0.8 + diff*0.2; // Weer schuivend gemiddelde

  if(activeBeacon == 3) {
    calcPosition();
  }
}

void calcPosition() {
  float P_avg[N];

  for(int i = 0; i < 4; i++) {
    float B[N] = {
      (D[i      ]*D[i      ]) - (D[(i+1)%3]*D[(i+1)%3]) + B_static[i      ],
      (D[(i+1)%3]*D[(i+1)%3]) - (D[(i+2)%3]*D[(i+2)%3]) + B_static[(i+1)%3],
      (D[(i+2)%3]*D[(i+2)%3]) - (D[i      ]*D[i      ]) + B_static[(i+2)%3]
    };

    float P[N];
    Matrix.Multiply((float*)A,(float*)B,N,N,1,(float*)P);

    Matrix.((float*)P, (float*)P_avg, 1, N, (float*)P_avg);
  }

  P_avg[0] = P_avg[0]/4;
  P_avg[1] = P_avg[1]/4;
  P_avg[2] = P_avg[2]/4;
  P_avg[3] = P_avg[3]/4;

  Serial.println("\nPosition (P_avg):");
  Matrix.Print((float*)P_avg,N,1,"P_avg");
}




