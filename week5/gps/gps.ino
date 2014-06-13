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
#define Z 1.0

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

float matrix_A3[N][N] = { // Relatieve afstanden tussen de nodes; gebruikt node 3 nog niet!
    {2*pos[1][0] - 2*pos[0][0], 2*pos[1][1] - 2*pos[0][1], Z},
    {2*pos[2][0] - 2*pos[1][0], 2*pos[2][1] - 2*pos[1][1], Z},
    {2*pos[0][0] - 2*pos[2][0], 2*pos[0][1] - 2*pos[2][1], Z}
};

float matrix_A2[N][N] = { // Relatieve afstanden tussen de nodes; gebruikt node 2 nog niet!
    {2*pos[1][0] - 2*pos[0][0], 2*pos[1][1] - 2*pos[0][1], Z},
    {2*pos[3][0] - 2*pos[1][0], 2*pos[3][1] - 2*pos[1][1], Z},
    {2*pos[0][0] - 2*pos[3][0], 2*pos[0][1] - 2*pos[3][1], Z}
};
float matrix_A1[N][N] = { // Relatieve afstanden tussen de nodes; gebruikt node 1 nog niet!
    {2*pos[2][0] - 2*pos[0][0], 2*pos[2][1] - 2*pos[0][1], Z},
    {2*pos[3][0] - 2*pos[2][0], 2*pos[3][1] - 2*pos[2][1], Z},
    {2*pos[0][0] - 2*pos[3][0], 2*pos[0][1] - 2*pos[3][1], Z}
};
float matrix_A0[N][N] = { // Relatieve afstanden tussen de nodes; gebruikt node 0 nog niet!
    {2*pos[2][0] - 2*pos[1][0], 2*pos[2][1] - 2*pos[1][1], Z},
    {2*pos[3][0] - 2*pos[2][0], 2*pos[3][1] - 2*pos[2][1], Z},
    {2*pos[1][0] - 2*pos[3][0], 2*pos[1][1] - 2*pos[3][1], Z}
};

// Static; just add d_0 and d_1
float B_static3[N] = {
    /*d_0^2 - d_1^2 */ - (pos[0][0]*pos[0][0]) + (pos[1][0]*pos[1][0]) - (pos[0][1]*pos[0][1]) + (pos[1][1]*pos[1][1]) /* dz = 0 */,
    /*d_1^2 - d_2^2 */ - (pos[1][0]*pos[1][0]) + (pos[2][0]*pos[2][0]) - (pos[1][1]*pos[1][1]) + (pos[2][1]*pos[2][1]) /* dz = 0 */,
    /*d_2^2 - d_0^2 */ - (pos[2][0]*pos[2][0]) + (pos[0][0]*pos[0][0]) - (pos[2][1]*pos[2][1]) + (pos[0][1]*pos[0][1]) /* dz = 0 */
};

float B_static2[N] = {
    /*d_0^2 - d_1^2 */ - (pos[0][0]*pos[0][0]) + (pos[1][0]*pos[1][0]) - (pos[0][1]*pos[0][1]) + (pos[1][1]*pos[1][1]) /* dz = 0 */,
    /*d_1^2 - d_3^2 */ - (pos[1][0]*pos[1][0]) + (pos[3][0]*pos[3][0]) - (pos[1][1]*pos[1][1]) + (pos[3][1]*pos[3][1]) /* dz = 0 */,
    /*d_3^2 - d_0^2 */ - (pos[3][0]*pos[3][0]) + (pos[0][0]*pos[0][0]) - (pos[3][1]*pos[3][1]) + (pos[0][1]*pos[0][1]) /* dz = 0 */
};
float B_static1[N] = {
    /*d_0^2 - d_1^2 */ - (pos[0][0]*pos[0][0]) + (pos[2][0]*pos[2][0]) - (pos[0][1]*pos[0][1]) + (pos[2][1]*pos[2][1]) /* dz = 0 */,
    /*d_1^2 - d_3^2 */ - (pos[2][0]*pos[2][0]) + (pos[3][0]*pos[3][0]) - (pos[2][1]*pos[2][1]) + (pos[3][1]*pos[3][1]) /* dz = 0 */,
    /*d_3^2 - d_0^2 */ - (pos[3][0]*pos[3][0]) + (pos[0][0]*pos[0][0]) - (pos[3][1]*pos[3][1]) + (pos[0][1]*pos[0][1]) /* dz = 0 */
};
float B_static0[N] = {
    /*d_0^2 - d_1^2 */ - (pos[1][0]*pos[1][0]) + (pos[2][0]*pos[2][0]) - (pos[1][1]*pos[1][1]) + (pos[2][1]*pos[2][1]) /* dz = 0 */,
    /*d_1^2 - d_3^2 */ - (pos[2][0]*pos[2][0]) + (pos[3][0]*pos[3][0]) - (pos[2][1]*pos[2][1]) + (pos[3][1]*pos[3][1]) /* dz = 0 */,
    /*d_3^2 - d_0^2 */ - (pos[3][0]*pos[3][0]) + (pos[1][0]*pos[1][0]) - (pos[3][1]*pos[3][1]) + (pos[1][1]*pos[1][1]) /* dz = 0 */
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

  Matrix.Invert((float*)matrix_A3,N);
  Serial.println("\nInverted matrix_A3:");
  Matrix.Print((float*)matrix_A3,N,N,"matrix_A3");
  
  Matrix.Invert((float*)matrix_A2,N);
  Serial.println("\nInverted matrix_A2:");
  Matrix.Print((float*)matrix_A2,N,N,"matrix_A2");
  
  Matrix.Invert((float*)matrix_A1,N);
  Serial.println("\nInverted matrix_A1:");
  Matrix.Print((float*)matrix_A1,N,N,"matrix_A1");
  
  Matrix.Invert((float*)matrix_A0,N);
  Serial.println("\nInverted matrix_A0:");
  Matrix.Print((float*)matrix_A0,N,N,"matrix_A0");
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

  /* Zwak uitschieters een beetje af: max 30% increase
  if(diff > (D[activeBeacon]*1.15) && D[activeBeacon] > 0) {
    diff = D[activeBeacon] * 1.15;
  }
  
  if(diff < (D[activeBeacon]*0.85)) {
    diff = D[activeBeacon]*0.85;
  }
  
  D[activeBeacon] = D[activeBeacon]*0.8 + diff*0.2; // Weer schuivend gemiddelde */
  D[activeBeacon] = diff;

  if(activeBeacon == 3) {
    calcPosition();
  }
}

void calcPosition() {
    float Bdrie[N] = {
        (D[0]*D[0]) - (D[1]*D[1]) + B_static3[0],
        (D[1]*D[1]) - (D[2]*D[2]) + B_static3[1],
        (D[2]*D[2]) - (D[0]*D[0]) + B_static3[2]
    };
    
    float Btwee[N] = {
        (D[0]*D[0]) - (D[1]*D[1]) + B_static2[0],
        (D[1]*D[1]) - (D[3]*D[3]) + B_static2[1],
        (D[3]*D[3]) - (D[0]*D[0]) + B_static2[2]
    };
    
    float Been[N] = {
        (D[0]*D[0]) - (D[2]*D[2]) + B_static2[0],
        (D[2]*D[2]) - (D[3]*D[3]) + B_static2[1],
        (D[3]*D[3]) - (D[0]*D[0]) + B_static2[2]
    };
    
    float Bnul[N] = {
        (D[1]*D[1]) - (D[2]*D[2]) + B_static2[0],
        (D[2]*D[2]) - (D[3]*D[3]) + B_static2[1],
        (D[3]*D[3]) - (D[1]*D[1]) + B_static2[2]
    };

    float P3[N];
    Matrix.Multiply((float*)matrix_A3,(float*)Bdrie,N,N,1,(float*)P3);
    Serial.println("\nPosition (P3):");
    Matrix.Print((float*)P3,N,1,"P3");
    
    float P2[N];
    Matrix.Multiply((float*)matrix_A2,(float*)Btwee,N,N,1,(float*)P2);
    Serial.println("\nPosition (P2):");
    Matrix.Print((float*)P2,N,1,"P2");
    
    /*float P1[N];
    Matrix.Multiply((float*)matrix_A1,(float*)Been,N,N,1,(float*)P1);
    Serial.println("\nPosition (P1):");
    Matrix.Print((float*)P1,N,1,"P1");
    
    float P0[N];
    Matrix.Multiply((float*)matrix_A0,(float*)Bnul,N,N,1,(float*)P0);
    Serial.println("\nPosition (P0):");
    Matrix.Print((float*)P0,N,1,"P0");
    */
    int avg[N];
    avg[0] = (int)(P2[0] + P3[0]) / 2.0;
    avg[1] = (int)(P2[1] + P3[1]) / 2.0;
    avg[2] = (int)(P2[2] + P3[2]) / 2.0;
  
    printf("\nPosition: (%d,%d)\n", avg[0], avg[1]);
}
