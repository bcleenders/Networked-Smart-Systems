/*

 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
  
  // initialize the LED as output
  pinMode(led, OUTPUT); 
}

void loop() {
  // send the value of analog input 0:
  Serial.println(analogRead(A0));
  
  if(analogRead(A0) > 250) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
  // wait a bit for the analog-to-digital converter 
  // to stabilize after the last reading:
  delay(100);
}
