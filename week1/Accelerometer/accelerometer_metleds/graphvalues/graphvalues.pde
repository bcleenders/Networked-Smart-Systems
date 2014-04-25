// Graph Multiple Sensors in Processing
 
import processing.serial.*;
Serial myPort;
 
int numValues = 3;    // number of input values or sensors
 
// rename these variables for your input:
float A = 0;          // red value
int Amin = -512;
int Amax = 512;
 
float B = 0;          // green value
int Bmin = -512;
int Bmax = 512;
 
float C = 0;          // blue value
int Cmin = -512;
int Cmax = 512;
 
float partH;          // partial screen height
 
int xPos = 1;         // horizontal position of the graph
 
 
void setup() {
  size(800, 600);
  partH = height / numValues;    // to divide screen up to draw the number of sensors
  
  // List all the available serial ports:
  println(Serial.list());
  // First port [0] in serial list usually Arduino, but check your own:
  myPort = new Serial(this, Serial.list()[0], 115200);
  // don't generate a serialEvent() until you get a newline character:
  myPort.bufferUntil('\n');
  background(0);
}
 
void draw() {
   // in this example, everything happens inside serialEvent()
   // but you can also do stuff in every frame if you wish
}
 
 
void serialEvent(Serial myPort) { 
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  //println(inString);        // < - uncomment this to debug serial input from Arduino
  
  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    // split the string on the commas and convert the resulting substrings into an integer array:
    int[] values = int(split(inString, ","));
    // if the array has at least the # of elements as your # of sensors, you know
    // you got the whole data packet.  Map the numbers and put into the variables:
    if (values.length >= numValues) {
      // map them to the range of partial screen height:
      A = map(values[0], Amin, Amax, 0, partH);
      B = map(values[1], Bmin, Bmax, 0, partH);
      C = map(values[2], Cmin, Cmax, 0, partH);
      //println(A + "\t" + B + "\t" + C);   // < - uncomment this to debug mapped values
      
      // draw lines:
      stroke(255, 0, 0);  // red
      line(xPos, partH,   xPos, partH    - A);
      stroke(0, 255, 0);  // green
      line(xPos, partH*2, xPos, partH*2  - B);
      stroke(0, 0, 255);  // blue
      line(xPos, partH*3, xPos, partH*3  - C);
      
      // draw dividing lines:
      stroke(255);
      line(0, partH,   width, partH);
      line(0, partH*2, width, partH*2);
 
      // if at the edge of the screen, go back to the beginning:
      if (xPos >= width) {
        xPos = 0;
        background(0);   // erase screen with black
      } 
      else {
        xPos++;          // increment the graph's horizontal position
      }
    }
  }
}
