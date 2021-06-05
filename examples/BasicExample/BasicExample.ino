/*
 * Test Arduino Manager for iPad / iPhone / Mac
 *
 * A simple test program to show the Arduino Manager
 * features.
 *
 * Author: Fabrizio Boco - fabboco@gmail.com
 *
 * Version: 1.0
 *
 * 06/02/2021
 *
 * All rights reserved
 *  
 */

/*
*
 * AMCOntroller libraries, example sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you 
 * by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation 
 * constitutes acceptance of these terms.  
 * If you do not agree with these terms, please do not use or install The Software.
 * The Author grants you a personal, non-exclusive license, under author's copyrights in this original software, to use The Software. 
 * Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any 
 * patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
 * The Software and the Documentation are provided by the Author on an "AS IS" basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT 
 * LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION 
 * ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, 
 * REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), 
 * STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "AM_BLESPIFriend.h"

#define YELLOWLEDPIN     4
#define CONNECTIONPIN    2
#define TEMPERATUREPIN   0
#define VARIABLELED      3
#define POTENTIOMETERPIN 3

boolean yellowLed;
int connectionLed;
int pot;
float temperature;
int ledBrightness;
/*
 *
 * Prototypes of AMController’s callbacks
 *
 *
 */
void doWork();
void doSync();
void processIncomingMessages(char *variable, char *value);
void processOutgoingMessages();
void deviceConnected();
void deviceDisconnected();


/*
 *
 * AMController Library initialization
 *
*/
AMController amController(&doWork,&doSync,&processIncomingMessages,&processOutgoingMessages,&deviceConnected,&deviceDisconnected); 
 
void setup()
{
  Serial.begin(115200);
  
  // Yellow LED on
  pinMode(YELLOWLEDPIN,OUTPUT);
  yellowLed=HIGH;
  digitalWrite(YELLOWLEDPIN,yellowLed);

  
  // Red LED OFF
  pinMode(CONNECTIONPIN,OUTPUT);
  connectionLed=LOW;
  digitalWrite(CONNECTIONPIN,connectionLed);
    
  pinMode(VARIABLELED,OUTPUT);  
  ledBrightness = 0;
  analogWrite(VARIABLELED,0);
  
  pinMode(POTENTIOMETERPIN,INPUT);
  pinMode(TEMPERATUREPIN,INPUT);

  Serial.println(F("Setup completed"));
}

void loop() {
  
  amController.loop();
}


void doWork() {
  
  temperature = getVoltage(TEMPERATUREPIN);  //getting the voltage reading from the temperature sensor
  temperature = (temperature - 0.5) * 100;  // converting from 10 mv per degree with 500 mV offset
  digitalWrite(YELLOWLEDPIN,yellowLed);
  pot = analogRead(POTENTIOMETERPIN);
}

void doSync() {
  //Serial.println("Sync");
  amController.writeMessage("S1",yellowLed);
  amController.writeMessage("Knob1",ledBrightness);
}

void processIncomingMessages(char *variable, char *value) {
  
  Serial.print("Variable >"); Serial.print(variable); Serial.print("< "); Serial.print(" Value >"); Serial.print(value); Serial.println("<"); 
  
  if (strcmp(variable,"S1")==0) {
    yellowLed = atoi(value);
  }
    
  if (strcmp(variable,"Push1") == 0  && atoi(value)==1) {    
    // When the button is pressed the LED is momentarily turned off
    // When the button is released nothing is done
    amController.temporaryDigitalWrite(CONNECTIONPIN,LOW,500);
  }

//   if (strcmp(variable,"Push1") == 0 && atoi(value)==1) {
//     // Button Pressed
//     digitalWrite(CONNECTIONPIN,LOW);
//   }
// 
//   if (strcmp(variable,"Push1") == 0 && atoi(value)==0) {
//     // Button Released
//     digitalWrite(CONNECTIONPIN,HIGH);
//   }
  
  if (strcmp(variable,"Knob1") == 0) {

    ledBrightness = atoi(value)/4;
    analogWrite(VARIABLELED,ledBrightness);
  }
  
}

void processOutgoingMessages() {
  
  //Serial.println("Outgoing");
  
  amController.writeMessage("T",temperature);
  amController.writeMessage("Led13",yellowLed);
  amController.writeMessage("Pot",pot);
}

void deviceConnected() {
  
  digitalWrite(CONNECTIONPIN,HIGH);  
  Serial.println(F("deviceConnected"));
}

void deviceDisconnected() {

  digitalWrite(CONNECTIONPIN,LOW);  
  Serial.println(F("deviceDisconnected"));
}

/**
*  
* Auxiliary functions
*
*/

/*
 * getVoltage() – returns the voltage on the analog input defined by pin
 * 
 */
float getVoltage(int pin) {

#if defined (_VARIANT_ARDUINO_ZERO_)

  return (analogRead(pin) * 3.3/1024.);  // converting from a 0 to 1023 digital range
                                        // to 0 to 3.3 volts (each 1 reading equals ~ 3 millivolts)
#else    

  return (analogRead(pin) * 4.92/1024.);  // converting from a 0 to 1023 digital range
                                          // to 0 to 5 volts (each 1 reading equals ~ 5 millivolts)

#endif                                      
}