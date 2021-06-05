/*
 * Arduino Manager for iPad / iPhone / Mac
 *
 * Skeleton
 *
 * Author: Fabrizio Boco - fabboco@gmail.com
 *
 * Version: 1.0
 *
 * 06/05/2021
 *
 * All rights reserved
 *  
 */

/*
*
 * IOSController libraries, test sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you 
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

/*
 *
 * Defines go here
 *
 *
 */


/*
 *
 * Global variables go here
 *
 *
 */

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
 * IOSControllerForForBluefruitSPIFriend Library initialization
 *
*/
AMController amController(&doWork,&doSync,&processIncomingMessages,&processOutgoingMessages,&deviceConnected,&deviceDisconnected); 
 
void setup()
{
  Serial.begin(115200);
  
  // Your setup goes here
      
  Serial.println(F("Setup completed"));
}

void loop() {
  amController.loop();
}


void doWork() {
   // Main 
  
}

void doSync() {  
  //Serial.print("Sync "); 

  
}

void processIncomingMessages(char *variable, char *value) {
  //Serial.print("Variable "); Serial.print(variable); Serial.print(" "); Serial.print(" Value "); Serial.println(value); 
  
}

void processOutgoingMessages() {
  //Serial.println("Outgoing");
  
}

void deviceConnected() {
  
  Serial.println(F("Device Connected"));
}

void deviceDisconnected() {

  Serial.println(F("Device Disconnected"));
}

/**
*  
* Auxiliary functions
*
*/
