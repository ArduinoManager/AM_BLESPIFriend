
/*
   AMController libraries, example sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you
   by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation
   constitutes acceptance of these terms.
   If you do not agree with these terms, please do not use or install The Software.
   The Author grants you a personal, non-exclusive license, under author's copyrights in this original software, to use The Software.
   Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any
   patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
   The Software and the Documentation are provided by the Author on an "AS IS" basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION
   ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
   REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
   STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   Author: Fabrizio Boco - fabboco@gmail.com

   Version: 1.0.0

   All rights reserved

*/

#ifndef AM_BLESPIFriend_H
#define AM_BLESPIFriend_H

//#define DEBUG           // uncomment to enable debugging - You should not need it !


#include <Arduino.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"

#define VARIABLELEN 14
#define VALUELEN   14

class AMController {

  private:

    int32_t         _charid_tx;
    int32_t         _charid_rx;

    volatile bool _dataAvailable;
    char          _remainBuffer[128];
    bool          _initialized;
    bool          _connected;
    bool          _sync;

    /**
      Pointer to the function where to put code in place of loop()
    **/
    void (*_doWork)(void);

    /*
      Pointer to the function where Switches, Knobs and Leds are syncronized
    */
    void (*_doSync)(void);

    /*
      Pointer to the function where incoming messages are processed

      variable

      value

    */
    void (*_processIncomingMessages)(char *variable, char *value);

    /*
      Pointer to the function where outgoing messages are processed

    */
    void (*_processOutgoingMessages)(void);

#ifdef ALARMS_SUPPORT
    /*
      Pointer to the function where alerts are processed

    */
    void (*_processAlarms)(char *alarm);
#endif

    /*
      Pointer to the function called when a device connects to Arduino

    */
    void (*_deviceConnected)(void);

    /*
      Pointer to the function called when a device disconnects from Arduino

    */
    void (*_deviceDisconnected)(void);

    void readVariable(void);

  public:

    AMController(
      void (*doWork)(void),
      void (*doSync)(void),
      void (*processIncomingMessages)(char *variable, char *value),
      void (*processOutgoingMessages)(void),
      void (*deviceConnected)(void),
      void (*deviceDisconnected)(void));

    void loop();
    void loop(unsigned long delay);

    void writeMessage(const char *variable, int value);
    void writeMessage(const char *variable, float value);
    void writeTripleMessage(const char *variable, float vX, float vY, float vZ);
    void writeTxtMessage(const char *variable, const char *value);
    
    void updateBatteryLevel(uint8_t level); // A value between 0% and 100%

    void log(const char *msg);
    void log(int msg);

    void logLn(const char *msg);
    void logLn(int msg);
    void logLn(long msg);
    void logLn(unsigned long msg);

    void temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms);

    void processIncomingData(void);

    void connected();
    void disconnected();
    void dataAvailable(char *data, size_t len);
    void writeBuffer(uint8_t *buffer, int l);
};

#endif
