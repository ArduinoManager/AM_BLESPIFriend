#include <stdlib.h>
#include "AM_BLESPIFriend.h"

#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              6    // Optional but recommended, set to -1 if unused

#define VERBOSE_MODE                   false

#if defined (_VARIANT_ARDUINO_ZERO_)
char *dtostrf (double val, signed char width, unsigned char prec, char *sout);
#endif

#define _HW_SPI_

#if defined(_HW_SPI_)

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

#else

#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO, BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

#endif

void connectionHandler(void);
void disconnectionHandler(void);
void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len);

AMController *myGlobal;

AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(void),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void)
)
{
  _doWork = doWork;
  _doSync = doSync;
  _processIncomingMessages = processIncomingMessages;
  _processOutgoingMessages = processOutgoingMessages;
  _deviceConnected = deviceConnected;
  _deviceDisconnected = deviceDisconnected;

  _remainBuffer[0] = '\0';

  _dataAvailable = false;
  _connected = false;
  _initialized = false;
  _sync = false;
  myGlobal = this;
}

void AMController::loop() {

  if (!_initialized) {

    if ( !ble.begin(VERBOSE_MODE) ) {
      Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
      while (1)
        ;
    }

    ble.echo(false);
    ble.verbose(false);

    ble.println("AT+GAPDEVNAME=AManager\n");
    ble.waitForOK();

    ble.sendCommandCheckOK("AT+GATTCLEAR");
    ble.sendCommandCheckOK( F("AT+GATTADDSERVICE=uuid=0x1020") );
    ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x0001,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=20,DATATYPE=bytearray,DESCRIPTION=out,VALUE=00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00"), &_charid_tx);
    ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x0002,PROPERTIES=0x04,MIN_LEN=1,MAX_LEN=20,DATATYPE=bytearray,DESCRIPTION=in,VALUE=00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00"), &_charid_rx);
    ble.reset();
    ble.echo(false);

    ble.setConnectCallback(connectionHandler);
    ble.setDisconnectCallback(disconnectionHandler);

    ble.setBleGattRxCallback(_charid_rx, BleGattRX);

    _initialized = true;
#ifdef DEBUG
    Serial.println("\tLibrary initialized");
#endif
    return;
  }

  ble.update(0);

  if (_connected && _dataAvailable) {
    _dataAvailable = false;
    processIncomingData();
  }

  if (_sync) {
    _sync = false;
    _doSync();
  }

  // doWork when disconnected
  _doWork();

  if (_connected) {
    _processOutgoingMessages();
  }
}

void AMController::writeMessage(const char *variable, int value) {
  char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%d#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
}

void AMController::writeMessage(const char *variable, float value) {
  char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%.3f#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
}


void AMController::writeTripleMessage(const char *variable, float vX, float vY, float vZ) {
  char buffer[VARIABLELEN + VALUELEN + 3];

  if (!_connected) {
    return;
  }
  snprintf(buffer, VARIABLELEN + VALUELEN + 3, "%s=%.2f:%.2f:%.2f#", variable, vX, vY, vZ);
  writeBuffer((uint8_t *)&buffer, strlen(buffer)*sizeof(char));
}

void AMController::writeTxtMessage(const char *variable, const char *value) {
  char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%s#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
}


void AMController::writeBuffer(uint8_t *buffer, int l) {
  uint8_t zeroFilledBuffer[64];
  uint8_t messageBuffer[64];
  uint8_t idx = 0;

  while (idx < l) {

    uint8_t this_block_size = min(20, l - idx);
    memset(&zeroFilledBuffer, '\0', 22);
    memcpy(&zeroFilledBuffer, buffer + idx, this_block_size);
    sprintf((char *)messageBuffer, "AT+GATTCHAR=%ld,", _charid_tx);

    // Data buffer to HEX string
    char *pBuff = (char *)messageBuffer;
    pBuff += strlen((char *)messageBuffer);
    for (int i = 0; i < 20; i++) {
      if (i > 0) {
        pBuff += sprintf(pBuff, "-");
      }
      pBuff += sprintf(pBuff, "%02X", zeroFilledBuffer[i]);
    }
    *pBuff = '\0';

#ifdef DEBUG
    // Serial.print("Sending >"); Serial.print((char *)messageBuffer); Serial.print("<"); Serial.println();
#endif

    ble.sendCommandCheckOK((char *)messageBuffer);

    idx += this_block_size;
  }
}


void AMController::log(const char *msg) {
  this->writeTxtMessage("$D$", msg);
}

void AMController::log(int msg) {
  char buffer[11];
  snprintf(buffer, 10, "%d", msg);

  this->writeTxtMessage("$D$", buffer);
}


void AMController::logLn(const char *msg) {
  this->writeTxtMessage("$DLN$", msg);
}

void AMController::logLn(int msg) {
  char buffer[11];
  snprintf(buffer, 10, "%d", msg);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(long msg) {
  char buffer[11];
  snprintf(buffer, 10, "%ld", msg);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(unsigned long msg) {
  char buffer[11];
  snprintf(buffer, 10, "%ld", msg);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms) {

#if defined (_VARIANT_ARDUINO_ZERO_)

  PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg = (uint8_t)(PORT_PINCFG_INEN) ;
  PORT->Group[g_APinDescription[pin].ulPort].DIRSET.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin) ;

  boolean previousValue = digitalRead(pin);

  switch ( value )
  {
    case LOW:
      PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[pin].ulPin) ;
      break ;

    default:
      PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg = (1ul << g_APinDescription[pin].ulPin) ;
      break ;
  }

  delay(ms);

  switch ( previousValue )
  {
    case LOW:
      PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[pin].ulPin) ;
      break ;

    default:
      PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg = (1ul << g_APinDescription[pin].ulPin) ;
      break ;
  }

#else

  boolean previousValue = digitalRead(pin);

  digitalWrite(pin, value);
  delay(ms);
  digitalWrite(pin, previousValue);
#endif
}

void AMController::connected() {
  if (_deviceConnected != NULL) {
    _deviceConnected();
  }
  _connected = true;
}

void AMController::disconnected() {
  _remainBuffer[0] = '\0';
  if (_deviceDisconnected != NULL) {
    _deviceDisconnected();
  }
  _connected = false;
}

void AMController::dataAvailable(char *data, size_t len) {
  strncat(_remainBuffer, data, len);
  if (strchr(_remainBuffer,'#') != NULL) {
    _dataAvailable = true;
  }
}

void AMController::processIncomingData(void) {
  char      _variable[VARIABLELEN + 1];
  char      _value[VALUELEN + 1];
  bool      _var = true;
  uint8_t   _idx = 0;
  int        lastPound = -1;

  _variable[0] = '\0';
  _value[0] = '\0';

  uint8_t l = strlen(_remainBuffer);

  Serial.print("Full buffer before >"); Serial.print(_remainBuffer); Serial.println("<");

  for (uint8_t i = 0; i < l; i++) {

    if (_var) {
      if (_remainBuffer[i] != '=') {
        _variable[_idx++] = _remainBuffer[i];
      }
      else {
        _variable[_idx] = '\0';
        _var = false;
        _idx = 0;
      }
    }
    else {
      if (_remainBuffer[i] == '#') {
        lastPound = i;
      }
      if (_remainBuffer[i] != '#') {
        _value[_idx++] = _remainBuffer[i];
      }
      else {
        _value[_idx] = '\0';
        _var = true;
        _idx = 0;

        if (strlen(_value) > 0 && strcmp(_variable, "Sync") == 0) {
          _sync = true;
        }

        if (strlen(_variable) > 0 && strlen(_value) > 0) {

          // Process incoming messages
#ifdef DEBUG
          Serial.print("process "); Serial.print(_variable); Serial.print(" -> "); Serial.println(_value);
#endif
          _processIncomingMessages(_variable, _value);
        }
      }
    }
  }

  if (lastPound == l - 1) {
    _remainBuffer[0] = '\0';
  }
  else if (lastPound > 0) {
    char tmp[128];
    strcpy(tmp, &_remainBuffer[lastPound + 1]);
    strcpy(_remainBuffer, tmp);
  }

#ifdef DEBUG
  Serial.print("Full buffer after  >"); Serial.print(_remainBuffer); Serial.println("<");
#endif
}


#if defined (_VARIANT_ARDUINO_ZERO_)

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {

  uint32_t iPart = (uint32_t)val;
  sprintf(sout, "%ld", iPart);

  if (prec > 0) {
    uint8_t pos = strlen(sout);
    sout[pos++] = '.';
    uint32_t dPart = (uint32_t)((val - (double)iPart) * pow(10, prec));

    for (uint8_t i = (prec - 1); i > 0; i--) {
      size_t pow10 = pow(10, i);
      if (dPart < pow10) {
        sout[pos++] = '0';
      }
      else {
        sout[pos++] = '0' + dPart / pow10;
        dPart = dPart % pow10;
      }
    }

    sout[pos++] = '0' + dPart;
    sout[pos] = '\0';
  }

  return sout;
}

#endif

void connectionHandler(void) {
  myGlobal->connected();
}

void disconnectionHandler(void) {
  myGlobal->disconnected();
}

void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len) {

#ifdef DEBUG
  Serial.print( F("[BLE GATT RX] (" ) );
  Serial.print(chars_id);
  Serial.print(") ");

  for (uint16_t i = 0; i < len; i++) {
    Serial.print("0x"); Serial.print(data[i], HEX); Serial.print(" ");
  }
  Serial.println();
#endif
  myGlobal->dataAvailable((char *)data, len);
}