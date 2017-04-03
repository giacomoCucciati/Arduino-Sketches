

/*
  Firmata is a generic protocol for communicating with microcontrollers
  from software on a host computer. It is intended to work with
  any host computer software package.

  To download a host software package, please clink on the following link
  to open the list of Firmata client libraries your default browser.

  https://github.com/firmata/arduino#firmata-client-libraries

  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2015 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated by Jeff Hoefs: August 9th, 2015
*/


#include <Servo.h>
#include <Wire.h>
#include <Firmata.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <L3G4200D.h>
#include <Kalman.h>



#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL 10

//Da definire i valori di stallo dei singoli motori.
//Non potranno essere zero.
int stallPow2_ = 285;
int stallPow3_ = 285;
int stallPow6_ = 285;
int stallPow7_ = 285;

int numPedestals_ = 50; // Se cambi questo, devi cambiare anche la lunghezza
                        // dei vettori dei pedestals.
int stallPow2Vec_[50];
int stallPow3Vec_[50];
int stallPow6Vec_[50];
int stallPow7Vec_[50];

int calibration_power = 330;
int offset_6 = 0;
int offset_7 = 0;
int offset_2 = 0;
int offset_3 = 0;
int counter_calibration = 0;
double mean_deviation_x = 0;
double mean_deviation_y = 0;


double averageTime = 0;
int averageCounter = 0;
int averageNum = 10;
double baseAngleX = 0;
double baseAngleY = 0;
/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/
boolean isHigh = false;
boolean printValues_ = false;
boolean timeToStall_ = false;
boolean timeToTakeOff_ = false;
boolean timeToCalibrate_ = false;


/* Accelerometer & Gyroscope */
Adafruit_ADXL345_Unified accel;
L3G4200D gyroscope;
/* Accelerometer and gyro variables */
// Timers
unsigned long timer = 0;

//weight and Fstep
double weight = 1.56;
double Fstep = weight / (4*350);

/* From Kalman example */
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// Create the Kalman instances
Kalman kalmanX;
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXrate, gyroYrate, gyroZrate;
double pitch, roll;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
/* ------------------ */
/* Distance sensor */
const int triggerPort = 24;
const int echoPort = 22;


/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

boolean isResetting = false;

/* utility functions */
void wireWrite(byte data)
{
#if ARDUINO >= 100
  Wire.write((byte)data);
#else
  Wire.send(data);
#endif
}

byte wireRead(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void attachServo(byte pin, int minPulse, int maxPulse)
{
  if (servoCount < MAX_SERVOS) {
    // reuse indexes of detached servos until all have been reallocated
    if (detachedServoCount > 0) {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    } else {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0) {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    } else {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  } else {
    Firmata.sendString("Max servos attached");
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  // if we're detaching the last servo, decrement the count
  // otherwise store the index of the detached servo
  if (servoPinMap[pin] == servoCount && servoCount > 0) {
    servoCount--;
  } else if (servoCount > 0) {
    // keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }

  servoPinMap[pin] = 255;
}

void readAndReportData(byte address, int theRegister, byte numBytes) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    wireWrite((byte)theRegister);
    Wire.endTransmission();
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available()) {
    Firmata.sendString("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Firmata.sendString("I2C: Too few bytes received");
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++) {
    i2cRxData[2 + i] = wireRead();
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == IGNORE)
    return;

  if (pinConfig[pin] == I2C && isI2CEnabled && mode != I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_DIGITAL(pin) && mode != SERVO) {
    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
      detachServo(pin);
    }
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch (mode) {
    case ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        }
        pinConfig[pin] = ANALOG;
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        pinConfig[pin] = INPUT;
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        pinConfig[pin] = OUTPUT;
      }
      break;
    case PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PWM;
      }
      break;
    case SERVO:
      if (IS_PIN_DIGITAL(pin)) {
        pinConfig[pin] = SERVO;
        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
          // pass -1 for min and max pulse values to use default values set
          // by Servo library
          attachServo(pin, -1, -1);
        }
      }
      break;
    case I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        pinConfig[pin] = I2C;
      }
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void setMyTimersRegisters(void) {
  //D2, D3 controleld by Timer 3
  TCCR3A = _BV(WGM31) | _BV(WGM30);
  TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);
  //D6, D7 controleld by Timer 4
  TCCR4A = _BV(WGM41) | _BV(WGM40);
  TCCR4B = _BV(WGM42) | _BV(CS41) | _BV(CS40);
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (pinConfig[pin]) {
      case SERVO:
        if (IS_PIN_DIGITAL(pin))
          servos[servoPinMap[pin]].write(value);
        pinState[pin] = value;
        break;
      case PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), value);
        pinState[pin] = value;
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }

      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
            wireWrite(data);
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
            // too many queries, just ignore
            Firmata.sendString("too many queries");
            break;
          }
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = data;
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }

            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < I2C_MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));

      if (delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }

      if (!isI2CEnabled) {
        enableI2CPins();
      }

      break;
    case SERVO_CONFIG:
      if (argc > 4) {
        // these vars are here for clarity, they'll optimized away by the compiler
        byte pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (IS_PIN_DIGITAL(pin)) {
          if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
            detachServo(pin);
          }
          attachServo(pin, minPulse, maxPulse);
          setPinModeCallback(pin, SERVO);
        }
      }
      break;
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PWM);
          Firmata.write(8); // 8 = 8-bit resolution
        }
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write(SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(I2C);
          Firmata.write(1);  // TODO: could assign a number to map to SCL or SDA
        }
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write((byte)pinConfig[pin]);
          Firmata.write((byte)pinState[pin] & 0x7F);
          if (pinState[pin] & 0xFF80) Firmata.write((byte)(pinState[pin] >> 7) & 0x7F);
          if (pinState[pin] & 0xC000) Firmata.write((byte)(pinState[pin] >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;
    //MY CASES
    /*case 0x11:
      if(isHigh) digitalWrite(13,LOW);
      else digitalWrite(13,HIGH);
      break;*/
    case 0x12:
      if (argc > 1) {
        int val = argv[0];
        val |= (argv[1] << 7);
        setEnginesTo(val);
      }
      break;
    case 0x13:
      printValues_ = !printValues_;
      break;
    case 0x14:
      timeToStall_ = !timeToStall_;
      break;
    case 0x15:
      timeToTakeOff_ = !timeToTakeOff_;
      break;
    case 0x16:
      if (argc > 1) {
        int val = argv[0];
        val |= (argv[1] << 7);
        setPedestals(val);
      }
      break;
    case 0x17:
      if (argc > 2) {
        int motor = argv[0];
        int val = argv[1];
        val |= (argv[2] << 7);
        if(motor == 2) setEngine2To(val);
        if(motor == 3) setEngine3To(val);
        if(motor == 6) setEngine6To(val);
        if(motor == 7) setEngine7To(val);
      }
      break;
    case 0x18:
      timeToCalibrate_ = !timeToCalibrate_;
      break;
    
  }
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, I2C);
    }
  }

  isI2CEnabled = true;

  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void systemResetCallback()
{
  isResetting = true;

  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default

  if (isI2CEnabled) {
    disableI2CPins();
  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }

    servoPinMap[i] = 255;
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  detachedServoCount = 0;
  servoCount = 0;

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
  isResetting = false;
}


void startUpcalibration()//check motors positions
{
    setEngine6To(calibration_power+offset_6);
    setEngine7To(calibration_power+offset_7);
    setEngine2To(calibration_power+offset_2);
    setEngine3To(calibration_power+offset_3);
    if (counter_calibration < 100){
      mean_deviation_x = mean_deviation_x + kalAngleX;
      mean_deviation_y = mean_deviation_y + kalAngleY;
      counter_calibration = counter_calibration +1;
    }
    else{
      mean_deviation_x = mean_deviation_x/100;
      mean_deviation_y = mean_deviation_y/100;
      if (mean_deviation_x > 1 && mean_deviation_y > 1){
        offset_2 = offset_2 - 2;
      } 
      if(mean_deviation_x > 1 && mean_deviation_y < 1){
        offset_3 = offset_3 - 2;
      }
      if(mean_deviation_x < 1 && mean_deviation_y > 1){
        offset_6 = offset_6 - 2;
      } 
      if(mean_deviation_x < 1 && mean_deviation_y < 1){
        offset_7 = offset_7 - 2;
      } 
      calibration_power = calibration_power +1;
      //if (distance > fly_value)
      // timeToCalibrate_ = false;
      // use it with a mean value?
      //
      if(calibration_power > 370 || offset_6 > 10 || offset_7 > 10 || offset_3 > 10 || offset_2 > 10){
        timeToCalibrate_ = false;
      }
      counter_calibration = 0;
      mean_deviation_x = 0;
      mean_deviation_y = 0;
      printEnginesAndAngles();
    }
}

void applyKalman()
{
  // New part, average
  if(averageCounter == 0) {
    accX = 0;
    accY = 0;
    accZ = 0;
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
    averageTime = 0;
  }

  sensors_event_t event;
  accel.getEvent(&event);
  accX += event.acceleration.x;
  accY += event.acceleration.y;
  accZ += event.acceleration.z;

  Vector norm = gyroscope.readNormalize();
  gyroX += norm.XAxis;
  gyroY += norm.YAxis;
  gyroZ += norm.ZAxis;

  double dt = (double)(micros() - timer);
  dt = dt/1000000;
  timer = micros();

  averageTime += dt;
  averageCounter += 1;

  if(averageCounter != averageNum){
    return;
  }

// Arduino will execute the following lines only if
// averageNum of measurements have been taken.
  accX = accX / averageNum;
  accY = accY / averageNum;
  accZ = accZ / averageNum;

  gyroX = gyroX / averageNum;
  gyroY = gyroY / averageNum;
  gyroZ = gyroZ / averageNum;

  averageCounter = 0;

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
   roll  = atan2(accY, accZ) * RAD_TO_DEG;
   pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
   roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
   pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  gyroXrate = gyroX / 131.0; // Convert to deg/s
  gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, averageTime); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, averageTime);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, averageTime); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, averageTime); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * averageTime; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * averageTime;

  //gyroXangle += kalmanX.getRate() * averageTime; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * averageTime;

  compAngleX = 0.93 * (compAngleX + gyroXrate * averageTime) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * averageTime) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  if(timeToStall_) correctEngineToStall();
  if(printValues_) printEnginesAndAngles();
}

void initSensorsAndKalman()
{
  //Initialize gyroscope L3G4200D
  Serial.println("Initialize L3G4200D");
  // Set scale 2000 dps and 400HZ Output data rate (cut-off 50)
  while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyroscope.calibrate(100);

  //Initialize accelerometer
  accel = Adafruit_ADXL345_Unified(12345);
  if(!accel.begin())
  {
    // There was a problem detecting the ADXL345 ... check your connections
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  //displaySensorDetails();
  //displayDataRate();
  //displayRange();

  //From Kalman example
  delay(100);
  sensors_event_t event;
  accel.getEvent(&event);
  accX = event.acceleration.x;
  accY = event.acceleration.y;
  accZ = event.acceleration.z;

#ifdef RESTRICT_PITCH // Eq. 25 and 26
   roll  = atan2(accY, accZ) * RAD_TO_DEG;
   pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
   roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
   pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void setEngine2To(int value){
  // pin 2
  sbi(TCCR3A, COM3B1);
  OCR3B = value;
  pinState[2] = value;

}

void setEngine3To(int value){
  // pin 3
  sbi(TCCR3A, COM3C1);
  OCR3C = value;
  pinState[3] = value;

}

void setEngine6To(int value){
  // pin 6
  sbi(TCCR4A, COM4A1);
  OCR4A = value;
  pinState[6] = value;

}

void setEngine7To(int value){
  // pin 7
  sbi(TCCR4A, COM4B1);
  OCR4B = value;
  pinState[7] = value;

}

void setEnginesTo(int value){

  setEngine2To(value);
  setEngine3To(value);
  setEngine6To(value);
  setEngine7To(value);

}

void printEnginesAndAngles(){
  double kalAlCorX = kalAngleX - baseAngleX;
  double kalAlCorY = kalAngleY - baseAngleY;
  String theMessage = String(pinState[2]);
  theMessage += " ";
  theMessage += String(pinState[3]);
  theMessage += " ";
  theMessage += String(pinState[6]);
  theMessage += " ";
  theMessage += String(pinState[7]);
  theMessage += " ";
  theMessage += String(kalAngleX);
  theMessage += " ";
  theMessage += String(kalAngleY);
  theMessage += " ";
  theMessage += String(baseAngleX);
  theMessage += " ";
  theMessage += String(baseAngleY);
  theMessage += " ";
  theMessage += String(kalAlCorX);
  theMessage += " ";
  theMessage += String(kalAlCorY);
  theMessage += " ";
  theMessage += String(averageTime);
  delay(10);
  Firmata.sendString(theMessage.c_str());
}

void correctEngineToStall(){
  double kalAlCorX = kalAngleX - baseAngleX;
  double kalAlCorY = kalAngleY - baseAngleY;

  int value2 = stallPow2_ - (int)kalAlCorX + (int)kalAlCorY;
  int value3 = stallPow3_ + (int)kalAlCorX + (int)kalAlCorY;
  int value6 = stallPow6_ + (int)kalAlCorX - (int)kalAlCorY;
  int value7 = stallPow7_ - (int)kalAlCorX - (int)kalAlCorY;

  value2 +=  -(int)gyroXrate + (int)gyroYrate;
  value3 +=  (int)gyroXrate + (int)gyroYrate;
  value6 +=  (int)gyroXrate - (int)gyroYrate;
  value7 +=  -(int)gyroXrate - (int)gyroYrate;

  //double angleFactor = cos(kalAlCorX * PI / 180.0) * cos(kalAlCorY * PI / 180.0);
  //double deltaForce = weight/angleFactor - ((value2+value3+value6+value7)*Fstep) ;
  //deltaW = int(deltaForce / (4 * Fstep));

  //value2 += deltaW;
  //value3 += deltaW;
  //value6 += deltaW;
  //value7 += deltaW;

  /*int value2 = stallPow2_ - (int)compAngleX + (int)compAngleY;
  int value3 = stallPow3_ + (int)compAngleX + (int)compAngleY;
  int value6 = stallPow6_ + (int)compAngleX - (int)compAngleY;
  int value7 = stallPow7_ - (int)compAngleX - (int)compAngleY;*/

  setEngine2To(value2);
  setEngine3To(value3);
  setEngine6To(value6);
  setEngine7To(value7);

}

void increasePedestals(){

  stallPow2_ += 1;
  stallPow3_ += 1;
  stallPow6_ += 1;
  stallPow7_ += 1;

}

void decreasePedestals(){

  stallPow2_ -= 1;
  stallPow3_ -= 1;
  stallPow6_ -= 1;
  stallPow7_ -= 1;

}

void savePedestals(int counter){

  stallPow2Vec_[counter] = stallPow2_;
  stallPow3Vec_[counter] = stallPow3_;
  stallPow6Vec_[counter] = stallPow6_;
  stallPow7Vec_[counter] = stallPow7_;
}

void setPedestals(int value){

  stallPow2_ = value;
  stallPow3_ = value;
  stallPow6_ = value;
  stallPow7_ = value;

}

void setPedestals(){

  // Prima azzero i valori
  stallPow2_ = 0;
  stallPow3_ = 0;
  stallPow6_ = 0;
  stallPow7_ = 0;

  // Poi calcolo la media e l'assegno ai vari pedestals
  for(int i = 0; i < numPedestals_; i++){
    stallPow2_ += stallPow2Vec_[i];
    stallPow3_ += stallPow3Vec_[i];
    stallPow6_ += stallPow6Vec_[i];
    stallPow7_ += stallPow7Vec_[i];
  }

  stallPow2_ = stallPow2_/numPedestals_;
  stallPow3_ = stallPow3_/numPedestals_;
  stallPow6_ = stallPow6_/numPedestals_;
  stallPow7_ = stallPow7_/numPedestals_;
}



void takeOff(){
  unsigned long distanza_prima = 999;
  unsigned long distanza_dopo = 0;
  timeToStall_ = true;
  int counter = 0;
  int counterDelay = 0;
  // Inizia il loop di calibrazione pedestals
  while(timeToTakeOff_){

    String theMessage = "Angolo totale maggiore di 5";
    // Controllo sempre possibili messaggi esterni
    while (Firmata.available())
      Firmata.processInput();

    // Durante il decollo il drone deve comunque bilanciarsi
    applyKalman();
    correctEngineToStall();
    counterDelay++;
    if(counterDelay > 100) {
      // Calcolo angolo totale rispetto alla verticale
      double angoloTot = acos(cos(kalAngleX * 71 / 4068) * cos(kalAngleY * 71 / 4068));

      // Se l'angolo complessivo e' minore di 5 gradi
      // procedi con il calcolo dei pedestal
      if(angoloTot < 0.087) {
        // Effettuo la misura di distanza
        // Porto bassa l'uscita del trigger
        digitalWrite( triggerPort, LOW );
        // Invio un impulso di 10us su trigger
        digitalWrite( triggerPort, HIGH );
        delayMicroseconds( 10 );
        digitalWrite( triggerPort, LOW );
        unsigned long durata = pulseIn( echoPort, HIGH );
        distanza_dopo = 0.034 * durata / 2;
        // Distanza corretta per la rotazione su x e y
        // distanza_dopo = distanza_dopo * cos(kalAngleX) * cos(kalAngleY);


         //dopo 38ms è fuori dalla portata del sensore
        if( durata > 38000 ){
          theMessage = "Fuori portata";
          // Il drone e' volato via? Effettuare un tentativo di atterraggio
          // lanciando una funzione apposita:
          // landing();
        } else{

          if (distanza_dopo < 20){ // Il drone non ha ancora raggiunto la quota di calibrazione
            if (distanza_dopo > distanza_prima) {
              //Nothing to do
            } else if (distanza_dopo < distanza_prima) {
              increasePedestals();
            } else {
              increasePedestals();
            }
          } else if (distanza_dopo > 20) { // Il drone ha superato la quota di calibrazione
            savePedestals(counter);
            counter ++;
            if (distanza_dopo > distanza_prima) {
              decreasePedestals();
            } else if (distanza_dopo < distanza_prima) {
              // Nothing to do
            } else {
              // Nothing to do
            }
          }
          // Aggiorno la distanza
          distanza_prima = distanza_dopo;
          //theMessage = distanza_dopo); //conversione a stringa non funzionante
          theMessage += " cm";
        }
        // Ho raccolto abbastanza pedestals per fare una buona media?
        if (counter == numPedestals_) {
          // Faccio una media dei pedestals e spengo il ciclo di decollo
          // Si torna nel loop principale con il timeToStall_ attivato
          setPedestals();
          timeToTakeOff_ = false;
        }
      } //End of if (angolo minore di 5 gradi)
      counterDelay = 0;
    } //End of if (counter delay)
    if(printValues_) Firmata.sendString(theMessage.c_str());
    delay(1);
  }//End of the while

}

void setStartingAngle()
{

  int tempCounter = 0;
  while (1){
    applyKalman();
    if (averageCounter == 0){
      baseAngleX += kalAngleX;
      baseAngleY += kalAngleY;
      tempCounter += 1;
    }
    if(tempCounter == 50){
      baseAngleX = baseAngleX/51.0;
      baseAngleY = baseAngleY/51.0;
      return;
    }
  }

}



void setup()
{
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  // to use a port other than Serial, such as Serial1 on an Arduino Leonardo or Mega,
  // Call begin(baud) on the alternate serial port and pass it to Firmata to begin like this:
  // Serial1.begin(57600);
  // Firmata.begin(Serial1);
  // then comment out or remove lines 701 - 704 below

  Firmata.begin(57600);

  //Set arduino timers to increase control on PWM

  setMyTimersRegisters();
  while (!Serial) {
    ; // wait for serial port to connect. Only needed for ATmega32u4-based boards (Leonardo, etc).
  }
  systemResetCallback();  // reset to default config

  initSensorsAndKalman();
  setStartingAngle();
  digitalWrite(12, LOW);

}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
  byte pin, analogPin;

  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  checkDigitalInputs();

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available())
    Firmata.processInput();

  applyKalman();
  //if(timeToTakeOff_) takeOff();
  if(timeToCalibrate_) startUpcalibration();


  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
      }
    }
  }
}
