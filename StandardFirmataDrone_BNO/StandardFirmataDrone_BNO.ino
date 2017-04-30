

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
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


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
#define BNO055_SAMPLERATE_DELAY_MS (25)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//  Drone geometry initalization:
//  assuming an equal force over all the motors, this has to be compensated by 
//  the arm applied by the single motor wrt to the different axes.
double armX_front = 10; 
double armY_left  = 5;
double armX_back  = 7;
double armY_right = 5;

//double correction_arm_2 = ;
//double correction_arm_3 = ;
//double correction_arm_6 = ;
//double correction_arm_7 = ;

float eulerX = 0;
float eulerY = 0;
float eulerZ = 0;
imu::Vector<3> euler;
unsigned long mytimer = 0;
unsigned long dt = 0;
int calibration_power = 0;
int offset_6 = 0;
int offset_7 = 0;
int offset_2 = 0;
int offset_3 = 0;
int counter_calibration = 0;
double mean_deviation_x = 0;
double mean_deviation_y = 0;
int number_mean = 20;
int another_counter = 0;
int print_counter = 0;
unsigned long distanza_prima = 999;
unsigned long distanza_dopo = 0;
int counter = 0;
int takeOffPower = 350;
int numPedestals_ = 10; 
int counterDelay = 0;
int value2 = 0;
int value3 = 0;
int value6 = 0;
int value7 = 0;
double angleCorrectedX = 0;
double angleCorrectedY = 0;

int pedestals[4];

int averageCounter = 0;
int averageNum = 10;
float baseAngleX = 0;
float baseAngleY = 0;
float baseAngleZ = 0;
/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/
boolean printValues_ = false;
boolean timeToStall_ = false;
boolean timeToTakeOff_ = false;
boolean timeToCalibrate_ = false;



/* Accelerometer and gyro variables */
// Timers
unsigned long timer = 0;

//weight and Fstep
double weight = 1.56;
double Fstep = weight / (4*350);


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
    // Set all engines to a custom value.  
    case 0x12:
      if (argc > 1) {
        int val = argv[0];
        val |= (argv[1] << 7);
        setEnginesTo(val);
      }
      break;
    // Enable/disable printing function
    case 0x13:
      printValues_ = !printValues_;
      break;
    // Enable engines auto-correction
    case 0x14:
      timeToStall_ = !timeToStall_;
      break;
    // Enable take off procedure
    case 0x15:
      timeToTakeOff_ = !timeToTakeOff_;
      //timeToStall_ = !timeToStall_;  //ToDo: uncomment this if you want automatic takeoff, otherwise first sysex x14 has to be called.
      break;
    // Set a specific engine to a custom value.
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
    // Enable take off calibration
    case 0x18:
      timeToCalibrate_ = !timeToCalibrate_;
      calibration_power = 330;
      another_counter = 0;
      break;
    // Start landing function
    case 0x19: //ramp down loop call
      ramp_down();
      break;
    // Set all pedestals to a custom value.
    case 0x1a:
      if (argc > 1) {
        int val = argv[0];
        val |= (argv[1] << 7);
        setPedestalsTo(val);
      }
      break;
    // Set single pedestal to a custom value.
    case 0x1b:
      if (argc > 2) {
        int motor = argv[0];
        int val = argv[1];
        val |= (argv[2] << 7);
        setPedestalTo(motor,val);
      }
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

void ramp_down()
{
  timeToCalibrate_ = false;
  timeToStall_     = false;
  timeToTakeOff_   = false;
  while(1){
    int minValue = 260;
    if(pinState[2]>minValue) setEngine2To(pinState[2]-2);
    if(pinState[3]>minValue) setEngine3To(pinState[3]-2);
    if(pinState[6]>minValue) setEngine6To(pinState[6]-2);
    if(pinState[7]>minValue) setEngine7To(pinState[7]-2);
    delay(200);
    if(pinState[2] <= minValue && pinState[3] <= minValue && pinState[6] <= minValue && pinState[7] <= minValue) break;
  }
}

void getDistance(){
  // Calcolo angolo totale rispetto alla verticale
  double angoloTot = acos(cos(eulerX * 71 / 4068) * cos(eulerY * 71 / 4068));

  // Se l'angolo complessivo e' minore di 5 gradi
  // procedi con il calcolo dei pedestal, altrimenti continua con il tentativo di stallo (loop principale)
  if(angoloTot < 0.087) {  // ToDo: test to verify the independence of the distance detected from a plane over different angles the angle (whithin 15 degrees)
    // Effettuo la misura di distanza
    // Porto bassa l'uscita del trigger
    digitalWrite( triggerPort, LOW );
    // Invio un impulso di 10us su trigger
    digitalWrite( triggerPort, HIGH );
    delayMicroseconds( 10 );
    digitalWrite( triggerPort, LOW );
    unsigned long durata = pulseIn( echoPort, HIGH ); // waint until the pulse is detected
    distanza_dopo = 0.034 * durata / 2;
    // Distanza corretta per la rotazione su x e y
    // distanza_dopo = distanza_dopo * cos(kalAngleX) * cos(kalAngleY);


    //dopo 38ms è fuori dalla portata del sensore
    if( durata > 38000 ){
          ramp_down();
    }
  }
}

void takeOff(){ //calibrazione pedestals
  counterDelay++;
  if(counterDelay > 50){
    getDistance();  
    if (distanza_dopo < 20){ // Il drone non ha ancora raggiunto la quota di calibrazione
      if (distanza_dopo > distanza_prima) {
      //Nothing to do
      } else if(distanza_dopo <= distanza_prima) {
        takeOffPower = takeOffPower + 1;
        } 
    } else if (distanza_dopo >= 20) { // Il drone ha superato la quota di calibrazione
      counter ++;
      if (distanza_dopo > distanza_prima) {
        takeOffPower = takeOffPower - 1;
        } 
      }
      // Aggiorno la distanza
      distanza_prima = distanza_dopo;
      counterDelay = 0;
  }
  // Ho raccolto abbastanza pedestals per fare una buona media?
  if (counter == numPedestals_) {
    // Faccio una media dei pedestals e spengo il ciclo di decollo
    // Si torna nel loop principale con il timeToStall_ attivato
    timeToTakeOff_ = false;
    ramp_down();
  }
} 

void getAngles(){  
  dt = (double)(millis() - mytimer);
  mytimer = millis();
  euler   = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  eulerX = euler.z() - baseAngleX;
  eulerY = euler.y() - baseAngleY;
  eulerZ = euler.x() - baseAngleZ;
  //eulerX = euler.x();
  //eulerY = euler.y();
  //eulerZ = euler.z();

  if(printValues_){ 
  printEnginesAndAngles();
  delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

void initSensors()
{
   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
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

void setPedestalsTo(int value){
  
  pedestals[0] = value;
  pedestals[1] = value;
  pedestals[2] = value;
  pedestals[3] = value;
}

void setPedestalTo(int motor, int value){
  
  if(motor == 2) pedestals[0] = value;
  if(motor == 3) pedestals[1] = value;
  if(motor == 6) pedestals[2] = value;
  if(motor == 6) pedestals[3] = value;
}

void printEnginesAndAngles(){
  print_counter += 1;
  if(print_counter == 10) {
    String theMessage = "";
    theMessage += String(pinState[2]);
    theMessage += " ";
    theMessage += String(pinState[3]);
    theMessage += " ";
    theMessage += String(pinState[6]);
    theMessage += " ";
    theMessage += String(pinState[7]);
    theMessage += " ";
    theMessage += String(eulerX);
    theMessage += " ";
    theMessage += String(eulerY);
    theMessage += " ";
    theMessage += String(eulerZ);
    theMessage += " ";
    theMessage += String(dt);
    theMessage += " ";
    theMessage += String(distanza_dopo);
    /*theMessage += " ";
    theMessage += String(baseAngleY);
    theMessage += " ";
    theMessage += String(baseAngleZ);*/
  
    Firmata.sendString(theMessage.c_str());
    print_counter = 0;
  }
}


// Calculate the starting X and Y angle of the 
// gyroscope when the drone still off.
void setStartingAngle()
{
  delay(1000);
  float tempBaseX = 0;
  float tempBaseY = 0;
  float tempBaseZ = 0;
  for (int tempCounter = 0; tempCounter < 100; tempCounter++) {
    // At this point baseAngleX and baseAngleY are still 0.
    getAngles();
    tempBaseX += eulerX;
    tempBaseY += eulerY;
    tempBaseZ += eulerZ;
  }
  baseAngleX = tempBaseX/100.0;
  baseAngleY = tempBaseY/100.0;
  baseAngleZ = tempBaseZ/100.0;
}

void correctEnginesToStall(){ // ToDo: setEngine2To(takeOffPower+offset_2);
  
  angleCorrectedX = eulerX;
  angleCorrectedY = eulerY;
  
  // the initial value (eg 367) should be implemented as the current power + offset_motor (calculated trough calibration)
  value2 = takeOffPower+offset_2+pedestals[0] - (int)angleCorrectedX + (int)angleCorrectedY;
  value3 = takeOffPower+offset_3+pedestals[1] + (int)angleCorrectedX + (int)angleCorrectedY;
  value6 = takeOffPower+offset_6+pedestals[2] - (int)angleCorrectedX - (int)angleCorrectedY;
  value7 = takeOffPower+offset_7+pedestals[3] + (int)angleCorrectedX - (int)angleCorrectedY;  

  /*double angleFactor = cos(kalAlCorX * PI / 180.0) * cos(kalAlCorY * PI / 180.0);
  double deltaForce = weight/angleFactor - ((value2+value3+value6+value7)*Fstep) ;
  deltaW = int(deltaForce / (4 * Fstep));

  value2 += deltaW;
  value3 += deltaW;
  value6 += deltaW;
  value7 += deltaW;

  int value2 = stallPow2_ - (int)compAngleX + (int)compAngleY;
  int value3 = stallPow3_ + (int)compAngleX + (int)compAngleY;
  int value6 = stallPow6_ + (int)compAngleX - (int)compAngleY;
  int value7 = stallPow7_ - (int)compAngleX - (int)compAngleY;*/

  setEngine2To(value2);
  setEngine3To(value3);
  setEngine6To(value6);
  setEngine7To(value7);

}

void startUpcalibration()//check motors positions
{

  if (counter_calibration < number_mean){
    mean_deviation_x = mean_deviation_x + eulerX; //mean
    mean_deviation_y = mean_deviation_y + eulerY;
    counter_calibration = counter_calibration +1; //increase counter needed for mean
  }
  else{
    double meanX = mean_deviation_x/number_mean;
    double meanY = mean_deviation_y/number_mean;
    if (meanX > 1 && meanY < 1 && meanY > -1){ //check which motor is pushing too much
      offset_2 = offset_2 - 1;
      offset_6 = offset_6 - 1;
      offset_3 = offset_3 + 1;
      offset_7 = offset_7 + 1;
    } 
    if(meanX < 1 && meanY < 1 && meanY > -1){
      offset_3 = offset_3 - 1;
      offset_7 = offset_7 - 1;
      offset_2 = offset_2 + 1;
      offset_6 = offset_6 + 1;
    }
    if(meanX < 1 && meanX > -1 && meanY > 1){
      offset_6 = offset_6 - 1;
      offset_7 = offset_7 - 1;
      offset_2 = offset_2 + 1;
      offset_3 = offset_3 + 1;
    } 
    if(meanX < 1 && meanX > -1 && meanY < 1){
      offset_2 = offset_2 - 1;
      offset_3 = offset_3 - 1;
      offset_7 = offset_7 + 1;
      offset_6 = offset_6 + 1;
    }
      
    if (meanX > 1 && meanY > 1){ //check which motor is pushing too much
      offset_6 = offset_6 - 1;
      offset_3 = offset_3 + 1;
    } 
    if(meanX > 1 && meanY < 1){
      offset_2 = offset_2 - 1;
      offset_7 = offset_7 + 1;
    }
    if(meanX < 1 && meanY > 1){
      offset_7 = offset_7 - 1;
      offset_2 = offset_2 + 1;
    } 
    if(meanX < 1 && meanY < 1){
      offset_3 = offset_3 - 1;
      offset_6 = offset_6 + 1;
    } 
    setEngine6To(calibration_power+offset_6);//set motors values
    setEngine7To(calibration_power+offset_7);
    setEngine2To(calibration_power+offset_2);
    setEngine3To(calibration_power+offset_3);

    //if(! (meanX >= 2 || meanX <= -2 || meanY>=2 || meanY <= -2)){
    //      calibration_power = calibration_power +1;
    //}
    //increase power for the next loop
        
    delay(500);
      
    getDistance();
    if (distanza_dopo > 5){ // implememting the distance sensor (ToDo: is it in cm?)
      timeToCalibrate_ = false;
      ramp_down();
      //use it with a mean value?
      }
    if(calibration_power > 380 || offset_6 > 40 || offset_7 > 40 || offset_3 > 40 || offset_2 > 40) {  //close calibration loop,
      calibration_power = 330;
      ramp_down();
      another_counter = another_counter+1; // another_counter will control the times the calibration will call,
      if(another_counter == 1){             // but keeping the offsets memory!
        timeToCalibrate_ = false; //close the calibration
      }
    }
    counter_calibration = 0;
    mean_deviation_x = 0;
    mean_deviation_y = 0;
  }
}

void setup()
{
  // Led on during Setup function
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

  initSensors();
  setStartingAngle();
  pedestals[0] = 260;
  pedestals[1] = 260;
  pedestals[2] = 260;
  pedestals[3] = 260;
  // Led off at the end of Setup function
  digitalWrite(12, LOW);
  timer = millis();
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
  while (Firmata.available()) {
    Firmata.processInput();
  };
  
  getAngles();
  if(timeToCalibrate_) startUpcalibration();
  if(timeToTakeOff_) takeOff();
  if(timeToStall_) correctEnginesToStall();

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
