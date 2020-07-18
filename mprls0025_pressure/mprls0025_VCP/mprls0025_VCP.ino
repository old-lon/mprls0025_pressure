#define MPRLS_DEFAULT_ADDR (0x18)   ///< Most common I2C address
#define MPRLS_STATUS_POWERED (0x40) ///< Status SPI powered bit
#define MPRLS_STATUS_BUSY (0x20)    ///< Status busy bit
#define MPRLS_STATUS_FAILED (0x04)  ///< Status bit for integrity fail
#define MPRLS_STATUS_MATHSAT (0x01) ///< Status bit for math saturation
#define _PSI_max 25
#define _PSI_min 0

#include <SoftWire.h>
SoftWire SWire(PB8, PB9, SOFT_FAST);

#include <usb_serial.h>
USBSerial myUserial;

boolean MPRLS_begin(){
 SWire.beginTransmission(MPRLS_DEFAULT_ADDR);
 delay(10); // startup timing

  // Serial.print("Status: ");
  uint8_t stat = MPRLS_readStatus();
  // Serial.println(stat);
  return !(stat & 0b10011110);
};

/**************************************************************************/
/*!
    @brief Read and calculate the pressure
    @returns The measured pressure, in hPa on success, NAN on failure
*/
/**************************************************************************/
float MPRLS_readPressure(void) {
  uint32_t raw_psi = MPRLS_readData();
  if (raw_psi == 0xFFFFFFFF) {
    return -1;
  };

  // All is good, calculate the PSI and convert to hPA
  // use the 10-90 calibration curve
  float psi = (raw_psi - 0x19999A) * (_PSI_max - _PSI_min);
  psi /= (float)(0xE66666 - 0x19999A);
  psi += _PSI_min;
  // convert PSI to PA
  return psi * 68.947572932*100;
}

/**************************************************************************/
/*!
    @brief Read 24 bits of measurement data from the device
    @returns -1 on failure (check status) or 24 bits of raw ADC reading
*/
/**************************************************************************/
uint32_t MPRLS_readData(void) {
   SWire.beginTransmission(MPRLS_DEFAULT_ADDR);
  SWire.write(0xAA); // command to read pressure
  SWire.write((byte)0x0);
  SWire.write((byte)0x0);
  SWire.endTransmission();


    // check the status byte
    uint8_t stat;
    while ((stat = MPRLS_readStatus()) & MPRLS_STATUS_BUSY) {
      stat = MPRLS_readStatus();
      myUserial.print("Status: "); myUserial.println(stat, HEX);
      delay(10);
    };
   
  SWire.requestFrom(MPRLS_DEFAULT_ADDR,4);
  uint8_t status = SWire.read();
    
  if (status & MPRLS_STATUS_MATHSAT) {
    return 0xFFFFFFFF;
  }
  if (status & MPRLS_STATUS_FAILED) {
    return 0xFFFFFFFF;
  }

  uint32_t ret;
  ret = SWire.read();
  ret <<= 8;
  ret |= SWire.read();
  ret <<= 8;
  ret |= SWire.read();
  SWire.endTransmission();
  return ret;
}

/**************************************************************************/
/*!
    @brief Read just the status byte, see datasheet for bit definitions
    @returns 8 bits of status data
*/
/**************************************************************************/
uint8_t MPRLS_readStatus(void) {
   SWire.requestFrom(MPRLS_DEFAULT_ADDR,1);
   return SWire.read();
}


void setup() {
    myUserial.begin(115200);
    SWire.begin();
    MPRLS_begin();
}


void loop() {

  float pressure_Pa = MPRLS_readPressure();
  myUserial.print("Pressure (Pa): "); myUserial.println(pressure_Pa);
  delay(100);
}
