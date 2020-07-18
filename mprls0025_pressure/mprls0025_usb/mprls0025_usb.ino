#include <USBComposite.h>
#define DEFAULT_SERIAL_STRING "00000000001A"
#define PRODUCT "STM32 DAQ BOARD"
#define MANUF "STMicroelectronics"
#define TXSIZE 64
#define RXSIZE 64
#define VID 0x0483
#define PID 0x5750

#define BOARD_LED PC13

#define MPRLS_DEFAULT_ADDR (0x18)   ///< Most common I2C address
#define MPRLS_STATUS_POWERED (0x40) ///< Status SPI powered bit
#define MPRLS_STATUS_BUSY (0x20)    ///< Status busy bit
#define MPRLS_STATUS_FAILED (0x04)  ///< Status bit for integrity fail
#define MPRLS_STATUS_MATHSAT (0x01) ///< Status bit for math saturation
#define _PSI_max 25
#define _PSI_min 0


const uint8_t reportDescription[] = {
      0x06, 0x00, 0xff,              //   USAGE_PAGE (Generic Desktop)
      0x09, 0x01,                    //   USAGE (Vendor Usage 1)
      // System Parameters
    //Прием в мк
      0xa1, 0x01,                    //   COLLECTION (Application)
      0x85, 0x01,                    //   REPORT_ID (2)
      0x75, 0x08,                    //   REPORT_SIZE (8)
      0x95, 64,                        //   REPORT_COUNT (4)
      0x09, 0x01,                    //   USAGE (Vendor Usage 1)
      0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
      0xC0    /*     END_COLLECTION              */
};

USBHIDDevice STM_HID;
uint8 buf_in[RXSIZE];
uint8 buf_out[TXSIZE];
char buf_[TXSIZE];
HIDReporter STM_HID_REP(buf_out,TXSIZE);


#include <SoftWire.h>
SoftWire SWire(PB8, PB9, SOFT_FAST);

#include <usb_serial.h>
USBSerial myUserial;

boolean MPRLS_begin(){
 SWire.beginTransmission(MPRLS_DEFAULT_ADDR);
 SWire.endTransmission();
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
  }

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
      // Serial.print("Status: "); Serial.println(stat, HEX);
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


void toggle_led()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
};


void setup() {
 STM_HID.begin(reportDescription, sizeof(reportDescription),PID,VID,MANUF ,PRODUCT ,DEFAULT_SERIAL_STRING);

    SWire.begin();
    MPRLS_begin();

     pinMode(BOARD_LED, OUTPUT);
   digitalWrite(BOARD_LED, HIGH);
   delay(1000);digitalWrite(BOARD_LED, LOW);
   delay(1000);digitalWrite(BOARD_LED, HIGH);

}


void loop() {
  float pressure_Pa = MPRLS_readPressure();
   buf_out[0]=1; 
     sprintf(buf_," P1 %d T1 %d P2 %d T2 %d ",(int)pressure_Pa,0,0,0); 
  for(int i(1);i<TXSIZE;i++)buf_out[i]=buf_[i];             
        toggle_led();
   STM_HID_REP.sendReport();
   };
