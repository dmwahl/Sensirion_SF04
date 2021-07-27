/// Start of sample code--------------------------------------------------------
/*
#include "mbed.h"
#include "sensirion_sf04.h"

#define SFM7033_ADDR 0x40

// an I2C sub-class that provides a constructed default
class I2CPreInit : public I2C
{
public:
    I2CPreInit(PinName sda, PinName scl, int freq) : I2C(sda, scl) {
        frequency(freq);
        //start();
    };
};

//I2CPreInit gI2C1(I2C_SDA, I2C_SCL, I2C frequency);
//I2CPreInit gI2C1(PB_9, PB_8, 100000);
I2CPreInit gI2C2(PB_3, PB_10, 100000);
//I2CPreInit gI2C3(PB_4, PA_8);

// (I2C object, address, cal field(0-4), resolution(9-16bit))
SF04 sfm(gI2C2, SFM7033_ADDR, 0, 16);

DigitalOut myled(LED1);
Serial pc(SERIAL_TX, SERIAL_RX);

int main()
{
    pc.baud(250000);
    pc.printf("Starting up...\n\r");

    pc.printf("SN: %u Scale: %d Unit:%s\n\r", sfm.serialNumber.u32, sfm.scaleFactor.u16, sfm.flowUnitStr);
    
    //wait(3);
    while (1) {
        myled = !myled;
        wait(1.0);
        
        sfm.Measure(FLOW);
        sfm.Measure(TEMP);
        sfm.Measure(VDD);
        pc.printf("%.0f %s (raw: %u) Temp: %.1fC Vdd: %.2f\n\r", ((float)sfm.flow.i16 / sfm.scaleFactor.u16), sfm.flowUnitStr, sfm.flow.u16, (float)sfm.temperature.i16/10, (float)sfm.vdd.u16/1000);
    }
}
*/
/// End sample code-------------------------------------------------------------
#ifndef SENSIRION_SF04_H
#define SENSIRION_SF04_H

#include "mbed.h"
#include "sensirion_sf04_typedefs.h"

#define I2C_READ 1
#define I2C_WRITE 0

//---------- Defines -----------------------------------------------------------
// CRC
#define SF04_POLYNOMIAL 0x131 //P(x)=x^8+x^5+x^4+1 = 100110001

// SF04 eeprom map
#define SF04_EE_ADR_SN_CHIP 0x02E4
#define SF04_EE_ADR_SN_PRODUCT 0x02F8
#define SF04_EE_ADR_SCALE_FACTOR 0x02B6
#define SF04_EE_ADR_FLOW_UNIT 0x02B7

// sensor command
typedef enum {
    SF04_USER_REG_W = 0xE2, // command writing user register
    SF04_USER_REG_R = 0xE3, // command reading user register
    SF04_ADV_USER_REG_W = 0xE4, // command writing advanced user register
    SF04_ADV_USER_REG_R = 0xE5, // command reading advanced user register
    SF04_READ_ONLY_REG1_R = 0xE7, // command reading read-only register 1
    SF04_READ_ONLY_REG2_R = 0xE9, // command reading read-only register 2
    SF04_TRIGGER_FLOW_MEASUREMENT = 0xF1, // command trig. a flow measurement
    SF04_TRIGGER_TEMP_MEASUREMENT = 0xF3, // command trig. a temperature measurement
    SF04_TRIGGER_VDD_MEASUREMENT = 0xF5, // command trig. a supply voltage measurement
    SF04_EEPROM_W = 0xFA, // command writing eeprom
    SF04_EEPROM_R = 0xFA, // command reading eeprom
    SF04_SOFT_RESET = 0xFE // command soft reset
} etCommand;

// sensor register
typedef enum {
    SF04_USER_REG = SF04_USER_REG_R,
    SF04_ADV_USER_REG = SF04_ADV_USER_REG_R,
    SF04_READ_ONLY_REG1 = SF04_READ_ONLY_REG1_R,
    SF04_READ_ONLY_REG2 = SF04_READ_ONLY_REG2_R
} etSF04Register;

// measurement signal selection
typedef enum {
    FLOW = SF04_TRIGGER_FLOW_MEASUREMENT,
    TEMP = SF04_TRIGGER_TEMP_MEASUREMENT,
    VDD = SF04_TRIGGER_VDD_MEASUREMENT,
} etSF04MeasureType;

// This enum lists all available flow resolution (Advanced User Register [11:9])
typedef enum {
    eSF04_RES_9BIT = ( 0<<9 ),
    eSF04_RES_10BIT = ( 1<<9 ),
    eSF04_RES_11BIT = ( 2<<9 ),
    eSF04_RES_12BIT = ( 3<<9 ),
    eSF04_RES_13BIT = ( 4<<9 ),
    eSF04_RES_14BIT = ( 5<<9 ),
    eSF04_RES_15BIT = ( 6<<9 ),
    eSF04_RES_16BIT = ( 7<<9 ),
    eSF04_RES_MASK = ( 7<<9 ) // (0x0E00)
} etSF04Resolution;

// Error codes
typedef enum {
    SF04_ACK_ERROR = 0x01,
    SF04_TIME_OUT_ERROR = 0x02,
    SF04_CHECKSUM_ERROR = 0x04,
    SF04_UNIT_ERROR = 0x08
} etError;

// Calibration fields for different gases
typedef enum {
    eSF04_CF0_AIR = 0x0A00,
    eSF04_CF1_O2 = 0x0A10,
    eSF04_CF2_N2O = 0x0A20,
    eSF04_CF3_O2 = 0x0A30, // Only for concentration measurement
    eSF04_CF4_N2O = 0x0A40, // Only for concentration measurement
    eSF04_CFRAW = 0x0880, // Only for concentration measurement
    eSF04_CF_MASK = 0x70
} etSF04Calibration;

// Macro that returns number of elements in an array
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

class SF04
{
protected:
    I2C &i2c;
    u8t mi2cAddress; // 7-bit I2C address shifted left 1 bit

    u8t checkCRC(u8t data[], u8t numBytes, u8t checksum); // return true if checksum is good, else return false

    u8t ReadRegister(etSF04Register eSF04Register, nt16 *pRegisterValue);
    u8t WriteRegister(etSF04Register eSF04Register, nt16 *pRegisterValue);
    u8t ReadEeprom(u16t eepromStartAdr, u16t size, nt16 eepromData[]);
    u8t ReadSerialNumber(nt32 *serialNumber);
    u8t ReadScaleFactor(nt16 *scaleFactor);
    u8t ReadFlowUnit(char *flowUnitStr);
    u8t setTempVddCorrectionBit(u8t value);


public:
    SF04(I2C &i2c, int addr, int calField, int resolution);
    bool softReset(); // Reset sensor. Return true if slave sends ACK, false if not.

    u8t setMeasResolution(u16t resolution); // Set measurement resolution from 9-16 bits
    u8t setCalibrationField(u8t calfield); // Set active calibration field

    nt32 serialNumber;
    nt16 scaleFactor;
    char flowUnitStr[15]; //string for the flow unit

    // Measure FLOW, TEMP, or VDD
    u8t Measure(etSF04MeasureType eSF04MeasureType);
    nt16 flow, temperature, vdd, status, checksum; // raw readings
    
    bool ready; // signal whether or not an operation is in progress
};

#endif