/**
 * @file sensirion_sf04_v2.cpp
 * @brief Platform-independent Sensirion SF04 flow sensor driver implementation
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#include "sensirion_sf04_v2.h"
#include <cstring>

//===========================================================================
// Constructor
SF04::SF04(SF04HAL::II2C &i2c,
           SF04HAL::ITiming &timing,
           uint8_t address,
           int calField,
           int resolution,
           SF04HAL::IDebug *debug)
    : i2c_(i2c), timing_(timing), debug_(debug),
      i2cAddress_(address)
//===========================================================================
{
    u8t error = false;
    ready = false;

    // Initialize I2C
    if (!i2c_.begin()) {
        if (debug_) {
            debug_->println("ERROR: I2C initialization failed");
        }
        return;
    }

    delayBetweenCommands();
    error |= softReset();
    delayBetweenCommands();
    error |= ReadSerialNumber(&serialNumber);
    delayBetweenCommands();
    error |= ReadScaleFactor(&scaleFactor);
    delayBetweenCommands();
    error |= ReadFlowUnit(flowUnitStr);
    delayBetweenCommands();
    error |= setMeasResolution(resolution); // 9-16 bit
    delayBetweenCommands();
    error |= setCalibrationField(calField);
    delayBetweenCommands();

    if (error == false) {
        ready = true;
        if (debug_) {
            debug_->printf("SF04 initialized: SN=%lu, Scale=%u, Unit=%s\n",
                          serialNumber.u32, scaleFactor.u16, flowUnitStr);
        }
    } else {
        if (debug_) {
            debug_->printf("ERROR: SF04 initialization failed (error=0x%02X)\n", error);
        }
    }
}

//===========================================================================
void SF04::delayBetweenCommands()
//===========================================================================
{
    timing_.delayMicroseconds(SF04_DELAY_BETWEEN_COMMANDS_US);
}

//===========================================================================
// Set measurement resolution (9-16 bit), format is eSF04_RES_12BIT
u8t SF04::setMeasResolution(u16t resolution)
//===========================================================================
{
    ready = false;
    u8t error = false; //variable for error code
    nt16 registerValue; // variable for register value

    switch(resolution) {
        case 9 :
            resolution = eSF04_RES_9BIT;
            break;
        case 10 :
            resolution = eSF04_RES_10BIT;
            break;
        case 11 :
            resolution = eSF04_RES_11BIT;
            break;
        case 12 :
            resolution = eSF04_RES_12BIT;
            break;
        case 13 :
            resolution = eSF04_RES_13BIT;
            break;
        case 14 :
            resolution = eSF04_RES_14BIT;
            break;
        case 15 :
            resolution = eSF04_RES_15BIT;
            break;
        case 16 :
            resolution = eSF04_RES_16BIT;
            break;
        default:
            resolution = eSF04_RES_16BIT;
            break;
    }
    ReadRegister(SF04_ADV_USER_REG,&registerValue); // Read out current register value
    registerValue.u16 = (registerValue.u16 & ~eSF04_RES_MASK) | resolution; // Set resolution in Advanced User Register [11:9]
    WriteRegister(SF04_ADV_USER_REG,&registerValue); // Write out new register value
    ready = true;
    return error;
}

//===========================================================================
// Set calibration field (0-4)
u8t SF04::setCalibrationField(u8t calfield)
//===========================================================================
{
    ready = false;
    u8t error = false; //variable for error code
    nt16 registerValue; // variable for register value
    if (calfield > 8) {
        calfield = 0;
    }

    ReadRegister(SF04_USER_REG,&registerValue); // Read out current register value
    registerValue.u16 = (registerValue.u16 & ~eSF04_CF_MASK) | (calfield << 4); // Set calibration field in User Register [6:4]
    WriteRegister(SF04_USER_REG,&registerValue); // Write out new register value
    ready = true;
    return error;
}

//===========================================================================
// Enable Temperature/Vdd correction
u8t SF04::setTempVddCorrectionBit(u8t value)
//===========================================================================
{
    ready = false;

    u8t error = false; //variable for error code
    nt16 registerValue; // variable for register value

    if (value != 1) {
        value = 0;   // Ensure value is 1 or 0
    }
    ReadRegister(SF04_USER_REG,&registerValue); // Read out current register value
    registerValue.u16 = ((registerValue.u16 & ~0x400) | (value << 10)); // Set calibration field in User Register [bit 10]
    WriteRegister(SF04_USER_REG,&registerValue); // Write out new register value

    ready = true;
    return error;
}

//===========================================================================
// Force sensor reset
bool SF04::softReset()
//===========================================================================
{
    ready = false;
    u8t command = SF04_SOFT_RESET;

    SF04HAL::I2CResult result = i2c_.write(i2cAddress_, &command, 1, true);

    ready = true;

    if (result == SF04HAL::I2CResult::SUCCESS) {
        return true;
    } else {
        if (debug_) {
            debug_->println("ERROR: Soft reset failed");
        }
        return false;
    }
}

//===========================================================================
// Compute CRC and return true if correct and false if not
u8t SF04::checkCRC(u8t data[], u8t numBytes, u8t checksum)
//===========================================================================
{
    u8t crc = 0;
    u8t byteCtr;
    for (byteCtr = 0; byteCtr < numBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for (u8t bit = 8; bit > 0; bit--) {
            if (crc & 0x80) crc = (crc << 1) ^ SF04_POLYNOMIAL;
            else crc = (crc << 1);
        }
    }
    if (crc != checksum) return SF04_CHECKSUM_ERROR;
    else return 0;
}

//===========================================================================
u8t SF04::ReadRegister(etSF04Register eSF04Register, nt16 *pRegisterValue)
//===========================================================================
{
    ready = false;
    u8t error = 0;
    u8t command = eSF04Register;
    u8t dataRead[3];

    // Write register address, then read 3 bytes (2 data + 1 CRC)
    SF04HAL::I2CResult result = i2c_.writeRead(i2cAddress_,
                                               &command, 1,
                                               dataRead, 3);

    if (result != SF04HAL::I2CResult::SUCCESS) {
        error |= SF04_I2C_ERROR;
        if (debug_) {
            debug_->println("ERROR: ReadRegister I2C failed");
        }
    } else {
        // Verify CRC
        u8t data[2] = {dataRead[0], dataRead[1]};
        error |= checkCRC(data, 2, dataRead[2]);

        pRegisterValue->s16.u8H = dataRead[0];
        pRegisterValue->s16.u8L = dataRead[1];
    }

    ready = true;
    return error;
}

//===========================================================================
u8t SF04::WriteRegister(etSF04Register eSF04Register, nt16 *pRegisterValue)
//===========================================================================
{
    ready = false;
    u8t error = 1; //variable for error code

    //-- check if selected register is writable --
    if(!(eSF04Register == SF04_READ_ONLY_REG1 || eSF04Register == SF04_READ_ONLY_REG2)) {
        error=0;

        //-- write register to sensor --
        u8t dataWrite[3] = {
            (u8t)(eSF04Register & ~0x01),
            pRegisterValue->s16.u8H,
            pRegisterValue->s16.u8L
        };

        SF04HAL::I2CResult result = i2c_.write(i2cAddress_, dataWrite, 3, true);

        if (result != SF04HAL::I2CResult::SUCCESS) {
            error |= SF04_I2C_ERROR;
            if (debug_) {
                debug_->println("ERROR: WriteRegister I2C failed");
            }
        }
    }
    ready = true;
    return error;
}

//===========================================================================
u8t SF04::ReadEeprom(u16t eepromStartAdr, u16t size, nt16 eepromData[])
//===========================================================================
{
    ready = false;
    u8t error = 0;
    nt16 eepromStartAdrTmp;
    eepromStartAdrTmp.u16 = eepromStartAdr;

    // Prepare write buffer: command + address (shifted left 4 bits)
    eepromStartAdrTmp.u16 = (eepromStartAdrTmp.u16 << 4);
    u8t writeData[3] = {
        SF04_EEPROM_R,
        eepromStartAdrTmp.s16.u8H,
        eepromStartAdrTmp.s16.u8L
    };

    // Read buffer: size * 3 bytes (2 data + 1 CRC per word)
    u8t *readBuffer = new u8t[size * 3];

    SF04HAL::I2CResult result = i2c_.writeRead(i2cAddress_,
                                               writeData, 3,
                                               readBuffer, size * 3);

    if (result != SF04HAL::I2CResult::SUCCESS) {
        error |= SF04_I2C_ERROR;
        if (debug_) {
            debug_->println("ERROR: ReadEeprom I2C failed");
        }
    } else {
        // Parse and verify each word
        for (u16t i = 0; i < size; i++) {
            u8t data[2] = {readBuffer[i*3], readBuffer[i*3 + 1]};
            u8t checksum = readBuffer[i*3 + 2];

            error |= checkCRC(data, 2, checksum);

            eepromData[i].s16.u8H = data[0];
            eepromData[i].s16.u8L = data[1];
        }
    }

    delete[] readBuffer;
    ready = true;
    return error;
}

//===========================================================================
u8t SF04::Measure(etSF04MeasureType eSF04MeasureType)
//===========================================================================
{
    ready = false;
    u8t error=0;
    u8t command;
    u8t dataRead[3];
    nt16 *pMeasurement;

    //-- write measurement command --
    switch(eSF04MeasureType) {
        case FLOW :
            command = SF04_TRIGGER_FLOW_MEASUREMENT;
            pMeasurement = &flow;
            setTempVddCorrectionBit(0);
            break;
        case TEMP :
            command = SF04_TRIGGER_TEMP_MEASUREMENT;
            pMeasurement = &temperature;
            setTempVddCorrectionBit(1);
            break;
        case VDD :
            command = SF04_TRIGGER_VDD_MEASUREMENT;
            pMeasurement = &vdd;
            setTempVddCorrectionBit(1);
            break;
        default:
            ready = true;
            return SF04_ACK_ERROR;
    }

    SF04HAL::I2CResult result = i2c_.writeRead(i2cAddress_,
                                               &command, 1,
                                               dataRead, 3);

    if (result != SF04HAL::I2CResult::SUCCESS) {
        error |= SF04_I2C_ERROR;
        if (debug_) {
            debug_->println("ERROR: Measure I2C failed");
        }
    } else {
        u8t data[2] = {dataRead[0], dataRead[1]};

        if (checkCRC(data, 2, dataRead[2]) == 0) {
            pMeasurement->s16.u8H = dataRead[0];
            pMeasurement->s16.u8L = dataRead[1];
        } else {
            error |= SF04_CHECKSUM_ERROR;
        }
    }

    ready = true;
    return error;
}

//===========================================================================
u8t SF04::ReadSerialNumber( nt32 *serialNumber )
//===========================================================================
{
    ready = false;
    nt16 registerValue;
    u16t eepromBaseAdr;
    u16t eepromAdr;
    nt16 eepromData[2];
    u8t error=0;

    //-- read "Read-Only Register 2" to find out the active configuration field --
    error |= ReadRegister(SF04_READ_ONLY_REG2,&registerValue);

    //-- calculate eeprom address of product serial number --
    eepromBaseAdr=(registerValue.u16 & 0x0007)*0x0300; //RO_REG2 bit<2:0>*0x0300
    eepromAdr= eepromBaseAdr + SF04_EE_ADR_SN_PRODUCT;

    //-- read product serial number from SF04's eeprom--
    error |= ReadEeprom( eepromAdr, 2, eepromData);
    serialNumber->s32.u16H=eepromData[0].u16;
    serialNumber->s32.u16L=eepromData[1].u16;
    ready = true;
    return error;
}

//===========================================================================
u8t SF04::ReadScaleFactor(nt16 *scaleFactor)
//===========================================================================
{
    ready = false;
    u8t error = 0;
    nt16 registerValue;
    u16t eepromBaseAdr;
    u16t eepromAdr;

    //-- read "User Register " to find out the active calibration field --
    error |= ReadRegister(SF04_USER_REG ,&registerValue);

    //-- calculate eeprom address of scale factor --
    eepromBaseAdr = ((registerValue.u16 & 0x0070) >> 4) * 0x0300; //UserReg bit<6:4>*0x0300
    eepromAdr = eepromBaseAdr + SF04_EE_ADR_SCALE_FACTOR;

    //-- read scale factor from SF04's eeprom--
    error |= ReadEeprom( eepromAdr, 1, scaleFactor);

    ready = true;
    return error;
}

//===========================================================================
u8t SF04::ReadFlowUnit(char *flowUnitStr)
//===========================================================================
{
    ready = false;
    //-- table for unit dimension, unit time, unit volume (x=not defined) --
    const char *unitDimension[]= {"x","x","x","n","u","m","c","d","","-","h","k",
                                  "M","G","x","x"
                                 };
    const char *unitTimeBase[] = {"","us","ms","s","min","h","day","x","x","x","x",
                                  "x","x","x","x","x"
                                 };
    const char *unitVolume[] = {"ln","sl","x","x","x","x","x","x","l","g","x",
                                "x","x","x","x","x","Pa","bar","mH2O","inH2O",
                                "x","x","x","x","x","x","x","x","x","x","x","x"
                               };
    //-- local variables --
    nt16 registerValue;
    u16t eepromBaseAdr;
    u16t eepromAdr;
    nt16 flowUnit;
    u8t tableIndex;
    u8t error=0;

    //-- read "User Register" to find out the active calibration field --
    error |= ReadRegister(SF04_USER_REG ,&registerValue);

    //-- calculate eeprom address of flow unit--
    eepromBaseAdr=((registerValue.u16 & 0x0070)>>4)*0x0300; //UserReg bit<6:4>*0x0300
    eepromAdr= eepromBaseAdr + SF04_EE_ADR_FLOW_UNIT;

    //-- read flow unit from SF04's eeprom--
    error |= ReadEeprom( eepromAdr, 1, &flowUnit);

    //-- get index of corresponding table and copy it to unit string --
    tableIndex=(flowUnit.u16 & 0x000F)>>0; //flowUnit bit <3:0>
    strcpy(flowUnitStr, unitDimension[tableIndex]);
    tableIndex=(flowUnit.u16 & 0x1F00)>>8; //flowUnit bit <8:12>
    strcat(flowUnitStr, unitVolume[tableIndex]);
    tableIndex=(flowUnit.u16 & 0x00F0)>>4; //flowUnit bit <4:7>
    if(unitTimeBase[tableIndex] != "") { //check if time base is defined
        strcat(flowUnitStr, "/");
        strcat(flowUnitStr, unitTimeBase[tableIndex]);
    }

    //-- check if unit string is feasible --
    if(strchr(flowUnitStr,'x') != NULL ) error |= SF04_UNIT_ERROR;
    ready = true;
    return error;
}

//===========================================================================
// Convenience methods to get measurements in physical units
//===========================================================================

float SF04::getFlow() const
{
    if (scaleFactor.u16 == 0) return 0.0f;
    return (float)flow.i16 / scaleFactor.u16;
}

float SF04::getTemperature() const
{
    return (float)temperature.i16 / 10.0f;  // 0.1°C units
}

float SF04::getVdd() const
{
    return (float)vdd.u16 / 1000.0f;  // 1mV units
}
