#include "sensirion_sf04.h"

//===========================================================================
// Default constructor
SF04::SF04(I2C &i2c, int i2cAddress, int calField, int resolution) : i2c(i2c), mi2cAddress(i2cAddress << 1)
//===========================================================================
{
    u8t error = false;
    ready = false;
    int16_t wait = 200; // Unsure why, but without a wait between commands the values read out don't make sense.
    wait_us(wait);
    error |= softReset();
    wait_us(wait);
    error |= ReadSerialNumber(&serialNumber);
    wait_us(wait);
    error |= ReadScaleFactor(&scaleFactor);
    wait_us(wait);
    error |= ReadFlowUnit(flowUnitStr);
    wait_us(wait);
    error |= setMeasResolution(resolution); // 9-16 bit
    wait_us(wait);
    error |= setCalibrationField(calField);
    wait_us(wait);

    if (error == false) {
        ready = true;
    }
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
    char data_write = SF04_SOFT_RESET;
    if (i2c.write(mi2cAddress, &data_write, 1, false) == 0) {
        ready = true;
        return true;
    } else {
        ready = true;
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
    u8t checksum; //variable for checksum byte
    u8t error = false; //variable for error code

    char dataWrite[1] = {eSF04Register};
    char dataRead[3];
    u8t data[2]; //data array for checksum verification

    int writeAddr = (mi2cAddress | I2C_WRITE);
    int readAddr = (mi2cAddress | I2C_READ);

    i2c.write(writeAddr, dataWrite, 1, true);
    i2c.read(readAddr, dataRead, 3, false);
    checksum = dataRead[2];
    data[0] = dataRead[0];
    data[1] = dataRead[1];

    error |= checkCRC(data,2,checksum);
    pRegisterValue->s16.u8H = dataRead[0];
    pRegisterValue->s16.u8L = dataRead[1];
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
        int writeAddr = (mi2cAddress | I2C_WRITE);
        char dataWrite[3] = {(eSF04Register & ~0x01 | I2C_WRITE), pRegisterValue->s16.u8H, pRegisterValue->s16.u8L};
        i2c.write(writeAddr, dataWrite, 3);
    }
    ready = true;
    return error;
}

//===========================================================================
u8t SF04::ReadEeprom(u16t eepromStartAdr, u16t size, nt16 eepromData[])
//===========================================================================
{
    ready = false;
    u8t checksum; //checksum
    u8t data[2]; //data array for checksum verification
    u8t error = 0; //error variable
    u16t i; //counting variable
    nt16 eepromStartAdrTmp; //variable for eeprom adr. as nt16
    eepromStartAdrTmp.u16=eepromStartAdr;

    //-- write I2C sensor address and command --
    i2c.start();
    error |= i2c.write(mi2cAddress | I2C_WRITE);
    error |= i2c.write(SF04_EEPROM_R);

    eepromStartAdrTmp.u16=(eepromStartAdrTmp.u16<<4); // align eeprom adr left
    error |= i2c.write(eepromStartAdrTmp.s16.u8H);
    error |= i2c.write(eepromStartAdrTmp.s16.u8L);

    //-- write I2C sensor address for read --
    i2c.start();
    error |= i2c.write(mi2cAddress | I2C_READ);

    //-- read eeprom data and verify checksum --
    for(i=0; i<size; i++) {
        eepromData[i].s16.u8H = data[0] = i2c.read(1);
        eepromData[i].s16.u8L = data[1] = i2c.read(1);
        checksum=i2c.read( (i < size-1) ? 1 : 0 ); //NACK for last byte
        error |= checkCRC(data,2,checksum);
    }
    i2c.stop();
    ready = true;
    return error;
}

//===========================================================================
u8t SF04::Measure(etSF04MeasureType eSF04MeasureType)//, nt16 *pMeasurement)
//===========================================================================
{
    ready = false;
    u8t checksum; //checksum
    u8t error=0; //error variable

    char dataWrite[1];
    char dataRead[3];
    u8t data[2]; //data array for checksum verification

    nt16 *pMeasurement;

    //-- write measurement command --
    switch(eSF04MeasureType) {
        case FLOW :
            dataWrite[0] = SF04_TRIGGER_FLOW_MEASUREMENT;
            pMeasurement = &flow;
            setTempVddCorrectionBit(0);
            break;
        case TEMP :
            dataWrite[0] = SF04_TRIGGER_TEMP_MEASUREMENT;
            pMeasurement = &temperature;
            setTempVddCorrectionBit(1);
            break;
        case VDD :
            dataWrite[0] = SF04_TRIGGER_VDD_MEASUREMENT;
            pMeasurement = &vdd;
            setTempVddCorrectionBit(1);
            break;
        default:
            break;
    }

    int writeAddr = (mi2cAddress | I2C_WRITE);
    int readAddr = (mi2cAddress | I2C_READ);

    i2c.write(writeAddr, dataWrite, 1, true);
    i2c.read(readAddr, dataRead, 3, false);
    checksum = dataRead[2];

    data[0] = dataRead[0];
    data[1] = dataRead[1];

    if (checkCRC(data,2,checksum) == 0) {
        pMeasurement->s16.u8H = dataRead[0];
        pMeasurement->s16.u8L = dataRead[1];
    } else {
        error |= SF04_CHECKSUM_ERROR;
    }
    ready = true;
    return error;
}

//===========================================================================
u8t SF04::ReadSerialNumber( nt32 *serialNumber )
//===========================================================================
{
    ready = false;
    nt16 registerValue; //register value for register
    u16t eepromBaseAdr; //eeprom base address of active calibration field
    u16t eepromAdr; //eeprom address of SF04's scale factor
    nt16 eepromData[2]; //serial number
    u8t error=0; //error variable

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
    u8t error = 0; //variable for error code
    nt16 registerValue; //register value for user register
    u16t eepromBaseAdr; //eeprom base address of active calibration field
    u16t eepromAdr; //eeprom address of SF04's scale factor

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
    nt16 registerValue; //register value for user register
    u16t eepromBaseAdr; //eeprom base address of active calibration field
    u16t eepromAdr; //eeprom address of SF04's flow unit word
    nt16 flowUnit; //content of SF04's flow unit word
    u8t tableIndex; //index of one of the unit arrays
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