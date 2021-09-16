#include "mbed.h"
#include "sensirion_sf04.h"

//#define SFM7033_ADDR 0x40
#define SFM4100_ADDR 0x01

// an I2C sub-class that provides a constructed default
class I2CPreInit : public I2C {
public:
  I2CPreInit(PinName sda, PinName scl, int freq) : I2C(sda, scl) {
    frequency(freq);
    // start();
  };
};

// I2CPreInit gI2C1(I2C_SDA, I2C_SCL, I2C frequency);
// I2CPreInit gI2C1(PB_9, PB_8, 100000);
// I2CPreInit gI2C2(PB_3, PB_10, 100000);
I2CPreInit i2c3(PB_4, PA_8, 100000);

// (I2C object, address, cal field(0-4), resolution(9-16bit))
SF04 sfm(i2c3, SFM4100_ADDR, 0, 16);
DigitalOut myled(LED1);

static BufferedSerial pc(USBTX, USBRX, 115200);
char serialRxBuf[128];
char serialTxBuf[128];

int main() {
  sprintf(serialTxBuf, "Starting up SF04 test. SN: %lu\r\n", sfm.serialNumber.u32);
  pc.write(serialTxBuf, sizeof(serialTxBuf));
  memset(serialTxBuf, 0, sizeof(serialTxBuf));

  while (1) {
    myled = !myled;
    ThisThread::sleep_for(500ms);

    sfm.Measure(FLOW);
    sfm.Measure(TEMP);
    sfm.Measure(VDD);

    sprintf(serialTxBuf, "Flow: %.2f %s\r\n", ((float)sfm.flow.i16 / sfm.scaleFactor.u16), sfm.flowUnitStr);
  pc.write(serialTxBuf, sizeof(serialTxBuf));
  memset(serialTxBuf, 0, sizeof(serialTxBuf));
    // pc.printf("%.0f %s (raw: %u) Temp: %.1fC Vdd: %.2f\n\r",
    // ((float)sfm.flow.i16 / sfm.scaleFactor.u16), sfm.flowUnitStr,
    // sfm.flow.u16, (float)sfm.temperature.i16/10, (float)sfm.vdd.u16/1000);
  }
}
