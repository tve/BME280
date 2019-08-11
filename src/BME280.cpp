// Driver for a BME280 temperature / pressure / humidity sensor
#if JEEH
#include <jee.h>
#elif ARDUINO
#include <Arduino.h>
#include <Wire.h>
#define I2C TwoWire
#endif

#include "BME280.h"

// init connects to the device, initializes it and leaves it in sleep mode. It returns true if
// successful. The parameters are the oversampling factors for temp, press, and hum, as well as
// the standby times and filter. See enums above.
bool BME280::init (uint8_t t_os, uint8_t p_os, uint8_t h_os, uint8_t t_sb, uint8_t filter) {
    // make sure we can talk to the chip and that it smells like a bme280...
    uint8_t id = 0;
    if (!_regs.readBlock(reg_id, 1, &id)) return false;
    if (id != 0x60) return false;

    // read the calibration data into a buffer and then transfer to the struct variables.
    // While doing so, calculate a CRC across the data and verify with stored value.
    uint8_t buf[26+8];
    if (!_regs.readBlock(0x88, 26, buf)) { return false; }
    T1 = (buf[1]<<8) | buf[0];
    T2 = (buf[3]<<8) | buf[2];
    T3 = (buf[5]<<8) | buf[4];
    P1 = (buf[7]<<8) | buf[6];
    P2 = (buf[9]<<8) | buf[8];
    P3 = (buf[11]<<8) | buf[10];
    P4 = (buf[13]<<8) | buf[12];
    P5 = (buf[15]<<8) | buf[14];
    P6 = (buf[17]<<8) | buf[16];
    P7 = (buf[19]<<8) | buf[18];
    P8 = (buf[21]<<8) | buf[20];
    P9 = (buf[23]<<8) | buf[22];
    H1 = buf[25];
    if (!_regs.readBlock(0xE1, 8, buf+26)) { return false; }
    H2 = (buf[26+1]<<8) | buf[26+0];
    H3 = buf[26+2];
    H4 = (buf[26+3]<<4) | (buf[26+4]&0xf);
    H5 = (buf[26+5]<<4) | (buf[26+4]>>4);
    H6 = buf[26+6];
    uint8_t crc = calc_crc(buf, 26+7);
    if (crc != buf[26+7]) return false;

    printf("BME280 T: %d %d %d\n", T1, T2, T3);
    printf("BME280 P: %d %d %d %d %d %d %d %d %d\n", P1, P2, P3, P4, P5, P6, P7, P8, P9);
    printf("BME280 H: %d %d %d %d %d %d %d %d %d\n", H1, H2, H3, H4, H5, H6);

    // configure the chip
    _t_os = t_os; _p_os = p_os; _h_os = h_os;
    _regs.write(reg_config, ((t_sb&7)<<5) | ((filter&7)<<2));
    _regs.write(reg_ctrl_hum, (h_os&7));
    _regs.write(reg_ctrl_meas, ((t_os&7)<<5) | ((p_os&7)<<2) | mode_sleep);

    return true;
}

uint8_t BME280::calc_crc(uint8_t *buf, uint8_t len) {
    uint32_t crc_reg = 0xFF;
    uint8_t polynomial = 0x1D;
    uint8_t bitNo, index;
    uint8_t din = 0;

    for (index = 0; index < len; index++) {
        for (bitNo = 0; bitNo < 8; bitNo++) {
            din = ((crc_reg & 0x80) > 0) ^ ((buf[index] & 0x80) > 0);
            din &= 1;

            /* Truncate 8th bit for crc_reg and mem_values */
            crc_reg = (uint32_t)((crc_reg & 0x7F) << 1);
            buf[index] = (uint8_t)((buf[index] & 0x7F) << 1);
            crc_reg = (uint32_t)(crc_reg ^ (polynomial * din));
        }
    }

    return (uint8_t)(crc_reg ^ 0xFF);
}

// mode sets the operating mode and returns the time in ms until the next conversion is
// available, 0 for sleep mode.
int16_t BME280::mode (uint8_t opmode) const {
    _regs.write(reg_ctrl_meas, ((_t_os&7)<<5) | ((_p_os&7)<<2) | (opmode&3));
    if (opmode == mode_sleep) return 0;
    // calculate the time to first conversion according to datasheet sec 11.1
    uint32_t t = 1250; // in us
    if (_t_os != disable) t += 2300 * (1<<(_t_os-1));
    if (_p_os != disable) t += 2300 * (1<<(_p_os-1)) + 575;
    if (_h_os != disable) t += 2300 * (1<<(_h_os-1)) + 575;
    return (t+999)/1000; // round up to ms
}

// ready returns true if a measurement is complete and the sensor is idle. This is mostly useful
// in mode_forced.
bool BME280::ready() const {
    uint8_t status;
    if (!_regs.readBlock(reg_status, 1, &status)) return false;
    return (status & 0x8) == 0;
}

int BME280::getMode() const {
    uint8_t m;
    if (!_regs.readBlock(reg_ctrl_meas, 1, &m)) return -1;
    return m&0x3;
}

//  read returns the temperature in centi-centigrade, and optionally stores the pressure and
//  humidity in the two parameters as well.
int16_t BME280::read (uint32_t *press, uint16_t *hum) const {
    uint8_t buf[8];
    if (!_regs.readBlock(0xF7, 8, buf)) return 0x8000;
    int32_t adc_p = (buf[0]<<12) | (buf[1]<<4) | (buf[2]>>4);
    int32_t adc_t = (buf[3]<<12) | (buf[4]<<4) | (buf[5]>>4);
    int32_t adc_h = (buf[6]<<8) | buf[7];

    int32_t t_fine = calcT(adc_t);
    if (press) *press = calcP(adc_p, t_fine);
    if (hum) *hum = calcH(adc_h, t_fine);
    return (t_fine * 5 + 128) >> 8;
}

// calcT returns temperature as "t_fine" defined in the datasheet 6.2.3
int32_t BME280::calcT(int32_t adc_t) const {
    int32_t var1, var2;

    var1 = ((adc_t/8 - (int32_t)T1*2) * (int32_t)T2) / 2048;
    var2 = adc_t/16 - (int32_t)T1;
    var2 = ((var2*var2)/4096) * (int32_t)T3 >> 14;
    return var1 + var2;
}

// calcP returns pressure in Pa.
uint32_t BME280::calcP(int32_t adc_p, int32_t t_fine) const {
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)P6;
    var2 = var2 + ((var1*(int64_t)P5)<<17);
    var2 = var2 + ((int64_t)P4<<35);
    var1 = ((var1 * var1 * (int64_t)P3)>>8) + ((var1 * (int64_t)P2)<<12);
    var1 = ( (((int64_t)1<<47) + var1) * (int64_t)P1 ) >> 33;

    if (var1 == 0) return 0; // avoid exception caused by division by zero

    p = 1048576 - adc_p;
    p = ((p << 31) - var2) * 3125 / var1;
    var1 = ((int64_t)P9 * (p>>13) * (p>>13)) >> 25;
    var2 = ((int64_t)P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)P7<<4);

    return (uint32_t)(p >> 8);
}

// calcH returns humidity in 100th %RH.
uint16_t BME280::calcH(int32_t adc_h, int32_t t_fine) const {
    int32_t var1, var2, var3, var4, var5;
    uint32_t humidity;
    constexpr uint32_t humidity_max = 102400;

    var1 = t_fine - 76800;
    var2 = adc_h * 16384;
    var3 = (int32_t)H4 * 1048576;
    var4 = (int32_t)H5 * var1;
    var5 = (var2 - var3 - var4 + 16384) / 32768;
    var2 = (var1 * (int32_t)H6) / 1024;
    var3 = (var1 * (int32_t)H3) / 2048;
    var4 = var2 * (var3 + 32768) / 1024 + 2097152;
    var2 = (var4 * (int32_t)H2 + 8192) / 16384;
    var3 = var5 * var2;
    var4 = (var3 / 32768) * (var3 / 32768) / 128;
    var5 = var3 - (var4 * (int32_t)H1 / 16);
    var5 = var5 < 0 ? 0 : var5;
    var5 = var5 > 419430400 ? 419430400 : var5;
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
            humidity = humidity_max;

    return (uint16_t)(humidity*100>>10);
}
