#pragma once

struct BME280Regs {
    virtual bool readBlock (uint8_t start, uint8_t count, uint8_t *buf) const = 0; // read reg block
    virtual void write (uint8_t r, uint8_t v) const = 0; // write 8-bit register
};

#if JEEH

template< typename I2C, int addr =0x76 >
struct BME280Jeeh : BME280Regs {
    BME280Jeeh() {};

    // read a block of registers, return true if successul
    bool readBlock (uint8_t start, uint8_t count, uint8_t *buf) const {
        if (! I2C::start(addr<<1) ) return false;
        if (! I2C::write(start) ) { I2C::stop(); return false; }
        I2C::stop();
        if (! I2C::start((addr<<1)|1) ) return false;
        int i = 0;
        while (i < count-1) buf[i++] = I2C::read(false);
        if (i < count) buf[i] = I2C::read(true);
        I2C::stop();
        return true;
    }

    // write an 8-bit register
    void write (uint8_t r, uint8_t v) const {
        I2C::start(addr<<1);
        I2C::write(r);
        I2C::write(v);
        I2C::stop();
    }
};

#elif ARDUINO

struct BME280Arduino : BME280Regs {
    BME280Regs(TwoWire &i2c, uint8_t addr = 0x76) : _i2c(i2c), _addr(addr) {};

    // read a block of registers, return true if successul
    // FIXME!
    bool readBlock (uint8_t start, uint8_t count, uint8_t *buf) const {
        uint8_t c = _i2c.requestFrom(_addr, 2, r, 1, true);
        if (c != 2) return 0; // may not be the best error value...
        int v = _i2c.read();
        return uint16_t((v<<8) | _i2c.read());
    }

    // write an 8-bit register
    void write (uint8_t r, uint8_t v) const {
        _i2c.beginTransmission(_addr);
        _i2c.write(r);
        _i2c.write(v);
        _i2c.endTransmission();
    }

    //private:
    TwoWire &_i2c;
    uint8_t _addr;
    uint8_t _t_os, p_os, h_os;
};

#endif
struct BME280 {
    BME280 (BME280Regs &regs) : _regs(regs) {};

    // temperature/pressure/humidity conversion disable and oversampling constants.
    enum { disable = 0, os_1x, os_2x, os_4x, os_8x, os_16x };
    // filter coefficients
    enum { flt_off = 0, flt_2, flt_4, flt_8, flt_16 };
    // operating modes.
    enum { mode_sleep = 0, mode_forced = 1, mode_normal = 3 };
    // standby durations in ms.
    enum { sb_0_5 = 0, sb_62_5, sb_125, sb_250, sb_500, sb_1000, sb_10, sb_20 };

    // init connects to the device, initializes it and leaves it in normal mode. It returns true if
    // successful. The parameters are the oversampling factors for temp, press, and hum, as well as
    // the standby times and filter. See enums above.
    bool init (uint8_t t_os, uint8_t p_os, uint8_t h_os, uint8_t t_sb, uint8_t filter);

    // mode sets the operating mode and returns the time in ms until the next conversion is
    // available, 0 for sleep mode, and -1 if there is an error.
    int16_t mode (uint8_t opmode) const;

    //  read returns the temperature in centi-centigrade, and optionally stores the pressure and
    //  humidity in the two parameters as well. The pressure is returned in Pa, and the humidity in
    //  100th percent.
    int16_t read (uint32_t *press, uint16_t *hum) const;

    // ready returns true if a measurement is complete and the sensor is idle. This is mostly useful
    // in mode_forced.
    bool ready() const;

    int getMode() const;

    // private stuff

    // register addresses.
    enum { reg_id=0xd0, reg_config=0xf5, reg_status=0xf3, reg_ctrl_meas=0xf4, reg_ctrl_hum=0xf2 };

    // calibration values read during init.
    uint16_t T1;
    int16_t  T2, T3;
    uint16_t P1;
    int16_t  P2,  P3,  P4,  P5,  P6,  P7,  P8,  P9;
    uint8_t  H1;
    int16_t  H2;
    uint8_t  H3;
    int16_t  H4, H5;
    int8_t   H6;

    int32_t calcT(int32_t adc_t) const;
    uint32_t calcP(int32_t adc_P, int32_t t_fine) const;
    uint16_t calcH(int32_t adc_h, int32_t t_fine) const;
    uint8_t calc_crc(uint8_t *buf, uint8_t len);

    uint8_t _t_os, _p_os, _h_os;

    BME280Regs &_regs;
};
