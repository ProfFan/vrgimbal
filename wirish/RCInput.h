
#ifndef __RCINPUT_H__
#define __RCINPUT_H__

//#include <AP_HAL_VRBRAIN.h>
//#include <AP_HAL.h>
#include "wirish.h"

#define RCINPUT_NUM_CHANNELS 4

class RCInput {
public:
    RCInput();
    void init();
    uint8_t  valid_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

    unsigned char GetState(void);
    uint16_t InputCh(unsigned char ch);

private:
    void InitDefaultPPM(char board);
    void InitDefaultPPMSUM(char board);
    void InitPPM(void);
    void InitPPMSUM(void);


    unsigned int ppm_sum_channel;
    unsigned int input_channel_ch1;
    unsigned int input_channel_ch2;
    unsigned int input_channel_ch3;
    unsigned int input_channel_ch4;
    unsigned int input_channel_ch5;
    unsigned int input_channel_ch6;
    unsigned int input_channel_ch7;
    unsigned int input_channel_ch8;
    unsigned int input_channel_ch9;
    unsigned int input_channel_ch10;
    unsigned int input_channel_ch11;
    unsigned int input_channel_ch12;

    unsigned char _iboard;
    static volatile uint8_t  _valid;
};

#endif // __RCINPUT_H__
