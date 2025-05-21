#ifndef interpo_h
#define interpo_h

#include "Arduino.h"

class Ramp
{
	public:
        Ramp();
        void go(float val,uint32_t dur);
        float update();
        float p_val;
    private: 
        uint32_t p_dur;
        float p_A;
        float p_B;
        uint32_t p_t;
        uint32_t p_pos;
        uint32_t p_grain;
};

class Interpolation
{
    public:
        Interpolation(float StartVal);
        float go(float Input,uint32_t duration);
    private:
        Ramp _myramp;
        uint8_t _Flag;
        float _savedValue;
};

#endif
