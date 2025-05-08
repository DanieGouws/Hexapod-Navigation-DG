#include "interpo.h"

Ramp::Ramp()
{
    p_val = 0.0;
    p_dur = 0;
    p_A = 0.0;
    p_B = 0.0;
    p_t = 0;
    p_pos = 0;
    p_grain = 10;
}

void Ramp::go(float val,uint32_t dur)
{
    p_A = p_val;   
    p_B = val;
    p_dur = dur;
    p_t = millis(); 
    p_pos = 0;
}

float Ramp::update()
{
    uint32_t newTime = millis();
    uint32_t delta = newTime - p_t;
    bool doUpdate;
    
    if(p_dur <= 0)
    {
      p_val = p_B;
    }
    else
    {  
      if(delta >= p_grain)
      {
        doUpdate = true;
      }
      else
      {
        doUpdate = false;
      }
  
      if(doUpdate)
      {
          p_t = newTime;
          if ((p_pos + delta) < p_dur)
          {
              p_pos = p_pos + delta;
          }
          else
          {
              p_pos = p_dur;
          }
  
          float k = (float)p_pos/(float)p_dur;
          p_val = p_A + (p_B - p_A)*k;
      }
    }
    return p_val;
}

Interpolation::Interpolation(float StartVal)
{
    _myramp.p_val = StartVal;
    _Flag = 0;
    _savedValue = 0;
}

float Interpolation::go(float Input,uint32_t duration)
{
    float output;

    if (Input != _savedValue)
    {
        _Flag = 0;
    }
    _savedValue = Input;

    if (_Flag == 0)
    {
        _myramp.go(Input,duration);
        _Flag = 1;
    }
    
    output = _myramp.update();
    return output;
}
