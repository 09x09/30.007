#ifndef ENCDR
#define ENCDR

#include "BounceFilter.h"
#include "Types.h"

class Encoder {
  private:
    BounceFilter *flt;
    Pins          pin;
    Physical      measurements;
    Reading       a;
    Result        result;
    unsigned long stop_microseconds;
    unsigned long last_sample;
    unsigned long last_successful_sample;

    int  read_raw();

  public:
    Encoder(int dt, int clk, int samples, unsigned long stop_microseconds, float step_angle, float linear_distance);
    bool   sample_encoder();
    bool   stopped();
    float  ang_vel();
    float  lin_vel();
    Result raw_tick();
};

#endif