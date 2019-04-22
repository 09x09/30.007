#ifndef MOV_AVG
#define MOV_AVG
#include "CircularBuffer.h"

class MovingAverage {
  private:
    int    len;
    int    cursor = 0;
    // float *readings;
    float *decay;
    float  decay_sum;
    float  cache;
    bool   on_cache;

    CircularBuffer<float> *readings;

  public:
    MovingAverage(float samples, float decay_factor);
    float add_sample(float sample);
    float read();
};


#endif