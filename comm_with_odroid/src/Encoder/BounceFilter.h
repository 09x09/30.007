#ifndef BNCE_FLTR
#define BNCE_FLTR
#include "CircularBuffer.h"
#include "Types.h"

class BounceFilter {
  private:
    CircularBuffer<EncoderSample> *buff;
    int                            samples;
    unsigned long                  last_request;
    unsigned long                  last_sample;
    int                            last_state;

    EncoderSample calc_sample();
    int calc_state_at(int pos);

  public:
    BounceFilter(int samples);
    
    void   add_sample(EncoderSample val);
    Result sample(unsigned long time);
};

#endif