#include "BounceFilter.h"



BounceFilter::BounceFilter(int samples) {
  this->samples = (samples % 2 == 0) ? samples + 1 : samples;
  buff = new CircularBuffer<EncoderSample>(samples * 2 + 3);
  last_request = 0;
  last_sample  = 0;
  last_state   = 1;
};



int BounceFilter::calc_state_at(int pos) {
  int sample_state = 0;
  for (int i = pos; i < samples + pos; i++) {
    sample_state += buff->read(i).state;
  }
  return sample_state < 0 ? -1 : sample_state > 0 ? 1 : 0;
}



EncoderSample BounceFilter::calc_sample() {
  int next_state = calc_state_at(0);
  next_state += calc_state_at(1);
  next_state += calc_state_at(2);

  unsigned long positive = 0;
  unsigned long negative = 0;
  int positive_samples = 0;
  int negative_samples = 0;
  
  for (int i = 0; i < samples; i++) {
    EncoderSample current = buff->read(i);
    if (current.state == -1) {
      negative += current.time;
      negative_samples ++;
    }
    if (current.state == 1) {
      positive += current.time;
      positive_samples ++;
    }
  }
  
  EncoderSample result;

  if ((next_state == 3 && last_state == -1) || last_state == 1) {
    result.state = 1;
    result.time = positive / positive_samples;
    last_state = 1;
  }
  if ((next_state == -3 && last_state == 1) || last_state == -1) {
    result.state = -1;
    result.time = negative / negative_samples;
    last_state = -1;
  }  return result;
}



void BounceFilter::add_sample(EncoderSample val) {
  buff->push(val);
}



Result BounceFilter::sample(unsigned long time) {
  Result result;
  result.has = false;

  if (buff->length() <= samples + 3) return result;
  if (last_request + last_sample > time) return result;

  result.has = true;
  last_request += last_sample;

  buff->next();
  result.data = calc_sample();
  last_sample = result.data.time;
  return result;
}