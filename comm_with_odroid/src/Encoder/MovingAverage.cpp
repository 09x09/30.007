#include "MovingAverage.h"

MovingAverage::MovingAverage(float samples, float decay_factor) {
  len = samples;
  decay_sum = 0;

  readings = new CircularBuffer<float>(len);

  decay = new float [len] {0};

  float factor = 1;
  for (int i = 0; i < len; i++) {
    decay[i] = factor;
    decay_sum += decay[i];
    factor *= decay_factor;
  }
};



float MovingAverage::read() {
  if (on_cache) return cache;

  cache = 0;
  for (int i = 0; i < readings->length(); i++) {
    cache += readings->read(i) * decay[i];
  }
  cache /= decay_sum;
  on_cache = true;
  return cache;
};



float MovingAverage::add_sample(float sample) {
  on_cache = false;
  readings->force_push(sample);
  return read();
};