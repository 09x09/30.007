#include "Encoder.h"
#include "Arduino.h"



Encoder::Encoder(int dt, int clk, int samples, unsigned long stop_microseconds, float step_angle, float linear_distance) {
  pin.dt = dt;
  pin.clk = clk;
  flt = new BounceFilter(samples);
  measurements.linear_dist = linear_distance;
  measurements.step_angle = step_angle;
  this->a.last = false;
  this->a.curr = false;
  this->stop_microseconds = stop_microseconds;
  result.has = false;
  last_sample = micros();
  last_successful_sample = micros();
}



int Encoder::read_raw() {
  int reading = 0;
  a.curr = digitalRead(pin.clk);
  if (!a.last && a.curr) reading = digitalRead(pin.dt) ? 1 : -1;
  a.last = a.curr;
  return reading;
}



bool Encoder::sample_encoder() {
  int reading = read_raw();
  if (reading == 0) return false;
  unsigned long now = micros();

  EncoderSample sample;

  sample.time = now - last_sample;
  sample.state = reading;
  last_sample = now;

  flt->add_sample(sample);

  Result new_result = flt->sample(now);
  if (new_result.has) {
    result = new_result;
    last_successful_sample = now;
  }
  return new_result.has;
};



bool Encoder::stopped() {
  if (!this->result.has) return true;
  if (micros() - last_successful_sample > stop_microseconds) return true;
  return false;
}



float Encoder::ang_vel() {
  if (stopped()) return 0;
  return (measurements.step_angle / (this->result.data.time * 0.000001)) * this->result.data.state;
}

float Encoder::lin_vel() {
  if (stopped()) return 0;
  return ((measurements.linear_dist * measurements.step_angle )/ (this->result.data.time * 0.000001)) * this->result.data.state;
}


Result Encoder::raw_tick() {
  return result;
}