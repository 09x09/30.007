#ifndef ENCODER_TYPES
#define ENCODER_TYPES

struct Pins {
  int dt;
  int clk;
};

struct Reading {
  bool last;
  bool curr;
};

struct Physical {
  float step_angle;
  float linear_dist;
};


struct EncoderSample {
  unsigned long time;
  int           state;
};

struct Result {
  bool          has;
  EncoderSample data;
};

#endif