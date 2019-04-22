#ifndef CRIC_BUFF
#define CRIC_BUFF

#include "Types.h"

template<typename Data>
class CircularBuffer {
  private:
    Data *buffer;
    int  c_lower;
    int  c_upper;
    int  len;
    int  resolve_cursor(int pos);
    void drop_left();

  public:
    CircularBuffer<Data>(int size);
    void push(Data data);
    void force_push(Data data);
    void set(Data data, int pos);
    void cut(int pos);
    Data next();
    Data read(int pos);
    int  length();
};
template<typename Data>
CircularBuffer<Data>::CircularBuffer(int size) {
  if (size == 0) size = 1;
  if (size < 0) size = -size;
  len = size;
  buffer = new Data [len];
  c_lower = 0;
  c_upper = 0;
}

template<typename Data>
int CircularBuffer<Data>::length() {
  if (c_upper == c_lower) return 0;
  if (c_upper < c_lower) return len - c_lower + c_upper;
  return c_upper - c_lower;
}

template<typename Data>
int CircularBuffer<Data>::resolve_cursor(int pos) {
  if (pos == 0) return c_lower;
  if (pos >  0) return (c_lower + (pos % length())) % len;
  if (pos <  0) return (c_upper + 1 + (pos % length()) + len) % len;
}

template<typename Data>
void CircularBuffer<Data>::push(Data data) {
  if (length() == len) return;
  c_upper ++;
  c_upper %= len;
  buffer[resolve_cursor(-1)] = data;
}

template<typename Data>
void CircularBuffer<Data>::force_push(Data data) {
  if (length() == len - 1 && length() != 0) drop_left();
  push(data);
}

template<typename Data>
void CircularBuffer<Data>::set(Data data, int pos) {
  buffer[resolve_cursor(pos)] = data;
}

template<typename Data>
void CircularBuffer<Data>::drop_left() {
  c_lower ++;
  c_lower %= len;
}

template<typename Data>
Data CircularBuffer<Data>::next() {
  Data current = buffer[resolve_cursor(0)];
  drop_left();
  return current;
}

template<typename Data>
Data CircularBuffer<Data>::read(int pos) {
  return buffer[resolve_cursor(pos)];
}

template<typename Data>
void CircularBuffer<Data>::cut(int pos) {
  while (pos < length()) {
    buffer[resolve_cursor(pos)] = resolve_cursor(pos + 1);
    pos ++;
  }
  c_upper == (c_upper + len - 1) % len;
}

#endif