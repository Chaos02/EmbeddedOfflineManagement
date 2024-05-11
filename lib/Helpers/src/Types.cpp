#include <Arduino.h>
#include "Types.h"

Power::Power(float V, float A) {
  v = V;
  a = A;
}
Power Power::fromVW(float V, float W) {
  return Power(V, (W / V));
}
Power fromAW(float A, float W) {
  return Power(A, (W / A));
}
float Power::W() {
  return v * a;
}
float Power::V() {
  return v;
}
float Power::A() {
  return a;
}

ProfileSense::ProfileSense(float V, float A, double Vsense): Power::Power{ V, A }, vsense{ Vsense } {} // runs regular Constructor and setsown field
double ProfileSense::vSense() {
  return vsense;
}