
/*
class FanCurve {
public:
  double Temp[5];
  double Speed[5];
  FanCurve(const double* Temps, const double* Speeds) {
    Temp = Temps;
    Speed = Speeds;
  }
};
*/

class Power {
  float v;
  float a;

public:
  Power(float V, float A);
  Power fromVW(float V, float W);
  Power fromAW(float A, float W);
  float W();
  float V();
  float A();
};

/// @brief 
/// Contains a USB-PD Charging Power profile along with a Voltage of how it can be recognized.
class ProfileSense : public Power {
  float vsense;
public:
  ProfileSense(float V, float A, double Vsense);
  double vSense();
};