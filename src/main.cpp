#include <Arduino.h>
#include <CooperativeMultitasking.h>

#include <Wire.h>
#include <InterpolationLib.h>
// private libraries
#include <Types.h>
#include <HelperFunctions.h>
#include <SmartBattery.h>
#include <SMBus.h>

#define PINVin A0       // Connect between 10.7kΩ (R1) and 1.3kΩ (R2)
#define VinR1 10700
#define VinR2 1300
#define VMAX 48         // maximum expected Voltage on PINVin *before* voltage divider
#define VMaxDeviation 5 // how much (in percent) the voltage may deviate to still be considered in spec

// Connect Thermistors as R1 in a Voltage divider where R2 = 12000, each will pull 0,0015A
#define PINTempBat0 A5
#define PINTempBat1 A4
#define PINTempInner A3
#define PINTempOuter A2
#define PINTempWater A1

// Hook up smart Battery to D2, D3 (SCA, SCL / PIN20, PIN21)
// "Please note that a pull-up resistor is needed when connecting SDA/SCL pins"

#define PINPowerButton 7 // Interrupt 6
#define PINPCPowerButton 5
#define PINStatusLED 6
// Setting this pin LOW will set all PWM signals to 50%
#define PINMaintananceMode 7

#define PINPWMInner 9
#define PINPWMOuter 10
#define PINPWMPump 11

#define PINTachInner 8
#define PINTachOuter 12
#define PINTachPump 13

#define PINBuzzer MOSI // Connect a Buzzer to get audible alarms
#define PINConnSense MISO // Connect between 20kΩ and 2.7kΩ, Ground this pin in the cable plug

// USB PD (EPR) Profiles
ProfileSense USB_PD[] = {
  //           V, A, Voltage after Voltage divider
  ProfileSense(5, 3, 0.541),
  ProfileSense(9, 3, 0.975),
  ProfileSense(12, 16.6F, 1.3), // AC PSU
  ProfileSense(15, 3, 1.625),
  ProfileSense(20, 3, 2.167),
  ProfileSense(20, 5, 2.167),
  ProfileSense(28, 5, 3.030),
  ProfileSense(36, 5, 3.900),
  ProfileSense(48, 5, 5.200) // with 5% deviation this is still if we detect 4,94V
};

// ###### Temperature readout ####### //
// 3 points (Min, Somewhere middle, Max) of the NTC Thermistors Resistance/Temperature correlation
// adjust for the Thermistors used!
// Temp in Degree Celsius (°C)
int TemperatureAir[3] = { -30, 0, 105 };
// Resistance in Ohm (Ω)
unsigned long ResistanceAir[3] = { 137755, 32997, 600 }; // https://asset.conrad.com/media10/add/160267/c1/-/en/001570951DS00/datasheet-1570951-tru-components-mjsts-103-3950-1-600-3d-temperature-sensor-30-up-to-105-c-10-k-3950-k.pdf
double CoeffsAir[3] = { 0, 0, 0 }; // put coefficients here if you already know them: give A in 10^-3, B in 10^-4 and C in 10^-7
int TemperatureWater[3] = { -40, 0, 150 };
unsigned long ResistanceWater[3] = { 194300, 27380, 309 }; // https://download.alphacool.com/legacy/kOhm_Sensor_Table_Alphacool.pdf
double CoeffsWater[3] = { 0, 0, 0 };


// ####### FAN CURVES ####### //
// FanCurve FCInner(Temp, FanSpeed);
// Inner Curve
double TempsInner[] = { 20, 25, 40, 50, 60 };     // in °C  Inner Air Temp
double FanSpeedInner[] = { 0, 0, 50, 85, 100 };   // in percent
// Outer Curve
double TempsWater[] = { 20, 25, 30, 35, 45 };     // in °C  Water Temperature
double FanSpeedOuter[] = { 0, 0, 35, 65, 100 };   // in percent
// Pump Curve
double TempsWaterRadDelta[] = { 0, 2, 15 };       // in °C   Temperature delta Water to Outer Rad sensor
double PumpSpeed[] = { 0, 0, 100 };               // in percent
// ########################## //

const float samplerate = 5;   // How many seconds per Sample --> in Hz^(-1)
const float heuristic = 10; // temperature percentage, where no fan ramp is done

// ############### Battery Management System ##################
#define Bat0Address 0b11
#define Bat1Address Bat0Address // Find out via i2c scan!
#define SMBusSlaveAddr 0b10 // Common Smart Battery System manager address

#define Measure_Idle true // wether or not to measure and report the AVR "CPU usage"
#define samplerateIdle 10 // in seconds

// -----------------------------------------------------------------------internal-------------------------------------------------------------------

CooperativeMultitasking tasks;
Continuation StatusReport;
Continuation PDControl;
Continuation FanControl;
Continuation TachControl;
Continuation FanLocked;
Continuation SMBusControl;
Continuation BatControl;
Continuation ConnectivityControl;
// Continuation MeasureIdle;
Guard PDVolChange;
double PDDouble[9];
byte PDProfile = 1;

Battery bat0;
Battery bat1;

byte BatLevel = 0;
Battery_Status BatStatus;

bool MaxFans = false;
bool isSMBusHost = false;

struct CoolingSystem {
public:
  struct pins {
    struct temp {
      const byte Inner = PINTempInner;
      const byte Outer = PINTempOuter;
      const byte Water = PINTempWater;
    } Temp;
    struct pwm {
      const byte Inner = PINPWMInner;
      const byte Outer = PINPWMOuter;
      const byte Pump = PINPWMPump;
    } PWM;
    struct tach {
      const byte Inner = PINTachInner;
      const byte Outer = PINTachOuter;
      const byte Pump = PINTachPump;
    } Tach;
  } PINS;
  /// @brief in °C
  struct temp {
    float Inner;
    float Outer;
    float Water;
  } Temp;
  /// @brief in % duty cycle
  struct pwm {
    byte Inner;
    byte Outer;
    byte Pump;
  } PWM;
  /// @brief in RPM
  struct tach {
    unsigned short Inner = 0;
    unsigned short Outer = 0;
    unsigned short Pump = 0;
  } Tach;
  struct locked {
    bool Inner = false;
    bool Outer = false;
    bool Pump = false;
  } Locked;

  /// @brief calculate all PWMs from the stored Temperatures (with the same )
  /// @param Temps 
  /// @param FanSpeeds 
  void calcPWM() {
    if (MaxFans) {
      PWM.Inner = PWM.Outer = PWM.Pump = 255;
    } else if (!digitalRead(PINMaintananceMode)) {
      Serial.println("[Maintanance]");
      PWM.Inner = 77; // 30% lowest PWM speed
      PWM.Outer = 51; // 20% start from dead stop
      PWM.Pump = 26; // 10% 
    } else {
      if (Locked.Inner) PWM.Inner = 255; else
        PWM.Inner = (byte) (Interpolation::SmoothStep(TempsInner, FanSpeedInner, 5, Temp.Inner) * 2.55);
      if (Locked.Outer) PWM.Outer = 255; else
        PWM.Outer = (byte) (Interpolation::SmoothStep(TempsWater, FanSpeedOuter, 5, Temp.Water) * 2.55);
      if (Locked.Pump) PWM.Pump = 255; else
        PWM.Pump = (byte) (Interpolation::SmoothStep(TempsWaterRadDelta, PumpSpeed, 3, (Temp.Water - Temp.Outer)) * 2.55);
    }
    return;
  }
} CS;

volatile unsigned long BtnDown = 0;
volatile unsigned long BtnUp = 0;

Continuation PwrBtnUp;
void PwrBtnUp() {
  digitalWrite(PINPCPowerButton, HIGH);
}

void PowerButton() {
  unsigned long BtnChange = millis();

  if ((BtnChange - BtnDown < 50) || (BtnChange - BtnUp < 50)) return; //debounce

  if (digitalRead(PINPowerButton) == LOW) {
    BtnDown = BtnChange;

    if (!(DS.Codes && DS.ON)) {
      digitalWrite(PINPCPowerButton, LOW);
      tasks.after(500, PwrBtnUp, 10);
    }
  } else {
    BtnUp = BtnChange;
  }

  unsigned short held = BtnUp - BtnDown;

  if (DS.Codes && DS.ON) {
    if (held > 4000) {
      // send short pulse
      digitalWrite(PINPCPowerButton, LOW);
      tasks.after(500, PwrBtnUp, 10);

    } else if (held > 6000) {
      // send long pulse (force shutdown)
      digitalWrite(PINPCPowerButton, LOW);
      tasks.after(4500, PwrBtnUp, 10);

    }
  } else {
    if (held > 1000) {
      DS.Codes |= DS.MaintananceMode;
    }
  }
}

void setup() {
  for (byte i = 0; i < 9; i++) {
    PDDouble[i] = USB_PD[i].vSense();
  }

  Serial.begin(2400);

  pinMode(PINMaintananceMode, INPUT_PULLUP);
  pinMode(PINVin, INPUT);
  pinMode(PINPowerButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PINPowerButton), PowerButton, CHANGE);
  pinMode(PINPCPowerButton, OUTPUT);
  pinMode(PINConnSense, INPUT_PULLUP);
  pinMode(PINBuzzer, OUTPUT);

  pinMode(PINTempInner, INPUT);
  pinMode(PINTempOuter, INPUT);
  pinMode(PINTempWater, INPUT);
  pinMode(PINPWMInner, OUTPUT);
  pinMode(PINPWMOuter, OUTPUT);
  pinMode(PINPWMPump, OUTPUT);


  analogWrite(PINPWMOuter, 128); // as to not make so much noise during boot

  pinMode(LED_BUILTIN, OUTPUT);
  byte led_Brightness = 0;
  unsigned long now = millis();
  while ((millis() < now + 2000) && !Serial) {
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }

  while (millis() < now + 10000) {
    if (led_Brightness >= 250) { led_Brightness = 0; }
    analogWrite(LED_BUILTIN, led_Brightness);
    led_Brightness += 5;
    delay(20);
  }

  pinMode(PINTachInner, INPUT);
  pinMode(PINTachOuter, INPUT);
  pinMode(PINTachPump, INPUT);

  SMBus.begin();
  bat0 = Battery((Bat0Address | 0b001000) << 1, BatAlarmWarningHandler);
  bat1 = Battery((Bat1Address | 0b001000) << 1, BatAlarmWarningHandler);


  if (CoeffsAir[0] == 0) {
    Steinhart_Hart_Coefficients(CoeffsAir, ResistanceAir, TemperatureAir);
    Serial.println("Temp sens Coefficients Air:");
    Serial.println("A=" + String(CoeffsAir[0] * pow(10, 3)) + "E-3, B=" + String(CoeffsAir[1] * pow(10, 4)) + "E-4, C=" + String(CoeffsAir[2] * pow(10, 7)) + "E-7");
  } else {
    CoeffsAir[0] *= pow(10, -3);
    CoeffsAir[0] *= pow(10, -4);
    CoeffsAir[0] *= pow(10, -7);
  }
  if (CoeffsWater[0] == 0) {
    Steinhart_Hart_Coefficients(CoeffsWater, ResistanceWater, TemperatureWater);
    Serial.println("Temp sens Coefficients Water:");
    Serial.println("A=" + String(CoeffsWater[0] * pow(10, 3)) + "E-3, B=" + String(CoeffsWater[1] * pow(10, 4)) + "E-4, C=" + String(CoeffsWater[2] * pow(10, 7)) + "E-7");
  } else {
    CoeffsWater[0] *= pow(10, -3);
    CoeffsWater[0] *= pow(10, -4);
    CoeffsWater[0] *= pow(10, -7);
  }

  Serial.println("========= Temperature Sensor profiles: =========");
  Serial.print("        ");
  for (byte i = 0; i <= 105; i += 15) {
    Serial.print(StringAlign(String(i) + "°C", 'R', 11));
  }
  Serial.println();

  Serial.print(StringAlign("Air: ", 'R', 8));
  for (byte i = 0; i <= 105; i += 15) {
    Serial.print(StringAlign(String(Steinhart_Hart_Resistance(i, CoeffsAir) / 1000, 1) + "KΩ", 'R', 11));
  }
  Serial.println();

  Serial.print(StringAlign("Water: ", 'R', 8));
  for (byte i = 0; i <= 105; i += 15) {
    Serial.print(StringAlign(String(Steinhart_Hart_Resistance(i, CoeffsWater) / 1000, 1) + "KΩ", 'R', 11));
  }
  Serial.println();
  Serial.println("If these do not match the data sheets of your Sensors, please adjust the table in program.");

  Serial.println("10KΩ at " + String(temperatureRead(CoeffsAir, 10000)));

  /*
  if (Measure_Idle) {
    tasks.now(MeasureIdle, 4);
  }
  */
  tasks.after(1000, StatusReport);
  tasks.now(FanControl);
  tasks.after(10000, TachControl); // wait for fans to gain speed on startup
  tasks.now(PDControl, 5);
  tasks.now(SMBusControl);
  tasks.after(2000, ConnectivityControl);

}

struct DeviceStatus {
  uint16_t Codes;
  uint16_t Displaying;
  byte StatusLED;
public:
  enum Code {
    OFF = 0b0,
    ON = 0b1,
    NoConnection = 0b10,
    NoCable = 0b100,



    MaintananceMode = 0b10000000,
    // High Byte for Problems
    BatLow = 0b100000000,
    PowerError = 0b1000000000,
    CoolingError = 0b10000000000,
    BatAlarm = 0b100000000000,

  };
} DS;

Continuation StatusUp;
Continuation StatusDown;
void StatusUP() {
  analogWrite(PINStatusLED, DS.StatusLED++);
}
void StatusDown() {
  analogWrite(PINStatusLED, DS.StatusLED--);
}
Continuation NotifyHIGH;
Continuation NotifyLOW;
void NotifyHIGH() {
  digitalWrite(PINStatusLED, HIGH);
}
void NotifyLOW() {
  digitalWrite(PINStatusLED, LOW);
}
Continuation AlarmHIGH;
Continuation AlarmLOW;
void AlarmHIGH() {
  digitalWrite(PINStatusLED, HIGH);
  digitalWrite(PINBuzzer, HIGH);
}
void AlarmLOW() {
  digitalWrite(PINStatusLED, LOW);
  digitalWrite(PINBuzzer, LOW);
}

void StatusReport() {
  // refresh every 4 seconds, needs to be >= time of longest LED cycle (DS.OK)
  tasks.after(4000, StatusReport, 1);

  if (DS.Codes && DS.ON) {
    DS.Displaying = DS.ON;
    if (DS.Codes && DS.NoConnection)
      DS.Displaying = DS.NoConnection;
    if (DS.Codes && DS.NoCable)
      DS.Displaying = DS.NoCable;
    if (DS.Codes && DS.MaintananceMode)
      DS.Displaying = DS.MaintananceMode;
    if (DS.Codes && DS.BatLow)
      DS.Displaying = DS.BatLow;
    if (DS.Codes && DS.PowerError)
      DS.Displaying = DS.PowerError;
    if (DS.Codes && DS.CoolingError)
      DS.Displaying = DS.CoolingError;
  } else {
    DS.Displaying = DS.OFF;
  }
  if (DS.Codes && DS.BatAlarm)
    DS.Displaying = DS.BatAlarm;


  switch (DS.Displaying) {
    case DS.OFF: {
      tasks.now(AlarmLOW);
      break;
    }
    case DS.ON: {
      // slow pulse
      DS.StatusLED = 0;
      for (unsigned int i = 0; i < 255; i++) {
        tasks.after(i * 7, StatusUp);
        tasks.after(2000 + (i * 7), StatusDown);
      }
      break;
    } case DS.NoConnection: {
      DS.StatusLED = 255;
      for (unsigned int i = 0; i < 255; i++) {
        tasks.after(i * 7, StatusDown);
        tasks.after(2000 + (i * 7), StatusDown);
      }
      break;
    } case DS.NoCable: {
      DS.StatusLED = 0;
      for (unsigned int i = 0; i < 255; i++) {
        tasks.after(i * 7, StatusUp);
        tasks.after(2000 + (i * 7), StatusUp);
      }
      break;
    } case DS.MaintananceMode: {
      DS.StatusLED = 128;
      for (unsigned int i = 128; i < 255; i++) {
        tasks.after(i * 7, StatusUp);
        tasks.after(2000 + (i * 7), StatusUp);
      }
      break;
    } case DS.BatLow: {
      tasks.now(NotifyHIGH);
      tasks.after(100, NotifyLOW);
      break;
    } case DS.PowerError: {
      tasks.after(1000, NotifyHIGH);
      tasks.after(2000, NotifyLOW);
      break;
    } case DS.BatAlarm: {
      for (word i; i < 4000; i += 250) {
        tasks.after(i, AlarmHIGH);
        tasks.after(i + 120, AlarmLOW);
      }
      break;
    }
  }
}

void ConnectivityControl() {
  if (digitalRead(PINConnSense) != LOW) {
    DS.Codes |= DS.NoCable;
  } else {
    DS.Codes &= ~DS.NoCable;
  }
  tasks.after(samplerate * 1000, ConnectivityControl, 1);
}

void PDControl() {

  byte profile = inRange(analogVoltage(PINVin), PDDouble, VMaxDeviation, 9);
  if (PDProfile != profile) {
    PDProfile = profile;
    DS.Codes &= ~DS.PowerError;
    if (PDProfile > 0) {
      String watt = String(USB_PD[PDProfile - 1].W());
      Serial.println("[PD_W] (" + watt + ") Watts available to the system!");
    } else if (analogVoltage(PINVin) < 1.0f) { // its not a profile!
      Serial.println("[PD_W] (0) USB-C Disconnected!");
    } else { // we don't know what it is!
      String vReal = String(VoltageDivGetVin(analogVoltage(PINVin), VinR1, VinR2));

      Serial.println("[PD_V] (" + vReal + ") Unrecognized Voltage!");
      DS.Codes |= DS.PowerError;
    }
  } // nothing changed

  tasks.after(100, PDControl, 5);
}

/// @brief *Reeeaallyy* approximately reads all current PWM speeds
/// split into own Continuation because of really low priority.
void TachControl() {
  CS.Tach.Inner = 0;
  CS.Tach.Outer = 0;
  CS.Tach.Pump = 0;

  bool lastInner = digitalRead(CS.PINS.Tach.Inner);
  bool lastOuter = digitalRead(CS.PINS.Tach.Outer);
  bool lastPump = digitalRead(CS.PINS.Tach.Pump);

  uint32_t now = millis();
  while ((millis() - now) < 100) { //down to 600RPM support

    if (digitalRead(CS.PINS.Tach.Inner) != lastInner) {
      lastInner = !lastInner;
      CS.Tach.Inner++;
    }
    if (digitalRead(CS.PINS.Tach.Outer) != lastOuter) {
      lastOuter = !lastOuter;
      CS.Tach.Outer++;
    }
    if (digitalRead(CS.PINS.Tach.Pump) != lastPump) {
      lastPump = !lastPump;
      CS.Tach.Pump++;
    }
  }

  // 2 Pulse == 1 Revolution; 1 Pulse == HIGH & LOW
  CS.Tach.Inner *= 600 / 2 / 2;
  CS.Tach.Outer *= 600 / 2 / 2;
  CS.Tach.Pump *= 600 / 2 / 2;


  if (
    (CS.Tach.Inner < 300 && CS.PWM.Inner > FanSpeedInner[0]) ||
    (CS.Tach.Outer < 3000 && CS.PWM.Outer > FanSpeedOuter[0]) ||
    (CS.Tach.Pump < 500 && CS.PWM.Pump > PumpSpeed[0])) {
    tasks.after(samplerate * 5000, FanLocked, 3);
  } else {
    DS.Codes &= ~DS.CoolingError;
    CS.Locked.Inner = false;
    CS.Locked.Outer = false;
    CS.Locked.Pump = false;
  }

  tasks.after(samplerate * 2000, TachControl, 1);
}

/// @brief Do stuff when a fan is actually locked.
void FanLocked() {
  DS.Codes |= DS.CoolingError;
  if ((CS.Tach.Inner == 0)) {
    CS.Locked.Inner = true;
    Serial.println("[FAN] (Inner) Inner fan seems to be locked!!");
  }
  if ((CS.Tach.Outer == 0)) {
    CS.Locked.Outer = true;
    Serial.println("[FAN] (Outer) Outer fan seems to be locked!!");
  }
  if ((CS.Tach.Pump == 0)) {
    CS.Locked.Pump = true;
    Serial.println("[FAN] (Pump) Pump seems to be locked!!");
  }
}

void FanControl() {

  CS.Temp.Inner = temperatureRead(CoeffsAir, resistanceReadR1(CS.PINS.Temp.Inner, 12200));
  CS.Temp.Outer = temperatureRead(CoeffsAir, resistanceReadR1(CS.PINS.Temp.Outer, 12200));
  CS.Temp.Water = temperatureRead(CoeffsWater, resistanceReadR1(CS.PINS.Temp.Water, 12200));

  CS.calcPWM();

  analogWrite(CS.PINS.PWM.Inner, CS.PWM.Inner);
  analogWrite(CS.PINS.PWM.Outer, CS.PWM.Outer);
  analogWrite(CS.PINS.PWM.Pump, CS.PWM.Pump);


  String TInn = String(CS.Temp.Inner, 1) + "°C";
  String TOut = String(CS.Temp.Outer, 1) + "°C";
  String TWat = String(CS.Temp.Water, 1) + "°C";
  if (CS.Temp.Inner == INFINITY) TInn = "---";
  if (CS.Temp.Outer == INFINITY) TOut = "---";
  if (CS.Temp.Water == INFINITY) TWat = "---";

  Serial.println("       Inner | Outer | Pump");
  Serial.print("[TMP] (" + TInn + "  ");
  Serial.print(TOut + "  ");
  Serial.println(TWat + ")");

  Serial.print("[PWM] (" + String(round(CS.PWM.Inner / 2.55)) + "%" + "    ");
  Serial.print(String(round(CS.PWM.Outer / 2.55)) + "%" + "    ");
  Serial.println(String(round(CS.PWM.Pump / 2.55)) + "%" + ")");

  Serial.print("[TACH] (" + String(CS.Tach.Inner) + "  ");
  Serial.print(String(CS.Tach.Outer) + "  ");
  Serial.println(String(CS.Tach.Pump) + ")");

  tasks.after(samplerate * 1000, FanControl, 4);
}

void SMBusControl() {
  if (isSMBusHost) {
    //register as slave to see if system ACKs host as address
    Wire.end();
    Wire.begin((SMBusSlaveAddr | 0b0001000) << 1);

    Wire.beginTransmission(0b0001000 << 1); // reserved host address
    byte error = Wire.endTransmission(true);

    if (error = 0) {
      isSMBusHost = false;
      DS.Codes |= DS.ON;
      tasks.after(60000, SMBusControl); // when system has been turned on
    } else {
      isSMBusHost = true;
      DS.Codes &= ~DS.ON;
      Wire.end();
      Wire.begin(); // as Host
      tasks.after(15000, SMBusControl); // system is NOT online
      tasks.now(BatControl);
    }
  } else {
    Wire.beginTransmission(0b0001000 << 1); // reserved host address
    byte error = Wire.endTransmission(true);
    if (error != 5) { // timeout
      tasks.after(60000, SMBusControl); // wait for system shutdown
    } else {
      isSMBusHost = true;
      tasks.now(SMBusControl);
    }
  }
}

// Only called if system is offline - otherwise System receives AlarmWarning calls!
void BatAlarmWarningHandler(byte Address, byte Alarms) {
  Serial.println("[BAT_A] (" + String(Address, BIN) + "_" + String(Alarms) + ") Detected alarms: " + Battery_Status(Alarms << 8).asString());
  if (Alarms & highByte(Battery_Status::Alarm::OverTemp)) {
    // only cleared on System reset
    DS.Codes |= DS.BatAlarm;
    MaxFans = true;
  }
  if (Alarms & highByte(Battery_Status::Alarm::RemainingTime)) {
    Serial.println("[BAT] Low Battery!");
    DS.Codes |= DS.BatLow;
  }
  if (Alarms & highByte(Battery_Status::Alarm::OverCharged)) {

  }
}

void BatControl() {
  byte lvl = (bat0.RelativeStateOfCharge() + bat1.RelativeStateOfCharge()) / 2;
  if (BatLevel != lvl) {
    Serial.println("[BAT_%] (" + String(lvl) + ") Battery level is now " + String(lvl) + "%.");
    BatLevel = lvl;
  }

  Battery_Status bs0 = bat0.BatteryStatus();
  Battery_Status bs1 = bat1.BatteryStatus();


  Battery_Status stat = Battery_Status(bs0.asWord() | bs1.asWord());

  if (BatStatus != stat) {

    Serial.println("[BAT_S] (" + String(stat.asWord()) + ") Battery Status changed: " + stat.asString());

    uint16_t diff = bs0.asWord() ^ bs1.asWord();
    if (!!diff) { // if there's a difference
      Serial.print("[BAT_S] (" + String(diff, BIN) + ") Batteries have differing Status: " + *Battery_Status::meaning(diff));
    }
    BatStatus = stat;
  }
}

/// @brief DO NOT CHANGE!
void loop() {
  tasks.run();
}

