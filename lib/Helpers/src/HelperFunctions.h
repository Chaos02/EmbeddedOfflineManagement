
#ifndef HelperFunctions_h
#define HelperFunctions_h

#include <Arduino.h>

/// @brief reads the voltage from analog pins as a float
/// @param pin the pin to be read from
/// @return Voltage from analog pin as a float
float analogVoltage(uint8_t pin, float ControllerVoltage = 5.0f);

/// @brief calculates the Resistance. Connect R1 to +5V and an analog pin, R2 to the same analog pin and GND
/// @param pin The Analog pin, the middle of the voltage divider is connected to
/// @param R1 The Ohm value of the FIRST resistor in the voltage divider.
float resistanceReadR2(uint8_t pin, word R1, float Vin = 5.0f);

/// @brief calculates the Resistance. Connect R1 to +5V and an analog pin, R2 to the same analog pin and GND
/// @param pin The Analog pin, the middle of the voltage divider is connected to
/// @param R2 The Ohm value of the SECOND resistor in the voltage divider.
float resistanceReadR1(uint8_t pin, word R2, float Vin = 5.0f);

/// @brief calculates the input voltage of a voltage divider, if you measure between R2 and GND
/// @param Vout What voltage is between R2 and GND
/// @param R1 Ohm value of the FIRST resistor in the divider.
/// @param R2 Ohm value of the SECOND resistor in the divider.
float VoltageDivGetVin(float Vout, word R1 = 10700, word R2 = 1300);

/// @brief calculates the Temperature in °C. Connect a 10kOhm to +5V and an analog pin, the *linear* temp probe to the same analog pin and GND
/// @param pin The Analog pin, the middle of the voltage divider is connected to
/// @param R_neg20 Resistance in Ohm of the temperature probe at -20°C
/// @param R_pos100 Resistance in Ohm of the temperature probe at +100°C
/// @return 
float temperatureReadLinear(uint8_t pin, word R_neg20, word R_pos100);

/**
 * @brief Calculates the nth root of a number.
 * @param x The number to calculate the nth root of.
 * @param n The degree of the root.
 * @param tolerance The number of decimal places to use for the tolerance. Default is 6.
 * @return The nth root of x.
 */
double NthSqrt(double x, int n, int tolerance = 6);

void Steinhart_Hart_Coefficients(double Coefficients[3], unsigned long R1_R2_R3[3], int T1_T2_T3[3]);
float Steinhart_Hart_Resistance(float Temperature, double Coefficients[3]);

/// @brief Calculate temperature in °C from an NTC thermistor based on 3 values from the Steinhart-Hart equation.
/// @param Coefficients Coefficients A, B and C of the Steinhart-Hart equation
/// @param R2 Value of R2 in the Voltage divider
/// @return 
float temperatureRead(double Coefficients[3], double R);

/// @brief
/// @param val
/// @param list single value or list of doubles to be matched against
/// @param derivation how much (in percent) the value may be off to be considered in range
/// @return false without a match or the index + 1 if there was a match
byte inRange(double val, double list[], double derivation, byte listLength = 1);

/**
 * @brief Aligns the input string to the specified length.
 * @param in String to align.
 * @param mode char 'R', 'L' or 'C' for Right, Left and Center alignment respectively.
 * @param length Length to stretch the input string to.
 * @param padString Optional - what to pad the string with. Default is a space character.
 * @return The aligned string.
 */
String StringAlign(String in, char mode, byte length, String padString = " ");

double gcd(double a, double b);

#endif