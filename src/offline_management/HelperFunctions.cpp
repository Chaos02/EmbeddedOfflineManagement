#include "HelperFunctions.h"

#include <InterpolationLib.h>

/// @brief reads the voltage from analog pins as a float
/// @param pin the pin to be read from
/// @return Voltage from analog pin as a float
float analogVoltage(uint8_t pin, float ControllerVoltage) {
    return analogRead(pin) * (ControllerVoltage / 1023.0);
}

/// @brief calculates the Resistance. Connect R1 to +5V and an analog pin, R2 to the same analog pin and GND
/// @param pin The Analog pin, the middle of the voltage divider is connected to
/// @param R1 The Ohm value of the FIRST resistor in the voltage divider.
float resistanceReadR2(uint8_t pin, word R1, float Vin) {
    int Vout = analogVoltage(pin);
    return (Vout * R1) / (Vin - Vout);
}

/// @brief calculates the Resistance. Connect R1 to +5V and an analog pin, R2 to the same analog pin and GND
/// @param pin The Analog pin, the middle of the voltage divider is connected to
/// @param R2 The Ohm value of the SECOND resistor in the voltage divider.
float resistanceReadR1(uint8_t pin, word R2, float Vin) {
    int Vout = analogVoltage(pin);
    return ((Vin - Vout) * R2) / Vout;
}

/// @brief calculates the input voltage of a voltage divider, if you measure between R2 and GND
/// @param Vout What voltage is between R2 and GND
/// @param R1 Ohm value of the FIRST resistor in the divider.
/// @param R2 Ohm value of the SECOND resistor in the divider.
float VoltageDivGetVin(float Vout, word R1, word R2) {
    return (Vout * (R1 + R2)) / R2;
}

/// @brief calculates the Temperature in °C. Connect a 10kOhm to +5V and an analog pin, the *linear* temp probe to the same analog pin and GND
/// @param pin The Analog pin, the middle of the voltage divider is connected to
/// @param R_neg20 Resistance in Ohm of the temperature probe at -20°C
/// @param R_pos100 Resistance in Ohm of the temperature probe at +100°C
/// @return 
float temperatureReadLinear(uint8_t pin, word R_neg20, word R_pos100) {
    // map() = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return ((resistanceReadR1(pin, 10000) - R_neg20) * (100.0f - -20.0f) / (R_pos100 - R_neg20) + -20.0f);
}

void Steinhart_Hart_Coefficients(double Coefficients[3], unsigned long R1_R2_R3[3], int T1_T2_T3[3]) {

    double L1 = log(R1_R2_R3[0]);
    double L2 = log(R1_R2_R3[1]);
    double L3 = log(R1_R2_R3[2]);

    double Y1 = 1 / (T1_T2_T3[0] + 273.15);
    double Y2 = 1 / (T1_T2_T3[1] + 273.15);
    double Y3 = 1 / (T1_T2_T3[2] + 273.15);

    double gamma2 = (Y2 - Y1) / (L2 - L1);
    double gamma3 = (Y3 - Y1) / (L3 - L1);

    Coefficients[2] = ((gamma3 - gamma2) / (L3 - L2)) * pow((L1 + L2 + L3), -1); // C
    Coefficients[1] = gamma2 - (Coefficients[2] * (pow(L1, 2) + (L1 * L2) + pow(L2, 2))); // B
    Coefficients[0] = Y1 - ((Coefficients[1] + (pow(L1, 2) * Coefficients[2])) * L1); // A

    return;
}

// TODO fix. returns INF in all cases.
float Steinhart_Hart_Resistance(float Temperature, double Coefficients[3]) {

    double x = (1 / Coefficients[2]) * (Coefficients[0] - (1 / (Temperature + 273.15)));
    double y = sqrt(pow((Coefficients[1] / (3 * Coefficients[2])), 3) + (pow(x, 2) / 4));

    return exp(NthSqrt(y - (x / 2), 3) - NthSqrt(y + (x / 2), 3));
}

/// @brief Calculate temperature in °C from an NTC thermistor based on 3 values from the Steinhart-Hart equation.
/// @param Coefficients Coefficients A, B and C of the Steinhart-Hart equation
/// @param R2 Value of R2 in the Voltage divider
/// @return 
float temperatureRead(double Coefficients[3], double R) {
    float Temp = (1 / (Coefficients[0] + (Coefficients[1] * log(R)) + (Coefficients[2] * pow(log(R), 3)))) - 273.15;
    return Temp;
}

/**
 * @brief Calculates the nth root of a number.
 * @param x The number to calculate the nth root of.
 * @param n The degree of the root.
 * @param tolerance The number of decimal places to use for the tolerance. Default is 6.
 * @return The nth root of x.
 */
double NthSqrt(double x, int n, int tolerance) {
    if ((n == 0) || (x == 0)) {
        return 0;
    }
    double lowerBound = min(-1.0, x);
    double upperBound = max(1.0, x);
    double approxRoot = (lowerBound + upperBound) / 2;
    double prevApproxRoot = approxRoot;
    double epsilon = pow(10, -tolerance);
    while (abs(approxRoot - prevApproxRoot) > epsilon) {
        prevApproxRoot = approxRoot;
        approxRoot = ((n - 1) * approxRoot + x / pow(approxRoot, n - 1)) / n;
    }
    return approxRoot;
}



/// @brief
/// @param val
/// @param list single value or list of doubles to be matched against
/// @param derivation how much (in percent) the value may be off to be considered in range
/// @return false without a match or the index + 1 if there was a match
byte inRange(double val, double list[], double derivation, byte listLength) {
    double delta = val * (derivation / 100);
    for (byte i = 0; i < listLength; i++) {
        if ((val < list[i] + delta) && (val > list[i] - delta)) {
            return i + 1;
        }
    }
    return 0;
}

/**
 * @brief Aligns the input string to the specified length.
 * @param in String to align.
 * @param mode char 'R', 'L' or 'C' for Right, Left and Center alignment respectively.
 * @param length Length to stretch the input string to.
 * @param padString Optional - what to pad the string with. Default is a space character.
 * @return The aligned string.
 */
String StringAlign(String in, char mode, byte length, String padString) {
    if (in.length() >= length) {
        in.remove(length);
        return in;
    }
    byte paddingLength = length - in.length();
    String padding = "";
    for (byte i = 0; i < paddingLength; i++) {
        padding += padString;
    }
    switch (mode) {
        case 'R':
            return padding + in;
        case 'L':
            return in + padding;
        case 'C':
            byte leftPaddingLength = paddingLength / 2;
            byte rightPaddingLength = paddingLength - leftPaddingLength;
            return String(leftPaddingLength, padString[0]) + in + String(rightPaddingLength, padString[0]);
        default:
            return in;
    }
}




double gcd(double a, double b) {
    if (a < b) {
        return gcd(b, a);
    }

    // base case
    if (fabs(b) < 0.01) {
        return a;
    } else {
        return (gcd(b, a - floor(a / b) * b));
    }
}