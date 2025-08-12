#include <PID_v1.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// Pins
const int ldrPin = 34;
const int redPin = 25;
const int greenPin = 26;
const int bluePin = 27;

// Sensor & Output Variables
double input;

// Setpoints for PID mode
double setpointRed = 600;
double setpointGreen = 500;
double setpointBlue = 400;

// PID outputs
double outputRed, outputGreen, outputBlue;

// PID tuning
double Kp = 1.0, Ki = 0.1, Kd = 0.05;

// PID objects
PID pidRed(&input, &outputRed, &setpointRed, Kp, Ki, Kd, DIRECT);
PID pidGreen(&input, &outputGreen, &setpointGreen, Kp, Ki, Kd, DIRECT);
PID pidBlue(&input, &outputBlue, &setpointBlue, Kp, Ki, Kd, REVERSE);

// System states
bool systemOn = true;
bool manualMode = false;
int intensity = 255;
int manualR = 0, manualG = 0, manualB = 0;

enum ColorMode { RGB_MODE, WHITE_MODE };
ColorMode colorMode = RGB_MODE;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("HHS");

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pidRed.SetMode(AUTOMATIC);
  pidGreen.SetMode(AUTOMATIC);
  pidBlue.SetMode(AUTOMATIC);

  pidRed.SetOutputLimits(0, 255);
  pidGreen.SetOutputLimits(0, 255);
  pidBlue.SetOutputLimits(0, 255);
}

void loop() {
  handleBluetooth();

  if (systemOn) {
    input = analogRead(ldrPin);
    input = map(input, 0, 4095, 0, 1023);

    if (colorMode == WHITE_MODE) {
      if (!manualMode) {
        // AUTO WHITE: based on LDR
        const int threshold = 512;
        if (input > threshold) {
          outputRed = outputGreen = outputBlue = 0;
        } else {
          outputRed = outputGreen = outputBlue = 255;
        }
      } else {
        // MANUAL WHITE
        outputRed = outputGreen = outputBlue = manualR;
      }
    } else {
      if (!manualMode) {
        // AUTO RGB (PID)
        pidRed.Compute();
        pidGreen.Compute();
        pidBlue.Compute();
      } else {
        // MANUAL RGB
        outputRed = manualR;
        outputGreen = manualG;
        outputBlue = manualB;
      }
    }

    // Smooth transitions
    static double smoothRed = 0, smoothGreen = 0, smoothBlue = 0;
    smoothRed = 0.9 * smoothRed + 0.1 * outputRed;
    smoothGreen = 0.9 * smoothGreen + 0.1 * outputGreen;
    smoothBlue = 0.9 * smoothBlue + 0.1 * outputBlue;

    analogWrite(redPin, (int)(smoothRed * intensity / 255));
    analogWrite(greenPin, (int)(smoothGreen * intensity / 255));
    analogWrite(bluePin, (int)(smoothBlue * intensity / 255));

    // Debug
    Serial.printf("LDR: %d | R:%d G:%d B:%d\n", (int)input,
                  (int)(smoothRed * intensity / 255),
                  (int)(smoothGreen * intensity / 255),
                  (int)(smoothBlue * intensity / 255));
  } else {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
  }

  delay(50);
}

void handleBluetooth() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("ON")) {
      systemOn = true;
      SerialBT.println("System ON");

    } else if (cmd.equalsIgnoreCase("OFF")) {
      systemOn = false;
      SerialBT.println("System OFF");

    } else if (cmd.startsWith("INT:")) {
      int val = cmd.substring(4).toInt();
      intensity = constrain(val, 0, 255);
      SerialBT.printf("Intensity set to %d\n", intensity);

    } else if (cmd.equalsIgnoreCase("AUTO")) {
      manualMode = false;
      manualR = manualG = manualB = 0;
      SerialBT.println("Switched to AUTO mode. Manual values cleared.");

    } else if (cmd.equalsIgnoreCase("MODE:RGB")) {
      colorMode = RGB_MODE;
      SerialBT.println("Mode set to RGB");

    } else if (cmd.equalsIgnoreCase("MODE:WHITE")) {
      colorMode = WHITE_MODE;
      SerialBT.println("Mode set to WHITE");

    } else if (cmd.startsWith("MANUAL:")) {
      String valStr = cmd.substring(7);

      if (colorMode == WHITE_MODE && valStr.indexOf(',') == -1) {
        int val = constrain(valStr.toInt(), 0, 255);
        manualR = manualG = manualB = val;
        manualMode = true;
        SerialBT.printf("Manual WHITE set to %d (R=G=B)\n", val);
      } else {
        int r = 0, g = 0, b = 0;
        int firstComma = valStr.indexOf(',');
        int secondComma = valStr.indexOf(',', firstComma + 1);

        if (firstComma > 0 && secondComma > firstComma) {
          r = valStr.substring(0, firstComma).toInt();
          g = valStr.substring(firstComma + 1, secondComma).toInt();
          b = valStr.substring(secondComma + 1).toInt();

          r = constrain(r, 0, 255);
          g = constrain(g, 0, 255);
          b = constrain(b, 0, 255);

          manualR = r;
          manualG = g;
          manualB = b;
          manualMode = true;

          SerialBT.printf("Manual RGB set to R:%d G:%d B:%d\n", r, g, b);
        } else {
          SerialBT.println("Invalid MANUAL format. Use MANUAL:R,G,B or MANUAL:X in WHITE mode.");
        }
      }

    } else {
      SerialBT.println("Unknown command. Valid: ON, OFF, INT:X, AUTO, MODE:RGB, MODE:WHITE, MANUAL:X or MANUAL:R,G,B");
    }
  }
}
