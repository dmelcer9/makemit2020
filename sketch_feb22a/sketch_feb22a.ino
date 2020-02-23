#include <Servo.h>

#define mixAfterExtrudeMillis 1000
#define dyeAfterExtrudeMillis 10

#define pinExtrudeSensor 2 // TODO
#define pinHuePot A0 // TODO
#define pinSaturationPot A1 //TODO

#define pinDyeRedMotorController 3
#define pinMixerMotorController 5

#define pinDyeGreenServo 6
#define pinDyeBlueServo 7

#define mixerMotorOff 0
#define mixerMotorFullSpeed 254

#define dyeMotorOff 0
#define dyeMotorFullSpeed 254

#define servoMin 0
#define servoMax 60

#define redLEDPin 9
#define greenLEDPin 10
#define blueLEDPin 11

typedef struct rgb {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct hsv {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

unsigned long lastExtrudeSensorTime = 0;
bool extrudeSensorEverPressed = false;

Servo green;
Servo blue;

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);
  pinMode(pinExtrudeSensor, INPUT_PULLUP);
  pinMode(pinHuePot, INPUT);
  pinMode(pinSaturationPot, INPUT);

  pinMode(pinDyeRedMotorController, OUTPUT);
  pinMode(pinMixerMotorController, OUTPUT);
  green.attach(pinDyeGreenServo);
  blue.attach(pinDyeBlueServo);

  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  Serial.println("Setup");
}


bool shouldRunMixer() {
  return extrudeSensorEverPressed && (millis() < (lastExtrudeSensorTime + mixAfterExtrudeMillis));
}

bool shouldRunDye() {
  return extrudeSensorEverPressed && (millis() < (lastExtrudeSensorTime + dyeAfterExtrudeMillis));
}

bool getExtrudeSensor() {
  bool pressed = digitalRead(pinExtrudeSensor);
  if(pressed == LOW) {
    //Serial.println("Extruder pressed");
    extrudeSensorEverPressed = true;
    return true;
  }
  //Serial.println("Extruder not pressed");

  return false;
}


void updateExtrudeSensorTime() {
  if (getExtrudeSensor()) {
    lastExtrudeSensorTime = millis();
  }
}

rgb getFrostingColor() {
  hsv color;
  color.h = float(analogRead(pinHuePot)) * 360.0 / 1023.0;
  color.s = 1.0;
  color.v = (float(analogRead(pinSaturationPot)) / 1023.0);
  return hsv2rgb(color);
}

rgb getLEDColor() {
  hsv color;
  color.h = float(analogRead(pinHuePot)) * 360.0 / 1023.0;
  color.s = (float(analogRead(pinSaturationPot)) / 1023.0);
  color.v = 1.0;
  return hsv2rgb(color);
}

void runMixer() {
  analogWrite(pinMixerMotorController, mixerMotorFullSpeed);
}

void turnOffMixer() {
  analogWrite(pinMixerMotorController, mixerMotorOff);
}

void runDye(rgb color) {

  float dyeRedVal = map_float(color.r, 0.0, 1.0, dyeMotorOff, dyeMotorFullSpeed);
  float dyeGreenVal = map_float(color.g, 0.0, 1.0, servoMin, servoMax);
  float dyeBlueVal = map_float(color.b, 0.0, 1.0, servoMin, servoMax);
  
  analogWrite(pinDyeRedMotorController, (int) dyeRedVal);
  green.write((int) dyeGreenVal);
  blue.write((int) dyeBlueVal);
}

void turnOffDye() {
  analogWrite(pinDyeRedMotorController, dyeMotorOff);
}

void updateLED(rgb color) {
  Serial.println("---");
  Serial.println(color.r);
  Serial.println(color.g);
  Serial.println(color.b);
  analogWrite(redLEDPin, color.r);
  analogWrite(greenLEDPin, color.g);
  analogWrite(blueLEDPin, color.b);
}

void loop() {

  updateExtrudeSensorTime();

  if (shouldRunMixer()) {
    runMixer();
  } else {
    turnOffMixer();
  }

  if (shouldRunDye()) {
    runDye(getFrostingColor());
  } else {
    turnOffDye();
  }

  updateLED(getLEDColor());
}
