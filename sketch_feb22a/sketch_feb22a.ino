#include "./hsv.hpp";

long MIX_AFTER_FORCE_MILLIS = 1000;
long DYE_AFTER_FORCE_MILLIS = 10;

long lastForceSensorTime;

#define pinMotorController 3

rgb currentColor; // doubles between 0 and 1

void setup() {
  // put your setup code here, to run once:
  lastForceSensorTime = -10000000;
  currentColor.r = 0.0;
  currentColor.g = 0.0;
  currentColor.b = 1.0;

  pinMode(pinMotorController, OUTPUT);

}

bool shouldRunMixer() {
  return (millis() > (lastForceSensorTime + MIX_AFTER_FORCE_MILLIS));
}

bool shouldRunDye() {
  return (millis() > (lastForceSensorTime + DYE_AFTER_FORCE_MILLIS));
}

bool getForceSensor() {
  return true;
}


void updateForceSensorTime() {
  if (getForceSensor()) {
    lastForceSensorTime = millis();
  }
}

rgb getColor() {
  hsv color;
  // TODO Get hue and saturation from pots
  color.v = 0.0;
  return hsv2rgb(color);
}

void runMixer() {
  // TODO
}

void turnOffMixer() {
  // TODO
}

void runDye(rgb color) {
 // TODO
  
}

void turnOffDye() {
  // TODO
}

void updateLED(rgb color) {
  // TODO Set LED to color
}

void loop() {
  updateForceSensorTime();
  rgb color = getColor();

  if (shouldRunMixer()) {
    runMixer();
  } else {
    turnOffMixer();
  }

  if (shouldRunDye()) {
    runDye(color);
  } else {
    turnOffDye();
  }

  updateLED(color);
}
