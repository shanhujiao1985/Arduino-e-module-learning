#include <AccelStepper.h>

#define STEP_ANGLE 1.8
#define MICROSTEPS 1

const int enablePin = 8;
const int xdirPin = 2, xstepPin = 5;
const int ydirPin = 3, ystepPin = 6;
const int zdirPin = 4, zstepPin = 7;
const int xLimitPin = 9, yLimitPin = 10, zLimitPin = 11, D12Pin = 12;
const int stepsPerRev = int(360.0 / STEP_ANGLE * MICROSTEPS);

AccelStepper stepper1(1, xstepPin, xdirPin);
AccelStepper stepper2(1, ystepPin, ydirPin);
AccelStepper stepper3(1, zstepPin, zdirPin);

struct FlashTask {
  unsigned long lastToggle = 0;
  int state = 0;
  int count = 0;
  int onTime = 0;
  int offTime = 0;
  int repeat = 0;
  bool active = false;
};

FlashTask ledTask;

void startFlash(int onTime, int offTime, int repeat) {
  ledTask.onTime = onTime;
  ledTask.offTime = offTime;
  ledTask.repeat = repeat * 2;
  ledTask.state = 0;
  ledTask.count = 0;
  ledTask.lastToggle = millis();
  ledTask.active = true;
  digitalWrite(LED_BUILTIN, HIGH);
}

void updateFlash() {
  if (!ledTask.active) return;
  unsigned long now = millis();
  int duration = (ledTask.state % 2 == 0) ? ledTask.onTime : ledTask.offTime;

  if (now - ledTask.lastToggle >= duration) {
    ledTask.state++;
    ledTask.lastToggle = now;
    ledTask.count++;

    if (ledTask.count >= ledTask.repeat) {
      digitalWrite(LED_BUILTIN, LOW);
      ledTask.active = false;
    } else {
      digitalWrite(LED_BUILTIN, ledTask.state % 2 == 0);
    }
  }
}

unsigned long waitUntil1 = 0, waitUntil2 = 0, waitUntil3 = 0;
bool waiting1 = false, waiting2 = false, waiting3 = false;
int direction1 = 1, direction2 = 1, direction3 = 1;

void setup() {
  pinMode(xstepPin, OUTPUT); pinMode(xdirPin, OUTPUT);
  pinMode(ystepPin, OUTPUT); pinMode(ydirPin, OUTPUT);
  pinMode(zstepPin, OUTPUT); pinMode(zdirPin, OUTPUT);
  pinMode(enablePin, OUTPUT); digitalWrite(enablePin, LOW);

  pinMode(xLimitPin, INPUT_PULLUP);
  pinMode(yLimitPin, INPUT_PULLUP);
  pinMode(zLimitPin, INPUT_PULLUP);
  pinMode(D12Pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  stepper1.setMaxSpeed(1200); stepper1.setAcceleration(400);
  stepper2.setMaxSpeed(1200); stepper2.setAcceleration(400);
  stepper3.setMaxSpeed(1200); stepper3.setAcceleration(400);

  stepper1.moveTo(stepsPerRev);
  stepper2.moveTo(stepsPerRev);
  stepper3.moveTo(stepsPerRev);
}

void loop() {
  updateFlash();

  if (digitalRead(xLimitPin) == LOW && !ledTask.active)
    startFlash(1000, 0, 1);

  if (digitalRead(yLimitPin) == HIGH && !ledTask.active)
    startFlash(100, 100, 1);

  if (digitalRead(zLimitPin) == LOW && !ledTask.active)
    startFlash(100, 100, 1);

  if (digitalRead(D12Pin) == LOW && !ledTask.active)
    startFlash(100, 100, 2);

  if (stepper1.distanceToGo() == 0 && !waiting1) {
    waiting1 = true; waitUntil1 = millis() + 1000;
  } else if (waiting1 && millis() >= waitUntil1) {
    direction1 *= -1;
    stepper1.moveTo(stepper1.currentPosition() + direction1 * stepsPerRev);
    waiting1 = false;
  } else {
    stepper1.run();
  }

  if (stepper2.distanceToGo() == 0 && !waiting2) {
    waiting2 = true; waitUntil2 = millis() + 1000;
  } else if (waiting2 && millis() >= waitUntil2) {
    direction2 *= -1;
    stepper2.moveTo(stepper2.currentPosition() + direction2 * stepsPerRev);
    waiting2 = false;
  } else {
    stepper2.run();
  }

  if (stepper3.distanceToGo() == 0 && !waiting3) {
    waiting3 = true; waitUntil3 = millis() + 1000;
  } else if (waiting3 && millis() >= waitUntil3) {
    direction3 *= -1;
    stepper3.moveTo(stepper3.currentPosition() + direction3 * stepsPerRev);
    waiting3 = false;
  } else {
    stepper3.run();
  }
}
