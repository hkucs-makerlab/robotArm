#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>
#include "pinout.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "command.h"
#include "GoBLE.hpp"

//
#define __ZUP '1'
#define __ZDOWN '2'
#define __STEPER_ON '3'
#define __STEPER_OFF '4'
#define __GRIPPER_ON '5'
#define __GRIPPER_OFF '6'
#define __FORWARD 'f'
#define __BACKWARD 'b'
#define __LEFT 'l'
#define __RIGHT 'r'
#define __CENTER 'c'
#define __HALT 'a'
#define __HOME 'h'
#define __END_STOP 'e'
#define __REST 't'
#define __BOTTOM 'o'

String M17 = "M17 "; //stepper on
String M18 = "M18 "; //stepper off
String M3 = "M3 "; //grepper on
String M5 = "M5 "; //grepper off
String M106 = "M106 ";     //fan on
String M107 = "M107 ";     //fan off
String G1 = "G1 X";   // move steppers
double xpos = 0, ypos = 120, zpos = 120;
bool stepperEnable = false;

#define Console Serial
#define BlueTooth Serial1
_GoBLE<HardwareSerial, HardwareSerial> Goble(BlueTooth, Console);

const int xDeltaInterval=10;
const int yDeltaInterval=10;
const int zDeltaInterval=10;

Stepper stepper(2400, STEPPER_GRIPPER_PIN_0, STEPPER_GRIPPER_PIN_1, STEPPER_GRIPPER_PIN_2, STEPPER_GRIPPER_PIN_3);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
RampsStepper stepperExtruder(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN);
FanControl fan(FAN_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Command command;

Servo servo;
int angle = 170;
int angle_offset = 0; // offset to compensate deviation from 90 degree(middle position)
// which should gripper should be full closed.

void setup() {
  while(!Serial || !Serial1);
  Goble.begin(9600);
  //Console.begin(115200);
  
  //various pins..
  pinMode(HEATER_0_PIN  , OUTPUT);
  pinMode(HEATER_1_PIN  , OUTPUT);
  pinMode(LED_PIN       , OUTPUT);

  //unused Stepper..
  pinMode(E_STEP_PIN   , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN , OUTPUT);

  //unused Stepper..
  pinMode(Q_STEP_PIN   , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN , OUTPUT);

  //GripperPins
  pinMode(STEPPER_GRIPPER_PIN_0, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_3, OUTPUT);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);

  //  vaccum motor control
  pinMode(MOTOR_IN1  , OUTPUT);
  pinMode(MOTOR_IN2  , OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  servo.attach(SERVO_PIN);
  servo.write(angle + angle_offset);


  //reduction of steppers..
  stepperHigher.setReductionRatio(32.0 / 9.0, 200 * 16);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
  stepperLower.setReductionRatio( 32.0 / 9.0, 200 * 16);
  stepperRotate.setReductionRatio(32.0 / 9.0, 200 * 16);
  stepperExtruder.setReductionRatio(32.0 / 9.0, 200 * 16);

  //start positions..
  stepperHigher.setPositionRad(PI / 2.0);  //90°
  stepperLower.setPositionRad(0);          // 0°
  stepperRotate.setPositionRad(0);         // 0°
  stepperExtruder.setPositionRad(0);

  //enable and init..
  setStepperEnable(false);
  interpolator.setInterpolation(0, 120, 120, 0, 0, 120, 120, 0);

  Console.println("started");
}

void loop () {
  static unsigned long prevTime = 0;
  static unsigned long curTime;
  
  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad (geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  stepperExtruder.stepToPositionRad(interpolator.getEPosmm());
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update();
  fan.update();

  if (millis() % 500 < 250) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  char cmd;
  if (check_goble(&cmd)) {
    prevTime = 0;
  }
  curTime = millis();
  if (curTime - prevTime > 100) {
    String gcode;
    prevTime = curTime;
    if (interpolator.isFinished()) {
      bool execute = true;
      switch (cmd) {
        case __HOME:
          xpos = 0, ypos = 120, zpos = 120;
          gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          break;
        case __BOTTOM:
          xpos = 0, ypos = 100, zpos = 0;
          gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          break;
        case __REST:
          xpos = 0, ypos = 40, zpos = 70;
          gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          break;
        case __END_STOP:
          xpos = 0, ypos = 19.5, zpos = 134;
          gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          break;
        case __FORWARD:
          if (ypos + yDeltaInterval <= 190) {
            ypos += yDeltaInterval;
            gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          } else
            execute = false;
          break;
        case __BACKWARD:
          if (ypos - yDeltaInterval >= 19) {
            ypos -= yDeltaInterval;
            gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          }
          else
            execute = false;
          break;
        case __RIGHT:
          if (xpos + xDeltaInterval  <= 150) {
            xpos += xDeltaInterval;
            gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          } else
            execute = false;
          break;
        case __LEFT:
          if (xpos - xDeltaInterval >= -150) {
            xpos -= xDeltaInterval;
            gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          } else
            execute = false;
          break;
        case __ZUP:
          if (zpos + zDeltaInterval <= 200) {
            zpos += zDeltaInterval;
            gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          } else
            execute = false;
          break;
        case __ZDOWN:
          if (zpos - zDeltaInterval >= -110) {
            zpos -= zDeltaInterval;
            gcode = G1 + String(xpos) + " Y" + String(ypos) + " Z" + String(zpos);
          }
          else
            execute = false;
          break;
        case __STEPER_ON:
          gcode = M17;
          break;
        case __STEPER_OFF:
          gcode = M18;
          break;
        case __GRIPPER_ON:
          gcode = M3 + String("T10");
          break;
        case __GRIPPER_OFF:
          gcode = M5 + String("T10");
          break;
        default:
          execute = false;
          break;
      }
      if (execute ) {
        Console.println(gcode);
        command.processMessage(gcode);
        executeCommand(command.getCmd());
        printOk();
      }
    }
  } // if execute time


  if (interpolator.isFinished() && command.handleGcode()) {
    executeCommand(command.getCmd());
    printOk();
  }
}

boolean check_goble(char *output) {
  bool rc = false;
  static char cmd = __HALT;

  int joystickX = 0;
  int joystickY = 0;

  if (Goble.available()) {
    rc = true;
    joystickX = Goble.readJoystickX();
    joystickY = Goble.readJoystickY();

    if (joystickX > 190) {
      cmd =  __FORWARD;
    } else if (joystickX < 80) {
      cmd = __BACKWARD;
    } else if (joystickY > 190) {
      cmd =  __RIGHT;
    } else if (joystickY < 80) {
      cmd = __LEFT;
    } else
      cmd = __HALT;

    if (Goble.readSwitchUp() == PRESSED) {
      cmd = __ZUP;
    } else if (Goble.readSwitchDown() == PRESSED) {
      cmd = __ZDOWN;
    } else if (Goble.readSwitchLeft() == PRESSED) {
      cmd = __END_STOP;
    } else if (Goble.readSwitchRight() == PRESSED) {
      cmd = __BOTTOM;
    } else if (Goble.readSwitchAction() == PRESSED) {
      cmd = __REST;
    } else if (Goble.readSwitchSelect() == PRESSED) {
      cmd = __HOME;
    } else if (Goble.readSwitchMid() == PRESSED) {
      cmd = __HOME;
    } else if (Goble.readSwitchStart() == PRESSED) {
      if (stepperEnable) {
        cmd = __STEPER_OFF; // to turn off
      } else {
        cmd = __STEPER_ON; // to turn on
      }
    }
  }
  *output = cmd;
  return rc;
}


void setStepperEnable(bool enable) {
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);
  stepperExtruder.enable(enable);
  fan.enable(enable);
  stepperEnable = enable;
}

void cmdMove(Cmd (&cmd)) {
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
}
void cmdDwell(Cmd (&cmd)) {
  delay(int(cmd.valueT * 1000));
}
void cmdGripperOn(Cmd (&cmd)) {
  //Serial.print("Gripper on ");

  // vaccum griiper
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  // servo gripper
  if (0) {
    int diff = int(cmd.valueT);
    if ((angle - diff) >= 90) {
      angle -= diff;
      servo.write(angle + -angle_offset);
      //Serial.print(diff);
      //Serial.print(", ");
    }
    //Serial.println(angle);
  }
  servo.write(90 + -angle_offset);

  // stepper gripper
  stepper.setSpeed(5);
  stepper.step(int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
}
void cmdGripperOff(Cmd (&cmd)) {
  //Serial.print("Gripper off ");

  // vaccum griiper
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  // servo gripper
  if (0) {
    int diff = int(cmd.valueT);
    if ((angle + diff) <= 180) {
      angle += diff;
      servo.write(angle + angle_offset);
      //Serial.print(diff);100
      //Serial.print(", ");
    }
    //Serial.println(angle);
  }
  servo.write(170 + -angle_offset);

  // stepper gripper
  stepper.setSpeed(5);
  stepper.step(-int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
}
void cmdStepperOn() {
  setStepperEnable(true);
}
void cmdStepperOff() {
  setStepperEnable(false);
}
void cmdFanOn() {
  fan.enable(true);
}
void cmdFanOff() {
  fan.enable(false);
}

void handleAsErr(Cmd (&cmd)) {
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)");
  printFault();
}

void executeCommand(Cmd cmd) {
  if (cmd.id == -1) {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }

  if (cmd.valueX == NAN) {
    cmd.valueX = interpolator.getXPosmm();
  }
  if (cmd.valueY == NAN) {
    cmd.valueY = interpolator.getYPosmm();
  }
  if (cmd.valueZ == NAN) {
    cmd.valueZ = interpolator.getZPosmm();
  }
  if (cmd.valueE == NAN) {
    cmd.valueE = interpolator.getEPosmm();
  }

  //decide what to do
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0: cmdMove(cmd); break;
      case 1: cmdMove(cmd); break;
      case 4: cmdDwell(cmd); break;
      //case 21: break; //set to mm
      //case 90: cmdToAbsolute(); break;
      //case 91: cmdToRelative(); break;
      //case 92: cmdSetPosition(cmd); break;
      default: handleAsErr(cmd);
    }
  } else if (cmd.id == 'M') {
    switch (cmd.num) {
      //case 0: cmdEmergencyStop(); break;
      case 3: cmdGripperOn(cmd); break;
      case 5: cmdGripperOff(cmd); break;
      case 17: cmdStepperOn(); break;
      case 18: cmdStepperOff(); break;
      case 106: cmdFanOn(); break;
      case 107: cmdFanOff(); break;
      default: handleAsErr(cmd);
    }
  } else {
    handleAsErr(cmd);
  }
}


