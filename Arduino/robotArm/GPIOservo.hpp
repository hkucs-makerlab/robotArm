#ifndef __GPIO_SERVO__
#define __GPIO_SERVO__
#include <Servo.h>
class GPIOservo {
  private:

  public:
    GPIOservo(): angle(0), rev(false), attached(false), pinIndex(-1) {

    }
    GPIOservo(int pinIndex ): GPIOservo() {
      this->pinIndex = pinIndex;
#if defined(ESP32)
      this->attach(); // calling in constructor not working in AVR mcu
#endif
    }

    GPIOservo(int pinIndex, int min, int max ): GPIOservo(pinIndex) {
      gpioServoMin = min;
      gpioServoMax = max;
    }

    bool attach(uint8_t pinIndex) {
      this->pinIndex = pinIndex;
      return attach();
    }

    bool attach(int minPulse, int maxPulse ) {
      this->gpioServoMin = minPulse;
      this->gpioServoMax = maxPulse;
      return attach();
    }

    bool attach(uint8_t pinIndex, int minPulse, int maxPulse ) {
      this->pinIndex = pinIndex;
      this->gpioServoMin = minPulse;
      this->gpioServoMax = maxPulse;
      return attach();
    }

    bool attach() {
      if (pinIndex < 0) return false;
      if (attached) {
        gpioServos.detach();
      }
      gpioServos.attach(pinIndex, gpioServoMin, gpioServoMax);
      gpioServos.write(angle);
      prevTime = millis();
      return attached = true;
    }

    void detach() {
      if (attached) {
        gpioServos.detach();
        attached = false;
      }
    }

    void write(int angle) {
      if (!attached) {
        return;
      }
      gpioServos.write(angle);
      if (angle != this->angle) this->angle = angle;
    }

    void writeMicroseconds(long us) {
      if (!attached) {
        return;
      }
      int deg = map(us, gpioServoMin, gpioServoMax, 0, 180);
      if (deg != this->angle) this->angle = deg;
      gpioServos.writeMicroseconds(us);
    }

    int getAngle() {
      return angle;
    }

    bool move(int targetAngle) {
      if (angle == targetAngle)  {
        return true;
      }
      currentTime = millis();
      if (currentTime - prevTime >= angleTimeGap) {
        long diffTime = currentTime - prevTime;
        int diffAngle = diffTime / angleTimeGap;
        prevTime = currentTime;
        this->write(angle);
        if (targetAngle < angle) {
          angle -= diffAngle;
          if (angle < 0) angle = 0;
        } else {
          angle += diffAngle;
          if (angle > 180) angle = 180;
        }
      }
      return false;
    }

    void sweep() {
      currentTime = millis();
      if (currentTime - prevTime >= angleTimeGap) {
        long diffTime = currentTime - prevTime;
        int diffAngle = diffTime / angleTimeGap;
        prevTime = currentTime;
        this->write(angle);
        if (rev) {
          angle -= diffAngle;
          if (angle < 0) {
            rev = false;
            angle = 0;
          }
        } else {
          angle += diffAngle;
          if (angle > 180) {
            rev = true;
            angle = 180;
          }
        } //rev
      }
    }

  private:
    int pinIndex;
    int gpioServoMin = 550;
    int gpioServoMax = 2550;
    //const int gpioServoMax = 2350;
    Servo gpioServos;
    //
    const short angleTimeGap = 5;
    int angle;
    bool rev;
    long prevTime, currentTime;
    //
    bool attached;
};
#endif
