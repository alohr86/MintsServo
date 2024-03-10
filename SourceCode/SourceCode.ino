#include "Motor.hpp"

#include "PID_v1.h"
#include "AS5600.h"

class L293N : public Motor {
  private:
    uint8_t IN1, IN2;

    void forward() {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }

    void backward() {
      digitalWrite(IN2, HIGH);
      digitalWrite(IN1, LOW);
    }

    void stop() {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  public:
    L293N(uint8_t pwm, uint8_t IN1, uint8_t IN2) :
      Motor(pwm),
      IN1(IN1),
      IN2(IN2) {}

    void begin() {
      pinMode(pwm, OUTPUT);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

    void set(int speed) {
      if(speed < 0) {
        backward();
      } else if(speed > 0) {
        forward();
      } else {
        stop();
      }
      analogWrite(pwm, abs(speed * 2.55));
    }
};

L293N motor(6, 3, 2);

AS5600 encoder;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.4, Ki=0.2, Kd=0.001;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  motor.begin();
  Serial.begin(9600);

  Wire.begin();

  encoder.begin(4);
  encoder.setDirection(AS5600_CLOCK_WISE);
  while(!encoder.isConnected()) {
    Serial.println("Connecting...");
    delay(2000);
  }
  encoder.resetPosition();

  Input = encoder.getCumulativePosition();

  Serial.println(Input * AS5600_RAW_TO_DEGREES);
  delay(2000);

  Setpoint = 720;

  myPID.SetMode(AUTOMATIC);

  myPID.SetOutputLimits(-100, 100);
}

void loop() {
  // put your main code here, to run repeatedly:
  Input = encoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES / 3;
  if(abs(Input - Setpoint) > 2.0) {
    myPID.Compute();
  } else {}
  Serial.print(Input);
  Serial.print("\t");
  Serial.print(Output);
  Serial.print("\t");
  Serial.println(Setpoint);
  motor.set(-Output);
}
