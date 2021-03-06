
#include <AFMotor.h>

AF_DCMotor motorfr(1);
AF_DCMotor motorfl(2);
AF_DCMotor motorbl(3);
AF_DCMotor motorbr(4);
int command = 0 ;
int speed = 160;
void setup() {

  motorfr.setSpeed(speed);
  motorfl.setSpeed(speed);
  motorbl.setSpeed(speed);
  motorbr.setSpeed(speed);
  
  motorfr.run(RELEASE);
  motorfl.run(RELEASE);
  motorbl.run(RELEASE);
  motorbr.run(RELEASE);
  Serial.begin(9600); 
  Serial.println("Setup done ");
}

void forward()
{

  motorfr.run(FORWARD);
  motorfl.run(FORWARD);
  motorbl.run(FORWARD);
  motorbr.run(FORWARD);

  delay(2500);
  motorfr.run(RELEASE);
  motorfl.run(RELEASE);
  motorbl.run(RELEASE);
  motorbr.run(RELEASE);
  delay(200);

}

void backward()
{

  motorfr.run(BACKWARD);
  motorfl.run(BACKWARD);
  motorbl.run(BACKWARD);
  motorbr.run(BACKWARD);

  delay(2500);
  motorfr.run(RELEASE);
  motorfl.run(RELEASE);
  motorbl.run(RELEASE);
  motorbr.run(RELEASE);
  delay(200);
}

void loop()
{

  if (Serial.available() > 0)
  {
    command = Serial.parseInt();
    if (command <= 0)
    {
      return;
    }
    if (command == 1)
    {
      forward();
      Serial.println("moving forward");
    }
    else if(command == 2)
    {
      backward();
      Serial.println("moving backward");
    }
  }
}
