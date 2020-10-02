#include <AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
int command = 0;

void setup()
{
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  Serial.begin(9600);
}

void forward()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);

  delay(4000);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(200);
}

void backward()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);

  delay(4000);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
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
    }
    else if(command == 2)
    {
      backward();
    }
    
    
  }
}
