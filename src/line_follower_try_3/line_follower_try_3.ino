// This code is used for driving the robot on a line

#include <AFMotor.h>

#define set_point 2000
#define max_speed 160 //set Max Speed Value
// this is for setting pid parames
#define Kp 0 //set Kp Value
#define Ki 0 //set Ki Value
#define Kd 0 //set Kd Value

AF_DCMotor motorfr(1);
AF_DCMotor motorfl(2);
AF_DCMotor motorbl(3);
AF_DCMotor motorbr(4);

int command = 0;
int speed = 160;

int proportional = 0;
int integral = 0;
int derivative = 0;
int last_proportional = 0;
int right_speed = 0;
int left_speed = 0;
int sensors_sum = 0;
int sensors_average = 0;
int sensors[5] = {0, 0, 0, 0, 0};
int Position = 0;
int error_value = 0;

void setup()
{

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

bool detect_left_90_degree()
{
    if (sensors[4] > 900 && sensors[3] > 900 && sensors[2]>900)
        return true;
    else
        return false;
}

bool detect_right_90_degree()
{
    if (sensors[0] > 900 && sensors[1] > 900 && sensors[2]>900)
        return true;
    else
        return false;
}

bool detect_t_junction()
{
    if (sensors[0] > 900 && sensors[1] > 900 && sensors[2]>900 && sensors[3] >900 && sensors[4]>900)
        return true;
    else
        return false;
}


void forward()
{

  motorfr.run(FORWARD);
  motorfl.run(FORWARD);
  motorbl.run(FORWARD);
  motorbr.run(FORWARD);
}

void backward()
{

  motorfr.run(BACKWARD);
  motorfl.run(BACKWARD);
  motorbl.run(BACKWARD);
  motorbr.run(BACKWARD);
}
void Stop()

{
  motorfr.run(RELEASE);
  motorfl.run(RELEASE);
  motorbl.run(RELEASE);
  motorbr.run(RELEASE);
}

void pid_calc()
{

    proportional = Position - set_point;
    integral = integral + proportional;
    derivative = proportional - last_proportional;
    last_proportional = proportional;
    error_value = int(proportional * Kp + integral * Ki + derivative * Kd);
}
void calc_turn()
{
    //Restricting the error value between +256.
    Serial.print("Error value :");
    Serial.println(error_value);
    if (error_value < -256)
    {
        error_value = -256;
    }
    if (error_value > 256)
    {
        error_value = 256;
    }
    // If error_value is less than zero calculate right turn speed values
    if (error_value < 0)
    {
        right_speed = max_speed + error_value;
        left_speed = max_speed;
    }
    else
    {
        right_speed = max_speed;
        left_speed = max_speed - error_value;
    }
}

void motor_drive(int right_speed, int left_speed)
{
    if (right_speed > 255)
        right_speed = 255;
    if (right_speed < 0)
        right_speed = 0;
    if (left_speed > 255)
        left_speed = 255;
    if (left_speed < 0)
        left_speed = 0;

    motorfl.setSpeed(left_speed);
    motorbl.setSpeed(left_speed);

    motorbr.setSpeed(right_speed);
    motorfr.setSpeed(right_speed);
    delay(100);
}

void loop()
{
    Serial.println("[" + String(sensors[0]) + "," + String(sensors[1]) + "," + String(sensors[2]) + "," + String(sensors[3]) + "," + String(sensors[4]) + "] :" + "Sum > " + String(sensors_sum));
    if (Serial.available() > 0)
    {
        command = Serial.parseInt();
        if (command <= 0)
        {
            return;
        }
        if (command == 1)
        {
            Serial.println("Activated");
        }
        else
        {
            Serial.println("Deactivated");
        }
    }

    int sensors_adv = 0;
    sensors_sum = 0;

    for (int i = 0; i <= 4; i++)
    {
        sensors[i] = analogRead(i);
        sensors_adv += sensors[i] * (i) * 1000;
        sensors_sum += sensors[i];
    }

    if (sensors_sum > 4000)
    {
        Serial.println("derailed");
        Stop();
    }
    else if (sensors_sum > 3500 && sensors_sum < 4000)
    {
        forward();
        Serial.println("moving forward");
    }

     if (sensors_sum <= 3500 && sensors_sum > 0)
     {
         Position = int(sensors_average / sensors_sum);
         pid_calc();
         calc_turn();
         motor_drive(right_speed, left_speed);
     }
     if (detect_left_90_degree())
     {
         Serial.println("detect_left_90_degree");
     }

     if (detect_right_90_degree())
     {
         Serial.println("detect_right_90_degree");
     }
     if (detect_t_junction())
     {
         Serial.println("detect_t_junction");
     }

     delay(500);
}
