// This code is used for driving the robot on a line 


#include <AFMotor.h>

#define set_point 2000
#define max_speed 200 //set Max Speed Value
// this is for setting pid parames
#define Kp 0 //set Kp Value
#define Ki 0 //set Ki Value
#define Kd 0 //set Kd Value

AF_DCMotor left_motor(1);
AF_DCMotor right_motor(4);
int command = 0;

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
    left_motor.setSpeed(255);
    right_motor.setSpeed(255);
    left_motor.run(RELEASE);
    right_motor.run(RELEASE);
    Serial.begin(9600);
}

void forward()
{
    left_motor.run(FORWARD);
    right_motor.run(FORWARD);
    Stop();

}

void backward()
{
    left_motor.run(BACKWARD);
    right_motor.run(BACKWARD);
}

void Stop()

{
    left_motor.run(RELEASE);
    right_motor.run(RELEASE);
    delay(500);
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

    
    left_motor.setSpeed(left_speed);
    right_motor.setSpeed(right_speed);
    left_motor.run(FORWARD);
    right_motor.run(FORWARD);
    delay(100);
}

void loop()
{
    sensors_sum = 0;

    for (int i = 0; i <= 4; i++)
    {
        sensors[i] = analogRead(i);
        sensors_sum += sensors[i];
    }
    Serial.println("[" + String(sensors[0])+ "," + String(sensors[1])+ ","+ String(sensors[2])+ ","+ String(sensors[3])+ ","+ String(sensors[4])+ "] :" + "Sum > " + String(sensors_sum));
    if (sensors_sum < 4000 )
    {
        forward();
    }
    if (sensors_sum >= 4000)
    {
        Stop();
    }

    // if (sensors_sum < 4000 && sensors_sum > 0)
    // {
    //     Position = int(sensors_average / sensors_sum);
    //     pid_calc();
    //     calc_turn();
    //     motor_drive(right_speed, left_speed);
    // }

    delay(500);
}
