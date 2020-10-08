// This code is used for driving the robot on a line

#include <AFMotor.h>

#define set_point 2000
#define max_speed 80 //set Max Speed Value
// this is for setting pid parames
float kp = 0.02 ;
float ki = 0    ;
float kd = 0    ;

AF_DCMotor motorfr(1);
AF_DCMotor motorfl(4);
AF_DCMotor motorbl(3);
AF_DCMotor motorbr(2);

int command = 0;
int speed = 160;
bool not_turning = true;
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
    Serial1.begin(9600);
    Serial.println("Setup done ");
}

bool detect_left_90_degree()
{
    if (sensors[4] < 50 && sensors[3] < 500 && sensors[2] < 50)
        return true;
    else
        return false;
}

bool detect_right_90_degree()
{
    if (sensors[0] < 50 && sensors[1] < 50 && sensors[2] < 50)
        return true;
    else
        return false;
}

bool detect_t_junction()
{
    if (sensors[0] < 50 && sensors[1] < 50 && sensors[2] < 50 && sensors[3] < 500 && sensors[4] < 50)
        return true;
    else
        return false;
}

void forward()
{

    motorfr.setSpeed(speed);
    motorfl.setSpeed(speed);
    motorbl.setSpeed(speed);
    motorbr.setSpeed(speed);
    motorfr.run(FORWARD);
    motorfl.run(FORWARD);
    motorbl.run(FORWARD);
    motorbr.run(FORWARD);
}

void backward()
{

    motorfr.setSpeed(speed);
    motorfl.setSpeed(speed);
    motorbl.setSpeed(speed);
    motorbr.setSpeed(speed);

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

int ad = 50;
void left()
{
    motorfr.setSpeed(speed + ad);
    motorfl.setSpeed(speed + ad);
    motorbl.setSpeed(speed + ad);
    motorbr.setSpeed(speed + ad);

    motorfr.run(FORWARD);
    motorfl.run(BACKWARD);
    motorbl.run(BACKWARD);
    motorbr.run(FORWARD);
    delay(700);
}

void right()
{

    motorfr.setSpeed(speed + ad);
    motorfl.setSpeed(speed + ad);
    motorbl.setSpeed(speed + ad);
    motorbr.setSpeed(speed + ad);

    motorfr.run(BACKWARD);
    motorfl.run(FORWARD);
    motorbl.run(FORWARD);
    motorbr.run(BACKWARD);
    delay(700);
}
void pid_calc()
{

    proportional = Position - set_point;
    integral = integral + proportional;
    derivative = proportional - last_proportional;
    last_proportional = proportional;
    error_value = int(proportional * kp + integral * ki + derivative * kd);
}
void calc_turn()
{
    //Restricting the error value between +256.
    Serial1.println("E:" + String(error_value) + " l: " + String(left_speed) + " r: " + String(right_speed));
    Serial.println("E:" + String(error_value) + " l: " + String(left_speed) + " r: " + String(right_speed));
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
    motorfr.run(FORWARD);
    motorfl.run(FORWARD);
    motorbl.run(FORWARD);
    motorbr.run(FORWARD);
    delay(100);
}

long sensors_adv = 0.0;

int exec_serial_commands()
{
    command = 0;
    if (Serial1.available())
    {
        command = Serial1.parseInt();
    }
    if (Serial.available())
    {
        command = Serial.parseInt();
    }
    run_command(command);
    return command;
}

void run_command(long com)
{
    if (com == 0)
        return;

    if (com > 1000 && com <= 2000)
    {
        setkp(com-1000);
    }
    

    if (com > 2000 && com <= 3000)
    {
        setki(com-2000);
    }
    
    if (com > 3000 && com <= 4000)
    {
        setkd(com-3000);
    }
    
    command = 0;
}

void setkp(int com)
{
    kp = com / 10000.0;
    Serial1.println("Kp : " + String(kp));
    Serial.println("Kp : " + String(kp));
}

void setki(int com)
{
    ki = com / 10000.0;
    Serial1.println("Ki : " + String(ki));
    Serial.println("Ki : " + String(ki));
}
void setkd(int com)
{

    kd = com / 10000.0;
    Serial1.println("Kd : " + String(kd));
    Serial.println("Kd : " + String(kd));
}

void run_navigation_commands(int com)
{

    com = com - 400;
    switch (com)
    {

    case 1:
        print_message("forward");
        forward();
        break;

    case 2:
        print_message("left");
        left();
        break;

    case 3:
        print_message("Stop");
        Stop();
        break;

    case 4:
        print_message("right");
        right();
        break;

    case 5:
        print_message("reverse");
        backward();
        break;
    }
}

void print_message(String msg)
{
  Serial1.println(msg);
}

void loop()
{

    exec_serial_commands();
    // Serial1.println("[" + String(sensors[0]) + "," + String(sensors[1]) + "," + String(sensors[2]) + "," + String(sensors[3]) + "," + String(sensors[4]) + "] :" + "Sum > " + String(sensors_sum));

    sensors_sum = 0;
    sensors_adv = 0;

    for (int i = 0; i <= 4; i++)
    {
        sensors[i] = analogRead(i);
        sensors_adv += sensors[i] * (i)*1000.0;
        //Serial.println("adv : " + String(sensors_adv));
        sensors_sum += sensors[i];
    }

    if (sensors_sum > 4400 && not_turning)
    {
        // Serial.println("derailed");
        Stop();
    }
    else if (sensors_sum > 3500 && sensors_sum <= 4000)
    {
        forward();
        not_turning = true;
        // Serial.println("moving forward");
    }

    if (sensors[2] > 50)
    {
        Position = int(sensors_adv / sensors_sum);
        // Serial1.println(" POS : " + String(Position) + " adv " + String(sensors_adv));
        // Serial.println(" POS : " + String(Position) + " adv " + String(sensors_adv));
        pid_calc();
        calc_turn();
        motor_drive(right_speed, left_speed);
    }
    if (detect_t_junction())
    {
        // Serial.println("detect_t_junction");
        Stop();
    }
    else if (detect_left_90_degree())
    {
        // Serial.println("detect_left_90_degree");
        left();
        not_turning = false;
    }

    else if (detect_right_90_degree())
    {
        // Serial.println("detect_right_90_degree");
        right();
        not_turning = false;
    }
}
