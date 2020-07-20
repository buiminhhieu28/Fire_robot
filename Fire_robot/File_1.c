#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include<Servo.h>
#define flame_pin A0
#define flame_pin_1 A1
#define Max 185 
#define Min -185
#define set 100    
#define R 1
#define L 0
#define T 1000
int in1 = 4, in2 = 5, in3 = 6, in4 = 7, ena = 3, enb = 11, tri = 2, echo = 10;
int p_in1 = 12, p_in2 = 13, p_en = A4;
int servopin = 9, servopin1 = A2;
int delay_set_for_servo = 300; // thoi gian cho 1 pha quay
double d;
int time_one_angle = 7;



int Length[3];
Servo servo;
Servo servo1;
float error = 0, setpoint = 24, u_t = 0;
float sample_time = 1;       // Khai báo thời gian lấy mẫu dt

double  Kp = 5.0;

int vR, vL;
int angle_x, angle_y;
int blue_val;

void setup()
{
    Serial.begin(9600);
    pinMode(p_in1, OUTPUT);
    pinMode(p_in2, OUTPUT);
    pinMode(p_en, OUTPUT);

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);

    pinMode(tri, OUTPUT);
    pinMode(echo, INPUT);

    servo.attach(servopin);
    servo.write(90);
    servo1.attach(A2);
    servo1.write(90);
}

double Range()
{ //do khoang cach bang sonal
    digitalWrite(tri, 0);
    delayMicroseconds(2);
    digitalWrite(tri, 1);
    delayMicroseconds(10);
    digitalWrite(tri, 0);
    delayMicroseconds(2);
    double time_back = pulseIn(echo, HIGH, 10000);
    double range = time_back / 58.0;
    if (range == 0) {
        range = 100;
    }
    delay(100);
    return range;
}

int check(int speed)
{
    return speed > 0 ? 1 : 2;
}

void control(int motor, int speed)
{ //ham dieu khien dong co chung
    int d = check(speed);
    if (motor == 0) {
        if (d == 1) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            analogWrite(ena, speed);

        }
        else {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            analogWrite(ena, abs(speed));

        }
    }

    else {
        if (d == 1) {
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(enb, speed);
        }
        else {
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            analogWrite(enb, abs(speed));
        }
    }
}


void water_motor(int k, int v)
{
    if (k == 0)
    {
        digitalWrite(12, 0);
        digitalWrite(13, 0);
    }
    else if (k == 1)
    {
        digitalWrite(12, 1);
        digitalWrite(13, 0);
        analogWrite(A4, v);
    }
}

void Control_1() { // dieu khien cach vat mot doan
    if (u_t < -255)
    {
        u_t = -255;
    }
    else if (u_t < 0)
    {
        u_t *= Max / 10;
    }
    else if (u_t <= 45 && u_t >= 40)
    { // hàm chỉnh sửa
        u_t = set;
    }
    else if (u_t > Max)
    {
        u_t = Max;
    }
    control(0, (int)u_t);
    control(1, (int)u_t);
}

void distance_more() { //do khoang cach tung phia

    servo.write(0);
    Length[1] = Range();
    delay(delay_set_for_servo);
    servo.write(180);
    Length[2] = Range();
    delay(delay_set_for_servo);
    servo.write(90);
}

int check_d()
{                                     // kiem tra nên re hướng nào
    if (Length[1] >= Length[2])
    {
        return 1;                   // phai
    }
    else
    {
        return 2;                   // trai
    }
}

void compute_PID()
{                           //PID setting
    u_t = Kp * error;
}

void Control_2(int n)
{                                  //dieu khien re trai re phai
    int vl, vr;
    if (n == 1)
    {
        vl = 255;
        vr = -255;
        // có thể tinh chỉnh
    }
    else if (n == 2)
    {
        vl = -255;
        vr = 255;
        // tinh chỉnh lại cho chính xác
    }
    control(0, (int)vl);
    control(1, (int)vr);
    delay(T);
    servo.write(90);
}

void randomseed() { // ngẫu nhiên chạy và ngẫu nhiên chết (random run and random die)
    int i = 0;
    long k = random(150, 255);
    randomSeed(1);
    control(0, k);
    control(1, -k);
    delay(200);

    control(0, 0);
    control(1, 0);
    delay(10);
    while (Length[0] >= 100 && i <= 10)
    {
        control(0, Max);
        control(1, Max);
        delay(50);
        Length[0] = Range();
        i++;
    }
}


void loop()
{
    if (digitalRead(flame_pin) == 0 && digitalRead(flame_pin_1) == 1)
    {
        water_motor(0, 0);
        servo.write(90);
        Length[0] = Range();

        servo.write(90);
        Length[0] = Range();
        error = Length[0] - setpoint;
        if (error <= 4 && error >= -4)
        {                                              // có thể tinh chỉnh cho chính xác
            control(0, 0);
            control(1, 0);
            delay(1);
            distance_more(); // đo lại thế giới
            int m = check_d(); // quyết định di chuyển
            Control_2(m); // rẽ đi đâu đó ???
        }
        compute_PID(); // PID controler
        Control_1(); // di chuyển đại pháp
        delay(sample_time);
    }
    else  if (digitalRead(A0) == 1 || digitalRead(A1) == 0)
    {
        control(0, 200);
        control(1, 200);
        delay(100);
        control(0, 0);
        control(1, 0);
        int i = 0;
        water_motor(1, 30);
        Serial.println("Dangerous");
        delay(500);
        while (digitalRead(A0) == 1 || digitalRead(A1) == 0)
        {
            for (int j = 65; j <= 115; j++)
            {
                servo1.write(j);
                for (int i = 50; i <= 255; i += 10)
                {
                    water_motor(1, i);
                    delay(2);
                }
                for (int i = 255; i >= 150; i -= 10)
                {
                    water_motor(1, i);
                    delay(2);
                }
            }

            for (int j = 115; j >= 65; j--)
            {
                servo1.write(j);
                for (int i = 50; i <= 255; i += 10)
                {
                    water_motor(1, i);
                    delay(2);
                }
                for (int i = 255; i >= 150; i -= 10)
                {
                    water_motor(1, i);
                    delay(2);
                }
            }
            control(0, 0);
            control(1, 0);
            delay(250);
            servo1.write(90);
        }
    }
}