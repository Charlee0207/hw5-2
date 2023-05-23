#include "mbed.h"
#include "bbcar.h"
#include <string>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>

// #define _DEBUG_
#define CAR_WIDTH 11.1
#define WHEEL_DIAMETER 6.5
#define CAR_ANGLE_2_SINGLE_WHEEL_ANGLE (((CAR_WIDTH)*2)/(WHEEL_DIAMETER)) // CAR_WIDTH*2pi * (carAngle/360) = WHEEL_DIAMETER*pi * (wheelAngle/360)
#define CAR_ANGLE_2_BOTH_WHEEL_ANGLE ((CAR_WIDTH)/(WHEEL_DIAMETER)) // CAR_WIDTH*2pi * (carAngle/360) = WHEEL_DIAMETER*pi * (wheelAngle/360)
#define SERVO_SPEED 35
#define TURN_FILTER_SIZE 8
#define SIGN_FILTER_SIZE 90
#define ELAPSED_TIME 2
inline void DEBUG_PRINT(int &curPattern, string printContent){
    #ifdef _DEBUG_
        printf("%d %d %d %d", (curPattern&0b1000)!=0, (curPattern&0b0100)!=0, (curPattern&0b0010)!=0, (curPattern&0b0001)!=0);
        cout << printContent << "\n";
    #endif
};
Timer t;
Ticker servo_ticker;
Ticker servo_feedback_ticker;
DigitalOut LED(LED2);
PwmIn servo0_f(D10), servo1_f(D13);
PwmOut servo0_c(D11), servo1_c(D12);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);
BusInOut qti_pin(D4,D5,D6,D7);

int main() {
    parallax_qti qti1(qti_pin);

    queue<int> turnFIFO, signFIFO;
    int preTurnPattern, preSignPattern, curPattern;
    bool start=false, stop=false, inCircle=false;
    int signFlag=-1;
    int turnServo0Angle=0, turnServo1Angle=0, straightAngle=0;
    int preServo0Angle, preServo1Angle, curServo0Angle, curServo1Angle;
    float turnFactor=(WHEEL_DIAMETER*3.14)/720, straightFactor=WHEEL_DIAMETER*3.14/360;
    float distance;

    for(int i=0; i<TURN_FILTER_SIZE; i++) turnFIFO.push(curPattern);
    for(int i=0; i<SIGN_FILTER_SIZE; i++) signFIFO.push(curPattern);

    while(1) {
        curPattern = (int)qti1;
        
        preTurnPattern = turnFIFO.front();
        preSignPattern = signFIFO.front();
        turnFIFO.push(curPattern);
        signFIFO.push(curPattern);
        turnFIFO.pop();
        signFIFO.pop();
        
        if(start){
            curServo0Angle = abs(car.servo0.angle); curServo1Angle = abs(car.servo1.angle);
            if(curPattern==0b0001 || curPattern==0b0011 || (curPattern==0b0111&&!inCircle) || (curPattern==0b1110&&inCircle)){
                turnServo0Angle += curServo0Angle-preServo0Angle;
                preServo0Angle = curServo0Angle;
            }
            else if((curPattern==0b0111&&inCircle) || (curPattern==0b1110&&!inCircle) || curPattern==0b1100 || curPattern==0b1000){
                turnServo1Angle += curServo1Angle-preServo1Angle;
                preServo1Angle = curServo1Angle;
            }
            else{
                straightAngle += (curServo0Angle-preServo0Angle + curServo1Angle-preServo1Angle + 1)/2;
                preServo0Angle = curServo0Angle;
                preServo1Angle = curServo1Angle;
            }
        }

        switch (curPattern) {
             
            case 0b0001:  
                car.turn(SERVO_SPEED*1.23, -0.1);        DEBUG_PRINT(curPattern, "large right");
                break;
            case 0b0011:  
                car.turn(SERVO_SPEED*1.13, -0.2);        DEBUG_PRINT(curPattern, "right");
                break;
            case 0b0111:  
                if(inCircle) car.turn(SERVO_SPEED*1.2, 0.1);           
                else car.turn(SERVO_SPEED*1.23, -0.1);            DEBUG_PRINT(curPattern, "slight right");
                break;
            case 0b0110:  
                if(preTurnPattern==0b0110){
                    car.goStraight(SERVO_SPEED*0.85);        DEBUG_PRINT(curPattern, "");
                }
                break;
            case 0b1110:  
                if(inCircle) car.turn(SERVO_SPEED*1.21, -0.1);           
                else car.turn(SERVO_SPEED*1.2, 0.1);         DEBUG_PRINT(curPattern, "slight left");
                break;
            case 0b1100:  
                car.turn(SERVO_SPEED*1.1, 0.2);         DEBUG_PRINT(curPattern, "left");
                break;
            case 0b1000:  
                car.turn(SERVO_SPEED*1.2, 0.1);         DEBUG_PRINT(curPattern, "large left");
                break;

            case 0b1001: case 0b1101: case 0b1011: 
                if(preTurnPattern==0b1101){
                    car.turn(SERVO_SPEED*1.2, 0.1);    DEBUG_PRINT(curPattern, "intersection, turn large left");
                }
                else
                    car.turn(SERVO_SPEED*1.2, -0.1);    DEBUG_PRINT(curPattern, "intersection, turn large right");
                break;
    
            case 0b1111:
                car.goStraight(SERVO_SPEED); 
                if(preSignPattern==0b1111){
                    if(!start){
                        t.start();
                        LED = 1;
                        start = true;
                        preServo0Angle=abs(car.servo0.angle); preServo1Angle=abs(car.servo1.angle);
                        cout << "start!\n";
                    }
                    if(start && chrono::duration_cast<chrono::seconds>(t.elapsed_time()).count() > ELAPSED_TIME){
                        stop = true;
                        LED = 0;
                        car.stop();
                        cout << "stop\n"; 
                        break;
                    }
                }
                else inCircle = true;
                break;
            default: 
                car.goStraight(SERVO_SPEED);
                break;
        }
        

        
        if(stop) break;
        // ThisThread::sleep_for(1ms);
    }

    distance = turnFactor * (turnServo0Angle+turnServo1Angle) + straightFactor * straightAngle;
    cout << "distance: " << distance << "\n";
}