#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>
#define BLACK 1
#define WHITE 0
#define north 0
#define east 1
#define west 2
#define south 3

#if 1
void init_sensors(void){
    reflectance_start();
    Ultra_Start();  
    IR_Start();
    motor_start();      
    motor_forward(0,0);
    reflectance_set_threshold(15000, 15000, 15000, 15000, 15000, 15000);
}
void get_start_time(){
    TickType_t start;
    motor_forward(0,0);
    print_mqtt("Zumo10/ready", "maze");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt("Zumo10/start", "%d", start);
}
void get_end_time(void){
    TickType_t end;
    motor_forward(0,0);
    motor_stop();
    end = xTaskGetTickCount();
    print_mqtt("Zumo101/stop","%d", end);
}
void left_then_right(void){
    SetMotors(1, 0, 100, 100, 260); 
    motor_forward(100,580);
    SetMotors(0, 1, 100, 100, 260);
}
void right_then_left(void){
    SetMotors(0, 1, 100, 100, 260);
    motor_forward(100,580);
    SetMotors(1, 0, 100, 100, 260);
}
struct robot{
    int x;
    int y;
    int direction;
} robot_coordinate = {0, -1, north};
struct sensors_ dig;
void maze(void){
    int last_choice[10];
    if (robot_coordinate.x == 0){
        left_then_right();
        robot_coordinate.x--;
        last_choice[0]=0;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    else if (robot_coordinate.x == -1) {
        left_then_right();
        robot_coordinate.x--;
        last_choice[1] = -1;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    else if (robot_coordinate.x == -2) {
        left_then_right();
        robot_coordinate.x--;
        last_choice[2] = -2;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    else if (robot_coordinate.x == -3) {
        right_then_left();
        robot_coordinate.x++;
        last_choice[3] = -3;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    if (robot_coordinate.x == -2 && last_choice[3] == -3) {
        right_then_left();
        robot_coordinate.x++;
        last_choice[4] = -2;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    if ((robot_coordinate.x == -1) && (last_choice[4] == -2)){
        right_then_left();
        robot_coordinate.x++;
        last_choice[5] = -1;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    if (robot_coordinate.x == 0 && last_choice[5] == -1) {
        right_then_left();
        robot_coordinate.x++;
        last_choice[6] = 0;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    else if (robot_coordinate.x == 1 && last_choice[6] == 0) {
        right_then_left();
        robot_coordinate.x++;
        last_choice[7] = 1;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    else if (robot_coordinate.x == 2 && last_choice[7] == 1) {
        right_then_left();
        robot_coordinate.x++;
        last_choice[8] = 2;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
    else if (robot_coordinate.x == 3 && last_choice[8] == 2) {
        right_then_left();
        robot_coordinate.x++;
        print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
    }
}
void increment_y(void){
    robot_coordinate.y++;
    print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
}
void increment_x(void){
    robot_coordinate.x++;
    print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
}
void decrease_x(void){
    robot_coordinate.x--;
    print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
}
void coordinate_at_start(void){
    robot_coordinate.x = 0;
    robot_coordinate.y = -1;
    print_mqtt("Robot coordinate:", "(%d, %d)", robot_coordinate.x, robot_coordinate.y);
} 
void adjust_coordinate_when_at_rear(void){
    if ((dig.R3 == 1 && dig.L3 == 0) && (robot_coordinate.direction == north) && (robot_coordinate.x == -3)){
        increment_y();
        while (dig.R3 == 1 && dig.L3 == 0){
            reflectance_digital(&dig);
            motor_forward(70,0);    
        }
    }
    if ((dig.R3 == 0 && dig.L3 == 1) && (robot_coordinate.direction == north) && (robot_coordinate.x == 3)){
        increment_y();
        while (dig.R3 == 0 && dig.L3 == 1){
            reflectance_digital(&dig);
            motor_forward(70,0);    
        }
    }
}
void adjust_coordinate_to_x0(void){
    if ((robot_coordinate.y == 11) && (robot_coordinate.x <0)){
        right_then_left();
        increment_x();
    }
    if ((robot_coordinate.y == 11) && (robot_coordinate.x >0)){
        left_then_right();
        decrease_x();
    }
}
void go_when_see_black(void){
    while (dig.R3 == 1 && dig.L3 == 1){
        reflectance_digital(&dig);
        motor_forward(70,0);    
    }
}
void zmain(){
    init_sensors();
    while (SW1_Read()== 1){
    vTaskDelay(100);
    }
    int count = 0;
    TickType_t end;
    TickType_t start;
    while(true){
        reflectance_digital(&dig);
        int d = Ultra_GetDistance();
        motor_forward(70,0);
        if ((dig.L3 == 1) && (dig.R3 == 1)){ //when 6 sensors see black, robot counts 1 intersection passed
            count++;
            if ((dig.R3 == 1 && dig.L3 == 1) && (robot_coordinate.direction == north)){
                increment_y();
                go_when_see_black();
            }
            if (count == 1){
                get_start_time();
                start = xTaskGetTickCount();
                coordinate_at_start();
            }
        }
        if(d<7){
            maze();
        }
        adjust_coordinate_when_at_rear();   //controll robot when x at -3 or 3
        adjust_coordinate_to_x0();  //guide robot back to x at 0 when near the end of the map
        if (dig.L3 == 0 && dig.R3 == 0 && dig.L1 == 0 && dig.R1 ==0 && robot_coordinate.y == 13){   //When all 6 sensors see white, stop robot
            get_end_time(); 
            end = xTaskGetTickCount();
            TickType_t time = end - start;
            print_mqtt("Zumo101/time","%d", time);
            break;
        }
    }
}

#endif
