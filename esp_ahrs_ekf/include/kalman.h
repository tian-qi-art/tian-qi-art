#ifndef KALMEN_H
#define KALMEN_H

typedef struct 
{
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
}kalman_offset_gyro; //offset of acc and gyro

typedef struct 
{
    float acc_offset_x;
    float acc_offset_y;
    float acc_offset_z;
}kalman_offset_acc; //offset of acc and gyr


void kalman_init();
void kalman_filter_init();
void offset_init();
void kalman_gyro_real();
void kalaman_acc_real();
void kalman_filter_update();
void filter();
void print(void);

#endif