#include "mpu6050.h"
#include "include/kalman.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_timer.h"
#include  "config.h"
static const char *TAG = "kalman";
   
static float X[4][1];//quaternion matrix
static float Z[3];//observation matrix
static float F[4][4];//state transition matrix
static float H[3][4];//observation covariance matrix
static float P[4][4];//covariance matrix
static float Q[4][4];//process noise covariance matrix
static float R[3][3];//measurement noise covariance matrix
static float K[4][3];//Kalman gain matrix  
static double dt; 
float last_time=0;
static float roll;
static float pitch;
static float yaw;

mpu6050_gyro_value_t kalman_receive_data_gyro;//receive data of gyro from mpu6050.c  
mpu6050_acce_value_t kalman_receive_data_acce;//receive data of acc from mpu6050.c
kalman_offset_gyro kalman_offset_data = {0.0f, 0.0f, 0.0f};//initialize offset data to 0.0f
kalman_offset_acc  kalman_offset_data_acc = {0.0f, 0.0f, 0.0f};//initialize offset data to 0.0f
complimentary_angle_t angle_receive;
static mpu6050_handle_t sensor;//define mpu6050 handle(句柄)


//initialize mpu6050 and log the result
void kalman_init(){
    
    //configure I2C master mode and initialize mpu6050
    i2c_param_config(I2C_NUM_0, &(i2c_config_t){
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    });
    //install I2C driver
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_set_pin(I2C_NUM_0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 0, 0, I2C_MODE_MASTER);
    sensor = mpu6050_create(I2C_NUM_0, 0x68);
    if (NULL == sensor) {
        ESP_LOGE(TAG, "mpu6050 create failed");
        return;
    }
    mpu6050_wake_up(sensor);
    //setup mpu6050 with accelerometer full scale range of ±2g and gyroscope full scale range of ±250°/s
    mpu6050_config(sensor, ACCE_FS_2G, GYRO_FS_250DPS);
    ESP_LOGI(TAG, "mpu6050 initialized successfully");
}

void kalman_filter_init(){
    float X_init[4][1] ={
        {1.0f}, 
        {0.0f},
        {0.0f}, 
        {0.0f}
    }; 
    memcpy(X, X_init, sizeof(X_init));

    float F_init[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    memcpy(F, F_init, sizeof(F_init));

    float H_init[3][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f}
    };
    memcpy(H, H_init, sizeof(H_init));

    float P_init[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    memcpy(P, P_init, sizeof(P_init));

    float Q_init[4][4] = {
        {0.01f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.01f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.01f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.01f}
    };
    memcpy(Q, Q_init, sizeof(Q_init));

    float R_init[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    memcpy(R, R_init, sizeof(R_init));
    ESP_LOGI(TAG, "Kalman filter initialized successfully");
}
void offset_init(){
    //calculate gyro offset
    ESP_LOGI(TAG, "Calculating gyro offset, please keep the device stationary...");
    for(int i = 0; i < 1000; i++){
        mpu6050_get_gyro(sensor, &kalman_receive_data_gyro);
        kalman_offset_data.gyro_offset_x += kalman_receive_data_gyro.gyro_x;
        kalman_offset_data.gyro_offset_y += kalman_receive_data_gyro.gyro_y;    
        kalman_offset_data.gyro_offset_z += kalman_receive_data_gyro.gyro_z;

    }
    kalman_offset_data.gyro_offset_x /= 1000.0f;
    kalman_offset_data.gyro_offset_y /= 1000.0f;
    kalman_offset_data.gyro_offset_z /= 1000.0f;
    ESP_LOGI(TAG, "gyro offset calculated successfully"); 
}

void kalman_gyro_real(){
    //calculate gyro real data
    mpu6050_get_gyro(sensor, &kalman_receive_data_gyro);
    kalman_receive_data_gyro.gyro_x -= kalman_offset_data.gyro_offset_x;
    kalman_receive_data_gyro.gyro_y -= kalman_offset_data.gyro_offset_y;
    kalman_receive_data_gyro.gyro_z -= kalman_offset_data.gyro_offset_z;
}

void kalaman_acc_real(){
   mpu6050_get_acce(sensor, &kalman_receive_data_acce);  
}
void filter()
{
    kalman_gyro_real();
    kalaman_acc_real();
    roll= atan2f(kalman_receive_data_acce.acce_y, kalman_receive_data_acce.acce_z) * (180.0f / M_PI);
    pitch= atan2f(-kalman_receive_data_acce.acce_x, sqrtf(kalman_receive_data_acce.acce_y*kalman_receive_data_acce.acce_y + kalman_receive_data_acce.acce_z*kalman_receive_data_acce.acce_z)) * (180.0f / M_PI);
    printf(">Roll:%.2f°, Pitch:%.2f°, Yaw:%.2f°\n", roll, pitch, yaw);
    // mpu6050_complimentory_filter(sensor,&kalman_receive_data_acce,&kalman_receive_data_gyro,&angle_receive);
    // printf(">Roll:%.2f°, Pitch:%.2f°", angle_receive.roll, angle_receive.pitch);
}

void kalman_filter_update()
{
    kalman_gyro_real();
    kalaman_acc_real();

    // ================== 1. 计算 dt ==================
    if (last_time == 0) {
        last_time = esp_timer_get_time();
        return;
    }

    dt = (esp_timer_get_time() - last_time) / 1000000.0f;
    last_time = esp_timer_get_time();

    if (dt < 0.001f || dt > 0.02f)
        dt = 0.01f;

    // ================== 2. 读取gyro ==================
    float gx = kalman_receive_data_gyro.gyro_x * M_PI / 180.0f;
    float gy = kalman_receive_data_gyro.gyro_y * M_PI / 180.0f;
    float gz = kalman_receive_data_gyro.gyro_z * M_PI / 180.0f;

    // 当前四元数
    float q0 = X[0][0];
    float q1 = X[1][0];
    float q2 = X[2][0];
    float q3 = X[3][0];

    // ================== 3. 状态预测（唯一正确方式） ==================
    float dq0 = (-q1*gx - q2*gy - q3*gz) * 0.5f;
    float dq1 = ( q0*gx + q2*gz - q3*gy) * 0.5f;
    float dq2 = ( q0*gy - q1*gz + q3*gx) * 0.5f;
    float dq3 = ( q0*gz + q1*gy - q2*gx) * 0.5f;

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    // 归一化
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    X[0][0]=q0; X[1][0]=q1; X[2][0]=q2; X[3][0]=q3;

    // ================== 4. 计算F（Jacobian） ==================
    float F[4][4] = {
        {1, -gx*dt/2, -gy*dt/2, -gz*dt/2},
        {gx*dt/2, 1, gz*dt/2, -gy*dt/2},
        {gy*dt/2, -gz*dt/2, 1, gx*dt/2},
        {gz*dt/2, gy*dt/2, -gx*dt/2, 1}
    };

    // ================== 5. P预测 ==================
    float P_temp[4][4]={0};
    float FT[4][4];

    // 转置
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            FT[i][j]=F[j][i];

    // P = F P F^T + Q
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            for(int k=0;k<4;k++)
                P_temp[i][j]+=F[i][k]*P[k][j];

    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            P[i][j]=0;
            for(int k=0;k<4;k++)
                P[i][j]+=P_temp[i][k]*FT[k][j];

            P[i][j]+=Q[i][j];
        }
    }

    // ================== 6. 加速度处理 ==================
    float ax = kalman_receive_data_acce.acce_x;
    float ay = kalman_receive_data_acce.acce_y;
    float az = kalman_receive_data_acce.acce_z;
    static float ax_lpf=0, ay_lpf=0, az_lpf=0;
    float alpha = 0.2f;  // 0.1~0.3

    ax_lpf = (1-alpha)*ax_lpf + alpha*kalman_receive_data_acce.acce_x;
    ay_lpf = (1-alpha)*ay_lpf + alpha*kalman_receive_data_acce.acce_y;
    az_lpf = (1-alpha)*az_lpf + alpha*kalman_receive_data_acce.acce_z;

    ax = ax_lpf;
    ay = ay_lpf;
    az = az_lpf;

    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if(acc_norm < 1e-6f) return;

    ax/=acc_norm; ay/=acc_norm; az/=acc_norm;

    // ================== 7. 预测重力 ==================
    float hx = 2*(q1*q3 - q0*q2);
    float hy = 2*(q0*q1 + q2*q3);
    float hz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // ================== 8. H矩阵 ==================
    float H[3][4] = {
        {-2*q2,  2*q3, -2*q0, 2*q1},
        { 2*q1,  2*q0,  2*q3, 2*q2},
        { 2*q0, -2*q1, -2*q2, 2*q3}
    };

    // ================== 9. 自适应R ==================
    float error = fabsf(acc_norm - 1.0f);
    float R_val = 0.02f + 5.0f * error;

    float R[3][3] = {
        {R_val,0,0},
        {0,R_val,0},
        {0,0,R_val}
    };

    // ================== 10. 计算Kalman增益 ==================
    float S[3][3]={0};
    float K[4][3]={0};
    float HT[4][3];

    for(int i=0;i<4;i++)
        for(int j=0;j<3;j++)
            HT[i][j]=H[j][i];

    float HP[3][4]={0};

    for(int i=0;i<3;i++)
        for(int j=0;j<4;j++)
            for(int k=0;k<4;k++)
                HP[i][j]+=H[i][k]*P[k][j];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<4;k++)
                S[i][j]+=HP[i][k]*HT[k][j];

    for(int i=0;i<3;i++)
        S[i][i]+=R[i][i];

    // 简化：对角近似求逆
    float S_inv[3][3]={0};
    for(int i=0;i<3;i++)
        S_inv[i][i]=1.0f/S[i][i];

    float PH[4][3]={0};

    for(int i=0;i<4;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<4;k++)
                PH[i][j]+=P[i][k]*HT[k][j];

    for(int i=0;i<4;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                K[i][j]+=PH[i][k]*S_inv[k][j];

    // ================== 11. 更新 ==================
    float y[3] = {
        ax - hx,
        ay - hy,
        az - hz
    };

    for(int i=0;i<4;i++)
        for(int j=0;j<3;j++)
            X[i][0]+=K[i][j]*y[j];

    // 归一化
    norm = sqrtf(X[0][0]*X[0][0]+X[1][0]*X[1][0]+X[2][0]*X[2][0]+X[3][0]*X[3][0]);
    for(int i=0;i<4;i++)
        X[i][0]/=norm;

    // ================== 12. 输出角度 ==================
    roll  = atan2f(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*180/M_PI;
    pitch = asinf(2*(q0*q2-q3*q1))*180/M_PI;
    yaw   = atan2f(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*180/M_PI;
}


void print(void){

    printf("%.2f,%.2f,%.2f\n", roll, pitch, yaw);
    // 关键：必须强制刷新缓冲区（否则数据可能卡在缓存）
    fflush(stdout);
    // printf(">Roll:%.2f°, Pitch:%.2f°", angle_receive.roll,angle_receive.pitch);
    //printf(">P:%0.2f",P[0][0]);
}


