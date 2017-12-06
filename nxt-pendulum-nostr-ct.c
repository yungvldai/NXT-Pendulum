void go(int speed);
float dif(float now,float past);

//+----+-------+-------+------+-----------------+
//| ## | const | value | unit |      desc.      |
//+----+-------+-------+------+-----------------+
#define    L       0  //  cm  | pendulum length |
//+----+-------+-------+------+-----------------+
#define    k1      0  //  any | coefficient     |
//+----+-------+-------+------+-----------------+
#define    k2      0  //  any | coefficient     |
//+----+-------+-------+------+-----------------+
#define    k3      0  //  any | coefficient     |
//+----+-------+-------+------+-----------------+
#define   delay   30  //  ms  | cycle delay     |
//+----+-------+-------+------+-----------------+

int dt = 0;

task main(){
     SetSensorLowspeed(S1);
     ResetSensorHTAngle(S1, HTANGLE_MODE_CALIBRATE);
     float rpm,abs_angle,alg_angle,avg_mrc;
     // k2*` îò MRC + k3*` îò Sen + k1*Sen
     float motor_rot_cnt_now = 0,motor_rot_cnt_past = 0;
     float sensor_now = 0,sensor_past;
     float Fx = 0;
     int st = CurrentTick();
     while(1){
         dt = CurrentTick() - st;
         avg_mrc =
         (MotorRotationCount(OUT_A) + MotorRotationCount(OUT_B)) / 2;
         motor_rot_cnt_past = motor_rot_cnt_now;
         motor_rot_cnt_now = avg_mrc;
         ResetSensorHTAngle(S1, HTANGLE_MODE_CALIBRATE);
         ReadSensorHTAngle(S1, abs_angle,alg_angle,rpm);
         sensor_past = sensor_now;
         sensor_now = abs_angle;
         Fx = k1 * sensor_now +
              k2 * dif(motor_rot_cnt_now,motor_rot_cnt_past) +
              k3 * dif(sensor_now,sensor_past);
         go(Fx);
         Wait(delay);
     }
}

float dif(float now,float past){
    return (now - past) / dt;
}

void go(int speed){
     OnFwd(OUT_A,speed);
     OnFwd(OUT_B,-speed);
}
