struct diff_val{
    float val;
    float lval;
};

#define dv diff_val

void go(int speed);
void set_v(dv tmp,float val);
float dif(dv tmp);

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

int dt = 0;

task main(){
     SetSensorLowspeed(S1);
     ResetSensorHTAngle(S1, HTANGLE_MODE_CALIBRATE);
     float rpm,abs_angle,alg_angle,avg_mrc;
     // k2*` îò MRC + k3*` îò Sen + k1*Sen
     dv motor_rot_cnt;
     dv sensor;
     motor_rot_cnt.val = sensor.val = 0;
     float Fx = 0;
     int st = CurrentTick();
     while(1){
         dt = CurrentTick() - st;
         avg_mrc =
         (MotorRotationCount(OUT_A) + MotorRotationCount(OUT_B)) / 2;
         set_v(motor_rot_cnt,avg_mrc);
         ResetSensorHTAngle(S1, HTANGLE_MODE_CALIBRATE);
         ReadSensorHTAngle(S1, abs_angle,alg_angle,rpm);
         set_v(sensor,abs_angle);
         Fx = k1 * sensor.val +
              k2 * dif(motor_rot_cnt) +
              k3 * dif(sensor);
         go(Fx);
         Wait(1);
     }
}

float dif(dv tmp){
    return (tmp.val - tmp.lval) / dt;
}

void set_v(dv tmp,float val){
     tmp.lval = tmp.val;
     tmp.val = val;
}

void go(int speed){
     OnFwd(OUT_A,speed);
     OnFwd(OUT_B,-speed);
}
