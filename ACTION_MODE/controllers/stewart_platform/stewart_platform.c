#include <math.h>
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/joystick.h>

#define TIME_STEP 64
#define NUM_PISTONS 6

static WbDeviceTag pistons[NUM_PISTONS];

static int i;
static double t = 0.0;

double target_position = 0.0;
double target_yaw = 0.0;

double P [6] = { 0.0, 2.474, 0.0, 0.0, 0.0, 0.0};
double L [6] = {2.842663,2.842663,2.842663,2.842663,2.842663,2.842663};
double L_original [6] ={2.842663,2.842663,2.842663,2.842663,2.842663,2.842663};
double TARGET_P [6];

static void find_tp(){
  for (i = 0; i < NUM_PISTONS; i++) {
    TARGET_P[i] = L[i] - L_original[i];
  }
}


static void find_devices() {
  for (i = 0; i < NUM_PISTONS; i++) {
    char name[64];
    sprintf(name, "piston%d", i);
    pistons[i] = wb_robot_get_device(name);
  }
}

static void position(int p) {
      target_position = p*0.1;
      for (i = 0; i< NUM_PISTONS; i++)
        wb_motor_set_position(pistons[i],target_position);
}


static void vibrate(double a) {
  for (t = 0; t < 0.2*a*a; t += TIME_STEP / 1000.0) {
    const double FREQ = 10/a;  // Hz
    const double AMPL = 0.05*a*a;
    double phase = 2 * M_PI * FREQ * t;
    for (i = 0; i < NUM_PISTONS; i++)
      wb_motor_set_position(pistons[i], AMPL * sin(phase));

    wb_robot_step(TIME_STEP);
  }
}
  


static void jump() {
  for (t = 0; t < 0.2; t += TIME_STEP / 1000.0) {
    const double FREQ = 2.5;  // Hz
    const double AMPL = -0.3;
    double phase = 2 * M_PI * FREQ * t;
    for (i = 0; i < NUM_PISTONS; i++)
      wb_motor_set_position(pistons[i], AMPL * sin(phase));

    wb_robot_step(TIME_STEP);
  }
}

static void delay(double a) {
  for (t = 0; t < a; t += TIME_STEP / 1000.0) {
    
    wb_robot_step(TIME_STEP);
  }
}

void ik (const double position[6], double L[6])
{
  static const double b[24] = {-1.57453, 0.0, 1.20951, 1.0, 1.57453, 0.0,
                               1.20951, 1.0, 1.83473, 0.0, 0.75883,  1.0,
                               0.260201, 0.0, -1.96834,  1.0, 0-.260201,  0.0,
                               -1.96834, 1.0, -1.83473,   0.0, 0.75883, 1.0};
  static const double init_b[24] = {
      -0.1956, 0.0, 0.9667, 1.0, 0.1956, 0.0, 0.9667, 1.0, 0.9344, 0.0, -0.3133,  1.0,
      0.739,  0.0, -0.6533,  1.0, -0.739,  0.0, -0.6533, 1.0, -0.9344,  0.0, -0.3133, 1.0};
  double final_p[24];
  double d_RZ_tmp[16];
  double RY_tmp[9];
  double b_RZ_tmp[9];
  double c_RZ_tmp[9];
  double RX_tmp;
  double RZ_tmp;
  double absxk;
  double b_RX_tmp;
  double scale;
  double t;
  int RZ_tmp_tmp;
  int absxk_tmp;
  int i;

  RX_tmp = sin(position[3]);
  b_RX_tmp = cos(position[3]);
  scale = sin(position[4]);
  absxk = cos(position[4]);
  t = sin(position[5]);
  RZ_tmp = cos(position[5]);

  b_RZ_tmp[0] = RZ_tmp;
  b_RZ_tmp[3] = -t;
  b_RZ_tmp[6] = 0.0;
  b_RZ_tmp[1] = t;
  b_RZ_tmp[4] = RZ_tmp;
  b_RZ_tmp[7] = 0.0;
  RY_tmp[0] = absxk;
  RY_tmp[3] = 0.0;
  RY_tmp[6] = scale;
  b_RZ_tmp[2] = 0.0;
  RY_tmp[1] = 0.0;
  b_RZ_tmp[5] = 0.0;
  RY_tmp[4] = 1.0;
  b_RZ_tmp[8] = 1.0;
  RY_tmp[7] = 0.0;
  RY_tmp[2] = -scale;
  RY_tmp[5] = 0.0;
  RY_tmp[8] = absxk;
  for (i = 0; i < 3; i++) {
    t = b_RZ_tmp[i];
    RZ_tmp = b_RZ_tmp[i + 3];
    RZ_tmp_tmp = (int)b_RZ_tmp[i + 6];
    for (absxk_tmp = 0; absxk_tmp < 3; absxk_tmp++) {
      c_RZ_tmp[i + 3 * absxk_tmp] =
          (t * RY_tmp[3 * absxk_tmp] + RZ_tmp * RY_tmp[3 * absxk_tmp + 1]) +
          (double)RZ_tmp_tmp * RY_tmp[3 * absxk_tmp + 2];
    }
  }
  RY_tmp[0] = 1.0;
  RY_tmp[3] = 0.0;
  RY_tmp[6] = 0.0;
  RY_tmp[1] = 0.0;
  RY_tmp[4] = b_RX_tmp;
  RY_tmp[7] = -RX_tmp;
  RY_tmp[2] = 0.0;
  RY_tmp[5] = RX_tmp;
  RY_tmp[8] = b_RX_tmp;
  for (i = 0; i < 3; i++) {
    t = c_RZ_tmp[i];
    RZ_tmp = c_RZ_tmp[i + 3];
    scale = c_RZ_tmp[i + 6];
    for (RZ_tmp_tmp = 0; RZ_tmp_tmp < 3; RZ_tmp_tmp++) {
      b_RZ_tmp[i + 3 * RZ_tmp_tmp] =
          (t * RY_tmp[3 * RZ_tmp_tmp] + RZ_tmp * RY_tmp[3 * RZ_tmp_tmp + 1]) +
          scale * RY_tmp[3 * RZ_tmp_tmp + 2];
    }
  }
  for (i = 0; i < 3; i++) {
    RZ_tmp_tmp = i << 2;
    d_RZ_tmp[RZ_tmp_tmp] = b_RZ_tmp[3 * i];
    d_RZ_tmp[RZ_tmp_tmp + 1] = b_RZ_tmp[3 * i + 1];
    d_RZ_tmp[RZ_tmp_tmp + 2] = b_RZ_tmp[3 * i + 2];
  }
  d_RZ_tmp[12] = position[0];
  d_RZ_tmp[13] = position[1];
  d_RZ_tmp[14] = position[2];
  d_RZ_tmp[3] = 0.0;
  d_RZ_tmp[7] = 0.0;
  d_RZ_tmp[11] = 0.0;
  d_RZ_tmp[15] = 1.0;
  for (i = 0; i < 4; i++) {
    t = d_RZ_tmp[i];
    RZ_tmp = d_RZ_tmp[i + 4];
    scale = d_RZ_tmp[i + 8];
    absxk = d_RZ_tmp[i + 12];
    for (RZ_tmp_tmp = 0; RZ_tmp_tmp < 6; RZ_tmp_tmp++) {
      absxk_tmp = RZ_tmp_tmp << 2;
      final_p[i + absxk_tmp] = ((t * b[absxk_tmp] + RZ_tmp * b[absxk_tmp + 1]) +
                                scale * b[absxk_tmp + 2]) +
                               absxk * b[absxk_tmp + 3];
    }
  }
  for (RZ_tmp_tmp = 0; RZ_tmp_tmp < 6; RZ_tmp_tmp++) {
    scale = 3.3121686421112381E-170;
    absxk_tmp = RZ_tmp_tmp << 2;
    absxk = fabs(final_p[absxk_tmp] - init_b[absxk_tmp]);
    if (absxk > 3.3121686421112381E-170) {
      RZ_tmp = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      RZ_tmp = t * t;
    }
    absxk = fabs(final_p[absxk_tmp + 1] - init_b[absxk_tmp + 1]);
    if (absxk > scale) {
      t = scale / absxk;
      RZ_tmp = RZ_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      RZ_tmp += t * t;
    }
    absxk = fabs(final_p[absxk_tmp + 2] - init_b[absxk_tmp + 2]);
    if (absxk > scale) {
      t = scale / absxk;
      RZ_tmp = RZ_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      RZ_tmp += t * t;
    }
    absxk = fabs(final_p[absxk_tmp + 3] - init_b[absxk_tmp + 3]);
    if (absxk > scale) {
      t = scale / absxk;
      RZ_tmp = RZ_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      RZ_tmp += t * t;
    }
    L[RZ_tmp_tmp] = scale * sqrt(RZ_tmp);

  }
  }
  
  void set(){
    find_tp();
  for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
  }



// main函数
int main() {
  wb_robot_init();  // 初始化Webots
  find_devices();  
  printf("Click the 3D viewer to strat controll.\n");

   wb_joystick_enable(TIME_STEP);     
        //开始模拟视频里的动作
        delay(1);
        vibrate(1);
        delay(0.5);
        vibrate(1);
        delay(0.5);
        position(-4);
        delay(3);
        position(0);
        vibrate(2); 
        delay(0.3);
        
        P[2] = 0.8;
        ik(P,L);
        set();
        delay(1);
        position(0);
        
        delay(1);
        double P1 [6] = { 0, 2.374, -0.2, 0.0, 0.0, 0.0};
        ik(P1,L);        
        set();
        delay(0.05);
        double P2 [6] = { 0, 2.174, -0.2, 0.0, 0.0, 0.0};
        ik(P2,L);
        set();
        delay(0.2);
        double P3 [6] = { 0, 2.474, -0.1, 0.0, 0.0, 0.0};
        ik(P3,L);
        set();
        delay(0.1);
        position(0);
        delay(0.5);
        vibrate(0.5);
        delay(1.5);
        
        double P4 [6] = { 0, 2.474, -0.2, 0.0, 0.0, 0.0};
        ik(P4,L);
        set();
       delay(0.05);
        double P5 [6] = { 0, 2.574, -0.25, 0.0, 0.0, 0.0};
        ik(P5,L);
        set();
        delay(0.05);
        double P6 [6] = { 0, 2.674, -0.3, 0.0, 0.0, 0.0};
        ik(P6,L);
        set();
        delay(0.3);
        
        position(0);
        
          
      //开始手柄控制模式   
      //此模式键位对应的是守望先锋里末日铁拳的键位         
   delay(1);
      for (t = 0; ; t += TIME_STEP / 1000.0) {
        int a0 = wb_joystick_get_axis_value(0);
        int a1 = wb_joystick_get_axis_value(1);               
        int a4 = wb_joystick_get_axis_value(4);     
        int a5 = wb_joystick_get_axis_value(5);                    
        int b = wb_joystick_get_pressed_button();
        printf("\n");
        printf("%d\n", a0);
        printf("%d\n", a1);
        printf("%d\n", a4);
        printf("%d\n", a5);
        printf("%d\n",b);
             
        if(a4 > 10000){
        //button LT
        P[2] = 0.8;
        ik(P,L);
        find_tp();
        
        for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
        }
        
        if(a4 < 10000 && b<0 ){
        position(0);
        }

        if(a5 > 10){
        //button RT
        position(0);
        vibrate(1);    
        }           
        
        if(b > 7 && b < 9){
        //button A
        jump();     
        }
        
        if(b > 8 && b < 10){
        //button B
        position(2);
        }
        if(b > 10 && b < 12){
        //button Y
        position(-4);
        delay(4);
        position(0);
        vibrate(2);
        }
        if(b > -1 && b < 1){
        //button start
        position(0);
        }
        
        if(b > 3 && b < 5){
        //button LB
        double P1 [6] = { 0, 2.374, -0.2, 0.0, 0.0, 0.0};
        ik(P1,L);
        find_tp();        
        for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
       delay(0.05);
        double P2 [6] = { 0, 2.174, -0.2, 0.0, 0.0, 0.0};
        ik(P2,L);
        find_tp();
        for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
          delay(0.05);
        double P3 [6] = { 0, 2.1, -0.2, 0.0, 0.0, 0.0};
        ik(P3,L);
        find_tp();
        for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
          delay(0.1);
        }
        
        if(b > 4 && b < 6){
        //button RB
        double P1 [6] = { 0, 2.474, -0.2, 0.0, 0.0, 0.0};
        ik(P1,L);
        find_tp();        
        for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
       delay(0.05);
        double P2 [6] = { 0, 2.574, -0.25, 0.0, 0.0, 0.0};
        ik(P2,L);
        find_tp();
        for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
          delay(0.05);
        double P3 [6] = { 0, 2.674, -0.3, 0.0, 0.0, 0.0};
        ik(P3,L);
        find_tp();
        for (i = 0; i < NUM_PISTONS; i++) {
        wb_motor_set_position(pistons[i],TARGET_P[i]);
          }
          delay(0.1);
        }
        if(b > 2 && b < 4){
        //button R3
        vibrate(1);
        }
        wb_robot_step(TIME_STEP);    
        }

  for (;;)
    wb_robot_step(TIME_STEP);  // 无限循环
    
  wb_robot_cleanup();

  return 0;
}


