#ifndef CONTROLLER_H
#define CONTROLLER_H

#define ENCODER_A_PIN DDD2
#define ENABLE_A_PIN PD2
#define ENCODER_B_PIN DDD3
#define ENABLE_A_PIN PD3
#define PPR 100     //pulse per rev of encoder
#define V_BATT 12   // V battery

#define SIGN(a) ( (a) >= 0.0 ? 1.0 : -1.0 )

extern volatile int pulseCount;

void main();
void encoder_init();
void sensor_init();
float rpm_to_voltage(float speed);
float encoder_to_voltage(float &speed_rps);
float command_to_voltage(char data);
char inquiry();
void PID_controller(float x[2], float &u, float t_recent ,float t0, int prescaler_1, int &n, int mode);
void speed_controller(float u);
float speed_front(float &v_front);
void traction_controller(float Sd, float wr_current, float vf, float R, float &u);

#endif