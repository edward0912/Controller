
#include <Servo.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "BDCdrv.h"
#include "UART.h"
#include "controller.h"

volatile int pulseCount = 0;

void main() {
	char data;
	
	float voltage_d, voltage_m; // DESIRED VOLTAGE, MEASURE VOLTAGE
	float x1[2] = [voltage_d, voltage_m];
	float x2[2] = [w_rear, v_front];
	float v_front, w_front, w_rear;
	float u, S; 		//OUTPUT VOLTAGE - SLIP RATIO 
	
	//const float S_max = 2.0; 
	const float S_desired = 0.2;
	const float F = 0.2;    // F.O.S FOR V_BATT_MAX
	const float R = 0.025;	//WHEEL RADIUS (2.5 CM)
	const float tol_traction = 1/R;  //ALLOWABLE RANGE OF 4KM/H (~1M/S)	IN RAD/S
	const float R_rev = 1/R; //REVERSE RADIUS
	
	int t_recent, t0_count; //RECORD COUNT RIGHT BEFORE ENTERING PID -> CHECK OVERFLOW
	int i = 0;
	
	const int mode_speed = 1, mode_tract =2;
	const int prescaler_1 = 8;  //TIMER 1 INITIALIZED FROM SERVODRV.C
	
	static int n = 0; //OVERFLOW COUNTS
	
	t0_count = TCNT1;
	t0 = t0_count*prescaler / 16.0e6; // INITIAL TIME (S)
	
	encoder_init;
	sensor_init;
	
	while(1) 
	{
		data = inquiry();
		voltage_d = command_to_voltage(data);
		voltage_e = encoder_to_voltage(w_rear);
		
		t_recent = TCNT1; // CHECK COUNT BEFORE ENTERING PID
		if( t_recent < t0_count && i = 0) {// CHECK IF OVERFLOWS AFTER PROCESS DATA ABOVE - ONLY ONCE
			n++;
			i = 1;
		} 
		PID_controller(x1[2], u, t_recent, t0, prescaler_1, n, mode_speed); //RETURN VOLTAGE U
		speed_controller(u);  // SEND PW TO BDC
		speed_front(v_front); // CONVERT SENSOR TO SPEED
		
		if ( (w_rear - v_front*R_rev) > tol_traction){ //TURNED OFF WHEN DESIRED SPEED REACHED
			traction_controller(S_desired,w_rear, v_front,R, u); //RETURN VOLTAGE U
			
			t_recent = TCNT1; // CHECK COUNT BEFORE ENTERING PID
			
			PID_controller(x2[2], u, t_recent, t0, prescaler_1, n, mode_tract);
			speed_controller(u); // SEND PW TO BDC
		}
		//brake_controller
		if (data = 'S') break; //STOP PROGRAM //**ADJUST ACCORDINGLY
	}

	delay(1000);
	exit(0); // not using loop()	
}

ISR(INT0_vect) { //*its better to do I2C
	if ((PIND & (1<<PIND2)) > (PIND & (1<<PIND3))){ // IF A > B (CW)
		pulseCount++;
	} // IF B > A (CCW)
  	else pulseCount--;
}

void encoder_init(){
	cli();
	
	DDRD &= (~(1 << ENCODER_A_PIN)) & (~(1 << ENCODER_B_PIN));  // Set ENCODER_A pin as input
    PORTD |= (1 << ENABLE_A_PIN) | (1 << ENABLE_B_PIN);  // Enable internal pull-up resistor //why pull up

	
	EICRA |= (1<<ISC01) | (1<<ISC00) ; // Trigger INT0 on rising edge
	EIMSK |= (1<<INT0); // Enable INT0 - ATTACHED WITH ENCODER A (PIN 2)
	sei(); // Enable global interrupts
}
void sensor_init(){
	
}

float rpm_to_voltage(float speed){ //TODO
	float voltage_m;
	/*
	ENCODER CONVERSION FROM RPM TO VOLTAGE
	CHECK MOTOR RPM'S SPECS - 5 V
	*/
	return voltage_m;
}

// the entire program is delayed 1s every time this gets called? -> can we scale down?
// make this register level
float encoder_to_voltage(float &speed_rps){ //RETURNS VOLTAGE FROM ENCODER PULSES
	pulseCount = 0; 		//RESET COUNT
	unsigned long t_previous = micros();
	while (1) {
		unsigned long t_current = micros();
		if (t_current - t_previous >= 1.0E6){ 	//AFTER 1S
			speed_rps = (float)pulseCount / PPR; //100 PULSES PER REV
			break;
		}
	}
	return rpm_to_voltage(speed_rps*60); //ENCODER VOLTAGE
}

float command_to_voltage(char data){ //TODO
	// convert the receive data from bluetooth to rpm
	// switch case here?, in pwm -> voltage -> rpm?
}

char inquiry(){ //*takes in command from bluetooth //Ali's TODO
	 char data;
	 data = UART_Receive(); //ReadBluetooth(data) ( is it a pointer here?)
	 //Lidar();
	 return data;
}

void PID_controller(float x[2], float &u, float t_recent ,float t0, int prescaler_1, int &n, int mode){
	float t, dt;
	float kp, ki, kd;
	float e, ed, z, ei_max;
	float u, r, y; // R - DESIRED POSITION, Y - SENSOR OUTPUT
	float time;
	
	static float tp = 0.0; //PREVIOUS TIME
	static float ei = 0.0;
	static float ep = 0.0; // PREVIOUS ERROR
	
	const float k = 1/prescaler_1; 
	const float t1_of = prescaler_1 / 16.0e6 * 65536;
	
	int tc;
	
	tc = TCNT0; // get current timer count	
	if( tc < t_recent ) n++; // increase count n if overflow / roll occurs
	time = k * tc + n * t1_of; //current time(s)
	t = time - t0; 
	
	// PID CONTROLLER GAINS - *** TUNE THIS
	if (mode = 1){ // SPEED GAINS
		kp = 10;
		ki = 0.1;
		kd = 5;
	}
	else if (mode = 2){ //TRACTION GAINS
		kp = 0;
		ki = 0;
		kd = 0;
	}
	
	r = x[0]; //VOLTAGE_D
	y = x[1]; //VOLTAGE_E
	e = r - y; //VOLTAGE_D - VOLTAGE_E: DESIRED FROM CONTROLLER - CURRENT READING FROM ENCODER
	
	// time interval dt
	dt = t - tp;
	tp = t;
	
	//finite difference approximation for differential error
	ed = (e-ep)/dt; //* or ec = r_dot - ys_dot = 0 - w
	ep = e;

	// set maximum integration bounds
	// ki*ei_max = ui_max = F*V_batt
	// ei_max = F*V_batt/ki
	if (ki > 0){
		ei_max = F*V_BATT/ki;
	}
	else ei_max = 0;
	
	// set z to prevent out of bounds increase (too large ui)
	if ((ei > ei_max) && (e > 0)){ // positive out of bounds 
		z = 0;
	}
	else if ((ei < ei_max) && (e < 0)){ //negative out of bounds
		z = 0;
	}
	else z = e; // normal integration after checks cleared
	
	//standard integration I
	ei += z*dt;
	// PID
	u = kp*e + kd*ed + ki*ei;
	
	// cap u max
	if (u > V_BATT) u = V_BATT;
	if (u < -V_BATT) u = -V_BATT;
	
}

void speed_controller(float u){ // convert voltage to pw, send to bdc
	int speed;
	if ( u > 0) {
		speed = (int)(u*255.0/V_BATT);// assume this is forward with positive u
		bdcTurnLeft (speed); 
	else {
		speed = (int)(-u*255.0/V_BATT); // assume this is backwards with negative u
		bdcTurnRight(speed);
		}
	} 
}

float speed_front(float &v_front){
	/*
	conversion from sensor reading (count/voltage) to m/s
	*/
}

void traction_controller(float Sd, float wr_current, float vf, float R, float &u){ //stop when slip const (0.2 for accel, -0.2 for decel)
	//float tol = 1.0e-10;	
	//S = ( w*R - v ) / ( abs(v) + tol ); // rear wheel (rad/s), front wheel (m/s)
	
	float wr; //DESIRED W_REAR
	float x[2] = [wr, wr_current]; 
	
	if (vf > 0){ //FORWARD
		if (wr_current*R - vf > 0) wr = (1 + Sd)/R; //ACCELERATION 
		else wr = (1 - Sd)/R; //DECELERATION 
	}
	else { //BACKWARDS
		if (wr_current*R - vf > 0) wr = (1 - Sd)/R; //ACCELERATION 
		else wr = (1 + Sd)/R; //DECELERATION 
	}
}

 /*
float read_ADC_voltage(int channel, int n)
// average = digital low pass filter
// for some value of n it's very good (eg n=200)
{
	int i;
	float input, voltage;
	unsigned long int sum;
	const float ADC_to_V = 1.0/1023.0*5;
	
	sum = 0;
	for(i=0;i<n;i++) {
		sum += analogRead(channel);
	}
	
	input = (float)sum / n; // average analog input
	
	// note that the simple division of float below takes around 
	// 40 us compared to around 4 us for equivalent multiplication
	// -> avoid using float division if possible
	// -> use multiplication instead of division when possible
	// -> avoid putting float division in loops
//	voltage = input / 1023.0 * 5;
	voltage = input * ADC_to_V; // much faster than expression above
	
	return voltage;
}

*/