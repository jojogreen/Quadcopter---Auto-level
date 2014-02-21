/* QUADCOPTER STABLE CONTROL
   *
   *
   *CODED BY EVAN KAISER
   *
   *LAST MODIFIED FEBRUARY 19, 2014 @ 12:36PM
   *
   *
   *
   * FUNCTION:
   *
   * THIS ARDUINO SCRIPT IS DESIGNED TO AUTO-LEVEL A QUADCOPTER WHEN THERE ARE NO INPUTS FROM THE RC CONTROLLER.
   * THIS WILL HELP A NOVICE TO PILOT A QUADCOPTER. IF THE PILOT LOSES CONTROL, ALL HE HAS TO DO IS LET GO OF THE ROLL, PITCH,
   * AND YAW STICKS AND CONTROL ONLY THE THROTTLE. THE SCRIPT USES AN ARDUINO MEGA 2560 AND AN MPU6050 6-AXIS ACCELEROMETER/GYRO.
   *
   *
   * HOW IT WORKS:
   *
   * THIS IS A BASIC FEEDBACK CONTROL SYSTEM. THE ARDUINO TAKES READINGS FROM THE ACCELEROMETER. AS GRAVITY ALWAYS POINTS TOWARDS THE GROUND,
   * THE PITCH AND ROLL ANGLE CAN BE CALCULATED. A PID ALGORITHM IS USED TO DETERMINE WHAT NEEDS TO HAPPEN TO THE PITCH AND ROLL. THIS VALUE
   * IS OUTPUT IN A PULSE WIDTH MODULATION (PWM) SIGNAL TO A CONTROLLER BOARD, WHICH CONTROLS THE FOUR ROTORS. IF THE PILOT IS ADJUSTING
   * THE PITCH, ROLL, OR YAW, THE ARDUINO BYPASSES THE PID LOOP AND OUTPUTS THE RF SIGNAL DIRECTLY TO THE CONTROLLER BOARD. A BLOCK DIAGRAM
   * IS ALSO ATTACHED.
   * 
   *FEATURES -
   *         -ACCELERATION VALUES
   *         -ANGLE OF QUADCOPTER CALCULATED FROM ACCELERATION
   *         -PID TO STABILIZE FLIGHT
   *
   *
*/
 
//Libraries to include
#include<Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <PID_v1.h>
#include <math.h>;

 
MPU6050 accelgyro;
//SERVO

Servo CH1;
Servo CH2;
Servo CH3;
Servo CH4;

 
//pins
const int snr = A1; //sonar is on pin 1
const int IRfront = A2; //IR front is on pin 2
const int IRstbd = A3; // IR starboard is on pin 3
const int IRrear = A4; //IR rear is on pin 4
const int IRport = A5;//IR port is on pin 5
const int ch1 = 2; //CH1 output pin 9 ROLL
const int ch2 = 3; // CH2 output pin 10 PITCH
const int ch3 = 4; // CH3 output pin 11 THROTTLE
const int ch4 = 5; // CH4 output pin 12 YAW
const int Switch = 6; //switch = pin 13
const int ch1in = 7;//Roll in
const int ch2in = 8;//Pitch in
const int ch3in=9 ;//Throttle in
const int ch4in= 10;//Yaw in
 
//Variables for Sonar
long snrVolt, snrrange;
float snrsum = 0;
float prevsnr = 0;
 
//variables for IR front
float frntVolt, frntrange, avgfront;
float frntsum = 0;
float prevfrnt = 0;
 
//variables for IR starboard
float stbdVolt, stbdrange, avgstbd;
float stbdsum = 0;
float prevstbd = 0;
 
//variables for IR rear
float rearVolt, rearrange, avgrear;
float rearsum = 0;
float prevrear = 0;
 
//variables for IR port
float portVolt, portrange, avgport;
float portsum = 0;
float prevport = 0;
 
//variables for motor control
int throttle = 1000;
double pitch = 1500;
double roll = 1500;
double yaw = 1500;
 
//global variables
int avgrange = 300;
long TIMER = 0;
long TIMERF = 0;
int buttonstate = 0;
int count = 0;
 
//accelerometer and gyro variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
long sumAX, sumAY, sumAZ = 0;
long sumGX, sumGY, sumGZ = 0;
int avg = 50;
double avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ = 0;
long Acceleration = 0;
 
int countA = 0;
long time = 0;
long prevtime = 0;
long ANGx, ANGy, ANGz = 0;
long Ax, Ay, Az, Gx, Gy, Gz, prevAX, prevAY, prevAZ, prevGX, prevGY, prevGZ = 0;
double calcangpitch;
double calcangroll;
float GZint;

 
//START ACCELERATION TO VELOCITY VARIABLES
int w = 0; //counter
long T = 0;
long t = 3;
long AX, AY, AZ = 0;
long Vxf, Vyf, Vzf = 0;
long clock = 0;
long deltT = 0;
int value = 0;
 
int DIRECTIONA, DIRECTIONB, DIRECTIONC = 0;

//PID
//Define Variables we'll be connecting to
double Setpointpitch = 0;
double Setpointroll = 0;
double Setthrottle = 20;

//Define the aggressive and conservative Tuning Parameters
double consKp=.75, consKi=.4, consKd=0.08;
double aggKp=1, aggKi=.5, aggKd=0.1;
double consyawKp=0, consyawKi=0, consyawKd=.2;
double aggyawKp=0, aggyawKi=0, aggyawKd=.5;

//Specify the links and initial tuning parameters
PID pitchPID(&calcangpitch, &pitch, &Setpointpitch, consKp, consKi, consKd, DIRECT);
PID rollPID(&calcangroll, &roll, &Setpointroll, consKp, consKi, consKd, DIRECT);
PID yawPID(&avgGZ, &yaw, 0, consyawKp, consyawKi, consyawKd, DIRECT);

int K=0;
 
 
//START VOID SETUP
void setup(){
	Wire.begin(); //Join I2C bus
	Serial.begin(9600);

        //Take initial inputs from RC controller
        pinMode(ch1in,INPUT);
        pinMode(ch2in,INPUT);
        pinMode(ch3in,INPUT);
        pinMode(ch4in,INPUT);
        int originalPitch = 1500;
        int upperpitch = originalPitch*1.1;
        int lowerpitch = originalPitch*.9;
        int originalRoll = 1500;
        int upperroll = originalRoll*1.1;
        int lowerroll = originalRoll*.9;
        int originalThrottle = 1000;
        int upperthrottle = originalThrottle*1.1;
        int lowerthrottle = originalThrottle*.9;
        int originalYaw = 1500;
        int upperyaw = originalYaw*1.1;
        int loweryaw = originalYaw*.9;
        
	
	//attaching pins to servo library
	CH1.attach(ch1);
	CH2.attach(ch2);
	CH3.attach(ch3);
	CH4.attach(ch4);
	
	pinMode (Switch, INPUT);
	 
	//for accel gyro function
	Serial.println("Initializing I2C devices...");
	accelgyro.initialize(); // initialize device
	//verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection()? "MPU6050 connection successful" : "MPU6050 connection failed");
	long starttime = millis();
	prevtime = starttime;
        CH4.writeMicroseconds(2000);
        delay(5000);
        CH3.writeMicroseconds(1500);
        CH1.writeMicroseconds(1500);
        CH4.writeMicroseconds(1500);
        pitchPID.SetMode(AUTOMATIC);
        pitchPID.SetOutputLimits(1500, 2000);
        rollPID.SetMode(AUTOMATIC);
        rollPID.SetOutputLimits(1500, 2000);
        yawPID.SetMode(AUTOMATIC);
        yawPID.SetOutputLimits(1500, 2000);
        
        
}//END VOID SETUP
 
//START VOID LOOP
//
//
//
//
//
void loop(){
  K = K++;
  int contPitch = pulseIn(ch2in, HIGH);
        int contRoll = pulseIn(ch1in, HIGH);
        int contThrottle = pulseIn(ch3in, HIGH);
        int contYaw = pulseIn(ch4in, HIGH);

  
  if(K == 2){
    pitchPID.SetOutputLimits(1000, 2000);
        rollPID.SetOutputLimits(1000, 2000);
        yawPID.SetOutputLimits(1000, 2000);
  }
	count = count++;
	buttonstate = digitalRead(Switch);

	if (count == 1){
		delay (0);
	}
	//GET SENSOR DATA FROM SENSORDATA FUNCTION:
	snrrange = Sonardata();
	frntrange = IRdata(IRfront);
	stbdrange = IRdata(IRstbd);
	rearrange = IRdata(IRrear);
	portrange = IRdata(IRport);
	
	//GET ACCEL/GYRO/VELOCITY DATA FROM ACCEL_GYRO_VELOCITY FUNCTION

	avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ = getaccel();//GET ACCELERATION VALUES
if(K==1){
  GZint = avgGZ;
}
avgGZ = avgGZ-GZint;


        calcangpitch = ang(avgAZ, avgAX, calcangpitch, avgGY, prevtime, K);//GET THE ANGLE VALUES FROM ANG FUNCTION
        
        double timem = millis();
         time = timem;
         time = time/1000;
        prevtime = time;
        
        calcangroll = ang(avgAZ, avgAY, calcangroll, avgGX, prevtime, K);
        
        timem = millis();
        time = timem;
        time = time/1000;
        prevtime = time;
        
        //Serial.print(calcangpitch); Serial.print(",");
        //Serial.print(calcangroll); Serial.print(",");
        Serial.print(avgGZ); Serial.print(",");
        
	 

	
	//ACCEl_gyro_velocity
	 

  Setpointpitch = 0;
  Setpointroll = 0;
  	 
	//DETERMINE WHAT TO DO AFTER DATA HAS BEEN RECEIVED

	//pitch PID
 double gap = abs(calcangpitch-Setpointpitch); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    pitchPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     pitchPID.SetTunings(aggKp, aggKi, aggKd);
  }

  pitchPID.Compute();
  
   contPitch = pulseIn(ch2in, HIGH);
  
  if(1.1*1500>contPitch && .9*1500<contPitch)
    {
    CH2.writeMicroseconds(pitch);
    }
    else
    {
      CH2.writeMicroseconds(contPitch);
    }
  //Roll PID
  
   gap = abs(calcangroll-Setpointroll); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    rollPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     rollPID.SetTunings(aggKp, aggKi, aggKd);
  }
    rollPID.Compute();

 contRoll = pulseIn(ch1in, HIGH);
  
  if(1.1*1500>contRoll && .9*1500<contRoll)
    {
    CH1.writeMicroseconds(roll);
    }
    else
    {
      CH1.writeMicroseconds(contRoll);
    }
  
  //Yaw PID
  
   gap = abs(avgGZ); //distance away from setpoint
  Serial.print(gap); Serial.print(",");
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    yawPID.SetTunings(consyawKp, consyawKi, consyawKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     yawPID.SetTunings(aggyawKp, aggyawKi, aggyawKd);
  }

  yawPID.Compute();

 contYaw = pulseIn(ch4in, HIGH);
  
  if(1.1*1500>contYaw && .9*1500<contYaw)
    {
    CH4.writeMicroseconds(yaw);
    }
    else
    {
      CH4.writeMicroseconds(contYaw);
    }

//Throttle
 contThrottle = pulseIn(ch3in, HIGH);
  
      CH3.writeMicroseconds(contThrottle);


TIMER = millis();
TIMER = TIMER/1000;

  

	snrrange = Sonardata();
	frntrange = IRdata(IRfront);
	stbdrange = IRdata(IRstbd);
	rearrange = IRdata(IRrear);
	portrange = IRdata(IRport);
	
	//GET ACCEL/GYRO/VELOCITY DATA FROM ACCEL_GYRO_VELOCITY FUNCTION

	avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ = getaccel();//GET ACCELERATION VALUES

avgGZ = avgGZ-GZint;


        calcangpitch = ang(avgAZ, avgAX, calcangpitch, avgGY, prevtime, K);//GET THE ANGLE VALUES FROM ANG FUNCTION
        
         timem = millis();
         time = timem;
         time = time/1000;
        prevtime = time;
        
        calcangroll = ang(avgAZ, avgAY, calcangroll, avgGX, prevtime, K);
        
        timem = millis();
        time = timem;
        time = time/1000;
        prevtime = time;

	 
	 
	 

	
	 

	 
	 
	 
	//motor commands
	//throttle control
	 
	 
	 
	 
	prevfrnt = frntrange;

	//initialize end loop
	TIMER = millis();
	//Serial.print("TIME"); Serial.println(TIMER);
//	while (TIMER >= 25000){
//		END();
//	}
}//end void loop
 
 
 
/* FUNCTIONS TO BE ACCESSED BY VOID LOOP
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  _________________________________________
*/
//sensor loop
long Sonardata(){
	//Sonar readings
	//
	pinMode(snr, INPUT);
	for (int i=0; i<avgrange; i++){
		snrVolt = analogRead(snr)/2;
		snrsum += snrVolt;
		//delay (1);
	}
	 
	snrrange = snrsum/avgrange;
	snrsum = 0;
	return snrrange;
}
 
long IRdata(const int IR){
	//IR front reading
	for (int j = 0; j<avgrange; j++){
		frntVolt = analogRead(IR)*.0048828125;
		frntsum += frntVolt;
		//delay (1);
	}
	float avgfrnt = 0;
	avgfrnt = frntsum/avgrange;
	float frntrange = (65/2.54)*pow(avgfrnt, -1.10);
	if (frntrange >=60.0){
		frntrange = 999;
	}
	else if (frntrange <=8.85){
		frntrange = 5;
	}
	 
	frntsum = 0;
	return frntrange;
}
 
 
 
 
 
/*FUNCTIONS TO BE ACCESSED BY MAIN LOOP
*
*
*
*/
 
//velocity function to calculate velocity in X, Y, and Z direction
 

	 
	//angle function to calculate angle in X, Y, and Z direction
	float ang(float avgAZ, float avgAX, float calcang, float gangx, float prevtime, float K){
		//Serial.println(K);
                float angaccel = atan2(avgAZ, avgAX)-PI;
                angaccel = angaccel*(180/PI)+90;
                
                double timem = millis();
             
                float time = timem/1000;
                float delt = time-prevtime;
                if(K>=1 | K<=2){
                  gangx = angaccel;
                }
                gangx = gangx+avgGX*delt;
                
                calcang = .90*gangx+.1*angaccel;
//                Serial.print(angaccel);Serial.print(",");
//                Serial.print(gangx);Serial.print(",");
//                Serial.print(calcang);Serial.print(",");
                
                return calcang;
	}//END ANGLE LOOP
	 
	//code from accel_gyro_test_v1_0
	//used to get accel values (m/(s^2)) & gyro values (deg/s)
	float getaccel(){
		  for(int i = 0; i <avg; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAX = sumAX + ax;
    sumAY = sumAY + ay;
    sumAZ = sumAZ + az;
    sumGX = sumGX + gx;
    sumGY = sumGY + gy;
    sumGZ = sumGZ + gz;
    
   
  }

    //Serial.println("sumAX     "); Serial.println(sumAX);
  
		avgAX = sumAX/(16384);
                avgAX = avgAX/50;
                avgAX = avgAX*9.8;
		avgAY = sumAY/16384;
                avgAY = avgAY/50;
                avgAY = avgAY*9.8;
                avgAZ = sumAZ/16384;
                avgAZ = avgAZ/50;
                avgAZ = avgAZ*9.8;
		avgGX = sumGX/(131);
                avgGX = avgGX/50;
                avgGY = sumGY/(131);
                avgGY = avgGY/50;
                avgGZ = sumGZ/(131);
                avgGZ = avgGZ/50;
                avgGZ = avgGZ;
                if(abs(avgGY)<.2){
                  avgGY = 0;
                }
                if(abs(avgAX)<.2){
                  avgAX = 0;
                }
                  if(abs(avgAY)<.2){
                    avgAY=0;
                  }
                    if(abs(avgAZ)<.2){
                      avgAZ = 0;
                    }
                     if(abs(avgGX)<.2){
                      avgGX = 0;
                     }
                    if (abs(avgGZ)<.2){
                     avgGZ = 0;
                    } 
		 
		//return sums to 0
		sumAX = 0;
		sumAY = 0;
		sumAZ = 0;
		sumGX = 0;
		sumGY = 0;
		sumGZ = 0;
		 
		//acceleration scaler

		 
		Acceleration = sqrt(pow(avgAX, 2)+pow(avgAY, 2)+pow(avgAZ, 2));
		return avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ;
	}
	 

	//BEGIN END LOOP
	void END(){
	Serial.print("END LOOP   ");
		throttle = throttle-15;
		throttle = constrain(throttle, 1000, 2000);
		CH3.writeMicroseconds(throttle);
		delay(400);
		Serial.println(throttle);
	} //END the end loop
