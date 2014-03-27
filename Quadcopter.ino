/* QUADCOPTER STABLE AUTONOMOUS CONTROL
   *
   * VERSION 2.4.2
   *
   *CODED BY EVAN KAISER
   *
   *LAST MODIFIED January 12, 2014 @ 6:34PM
   *
   *FEATURES -
   *         -AUTO-LEVELING WHEN TRANSMITTER IS CENTERED (THROTTLE ALWAYS CONTROLLED MANUALLY)
   *         -ACCELERATION VALUES
   *         -ANGLE OF QUADCOPTER CALCULATED FROM ACCELERATION
   *         -PID TO STABILIZE FLIGHT
   *         -RUNNING AVERAGE TO FILTER ACCELERATION VALUES
   *         
   *
   *TASKS TO D0:
   *  1)ADD IR INPUTS
   *  2)DETERMINE WHETHER CLOSE TO OBJECT (THROUGH IR)
   *  3)AVOID OBJECTS
   *  4)ROUTE RECEIVER THROUGH ARDUINO FOR OPTIONAL MANUAL CONTROL
*/
 
//Libraries to include
#include<Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <PID_v1.h>
#include <math.h>;
#include <SD.h>;

 
MPU6050 accelgyro;
//SERVO

Servo CH1;
Servo CH2;
Servo CH3;
Servo CH4;


//SD Card
const int chipSelect = 53;


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
const int SDswitch = 6; //switch = pin 13
const int ch1in = 7;//Roll in
const int ch2in = 8;//Pitch in
const int ch3in=9 ;//Throttle in
const int ch4in= 6;//Yaw in
 
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
const int error = 30;
int avgrange = 300;
long TIMER = 0;
long SDTIME = 0;
long TESTTIME2 = 0;
long TIMERF = 0;
int SDSwitchState = 0;
int count = 0;
int LevelActivate = 0;
int SDswitch = 22;
 
//accelerometer and gyro variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
long sumAX, sumAY, sumAZ = 0;
long sumGX, sumGY, sumGZ = 0;
int avg = 50;
double avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ = 0;
const int numReadings = 100;
int readingsAX[numReadings];
int readingsAY[numReadings];
int readingsAZ[numReadings];
int readingsGX[numReadings];
int readingsGY[numReadings];
int readingsGZ[numReadings];

int indexAX = 0;
int indexAY = 0;
int indexAZ = 0;
int indexGX = 0;
int indexGY = 0;
int indexGZ = 0;

long Acceleration = 0;
 
int countA = 0;
long time = 0;
long prevtime = 0;
long ANGx, ANGy, ANGz = 0;
long Ax, Ay, Az, Gx, Gy, Gz, prevAX, prevAY, prevAZ, prevGX, prevGY, prevGZ,valAX,valAY,valAZ,valGX,valGY,valGZ = 0;
double calcangpitch;
double calcangroll;
float GZint;

 
//START ACCELERATION TO VELOCITY VARIABLES
int w = 0; //counter
int p = 0;
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
double aggKp=.875, aggKi=.5, aggKd=0.1;
double consyawKp=0, consyawKi=0, consyawKd=.2;
double aggyawKp=0, aggyawKi=0, aggyawKd=.5;
int pitchPIDmode=0;
int rollPIDmode=0;
int yawPIDmode=0;


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
	
	pinMode (SDswitch, INPUT);
        pinMode (error, OUTPUT);
//initialize SD Card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
 
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    digitalWrite(error, HIGH);
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
	 
	//for accel gyro function
	Serial.println("Initializing I2C devices...");
	accelgyro.initialize(); // initialize device
	//verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection()? "MPU6050 connection successful" : "MPU6050 connection failed");
if(accelgyro.testConnection()==0){
  digitalWrite(error, HIGH);
}
else
{digitalWrite(error, LOW);
}
	long starttime = millis();
	prevtime = starttime;
        CH4.writeMicroseconds(2000);
        delay(5000);
        CH3.writeMicroseconds(1500);
        CH1.writeMicroseconds(1500);
        CH4.writeMicroseconds(1500);
        pitchPID.SetMode(AUTOMATIC);
        //pitchPID.SetControllerDirection(REVERSE);
        pitchPID.SetOutputLimits(1500, 2000);
        rollPID.SetMode(AUTOMATIC);
        //rollPID.SetControllerDirection(REVERSE);
        rollPID.SetOutputLimits(1500, 2000);
        yawPID.SetMode(AUTOMATIC);
        yawPID.SetOutputLimits(1500, 2000);
        
        File dataFile = SD.open("datalog.txt", FILE_WRITE);


        
        
}//END VOID SETUP
 
//START VOID LOOP
//
//
//
//
//
void loop(){
K=K++;






  
  int contPitch = pulseIn(ch2in, HIGH);
        int contRoll = pulseIn(ch1in, HIGH);
        int contThrottle = pulseIn(ch3in, HIGH);
        int contYaw = pulseIn(ch4in, HIGH);

        //Serial.print(contPitch); Serial.print("  ");Serial.print(contRoll); Serial.print("  ");Serial.print(contThrottle); Serial.print("  ");Serial.println(contYaw);

while(LevelActivate ==0 && contThrottle<=1500)
{
  int contPitch = pulseIn(ch2in, HIGH);
        int contRoll = pulseIn(ch1in, HIGH);
        int contThrottle = pulseIn(ch3in, HIGH);
        int contYaw = pulseIn(ch4in, HIGH);
        CH1.writeMicroseconds(contRoll);
        CH2.writeMicroseconds(contPitch);
        CH3.writeMicroseconds(contThrottle);
        CH4.writeMicroseconds(contYaw);
        
  if (contThrottle>=1500)
  {
    LevelActivate = 1;
  }
}




  
  if(K == 2){
    pitchPID.SetOutputLimits(1000, 2000);
        rollPID.SetOutputLimits(1000, 2000);
        yawPID.SetOutputLimits(1000, 2000);
  }
//  Serial.println();
//  Serial.println(K);
//  Serial.println();
	count = count++;
	//SDSwitchState = digitalRead(SDswitch);
	//  while (SDSwitchState == LOW){
		//   throttle = 145;
		//   pitch = 12;
		//   roll = 188;
		//   yaw = 12;
	//  };

	//GET SENSOR DATA FROM SENSORDATA FUNCTION:
//	snrrange = Sonardata();
//	frntrange = IRdata(IRfront);
//	stbdrange = IRdata(IRstbd);
//	rearrange = IRdata(IRrear);
//	portrange = IRdata(IRport);
	
	//GET ACCEL/GYRO/VELOCITY DATA FROM ACCEL_GYRO_VELOCITY FUNCTION
for (p=0; p<=23;p++)
{
	avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ = getaccel();//GET ACCELERATION VALUES
Serial.print("GZ Returned     ");Serial.println(avgGZ);
}
//if(K==1){
//  GZint = avgGZ;
//  
//  Serial.print("GZINT");Serial.println(GZint);
//}
//avgGZ = avgGZ-GZint;


        calcangpitch = ang(avgAZ, avgAX, calcangpitch, avgGY, prevtime, K);//GET THE ANGLE VALUES FROM ANG FUNCTION
        if (abs(calcangpitch)<=.1){
          calcangpitch = 0;
        }
        
        double timem = millis();
         time = timem;
         time = time/1000;
        prevtime = time;
        
        calcangroll = ang(avgAZ, avgAY, calcangroll, avgGX, prevtime, K);
        if(abs(calcangroll)<=.1){
          calcangroll = 0;
        }
        
        timem = millis();
        time = timem;
        time = time/1000;
        prevtime = time;
        
        //Serial.print(calcangpitch); Serial.print(",");
        //Serial.print(calcangroll); Serial.print(",");
        //Serial.print(avgGZ); Serial.print(",");
        
	 

	
	//ACCEl_gyro_velocity
	 

  Setpointpitch = 0;
  Setpointroll = 0;
  	 
	//DETERMINE WHAT TO DO AFTER DATA HAS BEEN RECEIVED

	//pitch PID

//Serial.println(calcangpitch);
 double gap = abs(calcangpitch-Setpointpitch); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    pitchPID.SetTunings(consKp, consKi, consKd);
    pitchPIDmode=0;
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     pitchPID.SetTunings(aggKp, aggKi, aggKd);
     pitchPIDmode=1;
  }

  pitchPID.Compute();
  
   contPitch = pulseIn(ch2in, HIGH);
  
  if(1600>contPitch && 1400<contPitch)
    {
    CH2.writeMicroseconds(pitch);
    //Serial.print(pitch); Serial.print(","); Serial.print(" ");
    }
    else
    {
      CH2.writeMicroseconds(contPitch);
      //Serial.print(contPitch); Serial.print(","); Serial.print(" ");
    }
  //Roll PID
   gap = abs(calcangroll-Setpointroll); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    rollPID.SetTunings(consKp, consKi, consKd);
    rollPIDmode=0;
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     rollPID.SetTunings(aggKp, aggKi, aggKd);
     rollPIDmode=1;
  }
    rollPID.Compute();

 contRoll = pulseIn(ch1in, HIGH);
  
  if(1600>contRoll && 1400<contRoll)
    {
    CH1.writeMicroseconds(roll);
    //Serial.print(roll); Serial.print(","); Serial.print(" ");
    }
    else
    {
      CH1.writeMicroseconds(contRoll);
    //  Serial.print(contRoll); Serial.print(","); Serial.print(" ");
    }
  
  //Yaw PID
  
   gap = abs(avgGZ); //distance away from setpoint
  //Serial.print(gap); Serial.print(",");
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    yawPID.SetTunings(consyawKp, consyawKi, consyawKd);
    yawPIDmode=0;
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     yawPID.SetTunings(aggyawKp, aggyawKi, aggyawKd);
     yawPIDmode=1;
  }

  yawPID.Compute();

 contYaw = pulseIn(ch4in, HIGH);
  
  if(1600>contYaw && 1400<contYaw)
    {
    CH4.writeMicroseconds(yaw);
    //Serial.print(yaw); Serial.print(","); Serial.print(" ");
    }
    else if(isnan(yaw)==true){
      CH4.writeMicroseconds(contYaw);
      digitalWrite(error,HIGH);
      delay(100);
      digitalWrite(error, LOW);
    }
    else
    {
      CH4.writeMicroseconds(contYaw);
    //  Serial.print(contYaw); Serial.print(","); Serial.print(" ");
    }

//Throttle
 contThrottle = pulseIn(ch3in, HIGH);
  
      CH3.writeMicroseconds(contThrottle);
    //  Serial.println(contThrottle);
      


TIMER = millis();
TIMER = TIMER/1000;
//Serial.println(TIMER);

  

//	snrrange = Sonardata();
//	frntrange = IRdata(IRfront);
//	stbdrange = IRdata(IRstbd);
//	rearrange = IRdata(IRrear);
//	portrange = IRdata(IRport);
	
	//GET ACCEL/GYRO/VELOCITY DATA FROM ACCEL_GYRO_VELOCITY FUNCTION

//	avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ = getaccel();//GET ACCELERATION VALUES
//
//avgGZ = avgGZ-GZint;
//
//
//        calcangpitch = ang(avgAZ, avgAX, calcangpitch, avgGY, prevtime, K);//GET THE ANGLE VALUES FROM ANG FUNCTION
//        
//         timem = millis();
//         time = timem;
//         time = time/1000;
//        prevtime = time;
//        
//        calcangroll = ang(avgAZ, avgAY, calcangroll, avgGX, prevtime, K);
//        
//        timem = millis();
//        time = timem;
//        time = time/1000;
//        prevtime = time;
        //Serial.println(time);

	 
	 
	 

	
	 
SDTIME = millis();
SDTIME = SDTIME/1000;

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.


  File dataFile = SD.open("datalog.txt", O_CREAT | O_WRITE);

    // if the file is available, write to it:
  if(K==1){
    dataFile.println("avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ, calcangpitch, calcangroll, pitch, controller pitch, roll, controller roll, yaw, controller yaw, throttle, pitch PID Mode, Roll PID Mode, Yaw PID Mode, time");

    // print to the serial port too:
    Serial.println("success");
  }

  // if the file is available, write to it:
  if (dataFile) {


    dataFile.print(avgAX);dataFile.print( ",");
    dataFile.print(avgAY);dataFile.print(",");
    dataFile.print(avgAZ);dataFile.print(",");
    dataFile.print(avgGX);dataFile.print(",");
    dataFile.print(avgGY);dataFile.print(",");
    dataFile.print(avgGZ);dataFile.print(",");
    dataFile.print(calcangpitch);dataFile.print(",");
    dataFile.print(calcangroll);dataFile.print(",");
    dataFile.print(pitch);dataFile.print(",");
    dataFile.print(contPitch);dataFile.print(",");
    dataFile.print(roll);dataFile.print(",");
    dataFile.print(contRoll);dataFile.print(",");
    dataFile.print(yaw);dataFile.print(",");
    dataFile.print(contYaw);dataFile.print(",");
    dataFile.print(contThrottle);dataFile.print(",");
    dataFile.print(pitchPIDmode);dataFile.print(",");
    dataFile.print(rollPIDmode);dataFile.print(",");
    dataFile.print(yawPIDmode);dataFile.print(",");
    dataFile.println(SDTIME);
    dataFile.close();
    //Serial.print(avgGX); Serial.print("   "); Serial.print(avgGY); Serial.print("   "); Serial.println(avgGZ); 
//  Serial.println(calcangroll); Serial.print("     "); Serial.println(calcangroll);// Serial.print("     "); Serial.println(roll);

    // print to the serial port too:
   // Serial.println("success");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

//  Serial.print("PITCH ANGLE   "); Serial.println(calcangpitch);
//  Serial.print("ROLL ANGLE     "); Serial.println(calcangroll);
//  Serial.print("YAW RATE     "); Serial.println(avgGZ);
	 
	 
	 
	//motor commands
	//throttle control
	 
	 SDSwitchState = digitalRead(SDswitch);
//Serial.print("Switch State   "); Serial.println(SDSwitchState);
if (SDSwitchState==LOW){

      END();

}

	 

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
                float angaccel = atan2(avgAZ, avgAX);//-PI;
                angaccel = angaccel*(180/PI)-90;//+270;
                
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
                if(calcang<=-180){
                  calcang=calcang+360;
                }
                return calcang;
	}//END ANGLE LOOP
	 
	//code from accel_gyro_test_v1_0
	//used to get accel values (m/(s^2)) & gyro values (deg/s)
	float getaccel(){

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sumAX = sumAX - readingsAX[indexAX];
    sumAY = sumAY - readingsAY[indexAY];
    sumAZ = sumAZ - readingsAZ[indexAZ];
    sumGX = sumGX - readingsGX[indexGX];
    sumGY = sumGY - readingsGY[indexGY];
    sumGZ = sumGZ - readingsGZ[indexGZ];
    
    readingsAX [indexAX] = ax;
    readingsAY [indexAY] = ay;
    readingsAZ [indexAZ] = az;
    readingsGX [indexGX] = gx;
    readingsGY [indexGY] = gy;
    readingsGZ [indexGZ] = gz;
    
    sumAX = sumAX + readingsAX[indexAX];
    sumAY = sumAY + readingsAY[indexAY];
    sumAZ = sumAZ + readingsAZ[indexAZ];
    sumGX = sumGX + readingsGX[indexGX];
    sumGY = sumGY + readingsGY[indexGY];
    sumGZ = sumGZ + readingsGZ[indexGZ];
    
    indexAX=indexAX++;
    indexAY=indexAY++;
    indexAZ=indexAZ++;
    indexGX=indexGX++;
    indexGY=indexGY++;
    indexGZ=indexGZ++;
    
    if(indexAX>=numReadings)
    {
      indexAX=0;
    }
    if(indexAY>=numReadings)
    {
      indexAY=0;

    }
    if(indexAZ>=numReadings)
    {
      indexAZ=0;
    }
    if(indexGX>=numReadings)
    {
      indexGX=0;
    }
    if(indexGY>=numReadings)
    {
      indexGY=0;
    }
    if(indexGZ>=numReadings)
    {
      indexGZ=0;
    }    
      avgGZ=sumGZ/numReadings;
      avgGY=sumGY/numReadings;    
      avgGX=sumGX/numReadings;    
      avgAZ=sumAZ/numReadings;   
      avgAY=sumAY/numReadings;
      avgAX=sumAX/numReadings;
      



    //Serial.println("sumAX     "); Serial.println(sumAX);
  
		avgAX = avgAX/(16384);
                //avgAX = avgAX/50;
                avgAX = avgAX*9.8;
                avgAX=avgAX-.23;
		avgAY = avgAY/16384;
                //avgAY = avgAY/50;
                avgAY = avgAY*9.8;
                avgAY= avgAY-.1;
                avgAZ = avgAZ/16384;
                //avgAZ = avgAZ/50;
                avgAZ = avgAZ*9.8;
                avgAZ = avgAZ+1.26;
		avgGX = avgGX/(131);
                //avgGX = avgGX/50;
                avgGX = avgGX+.12;
                avgGY = avgGY/(131);
                //avgGY = avgGY/50;
                avgGY = avgGY-.4;
                avgGZ = avgGZ/(131);
                //avgGZ = avgGZ/50;
                avgGZ = avgGZ-2.49;
//                if(abs(avgGY)<.2){
//                  avgGY = 0;
//                }
//                if(abs(avgAX)<.2){
//                  avgAX = 0;
//                }
//                  if(abs(avgAY)<.2){
//                    avgAY=0;
//                  }
//                    if(abs(avgAZ)<.2){
//                      avgAZ = 0;
//                    }
//                     if(abs(avgGX)<.2){
//                      avgGX = 0;
//                     }
//                    if (abs(avgGZ)<.2){
//                     avgGZ = 0;
//                    } 
//		 
		//acceleration scaler

		 
		Acceleration = sqrt(pow(avgAX, 2)+pow(avgAY, 2)+pow(avgAZ, 2));
//      Serial.print(avgAX);Serial.print(","); Serial.print(" ");Serial.print(avgAY);Serial.print(","); Serial.print(" ");Serial.print(avgAZ);Serial.print(","); Serial.print(" ");Serial.print(avgGX);Serial.print(","); Serial.print(" ");Serial.print(avgGY);Serial.print(","); Serial.print(" ");Serial.println(avgGZ);
Serial.print("GZ   "); Serial.println(avgGZ);
		return avgAX, avgAY, avgAZ, avgGX, avgGY, avgGZ;
	}
	 

	//BEGIN END LOOP
	void END(){
while(SDSwitchState==LOW){
  digitalWrite(error, HIGH);
  Serial.println("end function initialized");
}
	} //END the end loop
