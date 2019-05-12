int LED8=8;
int pinHV=2;
int pinHU=7;
int pinHW=4;
bool HV=0;
bool HU=0;
bool HW=0;
int angle;
int absAngle;
int rpm=0;
uint32_t ts1 = 0;
uint32_t ts2 = 1;

int refsig = 200; //for converting the analog signal coming from hall sensor to digital through arduino code
 int val;//the digital value of the incoming analog signals
 int prev_val = 0;
 int t, cur_t; //time variables
 void setup()
 {
   Serial.begin(9600);
   pinMode(A0, INPUT);
  // initialize digital pin LED8 as an output.
  pinMode(LED8, OUTPUT);
  pinMode(pinHV,INPUT);
  pinMode(pinHU,INPUT);
  pinMode(pinHW,INPUT);
}

void rotateX(int deg){
 
}

void loop() {
  


  //get angle
  int updatedHV=digitalRead(pinHV);
  int updatedHU=digitalRead(pinHU);
  int updatedHW=digitalRead(pinHW);
  if((HV!=updatedHV)|(HU!=updatedHU)|(HW!=updatedHW)){      //if any value changed, add +60 deg
    ts1 = micros();
    HV=updatedHV;
    HU=updatedHU;
    HW=updatedHW;
    angle+=60;
    absAngle+=60;
    rpm=1000000 * 60*6/(ts2-ts1);
    //1000000 * 60 / (cur_t - t)
    if(angle==360){
      angle=0;
    }
      
      // ...TASK TO BE MEASURED GOES HERE
    ts2 = micros();
  }
      
   Serial.println(angle);
  // Serial.print(" ");
  // Serial.println(rpm/100);
  
}//loop
