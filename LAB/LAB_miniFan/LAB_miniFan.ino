// State definition
#define S0 0 // LED off , motor off
#define S1 1 // LED off , motor velocity 50
#define S2 2 // LED off , motor velocity 100

// Output definition
#define LED 0
#define Pwm 0 

const int ledPin = 13;
const int pwmPin = 11;
const int btnPin = 3;
const int trigPin = 10;
const int echoPin = 7;

unsigned char state = S0;
unsigned char nextstate = S0;
unsigned char input[2] = {0,0}; // LED, Detect
unsigned int ledOut = LOW;
unsigned int pwmOut = 0;
int bPressed = 0;

unsigned long duration;
float distance;
int thresh = 6;
// State table definition

typedef struct {
    
unsigned int motor[2][2];      // output    = FSM[state].motor out[input X][input Y]
unsigned int led[2][2];        // output    = FSM[state].led out[input X][input Y]
unsigned int next[2][2];       // nextstate = FSM[state].next[input X][input Y]
	
} State_t;

//FSM

State_t FSM[3] = {

{ {{0,0},  {0,125} },     {{LOW,LOW},  {HIGH,HIGH}}  ,  { {S0,S0},{S1,S1}}  },
{ {{0,125}, {0,250}},     {{HIGH,HIGH},  {HIGH,HIGH}}  ,  { {S1,S1},{S2,S2}}  },
{ {{0,250},{0,0}  },     {{HIGH,HIGH}, {LOW,LOW}  }  ,  { {S2,S2},{S0,S0}}  }

};

void setup(){
// initialize the LED pin as an output:
pinMode(ledPin, OUTPUT);

// initialize pwm pin as an output:
pinMode(pwmPin, OUTPUT);

// initialize the pushbutton pin as an interrupt input
pinMode(btnPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(btnPin), pressed, FALLING);

// initialize the trigger pin as an output
pinMode(trigPin, OUTPUT);

// initialize the echo pin as an input
pinMode(echoPin, INPUT);

Serial.begin(9600);

}

void loop() {
// generate pwm signal on the trigger pin.
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin,LOW);
delayMicroseconds(10);

stateOutput();
analogWrite(pwmPin, pwmOut);
digitalWrite(ledPin, ledOut);

// Distance is calculated using how much time it takes.
duration = pulseIn(echoPin, HIGH);
distance = (float)duration / 58.0;
 
  nextState();

  Serial.print("distance = ");
  Serial.print(distance);
  Serial.println(" [cm]");
  
//state check
  Serial.print("state=");
  Serial.print(state);

  Serial.print("inputbtn = ");
  Serial.print(input[0]);

  Serial.print("inputdect = ");
  Serial.print(input[1]);  
  
 

  delay(1000);
}


void pressed(){

bPressed = 1;
}

void nextState(){

if (distance < thresh)
  input[1] = 1; 
else
  input[1] = 0;

if (bPressed==1)
  input[0]=1;
else
  input[0]=0;

state = FSM[state].next[input[0]][input[1]];
bPressed = 0;  
}

void stateOutput(){
pwmOut = FSM[state].motor[input[0]][input[1]];
ledOut = FSM[state].led[input[0]][input[1]];  

}


