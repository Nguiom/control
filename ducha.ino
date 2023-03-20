#include <OneWire.h>                
#include <DallasTemperature.h>

const int pwmPin = 9;
const int bus=8;
const int Uunits =100;
const int pwmRes =12;  
const int pwmMax =4095;

OneWire ourWire(bus);
DallasTemperature sensors(&ourWire);

float tempF=0.0;
unsigned int pwmV=0;
float ref=37;
float U_op = 50.0; // Direct Control Output - FOR OPENLOOP or FEEDFORWARD - Transistor Collector Current [mA]
float U_t = 0.0; // Control Output
float U_id=0;
float errorP=0;
float errorI=0;
float errorD=0;
float errorL=0;
float m=3.8;
float kp, ki, kd;

// Execution Time Control
long unsigned int pTime = 0;
long unsigned int dTime = 0;
long previousMillis = 0;  // For main loop function
long Ts = 1000; // Sample time in ms
long previousMillis2 = 0; // For auxiliary functions (squarewaves)
bool up = true;
int i = 0;

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}

void compute(void){
  unsigned long currentMillis = millis(); // Update current time from the beginning
  if (currentMillis - previousMillis >= Ts) {
    previousMillis = currentMillis;
    sensors.requestTemperatures();  
    tempF = sensors.getTempCByIndex(0); 
    errorP=ref-tempF;
    errorI=errorI+(errorP*Ts);
    errorD=(errorP-errorL)/Ts;
    
    
    U_t = (kp*errorP)*m+(ki*errorI)*m+U_op;   
    float U_tl = min(max(U_t, 0), Uunits); // Saturated Control Output
    pwmV = int((U_tl/Uunits)*pwmMax);

    errorL=errorP;
    analogWriteADJ(pwmPin, pwmV);
    
  
    Serial.print("U:");
    Serial.print(U_t);
    Serial.print(",");

    Serial.print("tempF:");
    Serial.println(tempF);     
  }



  // Advanced Serial Input Functions
  recvWithStartEndMarkers();  
  if (newData == true) {
    parseData();
    newData = false;
  }  
  
}

void calibracion(void){
  // Measurement, Control, Output Command Signal, Serial Data Communication
  unsigned long currentMillis = millis(); // Update current time from the beginning
  if (currentMillis - previousMillis >= Ts) {
    previousMillis = currentMillis;
    sensors.requestTemperatures();  
    tempF = sensors.getTempCByIndex(0); 
    U_t = U_op;   
    float U_tl = min(max(U_t, 0), Uunits); // Saturated Control Output
    pwmV = int((U_tl/Uunits)*pwmMax);
    analogWriteADJ(pwmPin, pwmV);
    
  
    Serial.print("U:");
    Serial.print(U_t);
    Serial.print(",");

    Serial.print("tempF:");
    Serial.println(tempF);     
  }



  // Advanced Serial Input Functions
  recvWithStartEndMarkers();  
  if (newData == true) {
    parseData();
    newData = false;
  }
  
}

void ident(void){
  // Measurement, Control, Output Command Signal, Serial Data Communication
  unsigned long currentMillis = millis(); // Update current time from the beginning
  if (currentMillis - previousMillis >= Ts) {
    previousMillis = currentMillis;
    sensors.requestTemperatures();  
    tempF = sensors.getTempCByIndex(0); 
    U_t = U_id+U_op;   
    float U_tl = min(max(U_t, 0), Uunits); // Saturated Control Output
    pwmV = int((U_tl/Uunits)*pwmMax);
    analogWriteADJ(pwmPin, pwmV);
    
    if (currentMillis >= 60000 && currentMillis-previousMillis2 >= 30000) {
    i++;
    previousMillis2 = currentMillis; // refresh the last time you RUN
    if (up){
      U_id = 10;
      up = false;
    } else {
       U_id = 0;
      up = true;
    }
  }
  
    Serial.print("U:");
    Serial.print(U_t);
    Serial.print(",");

    Serial.print("tempF:");
    Serial.println(tempF);     
  }



  // Advanced Serial Input Functions
  recvWithStartEndMarkers();  
  if (newData == true) {
    parseData();
    newData = false;
  }
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  sensors.begin();
  sensors.setResolution(12);
  setupPWMadj();
  analogWriteADJ(pwmPin, pwmV);
  int clear=0;
  while(clear<10){
    Serial.println();
  clear=clear +1;    
  }  
  SetTunings(1.3,0.05,0);
  delay(5000);
}

void loop() {
  compute();
}

/* Configure digital pins 9 and 10 as 12-bit PWM outputs (3905 Hz). */
void setupPWMadj() {
  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
      | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
      | _BV(CS10);                    /* no prescaling */
  ICR1 = 0x0fff;                      /* TOP counter value - SETS RESOLUTION/FREQUENCY */
}

/* 12-bit version of analogWrite(). Works only on pins 9 and 10. (MAX VAL=4095) */
void analogWriteADJ(uint8_t pin, uint16_t val){
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
    }
}

//============ Advanced Serial Input Functions

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<'; // Serial input must start with this character
    char endMarker = '>'; // Serial input must end with this character
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    ref = atof(receivedChars);     // convert serial input to a float and update System Reference value with that value

}
