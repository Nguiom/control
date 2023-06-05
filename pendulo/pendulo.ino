
const int pinMotor1=9;
const int pinInvMotor1=10;
const int Uunits =100;
const int pwmMax =4095;
long last=0;
long Ts=100;
const int sensor=1;
bool sentido=false;

float angle=0.0;
float ref =170;
float U_dc=10;
float U_t=0.0;

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void pruebas(){
 
  unsigned long tiempo = millis();

  if (tiempo - last >= Ts ){
    
  U_t=U_dc;  
  float U_tl = min(max(U_t,0), Uunits); // Saturated Control Output
  int pwmMotor1=int((U_tl/Uunits)*pwmMax);
  if(sentido){
    analogWriteADJ(pinInvMotor1,pwmMotor1);
    analogWriteADJ(pinMotor1,0);
  }else {
    analogWriteADJ(pinInvMotor1,0);
    analogWriteADJ(pinMotor1,pwmMotor1);
  }
  
  }
  

  int medida=analogRead(sensor);
  medida=map(medida,0,1024,0,359)-135;
  

    Serial.print("tiempo:");
    Serial.print(tiempo);
    Serial.print(",");
    Serial.print(",angulo:");
    Serial.println(medida); 

  recvWithStartEndMarkers();  
  if (newData == true) {
    parseData();
    newData = false;
  }  

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupPWMadj();
  int clear=0;
  while(clear<10){
    Serial.println();
  clear=clear +1;    
  }  
  delay(5000);


}

void loop() {
  // put your main code here, to run repeatedly:
  pruebas();
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