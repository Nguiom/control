
const int pinMotor1=9;
const int pinInvMotor1=10;
const int Uunits =100;
const int pwmMax =4095;
long last=0;
long Ts=0.01;
const int sensor=1;
bool sentido=false;

float angle=0.0;
float ref =170;
float U_dc=0;
float U_t=0.0;
float suerte=0.0;
float inercia=(0.14163*0.005+0.013*0.0445)*9.7774;

float x[]={0,0,0,0};
float u[]={0,0};
float gain=0;
float y=0;
float yR=0;
int medida=0;
unsigned long tiempo = 0;

float A[]={0.6189,0.08379,-7.003,0.5624};
float B[]={0.01298,0.2386};
float C[]={1,0};
float K[]={18.3959,4.5633};
float L[]={1.5146,-0.4732};
float v=0;
float vF[]={0,0};
int i=0;
float vI=0;
float vIF=0;

struct ganancias {
  float op;
  int angle;
  float *a;
  float *b;
  float *c;
  float *k;
  float *l;  
};

ganancias neutro[]={{29.5,90,A,B,C,K,L},{28.910,100,A,B,C,K,L}};

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void control(ganancias op){

  
  yR=map(medida,0,1024,0,359)-(135-48+op.angle);
  yR=float(yR)*(3.14159/180.0);
  
  if (tiempo-last>= 500 && tiempo >=15000){
  
  v=(yR-y)/(float(tiempo-last)/1000);
  vF[1]=0.3679*vF[0]+0.6321*v;
  gain=-(yR*2.5485*1-vF[1]*(1.7545*1));

  U_t=gain+op.op;

  last=tiempo;
  y=yR;
  vF[0]=vF[1];
  
  
  }else if(tiempo <10000 && tiempo-last>= 500){

    v=(yR-y)/(float(tiempo-last)/1000);
    vF[1]=0.3679*vF[0]+0.6321*v;
    y=yR;
    last=tiempo;
    U_t=26;
  }else if(tiempo <15000 && tiempo>=10000&& tiempo-last>= 500){

    v=(yR-y)/(float(tiempo-last)/1000);
    vF[1]=0.3679*vF[0]+0.6321*v;
    y=yR;
    last=tiempo;
    U_t=29.5;
  }

  float U_tl = min(max(U_t,0), Uunits); // Saturated Control Output
  int pwmMotor1=int((U_tl/Uunits)*pwmMax);

  if(sentido){
    analogWriteADJ(pinInvMotor1,pwmMotor1);
    analogWriteADJ(pinMotor1,0);
  }else {
    analogWriteADJ(pinInvMotor1,0);
    analogWriteADJ(pinMotor1,pwmMotor1);
  }

  

  Serial.print("yR ");
  Serial.print(yR*(180/3.14159));
  Serial.print(" u_tl ");
  Serial.print(U_tl);
  Serial.print(" modo ");
  Serial.print(vIF);
  Serial.print('\n');

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
  pepe();
  
  delay(5000);


}

void loop() {
  // put your main code here, to run repeatedly:
  //simulacion();
  tiempo = millis();
  
  medida=analogRead(sensor);
  yR=map(medida,0,1024,0,359)-(135-51+90);

  control(neutro[0]);

  if(abs(yR-y)<=0.0785&&abs(yR)>=0.0785&&tiempo-last>= 500){
    i=i+1;
    if(i>=10000){
      suerte=(cos(yR)*inercia)/(0.014163*neutro[0].op);
      neutro[0].op=inercia/(0.14163*suerte);
    }
  }else{
    i=0;
  }
}

void pepe(){
Serial.println(F("  ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣴⡶⠟⠛⠉⠉⠉⠙⠛⠷⣦⣀⠀⢀⣠⣤⣶⠶⠾⠿⠶⣶⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡾⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⡟⠉⠁⠀⠀⠀⠀⠀⠀⠙⢿⣄⠀⠀⠀⠀⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⠏⠀⠀⢀⣤⣴⠶⠟⠛⠛⠛⠛⠶⢶⣤⣼⣿⡀⢀⣀⣀⣀⣀⣀⣀⣀⣈⣿⡆⠀⠀⠀⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⠏⠀⠀⠀⠛⠉⠀⠀⠀⠀⠀⠀⣀⣀⣠⣬⣽⣿⣿⡛⠋⠉⠉⣉⣉⣉⣉⣿⣿⣿⣶⣦⣄⡀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⠏⠀⠀⠀⠀⠀⠀⠀⣀⣤⣶⢶⣻⣯⣽⣿⣿⣿⣿⣿⣿⣷⣄⠀⣛⣉⣩⣭⣯⣿⣿⣿⣭⣝⣻⣦⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⢀⣤⣶⠟⠁⠀⠀⠀⠀⠀⣀⣶⢟⣻⣽⡾⠟⠋⠉⠀⢀⣀⣀⣠⣤⣭⣽⣿⡟⠛⣉⣉⣭⣴⣶⣤⣤⣬⣭⣿⣿⣦⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⢠⡿⠋⠀⠀⠀⠀⠀⠀⠀⣾⣿⣿⣿⣉⣁⣀⣤⣴⢶⣿⠿⣿⣿⣯⡉⠀⠈⣿⡿⠛⠛⠉⢉⣿⡛⣿⢿⣷⡄⠉⠙⣿⡆"));
Serial.println(F("⠀⠀⠀⠀⣠⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⢹⣿⣿⣭⣁⠀⣼⣿⡶⣿⣁⣸⣿⣤⣾⣿⣤⣀⣀⣀⣾⣿⣻⣧⣀⣿⣿⣤⣶⡟⠁"));
Serial.println(F("⠀⠀⢀⣴⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠋⠈⠉⠉⠛⠛⠛⠛⠛⠛⢋⣿⡟⠋⠀⠀"));
Serial.println(F("⠀⢀⣾⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⣉⣭⣿⠿⠋⠀⠀⠰⢷⣦⣄⣠⣤⣤⣤⣤⣶⠟⠋⠀⠀⠀⠀"));
Serial.println(F("⢀⣾⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⠾⠛⠛⠉⠀⠀⠀⠀⠀⠀⠀⠈⠙⠿⣯⠉⠀⠈⠻⣦⡀⠀⠀⠀⠀"));
Serial.println(F("⢸⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣷⡄⠀⠀⠀"));
Serial.println(F("⢸⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣤⣤⣴⣦⣤⣤⣤⣤⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣤⣤⡾⣷⡄⠀⠀"));
Serial.println(F("⠀⢿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⡟⠉⢀⣤⣤⣤⣄⣀⣀⣉⠉⠙⠛⠻⠷⠶⠶⠶⠶⠶⠶⠾⠿⠟⠛⠛⠋⠉⠀⢀⣼⡇⠀⠀"));
Serial.println(F("⠀⠘⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⠸⣧⣀⣀⣉⣀⣈⣉⣉⣙⡛⠻⠿⠶⠶⠶⣦⣤⣤⣤⣤⣤⣤⣤⣤⣤⡴⠶⠶⢾⡟⠋⠀⠀⠀"));
Serial.println(F("⠀⠀⠘⢿⣦⡀⠀⠀⠀⠀⠀⠀⠀⢿⣦⣈⣛⠛⠋⠉⠉⠉⠉⠙⠛⠛⠿⠶⠶⣶⣤⣤⣤⣤⣀⣀⣀⣀⣀⣀⣀⣀⣤⡾⠃⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠙⣿⣶⣤⣀⡀⠀⠀⠀⠀⠉⠙⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠉⣉⣿⠿⠋⠉⠉⠀⠀⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠈⠙⠻⢿⣿⣿⣷⣶⣤⣤⣄⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣠⣤⣤⣶⠾⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠛⠛⠿⠿⣯⣿⣿⣛⣛⣛⣛⣛⣛⣛⣛⣛⣛⣛⣛⣿⡿⠟⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"));
Serial.println(F("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠛⠛⠛⠋⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"));
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