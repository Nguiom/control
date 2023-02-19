#include <OneWire.h>                
#include <DallasTemperature.h>

int pwmPin = 6;
int pwmV=120;
float ref=45;
float refTol=ref*0.05;

OneWire ourWire(2);
bool inicio =1;

DallasTemperature sensors(&ourWire);

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(9600);
  sensors.begin();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (inicio){
    inicio=0;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10000);
    digitalWrite(LED_BUILTIN, LOW);
  } 
  analogWrite(pwmPin,pwmV);
  sensors.requestTemperatures();
  float temp= sensors.getTempCByIndex(0);
  Serial.print("Temperatura= ");
  Serial.print(temp);
  Serial.println(" C");
  if(temp>(ref+refTol)){
    pwmV=pwmV-1;
  } else if(temp<(ref-refTol)){
    pwmV=pwmV+1;
  }
  delay(1000);
}
