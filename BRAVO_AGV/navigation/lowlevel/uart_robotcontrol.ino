hw_timer_t *timer = NULL;
hw_timer_t *timer2 = NULL;

#define RXD2 16
#define TXD2 17

#define PI_BAUD 115200

HardwareSerial piSerial(2);

// Encoder Interrupt Pins
const int interruptPinA1 = 34;
const int interruptPinB1 = 35;
const int interruptPinA2 = 14;
const int interruptPinB2 = 13;

/***
Motor Driver SetUp
***/
int RPWM = 32;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int LPWM = 33;  // Digital/PWM pin 6 to the LPWM on the BTS7960

// Enable "Left" and "Right" movement
int L_EN = 25;  // connect Digital/PWM pin 7 to L_EN on the BTS7960
int R_EN = 26;  // connect Digital/PWM pin 8 to R_EN on the BTS7960

//second h bridge - mark pins up
int RPWM2 = 19;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int LPWM2 = 18;  // Digital/PWM pin 6 to the LPWM on the BTS7960

// Enable "Left" and "Right" movement
int L_EN2 = 23;  // connect Digital/PWM pin 7 to L_EN on the BTS7960
int R_EN2 = 22;  // connect Digital/PWM pin 8 to R_EN on the BTS7960

const int PWM_Channel0 = 0;
const int PWM_Freq = 31000;
const int PWM_Res = 8;

const int PWM_Channel1 = 1;

const int PWM_Channel2 = 2;

const int PWM_Channel3 = 3;

// General Variables and Control
int standard_speed = 155;
int incoming = 0;
double r_velocity = 0.0;
double l_velocity = 0.0;

////////////
// Control variables motor derecho
////////////
double tiempo=0.0;  
double tiempo_ant =0.0;

// Señal de entrada
double DutyCycle = 0.0;  
// Señales de control
double Error = 0, m=0, Error_D=0, Error_Ant=0;

double s = 0, u = 0, d_w = 0, w = 0;

//double c = 70;
double alpha = 100;
double beta = 20;
int act_control = 0;

double dt = 0.02;

/////////////////
// variables control motor izquierdo
/////////////////
double tiempo2 = 0.0;  
double tiempo_ant2 = 0.0;

// Señal de entrada
double DutyCycle2 = 0.0;  
// Señales de control
double Error2 = 0, m2 = 0, Error_D2 = 0, Error_Ant2 = 0;

double s2 = 0, u2 = 0, d_w2 = 0, w2 = 0;

//double c = 70;
double alpha2 = 100;
double beta2 = 20;
int act_control2 = 0;

double dt2 = 0.02;


// rpm contadores
double counter = 0;
double counter2 = 0;

int signa(float s){
  if (s > 0) {
    return 1;
  }
  else if (s < 0) {
    return (-1);
  }
  else {
    return 0;
  }
}

void setup() {
  // put your setup code here, to run once:

  // initialize all our pins to output
    pinMode(L_EN, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN2, OUTPUT);
    pinMode(R_EN2, OUTPUT);

  // El Pin interruptPinA es entrada, se conecta a la señal del encoder Canal A (tren de pulsos).
  pinMode(interruptPinA1, INPUT);
  // El Pin interruptPinB es entrada, se conecta a la señal del encoder Canal B (tren de pulsos).
  pinMode(interruptPinB1, INPUT);
    // El Pin interruptPinA es entrada, se conecta a la señal del encoder Canal A (tren de pulsos).
  pinMode(interruptPinA2, INPUT);
  // El Pin interruptPinB es entrada, se conecta a la señal del encoder Canal B (tren de pulsos).
  pinMode(interruptPinB2, INPUT);

  // Configuration of pin and timer interruptions
  attachInterrupt(digitalPinToInterrupt(interruptPinA1), ISR_countingA, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinB1), ISR_countingB, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinA2), ISR_countingA2, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinB2), ISR_countingB2, RISING);

  // Timer1: Se dispara cada 20000 microsegundos=0.02 Segundos.
  timer=timerBegin(2,80,true); 
  timerAttachInterrupt(timer, &ISR_RevolucionesPorMinuto, true); 
  // Activa la interrupción: Ejecuta la función RevolucionesPorMinuto            
  timerAlarmWrite(timer, 20000, true);
  timerAlarmEnable(timer);

  timer2 = timerBegin(3, 80, true);
  timerAttachInterrupt(timer2, &ISR_RevolucionesPorMinuto2, true); 
  timerAlarmWrite(timer2, 20000, true);
  timerAlarmEnable(timer2);

  // set all the pins you want to use to LOW
    digitalWrite(L_EN, LOW);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN2, LOW);
    digitalWrite(R_EN2, LOW);
  
  delay(1000);// wait a second
  Serial.begin(115200);
  
  digitalWrite(R_EN, HIGH);  
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN2, HIGH);  
  digitalWrite(L_EN2, HIGH);
  // Setup For PWM Channels
  ledcSetup(PWM_Channel0, PWM_Freq, PWM_Res);
  ledcAttachPin(RPWM, PWM_Channel0);
  ledcSetup(PWM_Channel1, PWM_Freq, PWM_Res);
  ledcAttachPin(LPWM, PWM_Channel1);
  ledcSetup(PWM_Channel2, PWM_Freq, PWM_Res);
  ledcAttachPin(RPWM2, PWM_Channel2);
  ledcSetup(PWM_Channel3, PWM_Freq, PWM_Res);
  ledcAttachPin(LPWM2, PWM_Channel3);

  piSerial.begin(PI_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 115200 baud rate");
}



void loop() {
  // put your main code here, to run repeatedly
  if (piSerial.available()){
    Serial.println("UART Connected");
    incoming = piSerial.parseInt();
    Serial.print("Received: ");
    Serial.println(incoming);
    if (incoming == 1){
      r_velocity = piSerial.parseFloat();
      Serial.print("right wheel: ");
      Serial.println(r_velocity);
    }
    else if (incoming == 2) {
      l_velocity = piSerial.parseFloat();
      Serial.print("left wheel: ");
      Serial.println(l_velocity);
    }
    else {
      r_velocity = 0.0;
      l_velocity = 0.0;
    }
  }
  else {
    r_velocity = 0.0;
    l_velocity = 0.0;
  }
  if (act_control == 1){supertright(r_velocity);}
  if (act_control2 == 1){supertleft(l_velocity);}
}

void supertright(float VelOb){
  double VelocidadEnRPM = 0.0;
 
  dt = (millis()-tiempo_ant)/1000;
  VelocidadEnRPM = 60*counter/(dt*495*2);   
                              
  tiempo = tiempo + dt; 

  //m = 120;
  //m = VelReq;
  VelOb = abs(VelOb);

  
  Error = VelOb - VelocidadEnRPM;
  Error_D = (Error - Error_Ant)/dt;
  Error_Ant = Error;

  // m = kp*Error + ki*Error_Int + kd*Error_D;

  //s = Error_D + c*Error;
  d_w = signa(Error) * dt;
  w = w + d_w;
  u = alpha * sqrt(abs(Error)) * signa(Error) + beta * w;

  
  if(u>255){
    u = 255;
  }
  else if(u<0){
    u = 0;
  }
  
  DutyCycle = u;
  //DutyCycle = 170;
  if(VelOb > 0){
    ledcWrite(PWM_Channel0, DutyCycle);
    ledcWrite(PWM_Channel1, 0);
  }
  else if(VelOb < 0){
    ledcWrite(PWM_Channel1, DutyCycle);
    ledcWrite(PWM_Channel0, 0);
  }
  
  
  // Se reinicia el contador para que vuelva a contar hasta la siguiente interrupción.
  
  counter = 0;
  act_control = 0;
  tiempo_ant = millis();
  
  
  // Se imprime el valor del DutyCycle y de la velocidad del motor en RPM
  Serial.println("right");
  Serial.print(DutyCycle);
  Serial.print(",");
  Serial.print(VelOb);
  Serial.print(",");
  Serial.println(VelocidadEnRPM);
}


void supertleft(float VelOb){
  double VelocidadEnRPM = 0.0;
  dt2 = (millis()-tiempo_ant2)/1000;
  VelocidadEnRPM = 60*counter2/(dt2*500*80*2);   
                              
  tiempo2 = tiempo2 + dt2;
  VelOb = abs(VelOb);

  
  Error2 = VelOb - VelocidadEnRPM;
  Error_D2 = (Error2 - Error_Ant2)/dt2;
  Error_Ant2 = Error2;

  // m = kp*Error + ki*Error_Int + kd*Error_D;

  //s = Error_D + c*Error;
  d_w2 = signa(Error2) * dt2;
  w2 = w2 + d_w2;
  u2 = alpha2 * sqrt(abs(Error2)) * signa(Error2) + beta2 * w2;

  
  if(u2>255){
    u2 = 255;
  }
  else if(u2<0){
    u2 = 0;
  }
  
  DutyCycle2 = u2;
  //DutyCycle = 170;
  if(VelOb > 0){
    ledcWrite(PWM_Channel2, DutyCycle2);
    ledcWrite(PWM_Channel3, 0);
  }
  else if(VelOb < 0){
    ledcWrite(PWM_Channel3, DutyCycle2);
    ledcWrite(PWM_Channel2, 0);
  }
  
  
  // Se reinicia el contador para que vuelva a contar hasta la siguiente interrupción.
  
  counter2 = 0;
  act_control2 = 0;
  tiempo_ant2 = millis();
  
  
  // Se imprime el valor del DutyCycle y de la velocidad del motor en RPM
  Serial.println("left");
  Serial.print(DutyCycle2);
  Serial.print(",");
  Serial.print(VelOb);
  Serial.print(",");
  Serial.println(VelocidadEnRPM);
}
void ISR_countingA() {
  //Contador
  if(digitalRead(interruptPinB1)){
    counter++;                
  }
  else{
    counter--;
  }         
}

void ISR_countingB() {
  //Contador
  if(digitalRead(interruptPinA1)){
    counter--;                   
  }
  else{
    counter++;
 }         
}

void ISR_countingA2() {
  //Contador
  if(digitalRead(interruptPinB2)){
    counter2++;                
  }
  else{
    counter2--;
  }         
}

void ISR_countingB2() {
  //Contador
  if(digitalRead(interruptPinA2)){
    counter2--;                   
  }
  else{
    counter2++;
 }         
}

void ISR_RevolucionesPorMinuto(){
  // Velocidad en Revoluciones por minuto: esta parte "(counter/CuentasPorRevolucion)/Ts" 
  // da las revoluciones por segundo a las que gira el encoder, 
  // se multiplica por 60 segundos para obtener esta velocidad en RPM
  act_control = 1;                                                      
}

void ISR_RevolucionesPorMinuto2(){
  // Velocidad en Revoluciones por minuto: esta parte "(counter/CuentasPorRevolucion)/Ts" 
  // da las revoluciones por segundo a las que gira el encoder, 
  // se multiplica por 60 segundos para obtener esta velocidad en RPM
  act_control2 = 1;                                                      
}