#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>


// variaveis para contagem do tempo (ms)
unsigned long TOTAL_MillisA = 0;
unsigned long TOTAL_MillisB = 0;
unsigned long Millis = 0;
// Motor M1, Left Side
  const uint8_t pwm_M1 = 3;   // ENA1 - Enable and PWM
  const uint8_t fwd_M1 = 4;   // IN1 - Forward Drive 
  const uint8_t rwd_M1 = 2;   // IN2 - Reverse Drive 
  //-----------------
  // Motor M2, Right Side
  const uint8_t pwm_M2 = 5;   // ENB - Enable and PWM
  const uint8_t fwd_M2 = 7;   // IN3 - Forward Drive 
  const uint8_t rwd_M2 = 6;  //  IN4 - Reverse Drive
//Configurando HCSR04
Ultrasonic HCSR04(10,11);
//Configurando bluetooth
float Gi = 131;
float Dist = 0;
int G = 0;

//Endereco I2C do MPU6050
const int MPU = 0x68;
//SCL     ->     A5
//SDA     ->     A4
//Variaveis para armazenar valores do acelerometro e giroscópio
float eixoX, eixoY, eixoZ, Tmp, GiX, GiY, GiZ;
float eixoXA, eixoYA, eixoZA, GiXA, GiYA, GiZA;
float Z = 0;
int Ctt = 0;
int CttA = 0;
int C = 0;
float Giro_Z = 0;
float Giro_ZA = 0;
int speed = 250;

void Solic_MPU6050();
void Contagem_giro();
void Em_frente();
void All_stop();
void Giro_Esquerda();
void Giro_Direita();
void fim();
// define pinos motor de



void setup() { 
  // define banda de comunicação serial com computador
  Serial.begin(9600);
  // configura a escala de variação do giroscópio em graus/segundos
  Wire.begin();
  //Inicializa o MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0); 
  Wire.endTransmission(true);
  delay(5); 
  // configurando pinos do motor como OUT
  pinMode(pwm_M1, OUTPUT);
  pinMode(fwd_M1, OUTPUT);
  pinMode(rwd_M1, OUTPUT);
  pinMode(pwm_M2, OUTPUT);
  pinMode(fwd_M2, OUTPUT);
  pinMode(rwd_M2, OUTPUT);
}

void loop() {
  if(C == 0) {
    delay(2000);
    C = 1;
  }
  Dist = HCSR04.read(CM);
  while(Dist > 28) {
    Dist = HCSR04.read(CM);
    Em_frente();
  }
  All_stop();
  delay(600);
  G = 90;
  Giro_Direita();
  delay(600);
  Dist = HCSR04.read(CM);
  while(Dist > 25) {
    Dist = HCSR04.read(CM);
    Em_frente();
  }
  All_stop();
  delay(600);
  G = 70;
  Giro_Direita();
  delay(600);
  Dist = HCSR04.read(CM);
  while(Dist > 25) {
    Dist = HCSR04.read(CM);
    Em_frente();
  }
  All_stop();
  delay(600);
  G = 70;
  Giro_Esquerda();
  delay(600);
  Dist = HCSR04.read(CM);
  while(Dist > 28) {
    Dist = HCSR04.read(CM);
    Em_frente();
  }
  All_stop();
  fim();
}

void Giro_Direita(){    
  // M1 Left
  // Ox
  analogWrite(pwm_M1, speed/1);
  digitalWrite(fwd_M1, HIGH);
  digitalWrite(rwd_M1, LOW);
  // M2 Right 
  // xO
  analogWrite(pwm_M2, speed/1);    
  digitalWrite(fwd_M2, LOW);
  digitalWrite(rwd_M2, HIGH);
  Contagem_giro();
}
void All_stop(){
  // M1 Left
  // Ox
  analogWrite(pwm_M1, LOW);
  digitalWrite(fwd_M1, LOW);
  digitalWrite(rwd_M1, LOW);
  // M2 Right 
  // xO
  analogWrite(pwm_M2, LOW);    
  digitalWrite(fwd_M2, LOW);
  digitalWrite(rwd_M2, LOW);
}
void Giro_Esquerda(){
  // M1 Left
  // Ox
  analogWrite(pwm_M1, speed/1);
  digitalWrite(fwd_M1, LOW);
  digitalWrite(rwd_M1, HIGH);
  // M2 Right 
  // xO
  analogWrite(pwm_M2, speed/1);    
  digitalWrite(fwd_M2, HIGH);
  digitalWrite(rwd_M2, LOW);
  Contagem_giro();
  
}

void Em_frente(){
  // M1 Left
  // Ox
  analogWrite(pwm_M1, speed/1);
  digitalWrite(fwd_M1, HIGH);
  digitalWrite(rwd_M1, LOW);
  // M2 Right 
  // xO
  analogWrite(pwm_M2, speed/1);  
  digitalWrite(fwd_M2, HIGH);
  digitalWrite(rwd_M2, LOW);
}
void Solic_MPU6050(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,14,true);
  //Armazena o valor dos sensores nas variaveis correspondentes
  eixoX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  eixoY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  eixoZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GiX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GiY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GiZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  if(GiZ < 0 ) GiZ = GiZ * -1; // converte para + em caso de -
  GiZA = GiZ/131; // converte o dado puro do MPU6050 para um range de 0 a 250º/s
}

void Contagem_giro() {
  while(Giro_ZA <= G){
    TOTAL_MillisA = millis(); //pega o valor inicial em ms.
    while(Millis <= 25){// enquanto não se passar 100ms ele pega valores do MPU6050
      Ctt++; //contador para média dos graus por segundo em 100ms
      Solic_MPU6050(); //func para salvar os dados puros do MPU6050.
      Z = Z + GiZA; //faz somatório dos valores de º/s para tirar média depois
      TOTAL_MillisB = millis();//pega valor em ms atual
      Millis = TOTAL_MillisB - TOTAL_MillisA; //vai atualizando Millis até estourar em 100ms 
    }// após passar os 100ms, vamos calcular o quanto já girou
    Millis = 0;// zera millis para a proxima rotina
    Z = Z/Ctt; // tiramos a média dos valores em º/s
    Giro_Z = Z * 0.025; // multiplica o valor da média que é em º/s
                        // e multiplica por 0,1 que equivale a 0,1s ou 100ms
    Giro_ZA = Giro_ZA + Giro_Z; //a cada 100ms um valor já deslocado é adicionado ao giro.
    Ctt = 0; //zera o contador para média
    TOTAL_MillisA = millis(); //adiciona o novo valor para a próxima contagem dos 100ms
  }
  All_stop(); // após girar 90º no eixo Z, desliga os motores setando (LOW)
  Giro_ZA = 0;
}



void fim(){
  int x = 0; 
  All_stop();
  while(x == 0)x=0;
}