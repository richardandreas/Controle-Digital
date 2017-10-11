#include <Servo.h>
// o número do pino do botão
const int buttonPin = 3;
// o número do pino LED
const int ledPin =  4;
// variável para ler o status do botão de pressão
int buttonState = 0;

int LDR_Cima = A0;
int LDR_Baixo = A1;
int LDR_Esquerda = A2;
int LDR_Direita = A3;
int pont1 = A4; // potenciometro 1
int pont2 = A5; // potenciometro 2

float vpont1 = 0;// variável de leitura dos potenciometros
float vpont2 = 0;

float grau_vertical = 0;
float grau_horizontal = 0;

float erroVertical = 0;
float erroHorizontal = 0;

float iVertical = 0;
float iHorizontal = 0;


float kpVertical = 0.0007;
float kiVertical = 0.000045;
float kdVertical = 0.15;


float kpHorizontal = 0.0015;
float kiHorizontal = 0.000045;
float kdHorizontal = 0.15;


unsigned long TAnterior = 0;


float difVerticalAnterior = 0;
float difHorizontalAnterior = 0;


float leituraCima = 0;
float leituraBaixo = 0;
float leituraEsquerda = 0;
float leituraDireita = 0;

Servo vertical;
Servo horizontal;


void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);//pino de saida do led
  pinMode(buttonPin, INPUT);//pino de entrada do botão
  digitalWrite(buttonPin, HIGH);//aciona o resistor putt-up interno
  vertical.attach(6);
  horizontal.attach(5);
}


void loop() {
  unsigned long t = millis();
  int dt = t - TAnterior;
  TAnterior = t;


  buttonState = digitalRead(buttonPin);// leia o estado do valor do botão de pressão:


  if (buttonState == HIGH) {   // verifique se o botão está pressionado
    digitalWrite(ledPin, HIGH);  // liga o LED:

    vpont1 = analogRead(pont1);  //capturando valores analogicos de cada Potenciometro
    vpont1 = map(vpont1, 0, 1020, 0, 100);
    leituraCima = vpont1;
    leituraBaixo = 100 - leituraCima;
    vpont2 = analogRead(pont2);
    vpont2 = map(vpont2, 0, 1020, 0, 100);
    leituraDireita = vpont2;
    leituraEsquerda = 100 - leituraDireita;
    
  } else {
    
    digitalWrite(ledPin, LOW);// desliga o LED:
    leituraCima = analogRead(LDR_Cima);
    leituraCima = map(leituraCima, 10, 1019, 0, 100);
    leituraBaixo = analogRead(LDR_Baixo);
    leituraBaixo = map(leituraBaixo, 10, 1019, 0, 100);

    leituraEsquerda = analogRead(LDR_Esquerda);
    leituraEsquerda = map(leituraEsquerda, 10, 1019, 0, 100);
    leituraDireita = analogRead(LDR_Direita);
    leituraDireita = map(leituraDireita, 10, 1019, 0, 100);
  }


  float difVertical = leituraCima - leituraBaixo;
  erroVertical = difVertical;

  float pVertical = erroVertical * kpVertical;
        iVertical = kiVertical * erroVertical * dt;
  float dVertical = ((difVertical - difVerticalAnterior) * kdVertical) / dt;
  difVerticalAnterior = difVertical;


  float difHorizontal = leituraEsquerda - leituraDireita;
  erroHorizontal =  difHorizontal;

  float pHorizontal = erroHorizontal * kpHorizontal;
        iHorizontal = kiHorizontal * erroHorizontal * dt;
  float dHorizontal = ((difHorizontal - difHorizontalAnterior) * kdHorizontal) / dt;
  difHorizontalAnterior = difHorizontal;


  grau_vertical += pVertical + iVertical + dVertical;
  grau_horizontal += pHorizontal + iHorizontal + dHorizontal;


 /*  Serial.print("Esquerda = ");
  Serial.print(leituraEsquerda);
  Serial.print("      ");
  Serial.print("Direita = ");
  Serial.print(leituraDireita);
  Serial.print("      ");
  Serial.print("erroHorizontal = ");
  Serial.print(difHorizontal);
  Serial.print("      ");
  Serial.print("pVertical = ");
  Serial.print(pVertical);
  Serial.print("      ");
  Serial.print("iVertical = ");
  Serial.print(iVertical);
  Serial.print("      ");
  Serial.print("dVertical = ");
  Serial.print(dVertical);
  Serial.print("      ");
  Serial.print("grau_vertical = ");
  Serial.print(grau_vertical);
  Serial.println(" ");
*/

  if (abs(difVertical) > 3 && grau_vertical < 65 && grau_vertical > 3) {
    vertical.write(int(grau_vertical));
  } else if (grau_vertical > 65) {
    grau_vertical = 65;
  } else if (grau_vertical < 3) {
    grau_vertical = 3;
  }

  if (abs(difHorizontal) > 3 && grau_horizontal < 175 && grau_horizontal > 5) {
    horizontal.write(int(grau_horizontal));
  } else if (grau_horizontal > 175) {
    grau_horizontal = 175;
  } else if (grau_horizontal < 5) {
    grau_horizontal = 5;
  }

  delay(1);
}
