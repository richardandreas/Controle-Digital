class calculoPID {
  private:
    float error;
    float sample;
    float lastSample;
    float kp, ki, kd;
    float P, I, D;
    float sumPID;
    float setPoint;
    unsigned long lastTime;

    // Variaveis Sample Time

    int sampleTime;
    float sumError;
    float lastError;

    // Variaveis Reset Windup

    float outMin, outMax;

  public:
    calculoPID(float _kp, float _ki, float _kd, float _setPoint) {
      kp = _kp;
      ki = _ki;
      kd = _kd;
      setPoint = _setPoint;
      lastTime = 0;
      sampleTime = 1;
      I = 0;
    }

    void addNewSample(float _sample) {
      sample = _sample;
    }

    void Compute() {
      error = setPoint - sample;

      unsigned long t = millis();
      int dt = t - lastTime;
      lastTime = t;

      P = error * kp;
      I = I + (error * ki) * dt;
      D = (lastSample - sample) * kd / dt;
      sumPID = P + I + D;
      lastSample = sample;
    }

    float pid() {
      return sumPID;
    }

    void SetTunings(float _kp, float _ki, float _kd) {
      kp = _kp;
      ki = _ki;
      kd = _kd;
    }

    // Funções Sample Time

    void SetTuningsST(float _sampleTime) {
      sampleTime = _sampleTime;
      ki = ki * ((float)sampleTime / 1000);
      kd = kd / ((float)sampleTime / 1000);
    }

    void SetSampleTime(int NewSampleTime) {
      if (NewSampleTime > 0) {
        float ratio = (float)NewSampleTime / (float)sampleTime;
        ki *= ratio;
        kd /= ratio;
        sampleTime = (float)NewSampleTime;
      }
    }

    void ComputeST() {
      unsigned long now = millis();
      int timeChange = (now - lastTime);

      if (timeChange >= sampleTime) {
        float error = setPoint - sample;
        sumError += error;
        float dErr = (error - lastError);
        sumPID = kp * error + ki * sumError + kd * dErr;
        lastError = error;
        lastTime = now;
      }
    }

    // Funções Derivative Kick

    void ComputeDK() {
      unsigned long now = millis();
      int timeChange = (now - lastTime);

      if (timeChange >= sampleTime) {
        float error = setPoint - sample;
        sumError += error;
        float dErr = (sample - lastSample);
        sumPID = kp * error + ki * sumError + kd * dErr;
        lastSample = sample;
        lastTime = now;
      }
    }

    // Funções Reset Windup

    void ComputeRW() {
      unsigned long now = millis();
      int timeChange = (now - lastTime);

      if (timeChange >= sampleTime) {
        float error = setPoint - sample;
        sumError += (ki * error);
        if (sumError > outMax) {
          sumError = outMax;
        } else if (sumError < outMin) {
          sumError = outMin;
        }
        float dErr = (sample - lastSample);
        sumPID = kp * error + ki * sumError + kd * dErr;
        if (sumPID > outMax) {
          sumPID = outMax;
        } else if (sumError < outMin) {
          sumPID = outMin;
        }
        lastSample = sample;
        lastTime = now;
      }
    }

    void SetOutputLimits(float Min, float Max)
    {
      if (Min > Max) return;
      outMin = Min;
      outMax = Max;

      if (sumError > outMax) {
        sumError = outMax;
      } else if (sumError < outMin) {
        sumError = outMin;
      }
      if (sumPID > outMax) {
        sumPID = outMax;
      } else if (sumError < outMin) {
        sumPID = outMin;
      }
    }
};


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
int servoVerticalPin = 6;
int servoHorizontalPin = 5;

float vpont1 = 0;// variável de leitura dos potenciometros
float vpont2 = 0;

float grau_vertical = 0;
float grau_horizontal = 0;

float erroVertical = 0;
float erroHorizontal = 0;

float iVertical = 0;
float iHorizontal = 0;

float kpVertical = 0.3;
float kiVertical = 0.0001; // Quanto maior o ki, mais lento a reação do servo
float kdVertical = 0.01; // Quanto maior o kd, mais instável o movimento do servo

float kpHorizontal = 0.3;
float kiHorizontal = 0.0001;
float kdHorizontal = 0.01;


unsigned long TAnterior = 0;


float difVerticalAnterior = 0;
float difHorizontalAnterior = 0;


float leituraCima = 0;
float leituraBaixo = 0;
float leituraEsquerda = 0;
float leituraDireita = 0;

Servo vertical;
Servo horizontal;

calculoPID pid_vertical(kpVertical, kiVertical, kdVertical, 0);
calculoPID pid_horizontal(kpHorizontal, kiHorizontal, kdHorizontal, 0);

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT); //pino de saida do led
  pinMode(buttonPin, INPUT); //pino de entrada do botão
  digitalWrite(buttonPin, HIGH); //aciona o resistor putt-up interno
  vertical.attach(6);
  horizontal.attach(5);

  pid_vertical.SetTuningsST(50);
  pid_horizontal.SetTuningsST(50);

  pid_vertical.SetOutputLimits(-5, 5);
  pid_horizontal.SetOutputLimits(-5, 5);
}

void loop() {
  buttonState = digitalRead(buttonPin); // leia o estado do valor do botão de pressão:

  if (buttonState == HIGH) {   // verifique se o botão está pressionado
    digitalWrite(ledPin, HIGH);  // liga o LED:

    vpont1 = analogRead(pont1);  //capturando valores analogicos de cada Potenciometro
    vpont1 = map(vpont1, 0, 1020, 3, 65);

    vpont2 = analogRead(pont2);
    vpont2 = map(vpont2, 0, 1020, 4, 175);

    erroVertical = grau_vertical - vpont1;
    erroHorizontal = grau_horizontal - vpont2;

  } else {

    digitalWrite(ledPin, LOW); // desliga o LED:
    leituraCima = analogRead(LDR_Cima);
    leituraCima = map(leituraCima, 10, 1019, 0, 100);
    leituraBaixo = analogRead(LDR_Baixo);
    leituraBaixo = map(leituraBaixo, 10, 1019, 0, 100);

    leituraEsquerda = analogRead(LDR_Esquerda);
    leituraEsquerda = map(leituraEsquerda, 10, 1019, 0, 100);
    leituraDireita = analogRead(LDR_Direita);
    leituraDireita = map(leituraDireita, 10, 1019, 0, 100);

    erroVertical =  leituraBaixo - leituraCima;
    erroHorizontal =  leituraDireita - leituraEsquerda;
  }

  pid_vertical.addNewSample(erroVertical);
  pid_vertical.Compute();
  pid_horizontal.addNewSample(erroHorizontal);
  pid_horizontal.Compute();

  pid_vertical.ComputeRW();
  pid_horizontal.ComputeRW();

  grau_vertical = grau_vertical + pid_vertical.pid();
  grau_horizontal = grau_horizontal + pid_horizontal.pid();

  //Checagem de limite de angulo

  if (grau_vertical > 65) {
    grau_vertical = 65;
    vertical.write(int(grau_vertical));
    vertical.detach();
  } else if (grau_vertical < 3) {
    grau_vertical = 3;
    vertical.write(int(grau_vertical));
    vertical.detach();
  } else if (!vertical.attached()) {
    vertical.attach(servoVerticalPin);
  }

  //Checagem de diferença e escrita no servo

  if (abs(erroVertical) > 3) {
    if (!vertical.attached()) {
      vertical.attach(servoVerticalPin);
    }
    vertical.write(int(grau_vertical));
  } else {
    vertical.detach();
  }

  //Checagem de limite de angulo

  if (grau_horizontal > 175) {
    grau_horizontal = 175;
    horizontal.write(int(grau_horizontal));
    horizontal.detach();
  } else if (grau_horizontal < 5) {
    grau_horizontal = 5;
    horizontal.write(int(grau_horizontal));
    horizontal.detach();
  } else if (!horizontal.attached()) {
    horizontal.attach(servoHorizontalPin);
  }

  //Checagem de diferença e escrita no servo

  if (abs(erroHorizontal) > 3) {
    if (!horizontal.attached()) {
      horizontal.attach(servoHorizontalPin);
    }
    horizontal.write(int(grau_horizontal));
  } else {
    horizontal.detach();
  }

  Serial.print("erroVertical = ");
  Serial.print(erroVertical);
  Serial.print("      ");
  Serial.print("grau_vertical = ");
  Serial.print(grau_vertical);
  Serial.print("      ");
  Serial.print("leituraCima = ");
  Serial.print(leituraCima);
  Serial.print("      ");
  Serial.print("pidVertical = ");
  Serial.println(pid_vertical.pid());

  delay(1);
}
