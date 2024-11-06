#include <ESP32Servo.h>
#include <Arduino.h>

#define SENSOR1_PIN 14    // Pin do primeiro sensor
#define SENSOR2_PIN 34    // Pin do segundo sensor
#define SENSORGATE_PIN 32 // Pin do sensor de cancela
#define SERVO_PIN 33       // Pin do servomotor

float distancia = 0.16; // Distância entre sensores em metros (exemplo)

unsigned long tempo1 = 0;
unsigned long tempo2 = 0;
bool sensor1Detectado = false;
bool sensor2Detectado = false;
bool sensorCancela_Detectado = false;

Servo servo; // Mantenha como Servo, mesmo com a biblioteca ESP32Servo

// Protótipo da função abrirCancela
void abrirCancela(int delayAbertura);

void setup()
{
  Serial.begin(115200);
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(SENSORGATE_PIN, INPUT);

  // Inicializa o servomotor na posição fechada (0 graus)
  servo.attach(SERVO_PIN);
  servo.write(0); // Cancela fechada
}

void loop()
{
  if (digitalRead(SENSOR1_PIN) == LOW && !sensor1Detectado)
  { // Objeto detectado no Sensor 1
    tempo1 = micros();
    sensor1Detectado = true;
    Serial.println("Sensor 1 detectado");
  }

  if (digitalRead(SENSOR2_PIN) == LOW && sensor1Detectado && !sensor2Detectado)
  { // Objeto detectado no Sensor 2
    tempo2 = micros();
    sensor2Detectado = true;
    Serial.println("Sensor 2 detectado");

    // Calculo da velocidade
    float tempoSegundos = (tempo2 - tempo1) / 1000000.0; // Converte microssegundos para segundos
    float velocidade = distancia / tempoSegundos;

    Serial.print("Velocidade media: ");
    Serial.print(velocidade);
    Serial.println(" m/s");

    // Controle da velocidade de abertura da cancela
    int delayAbertura = map(velocidade * 375, 0, 500, 25, 1); // Ajuste de 50 ms para 5 ms
    delayAbertura = constrain(delayAbertura, 5, 50);          // Garante que esteja no intervalo

    abrirCancela(delayAbertura);

    // Reseta para a próxima medição
    sensor1Detectado = false;
    sensor2Detectado = false;
  }
}

void abrirCancela(int delayAbertura)
{
  Serial.println("Cancela abrindo");

  // Cancela abrindo até 90 graus
  for (int pos = 0; pos <= 90; pos += 2)
  {
    servo.write(pos);
    delay(delayAbertura);  // Ajuste de velocidade de abertura
  }

  // Verifica se o SENSORGATE_PIN está detectando um objeto
  while (digitalRead(SENSORGATE_PIN) == LOW)  // Enquanto houver objeto
  {
    Serial.println("Objeto detectado no SENSORGATE_PIN. Cancela continua aberta.");

    // Você pode colocar um pequeno delay para evitar o loop ser muito rápido
    delay(100);  // Aguarda 100ms antes de verificar novamente
  }

  // Quando o objeto não for mais detectado, começa a fechar a cancela
  Serial.println("Objeto não detectado, fechando a cancela...");

  delay(2000);

  // Cancela fechando até 0 graus
  for (int pos = 90; pos >= 0; pos -= 2)
  {
    servo.write(pos);
    delay(delayAbertura);  // Ajuste de velocidade de fechamento
  }

  Serial.println("Cancela fechada.");
}

