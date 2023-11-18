#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <ESP32Servo.h>

#define echoPin1 35
#define trigPin1 32
#define echoPin2 34
#define trigPin2 33

const int DIR = 15;
const int STEP = 25;

Servo servo1;
Servo servo2;
Servo servobraco;

unsigned long duration1, distance1, duration2, distance2;
int menordistancia1, menordistancia2;
int posicaobraco;
int servobracoangulo = 90;
int posaux = 0;
int cont = 0;

WiFiServer server(80);
ModbusIP mb;

void setup() {
  Serial.begin(115200);

  // Conectar ao Wi-Fi
  WiFi.begin("VIVOFIBRA-F2A6", "72232DF2A6");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Configurar pinos
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  servo1.attach(2);
  servo2.attach(4);
  servobraco.attach(13);
  servobraco.write(servobracoangulo);

  // Inicializar servidor Modbus IP
  mb.server();

  // Registradores de Input (Ireg) para armazenar distâncias mínimas
  mb.addIreg(100); // Registrador para armazenar a menor distância do sensor da direita
  mb.addIreg(101); // Registrador para armazenar a menor distância do sensor da esquerda

  // Registradores de Input Status (Ists) para indicar obstáculos em diferentes situações
  mb.addIsts(100); // Obstáculo à direita a menos de 30cm
  mb.addIsts(101); // Obstáculo à direita entre 30cm e 40cm
  mb.addIsts(102); // Obstáculo à esquerda a menos de 30cm
  mb.addIsts(103); // Obstáculo à esquerda entre 30cm e 40cm

  // Bobinas (Coils) para movimentação do braço e da estrutura
  mb.addCoil(100); // Movimentação da estrutura para a direita
  mb.addCoil(101); // Movimentação da estrutura para a esquerda
  mb.addCoil(102); // Movimentação do braço para a direita
  mb.addCoil(103); // Movimentação do braço para a esquerda


  server.begin();
}

// Função para medir a distância usando um sensor ultrassônico
long LeituraDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration / 58.2;

  return distance;
}

// Função para mover a estrutura para esquerda
void MoverEsquerda(int DIR, int STEP, int delayMicros) {
  digitalWrite(DIR, HIGH);
  digitalWrite(STEP, HIGH);
  delayMicroseconds(delayMicros);
  digitalWrite(STEP, LOW);
  delayMicroseconds(delayMicros);
}

// Função para mover a estrutura para direita
void MoverDireita(int DIR, int STEP, int delayMicros) {
  digitalWrite(DIR, LOW);
  digitalWrite(STEP, HIGH);
  delayMicroseconds(delayMicros);
  digitalWrite(STEP, LOW);
  delayMicroseconds(delayMicros);
}

void loop() {
  // Verificar a conexão Wi-Fi e reconectar se necessário
  while(WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin("VIVOFIBRA-F2A6", "72232DF2A6");
    Serial.print(".");
    delay(500);
  }

  // Loop para movimentar cada sensor de distância em 90 graus
  for (int pos = 0; pos <= 90; pos += 5) {
    mb.task();
    servo1.write(pos);
    servo2.write(pos);

    // Leitura de distâncias do sensor ultrassônico à direita
    distance1 = LeituraDistancia(trigPin1, echoPin1);

    // Na posição zero do servo, a variável menordistancia é igualada a distancia lida pelo sensor
    if (pos == 0) {
      menordistancia1 = distance1;
    }

    // a cada incremento de posição do servo a variável menordistancia é atualizada com a distância atual, caso seja menor que a mesma
    if (distance1 < menordistancia1 && pos != 0 ) {
      menordistancia1 = distance1;
    }
    
    // Leitura de distâncias do sensor ultrassônico à esquerda
    distance2 = LeituraDistancia(trigPin2, echoPin2);

    if (pos == 0) {
      menordistancia2 = distance2;
    }

    if (distance2 < menordistancia2 && pos != 0) {
      menordistancia2 = distance2;
    }

    // Movimentação do braço robótico para direita, quando não identificado obstáculos a menos de 30 cm de distância
    if (mb.Coil(102) == 1 && mb.Coil(103) == 0 && servobracoangulo < 180 && menordistancia1 > 30 && menordistancia2 > 30) {
      servobracoangulo += 2;
      servobraco.write(servobracoangulo);
    }
    
    // Movimentação do braço robótico para esquerda
    if (mb.Coil(103) == 1 && mb.Coil(102) == 0 && servobracoangulo > 0 && menordistancia1 > 30 && menordistancia2 > 30) {
      servobracoangulo -= 2;
      servobraco.write(servobracoangulo);
    }

    delay(100); // Aguarde 100 milissegundos antes de passar para a próxima posição
  }

  // Ao finalizar os 90 graus de varredura do sensor, a menor distancia lida é enviada para o supervisório
  mb.Ireg(100, menordistancia1);
  mb.Ireg(101, menordistancia2);

  // Bit nível alto enviado para o supervisório em caso de obstáculo identificado nas zonas de risco ou de alerta
  mb.Ists(100, menordistancia1 < 30 ? HIGH : LOW);
  mb.Ists(101, (menordistancia1 >= 30 && menordistancia1 <= 40) ? HIGH : LOW);
  mb.Ists(102, menordistancia2 < 30 ? HIGH : LOW);
  mb.Ists(103, (menordistancia2 >= 30 && menordistancia2 <= 40) ? HIGH : LOW);

  // Mover a estrutura com velocidade alta para direita em caso de obstáculo identificado acima de 40 cm de distância
  while (mb.Coil(100) == 1 && mb.Coil(101) == 0 && menordistancia1 > 40) {
    mb.task();
    servo1.write(posaux);
    servo2.write(posaux);

    if (posaux == 0) {
      menordistancia1 = distance1 ;
      menordistancia2 = distance2;
    }

    if (cont == 100) {
      distance1 = LeituraDistancia(trigPin1, echoPin1);
      distance2 = LeituraDistancia(trigPin2, echoPin2);

      if (distance1 < menordistancia1 && posaux != 0 ) {
        menordistancia1 = distance1;
      }

      if (distance2 < menordistancia2 && posaux != 0 ) {
        menordistancia2 = distance2;
      }

      posaux = posaux + 5;
      cont = 0;
    }

    if (posaux == 95) {
      posaux = 0;

      // Atualizar registradores e bobinas Modbus com as distâncias e status
      mb.Ireg(100, menordistancia1);
      mb.Ireg(101, menordistancia2);

      mb.Ists(100, menordistancia1 < 30 ? HIGH : LOW);
      mb.Ists(101, (menordistancia1 >= 30 && menordistancia1 <= 40) ? HIGH : LOW);
      mb.Ists(102, menordistancia2 < 30 ? HIGH : LOW);
      mb.Ists(103, (menordistancia2 >= 30 && menordistancia2 <= 40) ? HIGH : LOW);
    }

    MoverDireita(DIR, STEP, 500);

    cont++;
  }

  // Mover a estrutura com velocidade baixa para direita em caso de obstaculo identificado entre 30 cm e 40 cm de distância
  while (mb.Coil(100) == 1 && mb.Coil(101) == 0 && menordistancia1 > 30 && menordistancia1 <= 40) {
    mb.task();
    servo1.write(posaux);
    servo2.write(posaux);

    if (posaux == 0) {
      menordistancia1 = distance1 ;
      menordistancia2 = distance2;
    }

    if (cont == 30) {
      distance1 = LeituraDistancia(trigPin1, echoPin1);
      distance2 = LeituraDistancia(trigPin2, echoPin2);

      if (distance1 < menordistancia1 && posaux != 0) {
        menordistancia1 = distance1;
      }

      if (distance2 < menordistancia2 && posaux != 0 ) {
        menordistancia2 = distance2;
      }

      posaux = posaux + 5;
      cont = 0;
    }

    if (posaux == 95) {
      posaux = 0;

      // Atualizar registradores e bobinas Modbus com as distâncias e status medidos
      mb.Ireg(100, menordistancia1);
      mb.Ireg(101, menordistancia2);

      mb.Ists(100, menordistancia1 < 30 ? HIGH : LOW);
      mb.Ists(101, (menordistancia1 >= 30 && menordistancia1 <= 40) ? HIGH : LOW);
      mb.Ists(102, menordistancia2 < 30 ? HIGH : LOW);
      mb.Ists(103, (menordistancia2 >= 30 && menordistancia2 <= 40) ? HIGH : LOW);
    }

    MoverDireita(DIR, STEP, 2000);

    cont++;
  }

  // Mover a estrutura com velocidade alta para esquerda em caso de obstáculo identificado acima de 40 cm de distância
  while (mb.Coil(101) == 1 && mb.Coil(100) == 0 && menordistancia2 > 40) {
    mb.task();
    servo1.write(posaux);
    servo2.write(posaux);

    if (posaux == 0) {
      menordistancia1 = distance1;
      menordistancia2 = distance2;
    }

    if (cont == 100) {
      distance1 = LeituraDistancia(trigPin1, echoPin1);
      distance2 = LeituraDistancia(trigPin2, echoPin2);

      if (distance1 < menordistancia1 && posaux != 0) {
        menordistancia1 = distance1;
      }

      if (distance2 < menordistancia2 && posaux != 0 ) {
        menordistancia2 = distance2;
      }

      posaux = posaux + 5;
      cont = 0;
    }

    if (posaux == 95) {
      posaux = 0;

      // Atualizar registradores e bobinas Modbus com as distâncias e status
      mb.Ireg(100, menordistancia1);
      mb.Ireg(101, menordistancia2);

      mb.Ists(100, menordistancia1 < 30 ? HIGH : LOW);
      mb.Ists(101, (menordistancia1 >= 30 && menordistancia1 <= 40) ? HIGH : LOW);
      mb.Ists(102, menordistancia2 < 30 ? HIGH : LOW);
      mb.Ists(103, (menordistancia2 >= 30 && menordistancia2 <= 40) ? HIGH : LOW);
    }

    MoverEsquerda(DIR, STEP, 500);
    cont++;
  }

  // Mover a estrutura com velocidade baixa para esquerda em caso de obstaculo identificado entre 30 cm e 40 cm de distância
  while (mb.Coil(101) == 1 && mb.Coil(100) == 0 && menordistancia2 > 30 && menordistancia2 <= 40) {
    mb.task();
    servo1.write(posaux);
    servo2.write(posaux);

    if (posaux == 0) {
      menordistancia1 = distance1;
      menordistancia2 = distance2;
    }

    if (cont == 30) {
      distance1 = LeituraDistancia(trigPin1, echoPin1);
      distance2 = LeituraDistancia(trigPin2, echoPin2);

      if (distance1 < menordistancia1 && posaux != 0) {
        menordistancia1 = distance1;
      }

      if (distance2 < menordistancia2 && posaux != 0 ) {
        menordistancia2 = distance2;
      }

      posaux = posaux + 5;
      cont = 0;
    }

    if (posaux == 95) {
      posaux = 0;

      // Atualizar registradores e bobinas Modbus com as distâncias e status 
      mb.Ireg(100, menordistancia1);
      mb.Ireg(101, menordistancia2);

      mb.Ists(100, menordistancia1 < 30 ? HIGH : LOW);
      mb.Ists(101, (menordistancia1 >= 30 && menordistancia1 <= 40) ? HIGH : LOW);
      mb.Ists(102, menordistancia2 < 30 ? HIGH : LOW);
      mb.Ists(103, (menordistancia2 >= 30 && menordistancia2 <= 40) ? HIGH : LOW);
    }

    MoverEsquerda(DIR, STEP, 2000); 
    cont++;
  } 

  delay(10);
}
