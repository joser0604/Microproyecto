#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "SEMILLERO_ROBOTICA";
const char* password = "Chimuelo2018";

WiFiUDP Udp;
unsigned int localUdpPort = 5005;  // Mismo puerto que el nodo ROS2
char incomingPacket[64];

// Pines para puente H
#define ENA 26   // PWM motor izquierdo
#define IN1 25
#define IN2 33
#define ENB 27   // PWM motor derecho
#define IN3 32
#define IN4 14

// Configurar canales PWM
void setupMotorPWM() {
  ledcSetup(0, 5000, 8);  // canal, frecuencia, resolución
  ledcSetup(1, 5000, 8);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);
}

void setMotor(int channel, int in1, int in2, int pwm) {
  int value = abs(pwm);
  value = constrain(value, 0, 255);
  if (pwm >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  ledcWrite(channel, value);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  setupMotorPWM();

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConectado! IP ESP32: %s\n", WiFi.localIP().toString().c_str());

  Udp.begin(localUdpPort);
  Serial.printf("Escuchando UDP en puerto %d\n", localUdpPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(incomingPacket, 63);
    if (len > 0) incomingPacket[len] = 0;
    
    Serial.printf("Recibido: %s\n", incomingPacket);

    // Variables de PWM
    int pwmL = 0, pwmR = 0;
    // Parsear datos tipo L:120,R:-180
    sscanf(incomingPacket, "L:%d,R:%d", &pwmL, &pwmR);

    // Controlar motores
    setMotor(0, IN1, IN2, pwmL);
    setMotor(1, IN3, IN4, pwmR);
  }
}
