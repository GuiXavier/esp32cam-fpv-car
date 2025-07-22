#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "esp_camera.h"
#include <ArduinoJson.h>

// Inclua a biblioteca para o servidor RTSP aqui
#include "OV2640.h"
#include "OV2640Streamer.h" // <-- ADICIONADO: Necessário para a classe OV2640Streamer
#include "CRtspSession.h"
#include "CStreamer.h"


// =================================================================
// --- 1. CONFIGURAÇÕES GERAIS (Aproveitado do seu código) ---
// =================================================================

// --- DADOS DA REDE E MQTT ---
const char* ssid = "Recanto do Braz"; // SSID da sua rede Wi-Fi
const char* password = "brazax12";      // Senha da sua rede Wi-Fi
const char* mqtt_server = "168.75.109.220"; // IP do seu servidor Oracle
const int   mqtt_port = 1883;
const char* mqtt_user = "fallen";          // Seu usuário MQTT
const char* mqtt_password = "Residentevil4"; // Sua senha MQTT

// --- TÓPICOS MQTT PARA O CARRINHO ---
#define MQTT_TOPIC_COMANDO_MOVIMENTO "carrinho/comando/movimento"
#define MQTT_TOPIC_STATUS_HEALTH   "carrinho/status/health"

// =================================================================
// --- 2. PINOUT (Versão Simplificada sem SD Card) ---
// =================================================================

// --- PINOS DA CÂMERA (Padrão para AI-Thinker) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- PINOS DE CONTROLE ---
const int SERVO_PIN = 13;
const int MOTOR_R_PWM_PIN = 14;
const int MOTOR_L_PWM_PIN = 15;
const int MOTOR_R_EN_PIN = 2;
const int MOTOR_L_EN_PIN = 12;
const int STATUS_LED_PIN = 4; // LED do flash da câmera

// =================================================================
// --- 3. OBJETOS E VARIÁVEIS GLOBAIS ---
// =================================================================
WiFiClient espClient;
PubSubClient client(espClient);
Servo servo;

// --- Configurações para PWM (ledc) ---
const int PWM_FREQ = 5000; // Frequência em Hz para os motores
const int PWM_RESOLUTION = 8; // Resolução de 8 bits (0-255)
const int MOTOR_R_PWM_CHANNEL = 0;
const int MOTOR_L_PWM_CHANNEL = 1;

// --- Variáveis para o LED de status ---
enum SystemState { STATE_BOOTING, STATE_WIFI_CONNECTING, STATE_MQTT_CONNECTING, STATE_RUNNING, STATE_ERROR };
SystemState currentState = STATE_BOOTING;

// --- Objeto da Câmera ---
OV2640 cam;

// --- Servidor RTSP ---
CStreamer *streamer = NULL;
CRtspSession *rtspSession = NULL;
WiFiServer rtspServer(8554);


// =================================================================
// --- 4. DECLARAÇÃO DAS TAREFAS DO FREERTOS ---
// =================================================================
void controlLoopTask(void *pvParameters);
void videoStreamTask(void *pvParameters);

// =================================================================
// --- 5. FUNÇÕES DE DEBUG E STATUS ---
// =================================================================
void updateStatusLED() {
  // Deixaremos simples por enquanto: LED desligado se houver erro, ligado se estiver OK.
  digitalWrite(STATUS_LED_PIN, (currentState == STATE_ERROR) ? LOW : HIGH);
}

void publishHealthStatus() {
  // ✅ CORRIGIDO: 'StaticJsonDocument' foi atualizado para 'JsonDocument'
  JsonDocument doc;
  doc["uptime"] = millis() / 1000;
  doc["heap_free"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["state"] = currentState;

  char buffer[200];
  serializeJson(doc, buffer);
  client.publish(MQTT_TOPIC_STATUS_HEALTH, buffer, true);
}

// =================================================================
// --- 6. FUNÇÕES DE CONTROLE ---
// =================================================================
void setMotorState(int speed, int turn) {
    int speedLeft = speed + turn;
    int speedRight = speed - turn;

    speedLeft = constrain(speedLeft, -255, 255);
    speedRight = constrain(speedRight, -255, 255);

    digitalWrite(MOTOR_L_EN_PIN, speedLeft != 0);
    digitalWrite(MOTOR_R_EN_PIN, speedRight != 0);
    
    // Assumindo que a Ponte H controla direção com PWM reverso.
    // Adapte se for diferente (ex: pinos de direção separados).
    ledcWrite(MOTOR_L_PWM_CHANNEL, abs(speedLeft));
    ledcWrite(MOTOR_R_PWM_CHANNEL, abs(speedRight));

    Serial.printf("Motores: Esq=%d, Dir=%d\n", speedLeft, speedRight);
}

void setServoAngle(int angle) {
    angle = constrain(angle, 45, 135); // Limita o ângulo para segurança
    servo.write(angle);
    Serial.printf("Servo: %d graus\n", angle);
}

// =================================================================
// --- 7. FUNÇÃO DE CALLBACK DO MQTT ---
// =================================================================
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) { message += (char)payload[i]; }
  Serial.printf("Comando MQTT recebido no tópico %s: %s\n", topic, message.c_str());

  if (String(topic) == MQTT_TOPIC_COMANDO_MOVIMENTO) {
    // ✅ CORRIGIDO: 'StaticJsonDocument' foi atualizado para 'JsonDocument'
    JsonDocument doc;
    deserializeJson(doc, message);

    int motor = doc["motor"]; // -255 a 255
    int servo_angle = doc["servo"]; // 45 a 135

    setMotorState(motor, 0); // Simplificado: turn=0 por enquanto
    setServoAngle(servo_angle);
  }
}

// =================================================================
// --- 8. SETUP ---
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando o sistema do carrinho FPV...");

  // Configurar pinos
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(MOTOR_R_EN_PIN, OUTPUT);
  pinMode(MOTOR_L_EN_PIN, OUTPUT);
  servo.attach(SERVO_PIN);

  // Configurar canais PWM para os motores (ledc)
  ledcSetup(MOTOR_R_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_L_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_R_PWM_PIN, MOTOR_R_PWM_CHANNEL);
  ledcAttachPin(MOTOR_L_PWM_PIN, MOTOR_L_PWM_CHANNEL);

  // --- Conexão Wi-Fi ---
  currentState = STATE_WIFI_CONNECTING;
  updateStatusLED();
  Serial.printf("Conectando ao Wi-Fi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // --- Conexão MQTT ---
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // --- Inicialização da Câmera ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  // ✅ CORRIGIDO: Nomes dos pinos atualizados para 'sccb'
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; // Comece com VGA (640x480)
  config.jpeg_quality = 12; // 10-15 é um bom equilíbrio
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Falha na inicialização da câmera: 0x%x\n", err);
    currentState = STATE_ERROR;
    while(true) { delay(100); updateStatusLED(); } // Trava em erro
  }
  Serial.println("Câmera inicializada com sucesso!");

  // --- Criação das Tarefas do FreeRTOS ---
  xTaskCreatePinnedToCore(videoStreamTask, "VideoStream", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlLoopTask, "ControlLoop", 4096, NULL, 2, NULL, 1);

  Serial.println("Tarefas do FreeRTOS criadas. Sistema principal pronto.");
}

// =================================================================
// --- 9. LOOP PRINCIPAL E TAREFAS ---
// =================================================================
void loop() {
  vTaskDelay(portMAX_DELAY);
}

void controlLoopTask(void *pvParameters) {
  Serial.println("Tarefa de Controle iniciada no Core 1.");
  unsigned long lastHealthPublish = 0;
  
  for (;;) {
    if (!client.connected()) {
      currentState = STATE_MQTT_CONNECTING;
      updateStatusLED();
      Serial.print("Tentando conectar ao MQTT...");
      if (client.connect("ESP32_Carrinho_Client", mqtt_user, mqtt_password)) {
        Serial.println("Conectado!");
        client.subscribe(MQTT_TOPIC_COMANDO_MOVIMENTO);
        currentState = STATE_RUNNING;
        updateStatusLED();
      } else {
        Serial.printf(" falhou, rc=%d. Tentando de novo em 5s\n", client.state());
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    }
    client.loop();

    if (millis() - lastHealthPublish > 10000 && client.connected()) {
        lastHealthPublish = millis();
        publishHealthStatus();
    }
    
    vTaskDelay(pdMS_TO_TICKS(20)); // Roda 50 vezes por segundo
  }
}

void videoStreamTask(void *pvParameters) {
    Serial.println("Tarefa de Vídeo iniciada no Core 0.");
    rtspServer.begin();

    for (;;) {
        if (!streamer) {
            WiFiClient rtspClient = rtspServer.available();
            if (rtspClient) {
                // ✅ CORRIGIDO: Usando a classe concreta 'OV2640Streamer'
                streamer = new OV2640Streamer(&rtspClient, cam);
                rtspSession = new CRtspSession(&rtspClient, streamer);
            }
        }

        if (rtspSession) {
            rtspSession->handleRequests(0);
            if (!rtspSession->m_stopped) {
                streamer->streamImage(0);
            } else {
                delete rtspSession;
                delete streamer;
                rtspSession = NULL;
                streamer = NULL;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
