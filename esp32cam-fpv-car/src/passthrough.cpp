#include <Arduino.h>

void setup() {
  // Inicia a comunicação com o computador (Monitor Serial)
  Serial.begin(115200);
  Serial.println("Ponte UART iniciada. Pronta para gravar na ESP32-CAM.");
}

void loop() {
  // O ESP32 programador agora atua como uma ponte transparente.
  // A comunicação entre o computador e a ESP32-CAM é gerenciada
  // diretamente pelo hardware serial. Não é necessário código aqui.
  delay(1000); // Apenas para o loop não rodar vazio e consumir 100% da CPU.
}