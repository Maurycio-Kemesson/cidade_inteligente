// #include <WiFi.h>
// #include <WiFiUdp.h>
// #include <pb_encode.h>
// #include <pb_decode.h>
// #include "mensagem.pb.h"

// const char* ssid = "Kimnet Maurycio 2.4 ";
// const char* password = "@21022611";
// const char* gateway_ip = "192.168.1.12";
// const int udp_port = 7000;
// const int tcp_port = 6010;

// WiFiServer server(tcp_port);
// WiFiUDP udp;

// const int redPin = 25, yellowPin = 26, greenPin = 27;
// String estadoAtual = "vermelho";

// void mudaEstado(String estado) {
//   estadoAtual = estado;
//   digitalWrite(redPin, LOW);
//   digitalWrite(yellowPin, LOW);
//   digitalWrite(greenPin, LOW);
//   if (estado == "vermelho") digitalWrite(redPin, HIGH);
//   else if (estado == "amarelo") digitalWrite(yellowPin, HIGH);
//   else if (estado == "verde") digitalWrite(greenPin, HIGH);
// }

// void enviarEstado() {
//   uint8_t buffer[128];
//   pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
//   Estado msg = Estado_init_zero;
//   strcpy(msg.id, "esp32_semaforo");
//   strcpy(msg.estado, estadoAtual.c_str());
//   pb_encode(&stream, Estado_fields, &msg);
//   udp.beginPacket(gateway_ip, udp_port);
//   udp.write(buffer, stream.bytes_written);
//   udp.endPacket();
// }

// void escutarComandos() {
//   WiFiClient client = server.available();
//   if (client) {
//     uint8_t buffer[128];
//     size_t len = client.read(buffer, sizeof(buffer));
//     Comando cmd = Comando_init_zero;
//     pb_istream_t istream = pb_istream_from_buffer(buffer, len);
//     if (pb_decode(&istream, Comando_fields, &cmd)) {
//       mudaEstado(cmd.valor);
//     }
//     client.stop();
//   }
// }

// void anunciarDispositivo() {
//   DispositivoInfo info = DispositivoInfo_init_zero;
//   strcpy(info.tipo, "esp32");
//   strcpy(info.id, "esp32_semaforo");
//   strcpy(info.ip, WiFi.localIP().toString().c_str());
//   info.porta = tcp_port;
//   strcpy(info.estado, estadoAtual.c_str());

//   uint8_t buffer[256];
//   pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
//   if (!pb_encode(&stream, DispositivoInfo_fields, &info)) {
//     Serial.println("[ESP32] Erro ao codificar mensagem Protobuf");
//     return;
//   }

//   // Inicia suporte a multicast (corrigido)
//   udp.beginMulticast(IPAddress(224, 0, 0, 1), 5000);
//   udp.beginPacket(IPAddress(224, 0, 0, 1), 5000);
//   udp.write(buffer, stream.bytes_written);
//   udp.endPacket();

//   Serial.println("[ESP32] Dispositivo anunciado via multicast");
// }


// void setup() {
//   Serial.begin(115200);
//   pinMode(redPin, OUTPUT);
//   pinMode(yellowPin, OUTPUT);
//   pinMode(greenPin, OUTPUT);
//   mudaEstado("vermelho");

//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.print(".");
//   }
//   Serial.println("\n[ESP32] Conectado: " + WiFi.localIP().toString());

//   udp.begin(7001);
//   server.begin();

//   anunciarDispositivo();  // Anuncia ao Gateway logo no início
// }

// unsigned long ultimo = 0;
// unsigned long ultimoAnuncio = 0;

// void loop() {
//   escutarComandos();

//   // Envia estado a cada 5s
//   if (millis() - ultimo > 5000) {
//     enviarEstado();
//     ultimo = millis();
//   }

//   // Reanuncia dispositivo a cada 30s (para evitar sumir do gateway)
//   if (millis() - ultimoAnuncio > 30000) {
//     anunciarDispositivo();
//     ultimoAnuncio = millis();
//   }
// }

#include <WiFi.h>
#include <WiFiUdp.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "mensagem.pb.h"

const char* ssid = "Kimnet Maurycio 2.4 ";
const char* password = "@21022611";
const char* gateway_ip = "192.168.1.12";

const int udp_port = 7000;
const int tcp_port = 6010;

WiFiServer server(tcp_port);
WiFiUDP udp;

// Pinos do semáforo
const int redPin = 25, yellowPin = 26, greenPin = 27;

// Estado atual do semáforo (nome)
String estadoAtual = "vermelho";

// Controle automático de semáforo
enum EstadoSemaforo { VERMELHO, VERDE, AMARELO };
EstadoSemaforo estadoSemaforo = VERMELHO;
unsigned long ultimoEstadoMudado = 0;
const unsigned long duracaoVermelho = 10000;  // 10s
const unsigned long duracaoVerde   = 8000;    // 8s
const unsigned long duracaoAmarelo = 3000;    // 3s

// Envia estado a cada 5s
unsigned long ultimo = 0;
// Reanuncia a cada 30s
unsigned long ultimoAnuncio = 0;

// Troca LEDs conforme estado
void mudaEstado(String estado) {
  estadoAtual = estado;
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);

  if (estado == "vermelho") digitalWrite(redPin, HIGH);
  else if (estado == "amarelo") digitalWrite(yellowPin, HIGH);
  else if (estado == "verde") digitalWrite(greenPin, HIGH);
}

// Envia estado atual via UDP para o gateway
void enviarEstado() {
  uint8_t buffer[128];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  Estado msg = Estado_init_zero;
  strcpy(msg.id, "esp32_semaforo");
  strcpy(msg.estado, estadoAtual.c_str());

  if (pb_encode(&stream, Estado_fields, &msg)) {
    udp.beginPacket(gateway_ip, udp_port);
    udp.write(buffer, stream.bytes_written);
    udp.endPacket();
  }
}

// Ouve conexões TCP para comandos (ex: mudar estado manualmente)
void escutarComandos() {
  WiFiClient client = server.available();
  if (client) {
    uint8_t buffer[128];
    size_t len = client.read(buffer, sizeof(buffer));
    Comando cmd = Comando_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(buffer, len);
    if (pb_decode(&istream, Comando_fields, &cmd)) {
      mudaEstado(cmd.valor);  // Muda imediatamente conforme comando recebido
      enviarEstado();
    }
    client.stop();
  }
}

// Anuncia o dispositivo via multicast UDP
void anunciarDispositivo() {
  DispositivoInfo info = DispositivoInfo_init_zero;
  strcpy(info.tipo, "esp32");
  strcpy(info.id, "esp32_semaforo");

  String ipStr = WiFi.localIP().toString();
  strcpy(info.ip, ipStr.c_str());

  info.porta = tcp_port;
  strcpy(info.estado, estadoAtual.c_str());

  uint8_t buffer[256];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  if (!pb_encode(&stream, DispositivoInfo_fields, &info)) {
    Serial.println("[ESP32] Erro ao codificar mensagem Protobuf");
    return;
  }

  udp.beginMulticast(IPAddress(224, 0, 0, 1), 5000);
  udp.beginPacket(IPAddress(224, 0, 0, 1), 5000);
  udp.write(buffer, stream.bytes_written);
  udp.endPacket();

  Serial.println("[ESP32] Dispositivo anunciado via multicast");
}

void setup() {
  Serial.begin(115200);
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  mudaEstado("vermelho");

  // Conexão Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\n[ESP32] Conectado: " + WiFi.localIP().toString());

  udp.begin(7001);  // Porta local para enviar UDP
  server.begin();   // Inicia servidor TCP

  anunciarDispositivo();  // Primeiro anúncio logo após conexão
  ultimoEstadoMudado = millis();
}

void loop() {
  escutarComandos();

  unsigned long agora = millis();

  // Alterna o estado do semáforo automaticamente
  switch (estadoSemaforo) {
    case VERMELHO:
      if (agora - ultimoEstadoMudado >= duracaoVermelho) {
        mudaEstado("verde");
        enviarEstado();  // <- Notifica imediatamente
        estadoSemaforo = VERDE;
        ultimoEstadoMudado = agora;
      }
      break;
    case VERDE:
      if (agora - ultimoEstadoMudado >= duracaoVerde) {
        mudaEstado("amarelo");
        enviarEstado();  // <- Notifica imediatamente
        estadoSemaforo = AMARELO;
        ultimoEstadoMudado = agora;
      }
      break;
    case AMARELO:
      if (agora - ultimoEstadoMudado >= duracaoAmarelo) {
        mudaEstado("vermelho");
        enviarEstado();  // <- Notifica imediatamente
        estadoSemaforo = VERMELHO;
        ultimoEstadoMudado = agora;
      }
      break;
  }

  // Envia estado a cada 5 segundos, mesmo se não tiver mudado
  if (agora - ultimo >= 5000) {
    enviarEstado();
    ultimo = agora;
  }

  // Reanuncia o dispositivo a cada 30 segundos
  if (agora - ultimoAnuncio >= 30000) {
    anunciarDispositivo();
    ultimoAnuncio = agora;
  }
}
