# 🌐 Cidade Inteligente - Sistemas Distribuídos com Sockets

Este projeto simula uma Cidade Inteligente com sensores e atuadores que se comunicam com um Gateway central. Um aplicativo Flutter permite controle e monitoramento em tempo real.

---

## 🧠 O que é?

Um sistema distribuído para aprendizado de comunicação entre processos. Ele simula:

- **Dispositivos inteligentes** (sensores e atuadores)
- **Gateway central** para coordenação
- **App cliente** para interface de controle

---

## 🔧 Tecnologias Utilizadas

- **Python** (gateway e sensores) SDK Version 3.12.3
- **Node.js** (atuador/atuador extra) SDK Version 20.18.2
- **Flutter/Dart** (aplicativo cliente) SDK Version 3.27.2
- **Sockets TCP e UDP**
- **UDP Multicast** para descoberta de dispositivos
- **Protocol Buffers (protobuf)** para serialização de mensagens (SDK Python Version 6.31.1)

---

## 📦 Estrutura de Diretórios

cidade_inteligente/
├── gateway/ # Código do Gateway Python
│ └── gateway.py
├── dispositivos/
│ ├── sensor_temp.py # Sensor em Python
│ └── poste_node/ # Atuador em Node.js
│ └── poste.js
├── cliente_flutter/ # App Flutter
│ └── lib/
│ ├── main.dart
│ ├── simulador_falhas.dart
│ └── protos/
│ └── mensagem.pb.dart
├── protos/
│ └── mensagem.proto # Definições Protobuf
├── README.md
└── criar_ambiente.md # Instruções de configuração por linguagem