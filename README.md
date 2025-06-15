# 🌐 Cidade Inteligente - Sistemas Distribuídos com Sockets

Este projeto simula uma Cidade Inteligente com sensores e atuadores que se comunicam com um Gateway central. Um aplicativo Flutter permite controle e monitoramento em tempo real.

---

## 🧠 O que é?

Um sistema distribuído para aprendizado de comunicação entre processos. Ele simula:

- **Dispositivos inteligentes**: sensores e atuadores que interagem com o ambiente.
- **Gateway central**: responsável pela coordenação e comunicação entre dispositivos.
- **App cliente**: interface de controle e monitoramento em tempo real.

---

## 🔧 Tecnologias Utilizadas

- **Python**: desenvolvimento do gateway e sensores (SDK versão 3.12.3).
- **Node.js**: implementação de atuadores (SDK versão 20.18.2).
- **Flutter/Dart**: criação do aplicativo cliente (SDK versão 3.27.2).
- **Sockets TCP e UDP**: comunicação entre dispositivos.
- **UDP Multicast**: descoberta de dispositivos na rede.
- **Protocol Buffers (protobuf)**: serialização de mensagens (SDK Python versão 6.31.1).

---

## 📦 Estrutura de Diretórios

```
cidade_inteligente/
├── gateway/                # Código do Gateway em Python
│   └── gateway.py
├── dispositivos/           # Código dos dispositivos inteligentes
│   ├── sensor_temp.py      # Sensor de temperatura em Python
│   └── poste_node/         # Atuador em Node.js
│       └── poste.js
├── cliente_flutter/        # Aplicativo Flutter
│   └── lib/
│       ├── main.dart       # Arquivo principal do app
│       ├── simulador_falhas.dart # Simulação de falhas
│       └── protos/
│           └── mensagem.pb.dart # Arquivo gerado pelo Protobuf
├── protos/                 # Definições de mensagens Protobuf
│   └── mensagem.proto
├── README.md               # Documentação principal
└── criar_ambiente.md       # Instruções de configuração por linguagem
```