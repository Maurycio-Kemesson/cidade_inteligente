# 🌐 Cidade Inteligente - Sistemas Distribuídos com Sockets
Este projeto simula uma Cidade Inteligente com sensores e atuadores que se comunicam com um Gateway central. Um aplicativo Flutter permite controle e monitoramento em tempo real. Também há uma cliente CLI que oferece funcionalidades semelhantes ao cliente Flutter.


## 🧠 O que é?

Um sistema distribuído para aprendizado de comunicação entre processos. Ele simula:

- **Dispositivos inteligentes:** sensores e atuadores que interagem com o ambiente.
- **Gateway central:** responsável pela coordenação e comunicação entre dispositivos.
- **App cliente:** interface de controle e monitoramento em tempo real.

## 🔧 Tecnologias Utilizadas

- **Ubuntu-24.04:** os sockets foram configurados tendo em mente uma plataforma Unix;
- **Python v3.12.3:** desenvolvimento do gateway, cliente CLI, sensor de temperatura e semáforo (atuador);
- **Node.js v20.18.2:** poste de iluminação (lâmpada inteligente);
- **Flutter/Dart v3.27.2:** criação do aplicativo cliente;
- **Sockets TCP e UDP:** comunicação entre dispositivos;
- **UDP Multicast:** dispositivos inteligêntes descobrem a localização do gateway usando um grupo multicast;
- **libprotoc v31.1:** compilação das mensagens `.proto`.

## 📦 Estrutura de Diretórios

```
cidade_inteligente/
├── clients/
|   └── flutter_client/     # Cliente GUI Flutter
│   └── simple_client/      # Cliente CLI Python
├── devices/                # Código dos dispositivos inteligentes
│   ├── lamp_node/          # Lâmpada inteligente em Node.js
│   ├── semaphore/          # Semáforo em Python
│   └── temp_sensor/        # Sensor de temperatura em Python
├── exemplos/               # Code snippets
├── gateway/                # Código do Gateway em Python
│   ├── gateway.py               # Entry-point do Gateway
|   ├── db.py                    # Abstração de um banco de dados
|   ├── registration_handler.py  # Módulo responsável pelo multicast e registro de dispostivos
|   ├── sensors_handler.py       # Módulo responsável pelos sensores
|   ├── actuators_handler.py     # Módulo responsável pelos atuadores
|   └── clients_handler.py       # Módulo responsável pelos clientes
├── protos/                 
│   └── messages.proto       # Mensagens do Protobuf
├── python-requirements.txt  # Lista de dependência Python
└── README.md                # Documentação principal
```

## Diagrama de funcionamento
```mermaid
flowchart BT
    subgraph Gateway
        descobrimento([Serviço de Descobrimento])
        registro([Serviço de Registro])
        sensores([Serviço de Sensores])
        atuadores([Serviço de Atuadores])
        relatorios([Gerador de Relatórios])
    end

    desc_descobrimento[Socket UDP enviando o endereço do Serviço de Registro ao grupo multicast 224.0.1.0 na porta 50444. Envia a cada 5 segundos.]
    desc_descobrimento --- descobrimento

    desc_registro[Servidor TCP escutando na porta 50111. É responsável pelo registro de dispositivos inteligentes.]
    desc_registro --- registro

    desc_sensores[Socket UDP escutando na porta 50222. É responsável por receber leituras dos sensores registrados.]
    desc_sensores --- sensores

    desc_atuadores[Servidor TCP escutando na porta 50333. É responsável por receber atualizações dos atuadores registrados.]
    desc_atuadores --- atuadores

    desc_relatorios[Gera a cada 5 segundos relatórios sobre os dispositivos registrados. Os relatórios contêm informações como metadados, estado e disponibilidade. Os clientes podem solicitar os relatórios.]
    desc_relatorios --- relatorios
```

```mermaid
flowchart TD
    subgraph Sensor
        envio([Thread de envio de dados])
        descobrimento([Thread de descobrimento e checagem de disponibilidade])
    end

    enviar[Enviar leitura]
    esperar[Esperar conexão]
    conectar[Se registrar no Gateway]
    desconectar[Desconectar o sensor]
    continuar[Continuar escutando]
    continuar2[Continuar conectado]

    descp1{Conectado ao Gateway?}
    descp2{Escutou o endereço do Gateway no grupo multicast?}
    descp3{3 falhas seguidas ao tentar receber endereço?}

    descobrimento-->descp1
    descp1-->|Não|descp2
    descp1-->|Sim|descp3
    descp3-->|Sim|desconectar
    descp3-->|Não|continuar2
    descp2-->|Sim|conectar
    descp2-->|Nao|continuar

    envio-->envp1
    envp1{Conectado ao Gateway?}
    envp1-->|Sim|enviar
    envp1-->|Não|esperar
```

```mermaid
flowchart TD
    subgraph Atuador
        envio([Thread de envio de dados])
        descobrimento([Thread de descobrimento e checagem de disponibilidade])
        comandos([Servidor TCP esperando comandos do Gateway])
    end

    enviar[Enviar atualização ao Gateway]
    esperar[Esperar conexão]
    conectar[Se registrar no Gateway]
    desconectar[Desconectar o sensor]
    continuar[Continuar escutando]
    continuar2[Continuar conectado]

    descp1{Conectado ao Gateway?}
    descp2{Escutou o endereço do Gateway no grupo multicast?}
    descp3{3 falhas seguidas ao tentar receber endereço?}

    descobrimento-->descp1
    descp1-->|Não|descp2
    descp1-->|Sim|descp3
    descp3-->|Sim|desconectar
    descp3-->|Não|continuar2
    descp2-->|Sim|conectar
    descp2-->|Nao|continuar

    envio-->envp1
    envp1{Atualização de estado?}
    envp2{Conectado ao Gateway?}
    envp1-->|Sim|envp2
    envp2-->|Sim|enviar
    envp2-->|Não|esperar
```

## ▶️ Como Executar

### 1. Compilar o arquivo de mensagens

```bash
$ cd protos/
$ protoc --version
libprotoc 31.1
# Python
$ protoc --python_out=. --pyi_out=. messages.proto
# Node.js
$ protoc --js_out=import_style=commonjs,binary:. messages.proto
# Flutter
$ protoc --dart_out=. messages.proto
```

### 2. Rodar os processos

**Python:**
```bash
$ cd cidade_inteligente/
$ python3.12 -m venv venv
$ source venv/bin/activate
(venv) $ pip install -r python-requirements.txt
(venv) $ python gateway/gateway.py --help
```
ou
```bash
(venv) $ python devices/semaphore/semaphore.py --help
```
ou
```bash
(venv) $ python devices/temp_sensor/temp_sensor.py --help
```
ou
```bash
(venv) $ python clients/simple_client/simple_client.py
>>> help
```

**Node.js:**
```bash
$ cd cidade_inteligente/
$ npm install protobufjs
$ node devices/lamp_node/lamp.js
```

**Flutter:**
```bash
$ cd cidade_inteligente/clients/flutter_client
$ flutter pub get
$ flutter run
```
