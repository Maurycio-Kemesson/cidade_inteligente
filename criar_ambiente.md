
## ▶️ Como Executar

### 1. Pré-requisitos

- Python SDK Version 3.12.3
- Node.js 14+ SDK Version 20.18.2
- Flutter SDK SDK Version 3.27.2
- `protoc` (Protocol Buffers compiler) (Python / Version 6.31.1)

### 2. Instalação de dependências

```bash
# Python
pip install protobuf

# Node.js
npm install protobufjs

# Flutter
flutter pub get
```

### 2. Gerar arquivos Protobuf

# Python
protoc --python_out=. protos/mensagem.proto

# Node.js
protoc --js_out=import_style=commonjs,binary:. protos/mensagem.proto

# Flutter
protoc --dart_out=cliente_flutter/lib/protos protos/mensagem.proto

### 3. Rodar os componentes (em terminais separados)
# Gateway
python3 gateway/gateway.py

# Sensor de Temperatura
python3 dispositivos/sensor_temp.py

# Atuador (Poste)
node dispositivos/poste_node/poste.js

# Aplicativo Flutter
cd cliente_flutter
flutter run
