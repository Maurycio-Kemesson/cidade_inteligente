const grpc = require('@grpc/grpc-js');
const protoLoader = require('@grpc/proto-loader');
const path = require('path');

// Carregar o arquivo proto
const PROTO_PATH = path.join(__dirname, './protos/messages.proto');
const packageDefinition = protoLoader.loadSync(PROTO_PATH, {
    keepCase: true,
    longs: String,
    enums: String,
    defaults: true,
    oneofs: true
});
const messagesProto = grpc.loadPackageDefinition(packageDefinition);

// Obter os construtores das mensagens e serviços
const DeviceType = messagesProto.DeviceType;
const CommandType = messagesProto.CommandType;
const ComplyStatus = messagesProto.ComplyStatus;
const DeviceInfo = messagesProto.DeviceInfo;
const Address = messagesProto.Address;
const JoinRequest = messagesProto.JoinRequest;
const ActuatorUpdate = messagesProto.ActuatorUpdate;
const ActuatorComply = messagesProto.ActuatorComply;

// Serviço gRPC para o atuador
const ActuatorService = messagesProto.GatewayService;

/**
 * Informações do atuador
 */
const DEVICE_NAME = "LAMP";
let LAMP_STATE = '{"isOn": "yes" , "Color": "yellow", "Brightness": 10}';
const LAMP_METADATA = '{"isOn": "(yes or no)", "Color": "(yellow or white)", "Brightness": "(Between 1 and 10)", "Actions": ["turn_on", "turn_off"]}';
const PORT_ATUADOR = 60555;
const HOST_ATUADOR = '127.0.0.1';

// Variáveis para o cliente gRPC do gateway
let gatewayClient = null;

// Servidor gRPC para receber comandos
let actuatorServer = null;

function connectToGateway(ipGateway, portGateway) {
    // Criar um cliente para o serviço de registro do gateway
    const target = `${ipGateway}:${portGateway}`;
    gatewayClient = new messagesProto.GatewayService(target, grpc.credentials.createInsecure());
    
    // Construir a mensagem de registro
    const deviceInfo = new DeviceInfo();
    deviceInfo.setType(DeviceType.DT_ACTUATOR);
    deviceInfo.setName(DEVICE_NAME);
    deviceInfo.setState(LAMP_STATE);
    deviceInfo.setMetadata(LAMP_METADATA);
    deviceInfo.setTimestamp(formatToCustomISO(new Date()));

    const address = new Address();
    address.setIp(HOST_ATUADOR);
    address.setPort(PORT_ATUADOR);

    const joinRequest = new JoinRequest();
    joinRequest.setDeviceInfo(deviceInfo);
    joinRequest.setDeviceAddress(address);

    // Chamar o método RegisterDevice do gateway
    gatewayClient.registerDevice(joinRequest, (err, response) => {
        if (err) {
            console.error('Erro no registro:', err);
            return;
        }
        console.log(`Registrado com sucesso. Porta para atualizações: ${response.getReportPort()}`);
        portTrasferData = response.getReportPort();
        
        // Iniciar o servidor gRPC do atuador
        startActuatorServer();
    });
}

function startActuatorServer() {
    // Cria o servidor gRPC
    actuatorServer = new grpc.Server();
    
    // Adiciona o serviço ActuatorService
    actuatorServer.addService(ActuatorService.service, {
        sendCommand: (call, callback) => {
            const command = call.request;
            handleCommand(command, callback);
        }
    });
    
    // Inicia o servidor
    actuatorServer.bindAsync(
        `${HOST_ATUADOR}:${PORT_ATUADOR}`,
        grpc.ServerCredentials.createInsecure(),
        (err, port) => {
            if (err) {
                console.error('Erro ao iniciar o servidor do atuador:', err);
                return;
            }
            console.log(`Servidor do atuador rodando em ${HOST_ATUADOR}:${port}`);
            actuatorServer.start();
        }
    );
}

function handleCommand(command, callback) {
    const type = command.getType();
    const body = command.getBody();

    let complyStatus = ComplyStatus.CS_OK;
    let update = null;

    switch (type) {
        case CommandType.CT_GET_STATE:
            // Não precisa alterar estado, apenas retorna o estado atual
            break;
        case CommandType.CT_ACTION:
            if (body.toLowerCase() === "turn_on") {
                const jsonState = JSON.parse(LAMP_STATE);
                jsonState.isOn = "yes";
                LAMP_STATE = JSON.stringify(jsonState);
            } else if (body.toLowerCase() === "turn_off") {
                const jsonState = JSON.parse(LAMP_STATE);
                jsonState.isOn = "no";
                LAMP_STATE = JSON.stringify(jsonState);
            } else {
                complyStatus = ComplyStatus.CS_UNKNOWN_ACTION;
            }
            break;
        case CommandType.CT_SET_STATE:
            if (validarBody(body)) {
                const jsonBody = JSON.parse(body);
                const jsonState = JSON.parse(LAMP_STATE);
                for (const key in jsonBody) {
                    jsonState[key] = jsonBody[key];
                }
                LAMP_STATE = JSON.stringify(jsonState);
            } else {
                complyStatus = ComplyStatus.CS_INVALID_STATE;
            }
            break;
        default:
            complyStatus = ComplyStatus.CS_UNSPECIFIED;
    }

    // Criar a atualização para enviar no comply
    update = new ActuatorUpdate();
    update.setDeviceName(DEVICE_NAME);
    update.setState(LAMP_STATE);
    update.setMetadata(LAMP_METADATA);
    update.setTimestamp(formatToCustomISO(new Date()));
    update.setIsOnline(true);

    const comply = new ActuatorComply();
    comply.setStatus(complyStatus);
    comply.setUpdate(update);

    callback(null, comply);
}

function sendUpdateGateway() {
    if (!gatewayClient) {
        console.error('Cliente do gateway não inicializado');
        return;
    }

    const update = new ActuatorUpdate();
    update.setDeviceName(DEVICE_NAME);
    update.setState(LAMP_STATE);
    update.setMetadata(LAMP_METADATA);
    update.setTimestamp(formatToCustomISO(new Date()));
    update.setIsOnline(true);

    // Chamar o método SendActuatorUpdate do gateway
    gatewayClient.sendActuatorUpdate(update, (err, response) => {
        if (err) {
            console.error('Erro ao enviar atualização:', err);
        } else {
            console.log('Atualização enviada com sucesso');
        }
    });
}

function validarBody(body) {
    if (!body || typeof body !== 'string') return false;
    
    try {
        const jsonBody = JSON.parse(body);
        const jsonState = JSON.parse(LAMP_STATE);
        
        const chavesOriginais = Object.keys(jsonState);
        const chavesPassadas = Object.keys(jsonBody);
        
        // Verificar se todas as chaves passadas existem no estado original
        const todasAsChavesSaoValidas = chavesPassadas.every(chave => 
            chavesOriginais.includes(chave)
        );
        
        if (!todasAsChavesSaoValidas) return false;

        // Validações específicas por campo
        for (const chave of chavesPassadas) {
            switch (chave) {
                case "Color":
                    if (!["white", "yellow"].includes(jsonBody[chave].toLowerCase())) {
                        return false;
                    }
                    break;
                    
                case "isOn":
                    if (!["yes", "no"].includes(jsonBody[chave].toLowerCase())) {
                        return false;
                    }
                    break;
                    
                case "Brightness":
                    const brightness = parseInt(jsonBody[chave]);
                    if (isNaN(brightness) || brightness < 1 || brightness > 10) {
                        return false;
                    }
                    break;
                    
                default:
                    return false;
            }
        }
        
        return true;
    } catch (e) {
        return false;
    }
}

function formatToCustomISO(date) {
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    const hours = String(date.getHours()).padStart(2, '0');
    const minutes = String(date.getMinutes()).padStart(2, '0');
    const seconds = String(date.getSeconds()).padStart(2, '0');
    const milliseconds = String(date.getMilliseconds()).padStart(3, '0');

    return `${year}-${month}-${day}T${hours}:${minutes}:${seconds}.${milliseconds}000+00:00`;
}

function turnOffAtuador() {
    if (actuatorServer) {
        actuatorServer.tryShutdown(() => {
            console.log('Servidor do atuador desligado.');
        });
    }
    
    if (gatewayClient) {
        gatewayClient.close();
        gatewayClient = null;
    }
}

// Agendamento de atualizações periódicas
setInterval(sendUpdateGateway, 5000);

module.exports = {
  connectToGateway,
  turnOffAtuador
};