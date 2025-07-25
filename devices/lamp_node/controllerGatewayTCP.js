const net = require('net');
const protobuf = require("protobufjs");
const { ActuatorUpdate, JoinRequest, JoinReply, DeviceType, DeviceInfo, Address, ActuatorCommand, ActuatorComply, CommandType, ComplyStatus} = require('./protos/messages_pb');


/**
 * Informaçoes do atuador
 */
const DEVICE_NAME = "LAMP";
let LAMP_STATE = '{"isOn": "yes" , "Color": "yellow", "Brightness": 10}';
const LAMP_METADATA = '{"isOn": "(yes or no)", "Color": "(yellow or white)", "Brightness": "(Between 1 and 10)", "Actions": ["turn_on", "turn_off"]}';
const PORT_ATUADOR = 60555;
const HOST_ATUADOR = '127.0.0.1';

/**
 * Variavel servidor TCP
 */
let server = null;
let connectionGateway = null;
let portTrasferData = null;
let hostTrasferData = null;

function connectToGateway(ipGateway, portGateway) {

    if (connectionGateway != null) return;
    
    portGatewayToConnect = portGateway;
    hostTrasferData = ipGateway;

    // Criando conexao TCP com Gateway
    connectionGateway = net.createConnection({
        host: ipGateway,
        port: portGatewayToConnect
    }, () => {
        console.log(`Conectado ao gateway ${ipGateway}:${portGatewayToConnect}`);
    });

    // JoinRequest para o Gateway
    const dataRegisterActuator = new DeviceInfo();
    dataRegisterActuator.setType(DeviceType.DT_ACTUATOR);
    dataRegisterActuator.setName(DEVICE_NAME);
    dataRegisterActuator.setState(LAMP_STATE);
    dataRegisterActuator.setMetadata(LAMP_METADATA);
    dataRegisterActuator.setTimestamp(formatToCustomISO(new Date()));

    const address = new Address(); // porta do atuador 
    address.setIp(HOST_ATUADOR);
    address.setPort(PORT_ATUADOR);

    const joinRequest = new JoinRequest();
    joinRequest.setDeviceInfo(dataRegisterActuator);
    joinRequest.setDeviceAddress(address);

    const msgSend = joinRequest.serializeBinary();
    connectionGateway.write(msgSend); 

    // JoinReplay do Gateway
    connectionGateway.on('data', (data) => {
        const joinReplay = JoinReply.deserializeBinary(data);
        portTrasferData = joinReplay.getReportPort(); // porta do gateway que o atuador deve usar
        console.log(`Porta do gateway para transferencia de dados: ${portTrasferData}`);
        connectPortTrasferData();
    });
}

function connectPortTrasferData() {
    closeConnectionGateway();
    /* Criando uma nova conexao TCP com o Gateway em outra porta
    connectionGateway = net.createConnection({
        host: hostTrasferData,
        port: portTrasferData,
    }, () => {
        console.log(`Conectado ao gateway ${hostTrasferData}:${portTrasferData}`);
    });
    */
    sendUpdateGateway();

    // criando servidor atuador porta 60555
    startServer();
}

function startServer() {
    // Cria o servidor TCP
    server = net.createServer((socket) => {

        // Receber comandos
        socket.on('data', (data) => {

            socket.setTimeout(2000);
            
            // ActuatorCommand -> recebe
            const actuatorCommand = ActuatorCommand.deserializeBinary(data);

            if (actuatorCommand.getType() == CommandType.CT_GET_STATE) {
                sendDataGateway(ComplyStatus.CS_OK, socket);
            } else if (actuatorCommand.getType() == CommandType.CT_ACTION) {
                if (actuatorCommand.getBody().toLowerCase() == "turn_on") {
                    const jsonState = JSON.parse(LAMP_STATE);
                    jsonState.isOn = "yes";
                    LAMP_STATE = JSON.stringify(jsonState);
                    sendDataGateway(ComplyStatus.CS_OK, socket, socket);
                }
                else if (actuatorCommand.getBody().toLowerCase() == "turn_off") {
                    const jsonState = JSON.parse(LAMP_STATE);
                    jsonState.isOn = "no";
                    LAMP_STATE = JSON.stringify(jsonState);
                    sendDataGateway(ComplyStatus.CS_OK, socket);
                }
                else {
                    sendDataGateway(ComplyStatus.CS_UNKNOWN_ACTION, socket);
                }
            }
            else if (actuatorCommand.getType() == CommandType.CT_SET_STATE) {
                if (validarBody(actuatorCommand.getBody())) {
                    const jsonBody = JSON.parse(actuatorCommand.getBody());
                    const jsonState = JSON.parse(LAMP_STATE);
                    const chaveBody = Object.keys(jsonBody)[0];
                    jsonState[chaveBody] = jsonBody[chaveBody];
                    LAMP_STATE = JSON.stringify(jsonState);
                    sendDataGateway(ComplyStatus.CS_OK, socket);
                }
                else {
                    sendDataGateway(ComplyStatus.CS_INVALID_STATE, socket);
                }
                
            }
            else if (actuatorCommand.getType() == CommandType.CT_UNSPECIFIED) {
                sendDataGateway(ComplyStatus.CS_UNSPECIFIED, socket);
            }
        });

        // Evento: cliente desconectado
        socket.on('end', () => {
            console.log('Cliente desconectado');
        });

        // Evento: erro na conexão
        socket.on('error', (err) => {
            sendDataGateway(ComplyStatus.CS_FAIL);
            console.error('Erro na conexao:', err.message);
        });

    });

    // Inicia o servidor
    server.listen(PORT_ATUADOR, HOST_ATUADOR, () => {
        console.log(`Atuador rodando em ${HOST_ATUADOR}:${PORT_ATUADOR}`);
    });

    // Tratamento de erros gerais do servidor
    server.on('error', (err) => {
        sendDataGateway(ComplyStatus.CS_FAIL);
        console.error('Erro no atuador:', err.message);
    });

    server.on('listening', () => {
        console.log('Servidor está pronto para aceitar novas conexões.');
    });
}

function sendUpdateGateway() {
    closeConnectionGateway();
    openConnectionGateway();
    if (connectionGateway != null) {
        // Envia dados para o gateway
        const actuatorUpdate = new ActuatorUpdate();
        actuatorUpdate.setDeviceName(DEVICE_NAME);
        actuatorUpdate.setState(LAMP_STATE);
        actuatorUpdate.setMetadata(LAMP_METADATA);
        actuatorUpdate.setTimestamp(formatToCustomISO(new Date()));
        actuatorUpdate.setIsOnline(true);

        //Informa a atualizacao
        const updateCurrent = actuatorUpdate.serializeBinary();
        connectionGateway.write(updateCurrent);
    }
    closeConnectionGateway();
}

function sendDataGateway(complyStatus, socketServer) {
    closeConnectionGateway();
    openConnectionGateway();
    if (socketServer != null && connectionGateway != null) {
        // Envia dados para o gateway
        const actuatorUpdate = new ActuatorUpdate();
        actuatorUpdate.setDeviceName(DEVICE_NAME);
        actuatorUpdate.setState(LAMP_STATE);
        actuatorUpdate.setMetadata(LAMP_METADATA);
        actuatorUpdate.setTimestamp(formatToCustomISO(new Date()));
        actuatorUpdate.setIsOnline(true);

        //Informa a atualizacao
        const updateCurrent = actuatorUpdate.serializeBinary();
        connectionGateway.write(updateCurrent);

        // ActuatorComply -> manda uma mensagem o gateway
        const actuatorComply = new ActuatorComply();
        actuatorComply.setStatus(complyStatus);
        actuatorComply.setUpdate(actuatorUpdate);

        const msgResponse = actuatorComply.serializeBinary();

        // Envia a mensagem para gateway
        socketServer.write(msgResponse);
    }
}

function validarBody(body) {
    if (body == null || body == undefined || isInvalidJson(body)) return false;

    const jsonState = JSON.parse(LAMP_STATE);
    const jsonBody = JSON.parse(body);
    
    const chavesOriginais = Object.keys(jsonState);
    const chavesPassadas = Object.keys(jsonBody);
    const todasAsChavesSaoValidas = chavesPassadas.every(chave => chavesOriginais.includes(chave));
    
    if (!todasAsChavesSaoValidas) return false;

    if (chavesPassadas[0] == "Color" && (jsonBody[chavesPassadas[0]].toLowerCase() == "white" || jsonBody[chavesPassadas[0]].toLowerCase() == "yellow")) {
        return true;
    }

    if (chavesPassadas[0] == "isOn" && (jsonBody[chavesPassadas[0]].toLowerCase() == "yes" || jsonBody[chavesPassadas[0]].toLowerCase() == "no")) {
        return true;
    }

    if (chavesPassadas[0] == "Brightness" && jsonBody[chavesPassadas[0]] >= 1 && jsonBody[chavesPassadas[0]] <= 10) {
        return true;
    }

    return false;
}

function isInvalidJson(str) {
  try {
    JSON.parse(str);
    return false;
  } catch (e) {
    return true;
  }
}

function turnOffAtuador() {
    if(server != null) {
        server.close();
    }
}

function closeConnectionGateway() {
    if(connectionGateway != null) {
        connectionGateway.end();
        connectionGateway = null;
    }
}

function openConnectionGateway() {
    if (hostTrasferData && portTrasferData) {
        connectionGateway = net.createConnection({
            host: hostTrasferData,
            port: portTrasferData,
        }, () => {
            console.log(`Conectado ao gateway ${hostTrasferData}:${portTrasferData}`);
        });
        connectionGateway.on('error', (err) => {
            closeConnectionGateway();
        });
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

    const offset = '+00:00';

    return `${year}-${month}-${day}T${hours}:${minutes}:${seconds}.${milliseconds}000${offset}`;
}

// Run after 5s, then every 5s
setTimeout(() => {
    sendUpdateGateway(); // Initial call
    setInterval(sendUpdateGateway, 5000);
}, 5000);

module.exports = {
  server,
  connectToGateway,
  turnOffAtuador,
  closeConnectionGateway
};