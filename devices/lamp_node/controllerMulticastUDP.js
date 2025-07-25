const dgram = require('dgram');
const protobuf = require("protobufjs");
const CONTROLLER_GATEWAY = require('./controllerGatewayTCP');
const { Address } = require('./protos/messages_pb');

/**
 * Endereço multicast
 */
const PORT = 50444;
const MULTICAST_ADDRESS = '224.0.1.0';

/**
 * Cria um socket UDP IPv4
 */
let socket = null;

/**
 * Variáveis de controle
 */
let reconnectAttempts = 0;
const MAX_RECONNECT_ATTEMPTS = 10;
const INITIAL_RECONNECT_DELAY = 1000; // 1 segundo
const MAX_RECONNECT_DELAY = 60000; // 1 minuto
let isSended = false;
/**
 * Função para criar socket
 */
function createSocketMulticast() {
  socket = dgram.createSocket({type: 'udp4', reuseAddr: true});
}

/**
 * Função para conectar ao multicast
 */
function connectToMulticast() {
  try {
    // Configura o socket para receber mensagens multicast
    socket.on('listening', () => {
      socket.addMembership(MULTICAST_ADDRESS); // Se adicionando ao grupo multicast
      reconnectAttempts = 0; // Resetar tentativas após conexão bem-sucedida
    });

    // Manipulador de mensagens recebidas
    socket.on('message', (msg, rinfo) => {
      const message = Address.deserializeBinary(msg);
      const ipGateway = message.getIp();
      const portGateway = message.getPort();
      
      if (ipGateway && portGateway && !isSended) {
        isSended = true;
        CONTROLLER_GATEWAY.connectToGateway(ipGateway, portGateway);
      }
    });

    // Manipulador de erros
    socket.on('error', (err) => {
      console.error('Erro no socket:', err.message);
      CONTROLLER_GATEWAY.closeConnectionGateway();
      handleReconnection();
    });

    socket.bind(PORT); // Associa o socket à porta

  } catch (err) {
    console.error('Erro ao conectar no multicast:', err.message);
    CONTROLLER_GATEWAY.closeConnectionGateway();
    handleReconnection();
  }
}

// Função para lidar com a reconexão
function handleReconnection() {

  isSended = false;

  if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
    console.error('Número máximo de tentativas de reconexão atingido. Desligando...');
    process.exit(1);
  }

  const delay = Math.min(
    INITIAL_RECONNECT_DELAY * Math.pow(2, reconnectAttempts),
    MAX_RECONNECT_DELAY
  );

  reconnectAttempts++;
  console.log(`Tentando reconectar em ${delay/1000} segundos... Tentativa ${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS}`);

  // Fecha o socket antigo
  try {
    socket.close();
  } catch (err) {
    console.error('Erro ao fechar socket:', err.message);
  }

  setTimeout(() => {
    connectToMulticast(); // Tenta reconectar
  }, delay);
}

function closeConnectionAtuador() {
    if(socket != null) {
        socket.close();
    }
}

module.exports = {
  socket,
  createSocketMulticast,
  connectToMulticast,
  closeConnectionAtuador
};
