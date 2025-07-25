const CONTROLLER_MULTICAST = require('./controllerMulticastUDP');
const CONTROLLER_GATEWAY = require('./controllerGatewayTCP');


/**
 * Iniciar atuador
 */
console.log('INICIANDO ATUADOR');
CONTROLLER_MULTICAST.createSocketMulticast();
CONTROLLER_MULTICAST.connectToMulticast();

/**
 * Metodo para encerrar o atuador
 */
process.on('SIGINT', () => {
  console.log('Desligando atuador...');
  CONTROLLER_MULTICAST.closeConnectionAtuador();
  CONTROLLER_GATEWAY.closeConnectionGateway();
  CONTROLLER_GATEWAY.turnOffAtuador();
  process.exit(0);
});

