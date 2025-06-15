const dgram = require("dgram");
const net = require("net");
const protobuf = require("protobufjs");

const ID = "poste_node1";
let estado = "desligado";

protobuf.load("protos/mensagem.proto").then(root => {
  const DispositivoInfo = root.lookupType("DispositivoInfo");
  const Comando = root.lookupType("Comando");

  const info = DispositivoInfo.create({ tipo: "atuador", id: ID, ip: "127.0.0.1", porta: 6000, estado });
  const msg = DispositivoInfo.encode(info).finish();

  const udp = dgram.createSocket("udp4");
  udp.send(msg, 0, msg.length, 5000, "224.0.0.1");

  net.createServer(socket => {
    socket.on("data", data => {
      const comando = Comando.decode(data);
      console.log(`[Poste] Comando recebido: ${comando.acao} => ${comando.valor}`);
      if (comando.acao === "ligar" || comando.acao === "desligar") {
        estado = comando.valor;

         // Cria uma mensagem de resposta com o estado atualizado
      const resposta = DispositivoInfo.create({ tipo: "atuador", id: ID, ip: "127.0.0.1", porta: 6000, estado });
      const msgResposta = DispositivoInfo.encode(resposta).finish();

      // Envia a mensagem de resposta ao cliente
      socket.write(msgResposta)
      }
    });
  }).listen(6000, () => console.log("Poste TCP ativo na porta 6000"));
});
