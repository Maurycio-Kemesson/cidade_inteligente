import 'dart:io';

import 'package:cliente_flutter/protos/mensagem.pb.dart';
import 'package:flutter/material.dart';

class SimuladorFalhas extends StatefulWidget {
  final Socket socket;
  const SimuladorFalhas({super.key, required this.socket});

  @override
  _SimuladorFalhasState createState() => _SimuladorFalhasState();
}

class _SimuladorFalhasState extends State<SimuladorFalhas> {
  void desconectarGateway() {
    widget.socket.destroy();
    ScaffoldMessenger.of(context)
        .showSnackBar(SnackBar(content: Text("Desconectado do Gateway")));
  }

  void simularFalhaSensor(String id) {
    final cmd = Comando()
      ..id = id
      ..acao = "falha"
      ..valor = "true";
    widget.socket.add(cmd.writeToBuffer());
  }

  @override
  Widget build(BuildContext context) => Scaffold(
        appBar: AppBar(title: Text("Simulação de Falhas")),
        body: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            children: [
              ElevatedButton(
                onPressed: desconectarGateway,
                child: Text("Desconectar do Gateway"),
              ),
              SizedBox(height: 16),
              ElevatedButton(
                onPressed: () => simularFalhaSensor("sensor_temp1"),
                child: Text("Simular Falha no Sensor de Temperatura"),
              ),
            ],
          ),
        ),
      );
}
