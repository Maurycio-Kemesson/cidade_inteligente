import 'dart:io';

import 'package:cliente_flutter/protos/mensagem.pb.dart';
import 'package:cliente_flutter/simulador_falhas.dart';
import 'package:flutter/material.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) => MaterialApp(home: DispositivoUI());
}

class DispositivoUI extends StatefulWidget {
  const DispositivoUI({super.key});

  @override
  _DispositivoUIState createState() => _DispositivoUIState();
}

class _DispositivoUIState extends State<DispositivoUI> {
  Socket? socket;
  List<DispositivoInfo> dispositivos = [];
  bool isSocketConnected = false;

  @override
  void initState() {
    super.initState();
    // Socket.connect("192.168.1.12", 9000).then((s) {
    Socket.connect("10.5.81.18", 9000).then((s) {
      socket = s;
      setState(() {
        isSocketConnected = true;
      });
      solicitarDispositivos();
    }).catchError((error) {
      print("Failed to connect to socket: $error");
    });
  }

  void solicitarDispositivos() {
    if (socket == null) return;

    final cmd = Comando()
      ..id = "LISTAR"
      ..acao = "listar"
      ..valor = "";
    socket!.add(cmd.writeToBuffer());
    socket!.listen((event) {
      final lista = ListaDispositivos.fromBuffer(event);
      setState(() => dispositivos = lista.dispositivos);
    });
  }

  void enviarComando(String id, String acao, String valor) {
    if (socket == null) return;

    final cmd = Comando()
      ..id = id
      ..acao = acao
      ..valor = valor;
    socket!.add(cmd.writeToBuffer());
  }

  @override
  Widget build(BuildContext context) => Scaffold(
        appBar: AppBar(
          title: Text("Cliente Smart City"),
          actions: [
            IconButton(
              icon: Icon(Icons.warning),
              onPressed: () {
                if (!isSocketConnected) {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(content: Text("Socket not connected")),
                  );
                  return;
                }
                Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (context) => SimuladorFalhas(socket: socket!),
                  ),
                );
              },
            )
          ],
        ),
        body: isSocketConnected
            ? ListView.builder(
                itemCount: dispositivos.length,
                itemBuilder: (context, index) {
                  final d = dispositivos[index];
                  return ListTile(
                    title: Text(d.id),
                    subtitle: Text("Tipo: ${d.tipo} | Estado: ${d.estado}"),
                    trailing: Wrap(
                      spacing: 10,
                      children: [
                        ElevatedButton(
                          onPressed: () =>
                              enviarComando(d.id, "ligar", "ligado"),
                          child: Text("Ligar"),
                        ),
                        ElevatedButton(
                          onPressed: () =>
                              enviarComando(d.id, "desligar", "desligado"),
                          child: Text("Desligar"),
                        ),
                      ],
                    ),
                  );
                },
              )
            : Center(child: CircularProgressIndicator()),
      );
}
