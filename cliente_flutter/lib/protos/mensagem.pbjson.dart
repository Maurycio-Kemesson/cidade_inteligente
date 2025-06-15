//
//  Generated code. Do not modify.
//  source: protos/mensagem.proto
//
// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names

import 'dart:convert' as $convert;
import 'dart:core' as $core;
import 'dart:typed_data' as $typed_data;

@$core.Deprecated('Use dispositivoInfoDescriptor instead')
const DispositivoInfo$json = {
  '1': 'DispositivoInfo',
  '2': [
    {'1': 'tipo', '3': 1, '4': 1, '5': 9, '10': 'tipo'},
    {'1': 'id', '3': 2, '4': 1, '5': 9, '10': 'id'},
    {'1': 'ip', '3': 3, '4': 1, '5': 9, '10': 'ip'},
    {'1': 'porta', '3': 4, '4': 1, '5': 5, '10': 'porta'},
    {'1': 'estado', '3': 5, '4': 1, '5': 9, '10': 'estado'},
  ],
};

/// Descriptor for `DispositivoInfo`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List dispositivoInfoDescriptor = $convert.base64Decode(
    'Cg9EaXNwb3NpdGl2b0luZm8SEgoEdGlwbxgBIAEoCVIEdGlwbxIOCgJpZBgCIAEoCVICaWQSDg'
    'oCaXAYAyABKAlSAmlwEhQKBXBvcnRhGAQgASgFUgVwb3J0YRIWCgZlc3RhZG8YBSABKAlSBmVz'
    'dGFkbw==');

@$core.Deprecated('Use comandoDescriptor instead')
const Comando$json = {
  '1': 'Comando',
  '2': [
    {'1': 'id', '3': 1, '4': 1, '5': 9, '10': 'id'},
    {'1': 'acao', '3': 2, '4': 1, '5': 9, '10': 'acao'},
    {'1': 'valor', '3': 3, '4': 1, '5': 9, '10': 'valor'},
  ],
};

/// Descriptor for `Comando`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List comandoDescriptor = $convert.base64Decode(
    'CgdDb21hbmRvEg4KAmlkGAEgASgJUgJpZBISCgRhY2FvGAIgASgJUgRhY2FvEhQKBXZhbG9yGA'
    'MgASgJUgV2YWxvcg==');

@$core.Deprecated('Use estadoDescriptor instead')
const Estado$json = {
  '1': 'Estado',
  '2': [
    {'1': 'id', '3': 1, '4': 1, '5': 9, '10': 'id'},
    {'1': 'estado', '3': 2, '4': 1, '5': 9, '10': 'estado'},
  ],
};

/// Descriptor for `Estado`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List estadoDescriptor = $convert.base64Decode(
    'CgZFc3RhZG8SDgoCaWQYASABKAlSAmlkEhYKBmVzdGFkbxgCIAEoCVIGZXN0YWRv');

@$core.Deprecated('Use leituraSensorDescriptor instead')
const LeituraSensor$json = {
  '1': 'LeituraSensor',
  '2': [
    {'1': 'id', '3': 1, '4': 1, '5': 9, '10': 'id'},
    {'1': 'tipo', '3': 2, '4': 1, '5': 9, '10': 'tipo'},
    {'1': 'valor', '3': 3, '4': 1, '5': 9, '10': 'valor'},
    {'1': 'timestamp', '3': 4, '4': 1, '5': 9, '10': 'timestamp'},
  ],
};

/// Descriptor for `LeituraSensor`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List leituraSensorDescriptor = $convert.base64Decode(
    'Cg1MZWl0dXJhU2Vuc29yEg4KAmlkGAEgASgJUgJpZBISCgR0aXBvGAIgASgJUgR0aXBvEhQKBX'
    'ZhbG9yGAMgASgJUgV2YWxvchIcCgl0aW1lc3RhbXAYBCABKAlSCXRpbWVzdGFtcA==');

@$core.Deprecated('Use listaDispositivosDescriptor instead')
const ListaDispositivos$json = {
  '1': 'ListaDispositivos',
  '2': [
    {'1': 'dispositivos', '3': 1, '4': 3, '5': 11, '6': '.DispositivoInfo', '10': 'dispositivos'},
  ],
};

/// Descriptor for `ListaDispositivos`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List listaDispositivosDescriptor = $convert.base64Decode(
    'ChFMaXN0YURpc3Bvc2l0aXZvcxI0CgxkaXNwb3NpdGl2b3MYASADKAsyEC5EaXNwb3NpdGl2b0'
    'luZm9SDGRpc3Bvc2l0aXZvcw==');

