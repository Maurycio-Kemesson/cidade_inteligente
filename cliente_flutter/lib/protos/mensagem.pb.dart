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

import 'dart:core' as $core;

import 'package:protobuf/protobuf.dart' as $pb;

export 'package:protobuf/protobuf.dart' show GeneratedMessageGenericExtensions;

class DispositivoInfo extends $pb.GeneratedMessage {
  factory DispositivoInfo({
    $core.String? tipo,
    $core.String? id,
    $core.String? ip,
    $core.int? porta,
    $core.String? estado,
  }) {
    final result = create();
    if (tipo != null) result.tipo = tipo;
    if (id != null) result.id = id;
    if (ip != null) result.ip = ip;
    if (porta != null) result.porta = porta;
    if (estado != null) result.estado = estado;
    return result;
  }

  DispositivoInfo._();

  factory DispositivoInfo.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory DispositivoInfo.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'DispositivoInfo',
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'tipo')
    ..aOS(2, _omitFieldNames ? '' : 'id')
    ..aOS(3, _omitFieldNames ? '' : 'ip')
    ..a<$core.int>(4, _omitFieldNames ? '' : 'porta', $pb.PbFieldType.O3)
    ..aOS(5, _omitFieldNames ? '' : 'estado')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DispositivoInfo clone() => DispositivoInfo()..mergeFromMessage(this);
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DispositivoInfo copyWith(void Function(DispositivoInfo) updates) =>
      super.copyWith((message) => updates(message as DispositivoInfo))
          as DispositivoInfo;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static DispositivoInfo create() => DispositivoInfo._();
  @$core.override
  DispositivoInfo createEmptyInstance() => create();
  static $pb.PbList<DispositivoInfo> createRepeated() =>
      $pb.PbList<DispositivoInfo>();
  @$core.pragma('dart2js:noInline')
  static DispositivoInfo getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<DispositivoInfo>(create);
  static DispositivoInfo? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get tipo => $_getSZ(0);
  @$pb.TagNumber(1)
  set tipo($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasTipo() => $_has(0);
  @$pb.TagNumber(1)
  void clearTipo() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get id => $_getSZ(1);
  @$pb.TagNumber(2)
  set id($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasId() => $_has(1);
  @$pb.TagNumber(2)
  void clearId() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.String get ip => $_getSZ(2);
  @$pb.TagNumber(3)
  set ip($core.String value) => $_setString(2, value);
  @$pb.TagNumber(3)
  $core.bool hasIp() => $_has(2);
  @$pb.TagNumber(3)
  void clearIp() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.int get porta => $_getIZ(3);
  @$pb.TagNumber(4)
  set porta($core.int value) => $_setSignedInt32(3, value);
  @$pb.TagNumber(4)
  $core.bool hasPorta() => $_has(3);
  @$pb.TagNumber(4)
  void clearPorta() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.String get estado => $_getSZ(4);
  @$pb.TagNumber(5)
  set estado($core.String value) => $_setString(4, value);
  @$pb.TagNumber(5)
  $core.bool hasEstado() => $_has(4);
  @$pb.TagNumber(5)
  void clearEstado() => $_clearField(5);
}

class Comando extends $pb.GeneratedMessage {
  factory Comando({
    $core.String? id,
    $core.String? acao,
    $core.String? valor,
  }) {
    final result = create();
    if (id != null) result.id = id;
    if (acao != null) result.acao = acao;
    if (valor != null) result.valor = valor;
    return result;
  }

  Comando._();

  factory Comando.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Comando.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Comando',
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'id')
    ..aOS(2, _omitFieldNames ? '' : 'acao')
    ..aOS(3, _omitFieldNames ? '' : 'valor')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Comando clone() => Comando()..mergeFromMessage(this);
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Comando copyWith(void Function(Comando) updates) =>
      super.copyWith((message) => updates(message as Comando)) as Comando;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Comando create() => Comando._();
  @$core.override
  Comando createEmptyInstance() => create();
  static $pb.PbList<Comando> createRepeated() => $pb.PbList<Comando>();
  @$core.pragma('dart2js:noInline')
  static Comando getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Comando>(create);
  static Comando? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get id => $_getSZ(0);
  @$pb.TagNumber(1)
  set id($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasId() => $_has(0);
  @$pb.TagNumber(1)
  void clearId() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get acao => $_getSZ(1);
  @$pb.TagNumber(2)
  set acao($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasAcao() => $_has(1);
  @$pb.TagNumber(2)
  void clearAcao() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.String get valor => $_getSZ(2);
  @$pb.TagNumber(3)
  set valor($core.String value) => $_setString(2, value);
  @$pb.TagNumber(3)
  $core.bool hasValor() => $_has(2);
  @$pb.TagNumber(3)
  void clearValor() => $_clearField(3);
}

class Estado extends $pb.GeneratedMessage {
  factory Estado({
    $core.String? id,
    $core.String? estado,
  }) {
    final result = create();
    if (id != null) result.id = id;
    if (estado != null) result.estado = estado;
    return result;
  }

  Estado._();

  factory Estado.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Estado.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Estado',
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'id')
    ..aOS(2, _omitFieldNames ? '' : 'estado')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Estado clone() => Estado()..mergeFromMessage(this);
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Estado copyWith(void Function(Estado) updates) =>
      super.copyWith((message) => updates(message as Estado)) as Estado;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Estado create() => Estado._();
  @$core.override
  Estado createEmptyInstance() => create();
  static $pb.PbList<Estado> createRepeated() => $pb.PbList<Estado>();
  @$core.pragma('dart2js:noInline')
  static Estado getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Estado>(create);
  static Estado? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get id => $_getSZ(0);
  @$pb.TagNumber(1)
  set id($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasId() => $_has(0);
  @$pb.TagNumber(1)
  void clearId() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get estado => $_getSZ(1);
  @$pb.TagNumber(2)
  set estado($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasEstado() => $_has(1);
  @$pb.TagNumber(2)
  void clearEstado() => $_clearField(2);
}

class LeituraSensor extends $pb.GeneratedMessage {
  factory LeituraSensor({
    $core.String? id,
    $core.String? tipo,
    $core.String? valor,
    $core.String? timestamp,
  }) {
    final result = create();
    if (id != null) result.id = id;
    if (tipo != null) result.tipo = tipo;
    if (valor != null) result.valor = valor;
    if (timestamp != null) result.timestamp = timestamp;
    return result;
  }

  LeituraSensor._();

  factory LeituraSensor.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory LeituraSensor.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'LeituraSensor',
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'id')
    ..aOS(2, _omitFieldNames ? '' : 'tipo')
    ..aOS(3, _omitFieldNames ? '' : 'valor')
    ..aOS(4, _omitFieldNames ? '' : 'timestamp')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  LeituraSensor clone() => LeituraSensor()..mergeFromMessage(this);
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  LeituraSensor copyWith(void Function(LeituraSensor) updates) =>
      super.copyWith((message) => updates(message as LeituraSensor))
          as LeituraSensor;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static LeituraSensor create() => LeituraSensor._();
  @$core.override
  LeituraSensor createEmptyInstance() => create();
  static $pb.PbList<LeituraSensor> createRepeated() =>
      $pb.PbList<LeituraSensor>();
  @$core.pragma('dart2js:noInline')
  static LeituraSensor getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<LeituraSensor>(create);
  static LeituraSensor? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get id => $_getSZ(0);
  @$pb.TagNumber(1)
  set id($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasId() => $_has(0);
  @$pb.TagNumber(1)
  void clearId() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get tipo => $_getSZ(1);
  @$pb.TagNumber(2)
  set tipo($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasTipo() => $_has(1);
  @$pb.TagNumber(2)
  void clearTipo() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.String get valor => $_getSZ(2);
  @$pb.TagNumber(3)
  set valor($core.String value) => $_setString(2, value);
  @$pb.TagNumber(3)
  $core.bool hasValor() => $_has(2);
  @$pb.TagNumber(3)
  void clearValor() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.String get timestamp => $_getSZ(3);
  @$pb.TagNumber(4)
  set timestamp($core.String value) => $_setString(3, value);
  @$pb.TagNumber(4)
  $core.bool hasTimestamp() => $_has(3);
  @$pb.TagNumber(4)
  void clearTimestamp() => $_clearField(4);
}

class ListaDispositivos extends $pb.GeneratedMessage {
  factory ListaDispositivos({
    $core.Iterable<DispositivoInfo>? dispositivos,
  }) {
    final result = create();
    if (dispositivos != null) result.dispositivos.addAll(dispositivos);
    return result;
  }

  ListaDispositivos._();

  factory ListaDispositivos.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory ListaDispositivos.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'ListaDispositivos',
      createEmptyInstance: create)
    ..pc<DispositivoInfo>(
        1, _omitFieldNames ? '' : 'dispositivos', $pb.PbFieldType.PM,
        subBuilder: DispositivoInfo.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ListaDispositivos clone() => ListaDispositivos()..mergeFromMessage(this);
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ListaDispositivos copyWith(void Function(ListaDispositivos) updates) =>
      super.copyWith((message) => updates(message as ListaDispositivos))
          as ListaDispositivos;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static ListaDispositivos create() => ListaDispositivos._();
  @$core.override
  ListaDispositivos createEmptyInstance() => create();
  static $pb.PbList<ListaDispositivos> createRepeated() =>
      $pb.PbList<ListaDispositivos>();
  @$core.pragma('dart2js:noInline')
  static ListaDispositivos getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<ListaDispositivos>(create);
  static ListaDispositivos? _defaultInstance;

  @$pb.TagNumber(1)
  $pb.PbList<DispositivoInfo> get dispositivos => $_getList(0);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
