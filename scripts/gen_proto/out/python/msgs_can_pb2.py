# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: msgs_can.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='msgs_can.proto',
  package='msgs_can',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0emsgs_can.proto\x12\x08msgs_can\".\n\x07\x42\x61seVel\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\r\n\x05theta\x18\x03 \x01(\x02\"\xda\x02\n\x06Status\x12\x11\n\ttimestamp\x18\x01 \x01(\x02\x12+\n\x06status\x18\x02 \x01(\x0e\x32\x1b.msgs_can.Status.StatusType\x12)\n\x05\x65rror\x18\x03 \x01(\x0e\x32\x1a.msgs_can.Status.ErrorType\x12\x0f\n\x07message\x18\x04 \x01(\t\"3\n\nStatusType\x12\x06\n\x02OK\x10\x00\x12\x08\n\x04INIT\x10\x01\x12\x08\n\x04WARN\x10\x02\x12\t\n\x05\x45RROR\x10\x03\"\x9e\x01\n\tErrorType\x12\x08\n\x04NONE\x10\x00\x12\x14\n\x10INIT_PERIPHERALS\x10\x01\x12\x0c\n\x08INIT_CAN\x10\x02\x12\x10\n\x0cPROTO_ENCODE\x10\x03\x12\x10\n\x0cPROTO_DECODE\x10\x04\x12\x13\n\x0f\x43MD_VEL_TIMEOUT\x10\x05\x12\n\n\x06\x43\x41N_TX\x10\x06\x12\n\n\x06\x43\x41N_RX\x10\x07\x12\x12\n\x0eINVALID_CONFIG\x10\x08\"0\n\x0cStatusReport\x12 \n\x06status\x18\x01 \x01(\x0b\x32\x10.msgs_can.Status\"M\n\x03Log\x12$\n\x06\x63onfig\x18\x01 \x01(\x0b\x32\x14.msgs_can.BaseConfig\x12 \n\x06status\x18\x02 \x03(\x0b\x32\x10.msgs_can.Status\"c\n\nBaseConfig\x12\x11\n\tmax_accel\x18\x01 \x01(\x02\x12\x17\n\x0f\x63md_vel_timeout\x18\x02 \x01(\x02\x12\x14\n\x0cwheel_radius\x18\x03 \x01(\x02\x12\x13\n\x0b\x62\x61se_radius\x18\x04 \x01(\x02\"W\n\rRetBaseConfig\x12$\n\x06\x63onfig\x18\x01 \x01(\x0b\x32\x14.msgs_can.BaseConfig\x12 \n\x06status\x18\x02 \x01(\x0b\x32\x10.msgs_can.Status\"f\n\x07ImuData\x12\r\n\x05\x61\x63\x63_x\x18\x01 \x01(\x02\x12\r\n\x05\x61\x63\x63_y\x18\x02 \x01(\x02\x12\r\n\x05\x61\x63\x63_z\x18\x03 \x01(\x02\x12\x0e\n\x06gyro_x\x18\x04 \x01(\x02\x12\x0e\n\x06gyro_y\x18\x05 \x01(\x02\x12\x0e\n\x06gyro_z\x18\x06 \x01(\x02\"=\n\x06\x41\x63tCmd\x12$\n\x06\x61\x63tion\x18\x01 \x01(\x0e\x32\x14.msgs_can.ActActions\x12\r\n\x05value\x18\x02 \x01(\x02\"h\n\tActStatus\x12 \n\x06status\x18\x01 \x01(\x0b\x32\x10.msgs_can.Status\x12$\n\x06\x61\x63tion\x18\x02 \x01(\x0e\x32\x14.msgs_can.ActActions\x12\x13\n\x0bplant_count\x18\x03 \x01(\x05\"%\n\x10LedRingDistances\x12\x11\n\tdistances\x18\x01 \x03(\x02\"\xa7\x01\n\x12TrackingSensorData\x12\x37\n\x06status\x18\x01 \x01(\x0e\x32\'.msgs_can.TrackingSensorData.StatusType\x12\x11\n\tpose_x_mm\x18\x02 \x01(\x02\x12\x11\n\tpose_y_mm\x18\x03 \x01(\x02\x12\x11\n\ttheta_rad\x18\x04 \x01(\x02\"\x1f\n\nStatusType\x12\x06\n\x02OK\x10\x00\x12\t\n\x05\x45RROR\x10\x01\"N\n\x11TrackingSensorStd\x12\x12\n\npose_x_std\x18\x01 \x01(\x02\x12\x12\n\npose_y_std\x18\x02 \x01(\x02\x12\x11\n\ttheta_std\x18\x03 \x01(\x02\"C\n\x1fResetAndCalibrateTrackingSensor\x12\r\n\x05reset\x18\x01 \x01(\x08\x12\x11\n\tcalibrate\x18\x02 \x01(\x08*~\n\nActActions\x12\x15\n\x11START_GRAB_PLANTS\x10\x00\x12\x14\n\x10STOP_GRAB_PLANTS\x10\x01\x12\x11\n\rRELEASE_PLANT\x10\x02\x12\x14\n\x10TURN_SOLAR_PANEL\x10\x03\x12\x10\n\x0cINITIALIZING\x10\x04\x12\x08\n\x04\x46REE\x10\x05'
)

_ACTACTIONS = _descriptor.EnumDescriptor(
  name='ActActions',
  full_name='msgs_can.ActActions',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='START_GRAB_PLANTS', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STOP_GRAB_PLANTS', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RELEASE_PLANT', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TURN_SOLAR_PANEL', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INITIALIZING', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='FREE', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1375,
  serialized_end=1501,
)
_sym_db.RegisterEnumDescriptor(_ACTACTIONS)

ActActions = enum_type_wrapper.EnumTypeWrapper(_ACTACTIONS)
START_GRAB_PLANTS = 0
STOP_GRAB_PLANTS = 1
RELEASE_PLANT = 2
TURN_SOLAR_PANEL = 3
INITIALIZING = 4
FREE = 5


_STATUS_STATUSTYPE = _descriptor.EnumDescriptor(
  name='StatusType',
  full_name='msgs_can.Status.StatusType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INIT', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WARN', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=211,
  serialized_end=262,
)
_sym_db.RegisterEnumDescriptor(_STATUS_STATUSTYPE)

_STATUS_ERRORTYPE = _descriptor.EnumDescriptor(
  name='ErrorType',
  full_name='msgs_can.Status.ErrorType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INIT_PERIPHERALS', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INIT_CAN', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PROTO_ENCODE', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PROTO_DECODE', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CMD_VEL_TIMEOUT', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CAN_TX', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CAN_RX', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INVALID_CONFIG', index=8, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=265,
  serialized_end=423,
)
_sym_db.RegisterEnumDescriptor(_STATUS_ERRORTYPE)

_TRACKINGSENSORDATA_STATUSTYPE = _descriptor.EnumDescriptor(
  name='StatusType',
  full_name='msgs_can.TrackingSensorData.StatusType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1193,
  serialized_end=1224,
)
_sym_db.RegisterEnumDescriptor(_TRACKINGSENSORDATA_STATUSTYPE)


_BASEVEL = _descriptor.Descriptor(
  name='BaseVel',
  full_name='msgs_can.BaseVel',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='msgs_can.BaseVel.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='msgs_can.BaseVel.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='theta', full_name='msgs_can.BaseVel.theta', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=28,
  serialized_end=74,
)


_STATUS = _descriptor.Descriptor(
  name='Status',
  full_name='msgs_can.Status',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='msgs_can.Status.timestamp', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='status', full_name='msgs_can.Status.status', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='error', full_name='msgs_can.Status.error', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='message', full_name='msgs_can.Status.message', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _STATUS_STATUSTYPE,
    _STATUS_ERRORTYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=77,
  serialized_end=423,
)


_STATUSREPORT = _descriptor.Descriptor(
  name='StatusReport',
  full_name='msgs_can.StatusReport',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='msgs_can.StatusReport.status', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=425,
  serialized_end=473,
)


_LOG = _descriptor.Descriptor(
  name='Log',
  full_name='msgs_can.Log',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='config', full_name='msgs_can.Log.config', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='status', full_name='msgs_can.Log.status', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=475,
  serialized_end=552,
)


_BASECONFIG = _descriptor.Descriptor(
  name='BaseConfig',
  full_name='msgs_can.BaseConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='max_accel', full_name='msgs_can.BaseConfig.max_accel', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='cmd_vel_timeout', full_name='msgs_can.BaseConfig.cmd_vel_timeout', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='wheel_radius', full_name='msgs_can.BaseConfig.wheel_radius', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='base_radius', full_name='msgs_can.BaseConfig.base_radius', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=554,
  serialized_end=653,
)


_RETBASECONFIG = _descriptor.Descriptor(
  name='RetBaseConfig',
  full_name='msgs_can.RetBaseConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='config', full_name='msgs_can.RetBaseConfig.config', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='status', full_name='msgs_can.RetBaseConfig.status', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=655,
  serialized_end=742,
)


_IMUDATA = _descriptor.Descriptor(
  name='ImuData',
  full_name='msgs_can.ImuData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='acc_x', full_name='msgs_can.ImuData.acc_x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='acc_y', full_name='msgs_can.ImuData.acc_y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='acc_z', full_name='msgs_can.ImuData.acc_z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='gyro_x', full_name='msgs_can.ImuData.gyro_x', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='gyro_y', full_name='msgs_can.ImuData.gyro_y', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='gyro_z', full_name='msgs_can.ImuData.gyro_z', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=744,
  serialized_end=846,
)


_ACTCMD = _descriptor.Descriptor(
  name='ActCmd',
  full_name='msgs_can.ActCmd',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='action', full_name='msgs_can.ActCmd.action', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='msgs_can.ActCmd.value', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=848,
  serialized_end=909,
)


_ACTSTATUS = _descriptor.Descriptor(
  name='ActStatus',
  full_name='msgs_can.ActStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='msgs_can.ActStatus.status', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='action', full_name='msgs_can.ActStatus.action', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='plant_count', full_name='msgs_can.ActStatus.plant_count', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=911,
  serialized_end=1015,
)


_LEDRINGDISTANCES = _descriptor.Descriptor(
  name='LedRingDistances',
  full_name='msgs_can.LedRingDistances',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='distances', full_name='msgs_can.LedRingDistances.distances', index=0,
      number=1, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1017,
  serialized_end=1054,
)


_TRACKINGSENSORDATA = _descriptor.Descriptor(
  name='TrackingSensorData',
  full_name='msgs_can.TrackingSensorData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='msgs_can.TrackingSensorData.status', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pose_x_mm', full_name='msgs_can.TrackingSensorData.pose_x_mm', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pose_y_mm', full_name='msgs_can.TrackingSensorData.pose_y_mm', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='theta_rad', full_name='msgs_can.TrackingSensorData.theta_rad', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _TRACKINGSENSORDATA_STATUSTYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1057,
  serialized_end=1224,
)


_TRACKINGSENSORSTD = _descriptor.Descriptor(
  name='TrackingSensorStd',
  full_name='msgs_can.TrackingSensorStd',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='pose_x_std', full_name='msgs_can.TrackingSensorStd.pose_x_std', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pose_y_std', full_name='msgs_can.TrackingSensorStd.pose_y_std', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='theta_std', full_name='msgs_can.TrackingSensorStd.theta_std', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1226,
  serialized_end=1304,
)


_RESETANDCALIBRATETRACKINGSENSOR = _descriptor.Descriptor(
  name='ResetAndCalibrateTrackingSensor',
  full_name='msgs_can.ResetAndCalibrateTrackingSensor',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='reset', full_name='msgs_can.ResetAndCalibrateTrackingSensor.reset', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='calibrate', full_name='msgs_can.ResetAndCalibrateTrackingSensor.calibrate', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1306,
  serialized_end=1373,
)

_STATUS.fields_by_name['status'].enum_type = _STATUS_STATUSTYPE
_STATUS.fields_by_name['error'].enum_type = _STATUS_ERRORTYPE
_STATUS_STATUSTYPE.containing_type = _STATUS
_STATUS_ERRORTYPE.containing_type = _STATUS
_STATUSREPORT.fields_by_name['status'].message_type = _STATUS
_LOG.fields_by_name['config'].message_type = _BASECONFIG
_LOG.fields_by_name['status'].message_type = _STATUS
_RETBASECONFIG.fields_by_name['config'].message_type = _BASECONFIG
_RETBASECONFIG.fields_by_name['status'].message_type = _STATUS
_ACTCMD.fields_by_name['action'].enum_type = _ACTACTIONS
_ACTSTATUS.fields_by_name['status'].message_type = _STATUS
_ACTSTATUS.fields_by_name['action'].enum_type = _ACTACTIONS
_TRACKINGSENSORDATA.fields_by_name['status'].enum_type = _TRACKINGSENSORDATA_STATUSTYPE
_TRACKINGSENSORDATA_STATUSTYPE.containing_type = _TRACKINGSENSORDATA
DESCRIPTOR.message_types_by_name['BaseVel'] = _BASEVEL
DESCRIPTOR.message_types_by_name['Status'] = _STATUS
DESCRIPTOR.message_types_by_name['StatusReport'] = _STATUSREPORT
DESCRIPTOR.message_types_by_name['Log'] = _LOG
DESCRIPTOR.message_types_by_name['BaseConfig'] = _BASECONFIG
DESCRIPTOR.message_types_by_name['RetBaseConfig'] = _RETBASECONFIG
DESCRIPTOR.message_types_by_name['ImuData'] = _IMUDATA
DESCRIPTOR.message_types_by_name['ActCmd'] = _ACTCMD
DESCRIPTOR.message_types_by_name['ActStatus'] = _ACTSTATUS
DESCRIPTOR.message_types_by_name['LedRingDistances'] = _LEDRINGDISTANCES
DESCRIPTOR.message_types_by_name['TrackingSensorData'] = _TRACKINGSENSORDATA
DESCRIPTOR.message_types_by_name['TrackingSensorStd'] = _TRACKINGSENSORSTD
DESCRIPTOR.message_types_by_name['ResetAndCalibrateTrackingSensor'] = _RESETANDCALIBRATETRACKINGSENSOR
DESCRIPTOR.enum_types_by_name['ActActions'] = _ACTACTIONS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

BaseVel = _reflection.GeneratedProtocolMessageType('BaseVel', (_message.Message,), {
  'DESCRIPTOR' : _BASEVEL,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.BaseVel)
  })
_sym_db.RegisterMessage(BaseVel)

Status = _reflection.GeneratedProtocolMessageType('Status', (_message.Message,), {
  'DESCRIPTOR' : _STATUS,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.Status)
  })
_sym_db.RegisterMessage(Status)

StatusReport = _reflection.GeneratedProtocolMessageType('StatusReport', (_message.Message,), {
  'DESCRIPTOR' : _STATUSREPORT,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.StatusReport)
  })
_sym_db.RegisterMessage(StatusReport)

Log = _reflection.GeneratedProtocolMessageType('Log', (_message.Message,), {
  'DESCRIPTOR' : _LOG,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.Log)
  })
_sym_db.RegisterMessage(Log)

BaseConfig = _reflection.GeneratedProtocolMessageType('BaseConfig', (_message.Message,), {
  'DESCRIPTOR' : _BASECONFIG,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.BaseConfig)
  })
_sym_db.RegisterMessage(BaseConfig)

RetBaseConfig = _reflection.GeneratedProtocolMessageType('RetBaseConfig', (_message.Message,), {
  'DESCRIPTOR' : _RETBASECONFIG,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.RetBaseConfig)
  })
_sym_db.RegisterMessage(RetBaseConfig)

ImuData = _reflection.GeneratedProtocolMessageType('ImuData', (_message.Message,), {
  'DESCRIPTOR' : _IMUDATA,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.ImuData)
  })
_sym_db.RegisterMessage(ImuData)

ActCmd = _reflection.GeneratedProtocolMessageType('ActCmd', (_message.Message,), {
  'DESCRIPTOR' : _ACTCMD,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.ActCmd)
  })
_sym_db.RegisterMessage(ActCmd)

ActStatus = _reflection.GeneratedProtocolMessageType('ActStatus', (_message.Message,), {
  'DESCRIPTOR' : _ACTSTATUS,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.ActStatus)
  })
_sym_db.RegisterMessage(ActStatus)

LedRingDistances = _reflection.GeneratedProtocolMessageType('LedRingDistances', (_message.Message,), {
  'DESCRIPTOR' : _LEDRINGDISTANCES,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.LedRingDistances)
  })
_sym_db.RegisterMessage(LedRingDistances)

TrackingSensorData = _reflection.GeneratedProtocolMessageType('TrackingSensorData', (_message.Message,), {
  'DESCRIPTOR' : _TRACKINGSENSORDATA,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.TrackingSensorData)
  })
_sym_db.RegisterMessage(TrackingSensorData)

TrackingSensorStd = _reflection.GeneratedProtocolMessageType('TrackingSensorStd', (_message.Message,), {
  'DESCRIPTOR' : _TRACKINGSENSORSTD,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.TrackingSensorStd)
  })
_sym_db.RegisterMessage(TrackingSensorStd)

ResetAndCalibrateTrackingSensor = _reflection.GeneratedProtocolMessageType('ResetAndCalibrateTrackingSensor', (_message.Message,), {
  'DESCRIPTOR' : _RESETANDCALIBRATETRACKINGSENSOR,
  '__module__' : 'msgs_can_pb2'
  # @@protoc_insertion_point(class_scope:msgs_can.ResetAndCalibrateTrackingSensor)
  })
_sym_db.RegisterMessage(ResetAndCalibrateTrackingSensor)


# @@protoc_insertion_point(module_scope)
