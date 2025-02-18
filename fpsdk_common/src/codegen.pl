#!/usr/bin/perl
# ___    ___
# \  \  /  /
#  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
#  /  /\  \   License: see the LICENSE file
# /__/  \__\
#
# Fixposition SDK code generator
#
# You'll need: apt install libpath-tiny-perl libdigest-sha-perl
#
# Note that this is not run automatically at build time. Instead, you're supposed to run this manually and commit th
# modifed source files (and the changes to this script, obviously).
#
use strict;
use warnings;
use utf8;
use FindBin;
use Path::Tiny;
use Data::Dumper;
use Digest::SHA;

########################################################################################################################

my @FPB_MESSAGES =
(
    { name => 'GNSSSTATUS',          msgid =>  1201 },
    { name => 'SYSTEMSTATUS',        msgid =>  1301 },
    { name => 'MEASUREMENTS',        msgid =>  2001 },
    { name => 'VERSION',             msgid =>  2301 },
    { name => 'UNITTEST1',           msgid => 65001 },
    { name => 'UNITTEST2',           msgid => 65002 },
);

########################################################################################################################

my @NOVB_MESSAGES =
(
    { name => 'BESTGNSSPOS',         msgid =>  1429 },
    { name => 'BESTPOS',             msgid =>    42 },
    { name => 'BESTUTM',             msgid =>   726 },
    { name => 'BESTVEL',             msgid =>    99 },
    { name => 'BESTXYZ',             msgid =>   241 },
    { name => 'CORRIMUS',            msgid =>  2264 },
    { name => 'HEADING2',            msgid =>  1335 },
    { name => 'IMURATECORRIMUS',     msgid =>  1362 },
    { name => 'INSCONFIG',           msgid =>  1945 },
    { name => 'INSPVA',              msgid =>   507 },
    { name => 'INSPVAS',             msgid =>   508 },
    { name => 'INSPVAX',             msgid =>  1465 },
    { name => 'INSSTDEV',            msgid =>  2051 },
    { name => 'PSRDOP2',             msgid =>  1163 },
    { name => 'RAWDMI',              msgid =>  2269 },
    { name => 'RAWIMU',              msgid =>   268 },
    { name => 'RAWIMUSX',            msgid =>  1462 },
    { name => 'RXSTATUS',            msgid =>    93 },
    { name => 'TIME',                msgid =>   101 },
);

########################################################################################################################

my @UNIB_MESSAGES =
(
    { name => 'VERSION',             msgid =>    37 },
    { name => 'OBSVM',               msgid =>    12 },
    { name => 'OBSVH',               msgid =>    13 },
    { name => 'OBSVMCMP',            msgid =>   138 },
    { name => 'OBSVHCMP',            msgid =>   139 },
    { name => 'OBSVBASE',            msgid =>   284 },
    { name => 'BASEINFO',            msgid =>   176 },
    { name => 'GPSION',              msgid =>     8 },
    { name => 'BD3ION',              msgid =>    21 },
    { name => 'BDSION',              msgid =>     4 },
    { name => 'GALION',              msgid =>     9 },
    { name => 'GPSUTC',              msgid =>    19 },
    { name => 'BD3UTC',              msgid =>    22 },
    { name => 'BDSUTC',              msgid =>  2012 },
    { name => 'GALUTC',              msgid =>    20 },
    { name => 'GPSEPH',              msgid =>   106 },
    { name => 'QZSSEPH',             msgid =>   110 },
    { name => 'BD3EPH',              msgid =>  2999 },
    { name => 'BDSEPH',              msgid =>   108 },
    { name => 'GLOEPH',              msgid =>   107 },
    { name => 'GALEPH',              msgid =>   109 },
    { name => 'AGRIC',               msgid =>  1276 },
    { name => 'PVTSLN',              msgid =>  1021 },
    { name => 'BESTNAV',             msgid =>  2118 },
    { name => 'BESTNAVXYZ',          msgid =>   240 },
    { name => 'BESTNAVH',            msgid =>  2119 },
    { name => 'BESTNAVXYZH',         msgid =>   242 },
    { name => 'BESTSAT',             msgid =>  1041 },
    { name => 'ADRNAV',              msgid =>   142 },
    { name => 'ADRNAVH',             msgid =>  2117 },
    { name => 'PPPNAV',              msgid =>  1026 },
    { name => 'SPPNAV',              msgid =>    46 },
    { name => 'SPPNAVH',             msgid =>  2116 },
    { name => 'STADOP',              msgid =>   954 },
    { name => 'STADOPH',             msgid =>  2122 },
    { name => 'ADRDOP',              msgid =>   953 },
    { name => 'ADRDOPH',             msgid =>  2121 },
    { name => 'PPPDOP',              msgid =>  1025 },
    { name => 'SPPDOP',              msgid =>   173 },
    { name => 'SPPDOPH',             msgid =>  2120 },
    { name => 'SATSINFO',            msgid =>  2124 },
    { name => 'BASEPOS',             msgid =>    49 },
    { name => 'SATELLITE',           msgid =>  1042 },
    { name => 'SATECEF',             msgid =>  2115 },
    { name => 'RECTIME',             msgid =>   102 },
    { name => 'NOVHEADING',          msgid =>   972 },
    { name => 'NOVHEADING2',         msgid =>  1331 },
    { name => 'RTKSTATUS',           msgid =>   509 },
    { name => 'AGNSSSTATUS',         msgid =>   512 },
    { name => 'RTCSTATUS',           msgid =>   510 },
    { name => 'JAMSTATUS',           msgid =>   511 },
    { name => 'FREQJAMSTATUS',       msgid =>   519 },
    { name => 'RTCMSTATUS',          msgid =>  2125 },
    { name => 'HWSTATUS',            msgid =>   218 },
    { name => 'PPPSTATUS',           msgid =>  9000 },
    { name => 'AGC',                 msgid =>   220 },
    { name => 'INFOPART1',           msgid =>  1019 },
    { name => 'INFOPART2',           msgid =>  1020 },
    { name => 'MSPOS',               msgid =>   520 },
    { name => 'TROPINFO',            msgid =>  2318 },
    { name => 'PTOBSINFO',           msgid =>   221 },
    { name => 'PPPB2INFO1',          msgid =>  2302 },
    { name => 'PPPB2INFO2',          msgid =>  2304 },
    { name => 'PPPB2INFO3',          msgid =>  2306 },
    { name => 'PPPB2INFO4',          msgid =>  2308 },
    { name => 'PPPB2INFO5',          msgid =>  2310 },
    { name => 'PPPB2INFO6',          msgid =>  2312 },
    { name => 'PPPB2INFO7',          msgid =>  2314 },
);

########################################################################################################################

my @UBX_CLASSES =
(
    { name => 'ACK',    clsid => 0x05 },
    { name => 'CFG',    clsid => 0x06 },
    { name => 'ESF',    clsid => 0x10 },
    { name => 'INF',    clsid => 0x04 },
    { name => 'LOG',    clsid => 0x21 },
    { name => 'MGA',    clsid => 0x13 },
    { name => 'MON',    clsid => 0x0a },
    { name => 'NAV',    clsid => 0x01 },
    { name => 'NAV2',   clsid => 0x29 },
    { name => 'RXM',    clsid => 0x02 },
    { name => 'SEC',    clsid => 0x27 },
    { name => 'TIM',    clsid => 0x0d },
    { name => 'UPD',    clsid => 0x09 },
    # fake classes
    { name => 'NMEA',   clsid => 0xf0 },
    { name => 'RTCM3',  clsid => 0xf5 },
);

########################################################################################################################

my @UBX_MESSAGES =
(
    { class => 'ACK',    name => 'ACK',            msgid => 0x01 },
    { class => 'ACK',    name => 'NAK',            msgid => 0x00 },
    { class => 'CFG',    name => 'CFG',            msgid => 0x09 },
    { class => 'CFG',    name => 'PWR',            msgid => 0x57 },
    { class => 'CFG',    name => 'RST',            msgid => 0x04 },
    { class => 'CFG',    name => 'VALDEL',         msgid => 0x8c },
    { class => 'CFG',    name => 'VALGET',         msgid => 0x8b },
    { class => 'CFG',    name => 'VALSET',         msgid => 0x8a },
    { class => 'ESF',    name => 'ALG',            msgid => 0x14 },
    { class => 'ESF',    name => 'INS',            msgid => 0x15 },
    { class => 'ESF',    name => 'MEAS',           msgid => 0x02 },
    { class => 'ESF',    name => 'RAW',            msgid => 0x03 },
    { class => 'ESF',    name => 'STATUS',         msgid => 0x10 },
    { class => 'INF',    name => 'DEBUG',          msgid => 0x04 },
    { class => 'INF',    name => 'ERROR',          msgid => 0x00 },
    { class => 'INF',    name => 'NOTICE',         msgid => 0x02 },
    { class => 'INF',    name => 'TEST',           msgid => 0x03 },
    { class => 'INF',    name => 'WARNING',        msgid => 0x01 },
    { class => 'LOG',    name => 'CREATE',         msgid => 0x07 },
    { class => 'LOG',    name => 'ERASE',          msgid => 0x03 },
    { class => 'LOG',    name => 'FINDTIME',       msgid => 0x0e },
    { class => 'LOG',    name => 'INFO',           msgid => 0x08 },
    { class => 'LOG',    name => 'RETR',           msgid => 0x09 },
    { class => 'LOG',    name => 'RETRPOS',        msgid => 0x0b },
    { class => 'LOG',    name => 'RETRPOSX',       msgid => 0x0f },
    { class => 'LOG',    name => 'RETRSTR',        msgid => 0x0d },
    { class => 'LOG',    name => 'STR',            msgid => 0x04 },
    { class => 'MGA',    name => 'ACK',            msgid => 0x60 },
    { class => 'MGA',    name => 'BDS',            msgid => 0x03 },
    { class => 'MGA',    name => 'DBD',            msgid => 0x80 },
    { class => 'MGA',    name => 'GAL',            msgid => 0x02 },
    { class => 'MGA',    name => 'GLO',            msgid => 0x06 },
    { class => 'MGA',    name => 'GPS',            msgid => 0x00 },
    { class => 'MGA',    name => 'INI',            msgid => 0x40 },
    { class => 'MGA',    name => 'QZSS',           msgid => 0x05 },
    { class => 'MON',    name => 'COMMS',          msgid => 0x36 },
    { class => 'MON',    name => 'GNSS',           msgid => 0x28 },
    { class => 'MON',    name => 'HW',             msgid => 0x09 },
    { class => 'MON',    name => 'HW2',            msgid => 0x0b },
    { class => 'MON',    name => 'HW3',            msgid => 0x37 },
    { class => 'MON',    name => 'IO',             msgid => 0x02 },
    { class => 'MON',    name => 'MSGPP',          msgid => 0x06 },
    { class => 'MON',    name => 'PATCH',          msgid => 0x27 },
    { class => 'MON',    name => 'RF',             msgid => 0x38 },
    { class => 'MON',    name => 'RXBUF',          msgid => 0x07 },
    { class => 'MON',    name => 'RXR',            msgid => 0x21 },
    { class => 'MON',    name => 'SPAN',           msgid => 0x31 },
    { class => 'MON',    name => 'SYS',            msgid => 0x39 },
    { class => 'MON',    name => 'TEMP',           msgid => 0x0e },
    { class => 'MON',    name => 'TXBUF',          msgid => 0x08 },
    { class => 'MON',    name => 'VER',            msgid => 0x04 },
    { class => 'NAV',    name => 'ATT',            msgid => 0x05 },
    { class => 'NAV',    name => 'CLOCK',          msgid => 0x22 },
    { class => 'NAV',    name => 'COV',            msgid => 0x36 },
    { class => 'NAV',    name => 'DOP',            msgid => 0x04 },
    { class => 'NAV',    name => 'EELL',           msgid => 0x3d },
    { class => 'NAV',    name => 'EOE',            msgid => 0x61 },
    { class => 'NAV',    name => 'GEOFENCE',       msgid => 0x39 },
    { class => 'NAV',    name => 'HPPOSECEF',      msgid => 0x13 },
    { class => 'NAV',    name => 'HPPOSLLH',       msgid => 0x14 },
    { class => 'NAV',    name => 'ODO',            msgid => 0x09 },
    { class => 'NAV',    name => 'ORB',            msgid => 0x34 },
    { class => 'NAV',    name => 'PL',             msgid => 0x62 },
    { class => 'NAV',    name => 'POSECEF',        msgid => 0x01 },
    { class => 'NAV',    name => 'POSLLH',         msgid => 0x02 },
    { class => 'NAV',    name => 'PVAT',           msgid => 0x17 },
    { class => 'NAV',    name => 'PVT',            msgid => 0x07 },
    { class => 'NAV',    name => 'RELPOSNED',      msgid => 0x3c },
    { class => 'NAV',    name => 'RESETODO',       msgid => 0x10 },
    { class => 'NAV',    name => 'SAT',            msgid => 0x35 },
    { class => 'NAV',    name => 'SBAS',           msgid => 0x32 },
    { class => 'NAV',    name => 'SIG',            msgid => 0x43 },
    { class => 'NAV',    name => 'SLAS',           msgid => 0x42 },
    { class => 'NAV',    name => 'STATUS',         msgid => 0x03 },
    { class => 'NAV',    name => 'SVIN',           msgid => 0x3b },
    { class => 'NAV',    name => 'TIMEBDS',        msgid => 0x24 },
    { class => 'NAV',    name => 'TIMEGAL',        msgid => 0x25 },
    { class => 'NAV',    name => 'TIMEGLO',        msgid => 0x23 },
    { class => 'NAV',    name => 'TIMEGPS',        msgid => 0x20 },
    { class => 'NAV',    name => 'TIMELS',         msgid => 0x26 },
    { class => 'NAV',    name => 'TIMEQZSS',       msgid => 0x27 },
    { class => 'NAV',    name => 'TIMENAVIC',      msgid => 0x63 },
    { class => 'NAV',    name => 'TIMETRUSTED',    msgid => 0x64 },
    { class => 'NAV',    name => 'TIMEUTC',        msgid => 0x21 },
    { class => 'NAV',    name => 'VELECEF',        msgid => 0x11 },
    { class => 'NAV',    name => 'VELNED',         msgid => 0x12 },
    { class => 'NAV2',   name => 'CLOCK',          msgid => "UBX_NAV_CLOCK_MSGID" },
    { class => 'NAV2',   name => 'COV',            msgid => "UBX_NAV_COV_MSGID" },
    { class => 'NAV2',   name => 'DOP',            msgid => "UBX_NAV_DOP_MSGID" },
    { class => 'NAV2',   name => 'EOE',            msgid => "UBX_NAV_EOE_MSGID" },
    { class => 'NAV2',   name => 'ODO',            msgid => "UBX_NAV_ODO_MSGID" },
    { class => 'NAV2',   name => 'POSECEF',        msgid => "UBX_NAV_POSECEF_MSGID" },
    { class => 'NAV2',   name => 'POSLLH',         msgid => "UBX_NAV_POSLLH_MSGID" },
    { class => 'NAV2',   name => 'PVT',            msgid => "UBX_NAV_PVT_MSGID" },
    { class => 'NAV2',   name => 'SAT',            msgid => "UBX_NAV_SAT_MSGID" },
    { class => 'NAV2',   name => 'SBAS',           msgid => "UBX_NAV_SBAS_MSGID" },
    { class => 'NAV2',   name => 'SIG',            msgid => "UBX_NAV_SIG_MSGID" },
    { class => 'NAV2',   name => 'SLAS',           msgid => "UBX_NAV_SLAS_MSGID" },
    { class => 'NAV2',   name => 'STATUS',         msgid => "UBX_NAV_STATUS_MSGID" },
    { class => 'NAV2',   name => 'SVIN',           msgid => "UBX_NAV_SVIN_MSGID" },
    { class => 'NAV2',   name => 'TIMEBDS',        msgid => "UBX_NAV_TIMEBDS_MSGID" },
    { class => 'NAV2',   name => 'TIMEGAL',        msgid => "UBX_NAV_TIMEGAL_MSGID" },
    { class => 'NAV2',   name => 'TIMEGLO',        msgid => "UBX_NAV_TIMEGLO_MSGID" },
    { class => 'NAV2',   name => 'TIMEGPS',        msgid => "UBX_NAV_TIMEGPS_MSGID" },
    { class => 'NAV2',   name => 'TIMELS',         msgid => "UBX_NAV_TIMELS_MSGID" },
    { class => 'NAV2',   name => 'TIMEQZSS',       msgid => "UBX_NAV_TIMEQZSS_MSGID" },
    { class => 'NAV2',   name => 'TIMENAVIC',      msgid => "UBX_NAV_TIMENAVIC_MSGID" },
    { class => 'NAV2',   name => 'TIMEUTC',        msgid => "UBX_NAV_TIMEUTC_MSGID" },
    { class => 'NAV2',   name => 'VELECEF',        msgid => "UBX_NAV_VELECEF_MSGID" },
    { class => 'NAV2',   name => 'VELNED',         msgid => "UBX_NAV_VELNED_MSGID" },
    { class => 'RXM',    name => 'COR',            msgid => 0x14 },
    { class => 'RXM',    name => 'MEASX',          msgid => 0x14 },
    { class => 'RXM',    name => 'PMP',            msgid => 0x72 },
    { class => 'RXM',    name => 'PMREQ',          msgid => 0x41 },
    { class => 'RXM',    name => 'QZSSL6',         msgid => 0x73 },
    { class => 'RXM',    name => 'RAWX',           msgid => 0x15 },
    { class => 'RXM',    name => 'RLM',            msgid => 0x59 },
    { class => 'RXM',    name => 'RTCM',           msgid => 0x32 },
    { class => 'RXM',    name => 'SFRBX',          msgid => 0x13 },
    { class => 'RXM',    name => 'SPARTN',         msgid => 0x33 },
    { class => 'RXM',    name => 'SPARTNKEY',      msgid => 0x36 },
    { class => 'SEC',    name => 'OSNMA',          msgid => 0x0a },
    { class => 'SEC',    name => 'SIG',            msgid => 0x09 },
    { class => 'SEC',    name => 'SIGLOG',         msgid => 0x10 },
    { class => 'SEC',    name => 'UNIQID',         msgid => 0x03 },
    { class => 'TIM',    name => 'TM2',            msgid => 0x03 },
    { class => 'TIM',    name => 'TP',             msgid => 0x01 },
    { class => 'TIM',    name => 'VRFY',           msgid => 0x06 },
    { class => 'UPD',    name => 'FLDET',          msgid => 0x08 },
    { class => 'UPD',    name => 'POS',            msgid => 0x15 },
    { class => 'UPD',    name => 'SAFEBOOT',       msgid => 0x07 },
    { class => 'UPD',    name => 'SOS',            msgid => 0x14 },
    # fake classes
    { class => 'NMEA',   name => 'STANDARD_DTM',   msgid => 0x0a },
    { class => 'NMEA',   name => 'STANDARD_GAQ',   msgid => 0x45 },
    { class => 'NMEA',   name => 'STANDARD_GBQ',   msgid => 0x44 },
    { class => 'NMEA',   name => 'STANDARD_GBS',   msgid => 0x09 },
    { class => 'NMEA',   name => 'STANDARD_GGA',   msgid => 0x00 },
    { class => 'NMEA',   name => 'STANDARD_GLL',   msgid => 0x01 },
    { class => 'NMEA',   name => 'STANDARD_GLQ',   msgid => 0x43 },
    { class => 'NMEA',   name => 'STANDARD_GNQ',   msgid => 0x42 },
    { class => 'NMEA',   name => 'STANDARD_GNS',   msgid => 0x0d },
    { class => 'NMEA',   name => 'STANDARD_GPQ',   msgid => 0x40 },
    { class => 'NMEA',   name => 'STANDARD_GQQ',   msgid => 0x47 },
    { class => 'NMEA',   name => 'STANDARD_GRS',   msgid => 0x06 },
    { class => 'NMEA',   name => 'STANDARD_GSA',   msgid => 0x02 },
    { class => 'NMEA',   name => 'STANDARD_GST',   msgid => 0x07 },
    { class => 'NMEA',   name => 'STANDARD_GSV',   msgid => 0x03 },
    { class => 'NMEA',   name => 'STANDARD_RLM',   msgid => 0x0b },
    { class => 'NMEA',   name => 'STANDARD_RMC',   msgid => 0x04 },
    { class => 'NMEA',   name => 'STANDARD_TXT',   msgid => 0x41 },
    { class => 'NMEA',   name => 'STANDARD_VLW',   msgid => 0x0f },
    { class => 'NMEA',   name => 'STANDARD_VTG',   msgid => 0x05 },
    { class => 'NMEA',   name => 'STANDARD_ZDA',   msgid => 0x08 },
    { class => 'NMEA',   name => 'PUBX_CONFIG',    msgid => 0x41 },
    { class => 'NMEA',   name => 'PUBX_POSITION',  msgid => 0x00 },
    { class => 'NMEA',   name => 'PUBX_RATE',      msgid => 0x40 },
    { class => 'NMEA',   name => 'PUBX_SVSTATUS',  msgid => 0x03 },
    { class => 'NMEA',   name => 'PUBX_TIME',      msgid => 0x04 },
    { class => 'RTCM3',  name => 'TYPE1001',       msgid => 0x01 },
    { class => 'RTCM3',  name => 'TYPE1002',       msgid => 0x02 },
    { class => 'RTCM3',  name => 'TYPE1003',       msgid => 0x03 },
    { class => 'RTCM3',  name => 'TYPE1004',       msgid => 0x04 },
    { class => 'RTCM3',  name => 'TYPE1005',       msgid => 0x05 },
    { class => 'RTCM3',  name => 'TYPE1006',       msgid => 0x06 },
    { class => 'RTCM3',  name => 'TYPE1007',       msgid => 0x07 },
    { class => 'RTCM3',  name => 'TYPE1009',       msgid => 0x09 },
    { class => 'RTCM3',  name => 'TYPE1010',       msgid => 0x0a },
    { class => 'RTCM3',  name => 'TYPE1011',       msgid => 0xa1 },
    { class => 'RTCM3',  name => 'TYPE1012',       msgid => 0xa2 },
    { class => 'RTCM3',  name => 'TYPE1033',       msgid => 0x21 },
    { class => 'RTCM3',  name => 'TYPE1074',       msgid => 0x4a },
    { class => 'RTCM3',  name => 'TYPE1075',       msgid => 0x4b },
    { class => 'RTCM3',  name => 'TYPE1077',       msgid => 0x4d },
    { class => 'RTCM3',  name => 'TYPE1084',       msgid => 0x54 },
    { class => 'RTCM3',  name => 'TYPE1085',       msgid => 0x55 },
    { class => 'RTCM3',  name => 'TYPE1087',       msgid => 0x57 },
    { class => 'RTCM3',  name => 'TYPE1094',       msgid => 0x5e },
    { class => 'RTCM3',  name => 'TYPE1095',       msgid => 0x5f },
    { class => 'RTCM3',  name => 'TYPE1097',       msgid => 0x61 },
    { class => 'RTCM3',  name => 'TYPE1124',       msgid => 0x7c },
    { class => 'RTCM3',  name => 'TYPE1125',       msgid => 0x7d },
    { class => 'RTCM3',  name => 'TYPE1127',       msgid => 0x7f },
    { class => 'RTCM3',  name => 'TYPE1230',       msgid => 0xe6 },
    { class => 'RTCM3',  name => 'TYPE4072_0',     msgid => 0xfe },
    { class => 'RTCM3',  name => 'TYPE4072_1',     msgid => 0xfd },
);

########################################################################################################################

my @RTCM3_MESSAGES =
(
    { type => 1001,  desc => "L1-only GPS RTK observables" },
    { type => 1002,  desc => "Extended L1-only GPS RTK observables" },
    { type => 1003,  desc => "L1/L2 GPS RTK observables" },
    { type => 1004,  desc => "Extended L1/L2 GPS RTK observables" },
    { type => 1005,  desc => "Stationary RTK reference station ARP" },
    { type => 1006,  desc => "Stationary RTK reference station ARP with antenna height" },
    { type => 1007,  desc => "Antenna descriptor" },
    { type => 1008,  desc => "Antenna descriptor and serial number" },
    { type => 1009,  desc => "L1-only GLONASS RTK observables" },
    { type => 1010,  desc => "Extended L1-only GLONASS RTK observables" },
    { type => 1011,  desc => "L1/L2 GLONASS RTK observables" },
    { type => 1012,  desc => "Extended L1/L2 GLONASS RTK observables" },
    #
    { type => 1030,  desc => "GPS network RTK residual message" },
    { type => 1031,  desc => "GLONASS network RTK residual message" },
    { type => 1032,  desc => "Physical reference station position message" },
    { type => 1033,  desc => "Receiver and antenna descriptors" },
    { type => 1230,  desc => "GLONASS code-phase biases" },
    #
    { type => 1071,  desc => "GPS MSM1 (C)" },
    { type => 1072,  desc => "GPS MSM2 (L)" },
    { type => 1073,  desc => "GPS MSM3 (C, L)" },
    { type => 1074,  desc => "GPS MSM4 (full C, full L, S)" },
    { type => 1075,  desc => "GPS MSM5 (full C, full L, S, D)" },
    { type => 1076,  desc => "GPS MSM6 (ext full C, ext full L, S)" },
    { type => 1077,  desc => "GPS MSM7 (ext full C, ext full L, S, D)" },
    { type => 1081,  desc => "GLONASS MSM1 (C)" },
    { type => 1082,  desc => "GLONASS MSM2 (L)" },
    { type => 1083,  desc => "GLONASS MSM3 (C, L)" },
    { type => 1084,  desc => "GLONASS MSM4 (full C, full L, S)" },
    { type => 1085,  desc => "GLONASS MSM5 (full C, full L, S, D)" },
    { type => 1086,  desc => "GLONASS MSM6 (ext full C, ext full L, S)" },
    { type => 1087,  desc => "GLONASS MSM7 (ext full C, ext full L, S, D)" },
    { type => 1091,  desc => "Galileo MSM1 (C)" },
    { type => 1092,  desc => "Galileo MSM2 (L)" },
    { type => 1093,  desc => "Galileo MSM3 (C, L)" },
    { type => 1094,  desc => "Galileo MSM4 (full C, full L, S)" },
    { type => 1095,  desc => "Galileo MSM5 (full C, full L, S, D)" },
    { type => 1096,  desc => "Galileo MSM6 (full C, full L, S)" },
    { type => 1097,  desc => "Galileo MSM7 (ext full C, ext full L, S, D)" },
    { type => 1101,  desc => "SBAS MSM1 (C)" },
    { type => 1102,  desc => "SBAS MSM2 (L)" },
    { type => 1103,  desc => "SBAS MSM3 (C, L)" },
    { type => 1104,  desc => "SBAS MSM4 (full C, full L, S)" },
    { type => 1105,  desc => "SBAS MSM5 (full C, full L, S, D)" },
    { type => 1106,  desc => "SBAS MSM6 (full C, full L, S)" },
    { type => 1107,  desc => "SBAS MSM7 (ext full C, ext full L, S, D)" },
    { type => 1111,  desc => "QZSS MSM1 (C)" },
    { type => 1112,  desc => "QZSS MSM2 (L)" },
    { type => 1113,  desc => "QZSS MSM3 (C, L)" },
    { type => 1114,  desc => "QZSS MSM4 (full C, full L, S)" },
    { type => 1115,  desc => "QZSS MSM5 (full C, full L, S, D)" },
    { type => 1116,  desc => "QZSS MSM6 (full C, full L, S)" },
    { type => 1117,  desc => "QZSS MSM7 (ext full C, ext full L, S, D)" },
    { type => 1121,  desc => "BeiDou MSM1 (C)" },
    { type => 1122,  desc => "BeiDou MSM2 (L)" },
    { type => 1123,  desc => "BeiDou MSM3 (C, L)" },
    { type => 1124,  desc => "BeiDou MSM4 (full C, full L, S)" },
    { type => 1125,  desc => "BeiDou MSM5 (full C, full L, S, D)" },
    { type => 1126,  desc => "BeiDou MSM6 (full C, full L, S)" },
    { type => 1127,  desc => "BeiDou MSM7 (ext full C, ext full L, S, D)" },
    { type => 1131,  desc => "NavIC MSM1 (C)" },
    { type => 1132,  desc => "NavIC MSM2 (L)" },
    { type => 1133,  desc => "NavIC MSM3 (C, L)" },
    { type => 1134,  desc => "NavIC MSM4 (full C, full L, S)" },
    { type => 1135,  desc => "NavIC MSM5 (full C, full L, S, D)" },
    { type => 1136,  desc => "NavIC MSM6 (full C, full L, S)" },
    { type => 1137,  desc => "NavIC MSM7 (ext full C, ext full L, S, D)" },
    #
    { type => 1019,  desc => "GPS ephemerides" },
    { type => 1020,  desc => "GLONASS ephemerides" },
    { type => 1042,  desc => "BeiDou satellite ephemeris data" },
    { type => 1044,  desc => "QZSS ephemerides" },
    { type => 1045,  desc => "Galileo F/NAV satellite ephemeris data" },
    { type => 1046,  desc => "Galileo I/NAV satellite ephemeris data" },
    #
    { type => 4050,  desc => "STMicroelectronics proprietary" },
    { type => 4051,  desc => "Hi-Target proprietary" },
    { type => 4052,  desc => "Furuno proprietary" },
    { type => 4053,  desc => "Qualcomm proprietary" },
    { type => 4054,  desc => "Geodnet proprietary" },
    { type => 4055,  desc => "KRISO proprietary" },
    { type => 4056,  desc => "Dayou proprietary" },
    { type => 4057,  desc => "Sixents proprietary" },
    { type => 4058,  desc => "Anello proprietary" },
    { type => 4059,  desc => "NRCan proprietary" },
    { type => 4060,  desc => "ALES proprietary" },
    { type => 4061,  desc => "Geely proprietary" },
    { type => 4062,  desc => "SwiftNav proprietary" },
    { type => 4063,  desc => "CHCNAV proprietary" },
    { type => 4064,  desc => "NTLab proprietary" },
    { type => 4065,  desc => "Allystar proprietary" },
    { type => 4066,  desc => "Lantmateriet proprietary" },
    { type => 4067,  desc => "CIPPE proprietary" },
    { type => 4068,  desc => "Qianxun proprietary" },
    { type => 4069,  desc => "Veripos proprietary" },
    { type => 4070,  desc => "Wuhan MengXin proprietary" },
    { type => 4071,  desc => "Wuhan Navigation proprietary" },
    { type => 4072,  desc => "u-blox proprietary", },
    { type => 4073,  desc => "Mitsubishi proprietary" },
    { type => 4074,  desc => "Unicore proprietary" },
    { type => 4075,  desc => "Alberding proprietary" },
    { type => 4076,  desc => "IGS proprietary" },
    { type => 4077,  desc => "Hemisphere proprietary" },
    { type => 4078,  desc => "ComNav proprietary" },
    { type => 4079,  desc => "SubCarrier proprietary" },
    { type => 4080,  desc => "NavCom proprietary" },
    { type => 4081,  desc => "SNU GNSS proprietary" },
    { type => 4082,  desc => "CRCSI proprietary" },
    { type => 4083,  desc => "DLR proprietary" },
    { type => 4084,  desc => "Geodetics, Inc proprietary" },
    { type => 4085,  desc => "EUSPA proprietary" },
    { type => 4086,  desc => "inPosition proprietary" },
    { type => 4087,  desc => "Fugro proprietary" },
    { type => 4088,  desc => "IfEN proprietary" },
    { type => 4089,  desc => "Septentrio proprietary" },
    { type => 4090,  desc => "Geo++ proprietary" },
    { type => 4091,  desc => "Topcon proprietary" },
    { type => 4092,  desc => "Leica proprietary" },
    { type => 4093,  desc => "NovAtel proprietary" },
    { type => 4094,  desc => "Trimble proprietary" },
    { type => 4095,  desc => "Ashtech proprietary" },
);

my @RTCM3_SUBTYPES =
(
    { type => 4072, subtype => 0, desc => "u-blox proprietary: Reference station PVT" },
    { type => 4072, subtype => 1, desc => "u-blox proprietary: Additional reference station information" },
);

########################################################################################################################

my @SPARTN_MESSAGES =
(
    { name => 'OCB',   type =>   0,  desc => 'Orbits, clock, bias (OCB)' },
    { name => 'HPAC',  type =>   1,  desc => 'High-precision atmosphere correction (HPAC)' },
    { name => 'GAD',   type =>   2,  desc => 'Geographic area definition (GAD)' },
    { name => 'BPAC',  type =>   3,  desc => 'Basic-precision atmosphere correction (BPAC)' },
    { name => 'EAS',   type =>   4,  desc => 'Encryption and authentication support (EAS)' },
    { name => 'PROP',  type => 120,  desc => 'Proprietary message' },
);

my @SPARTN_SUBTYPES =
(
    { type => 'OCB',   name => 'GPS',    subtype => 0,  desc => 'Orbits, clock, bias (OCB) GPS' },
    { type => 'OCB',   name => 'GLO',    subtype => 1,  desc => 'Orbits, clock, bias (OCB) GLONASS' },
    { type => 'OCB',   name => 'GAL',    subtype => 2,  desc => 'Orbits, clock, bias (OCB) Galileo' },
    { type => 'OCB',   name => 'BDS',    subtype => 3,  desc => 'Orbits, clock, bias (OCB) BeiDou' },
    { type => 'OCB',   name => 'QZSS',   subtype => 4,  desc => 'Orbits, clock, bias (OCB) QZSS' },
    { type => 'HPAC',  name => 'GPS',    subtype => 0,  desc => 'High-precision atmosphere correction (HPAC) GPS' },
    { type => 'HPAC',  name => 'GLO',    subtype => 1,  desc => 'High-precision atmosphere correction (HPAC) GLONASS' },
    { type => 'HPAC',  name => 'GAL',    subtype => 2,  desc => 'High-precision atmosphere correction (HPAC) Galileo' },
    { type => 'HPAC',  name => 'BDS',    subtype => 3,  desc => 'High-precision atmosphere correction (HPAC) BeiDou' },
    { type => 'HPAC',  name => 'QZSS',   subtype => 4,  desc => 'High-precision atmosphere correction (HPAC) QZSS' },
    { type => 'GAD',   name => 'GAD',    subtype => 0,  desc => 'Geographic area definition (GAD)' },
    { type => 'BPAC',  name => 'POLY',   subtype => 0,  desc => 'Basic-precision atmosphere correction (BPAC)' },
    { type => 'EAS',   name => 'KEY',    subtype => 0,  desc => 'Encryption and authentication support (EAS): Dynamic key' },
    { type => 'EAS',   name => 'GROUP',  subtype => 1,  desc => 'Encryption and authentication support (EAS): Group authentication' },
    { type => 'PROP',  name => 'EST',    subtype => 0,  desc => 'Proprietary messages: test' },
    { type => 'PROP',  name => 'UBLOX',  subtype => 1,  desc => 'Proprietary messages: u-blox' },
    { type => 'PROP',  name => 'SWIFT',  subtype => 2,  desc => 'Proprietary messages: Swift' },
);

########################################################################################################################

my $FPSDK_COMMON_DIR = path("$FindBin::Bin/..")->canonpath();
my @CODEGEN_FILES = ();
my %CODEGEN_SECTIONS = ();

########################################################################################################################
# Generate code for FP_B
do
{
    push(@CODEGEN_FILES,
        path("$FPSDK_COMMON_DIR/include/fpsdk_common/parser/fpb.hpp"),
        path("$FPSDK_COMMON_DIR/src/parser/fpb.cpp"));
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_FPB_MESSAGES} = [];
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_FPB_MSGINFO} =
    [
        "static constexpr std::array<MsgInfo, " . ($#FPB_MESSAGES + 1) . "> MSG_INFO =\n",
        "{{\n",
    ];
    foreach my $entry (@FPB_MESSAGES)
    {
        my $name  = "FP_B-$entry->{name}";
        my $msgid = "FP_B_$entry->{name}_MSGID";
        my $strid = "FP_B_$entry->{name}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_FPB_MESSAGES}},
            sprintf("static constexpr uint16_t    %-30s = %5d;                    //!< $name message ID\n", $msgid, $entry->{msgid}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_FPB_MSGINFO}},
            sprintf("    { %-30s %-30s },\n", "$msgid,", $strid),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_FPB_MSGINFO}},
        "}};\n");
};

########################################################################################################################
# Generate code for NOV_B
do
{
    push(@CODEGEN_FILES,
        path("$FPSDK_COMMON_DIR/include/fpsdk_common/parser/novb.hpp"),
        path("$FPSDK_COMMON_DIR/src/parser/novb.cpp"));
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_NOVB_MESSAGES} = [];
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_NOVB_MSGINFO} =
    [
        "static constexpr std::array<MsgInfo, " . ($#NOVB_MESSAGES + 1) . "> MSG_INFO =\n",
        "{{\n",
    ];
    foreach my $entry (@NOVB_MESSAGES)
    {
        my $name  = "NOV_B-$entry->{name}";
        my $msgid = "NOV_B_$entry->{name}_MSGID";
        my $strid = "NOV_B_$entry->{name}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_NOVB_MESSAGES}},
            sprintf("static constexpr uint16_t    %-30s = %5d;                    //!< $name message ID\n", $msgid, $entry->{msgid}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_NOVB_MSGINFO}},
            sprintf("    { %-30s %-30s },\n", "$msgid,", $strid),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_NOVB_MSGINFO}},
        "}};\n");
};

########################################################################################################################
# Generate code for UNI_B
do
{
    push(@CODEGEN_FILES,
        path("$FPSDK_COMMON_DIR/include/fpsdk_common/parser/unib.hpp"),
        path("$FPSDK_COMMON_DIR/src/parser/unib.cpp"));
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UNIB_MESSAGES} = [];
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UNIB_MSGINFO} =
    [
        "static constexpr std::array<MsgInfo, " . ($#UNIB_MESSAGES + 1) . "> MSG_INFO =\n",
        "{{\n",
    ];
    foreach my $entry (@UNIB_MESSAGES)
    {
        my $name  = "UNI_B-$entry->{name}";
        my $msgid = "UNI_B_$entry->{name}_MSGID";
        my $strid = "UNI_B_$entry->{name}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UNIB_MESSAGES}},
            sprintf("static constexpr uint16_t    %-30s = %5d;                    //!< $name message ID\n", $msgid, $entry->{msgid}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UNIB_MSGINFO}},
            sprintf("    { %-30s %-30s },\n", "$msgid,", $strid),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UNIB_MSGINFO}},
        "}};\n");
};


########################################################################################################################
# Generate code for UBX
do
{
    push(@CODEGEN_FILES,
        path("$FPSDK_COMMON_DIR/include/fpsdk_common/parser/ubx.hpp"),
        path("$FPSDK_COMMON_DIR/src/parser/ubx.cpp"));
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_CLASSES} = [];
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP} = [
        "static const UbxClassesInfo CLS_INFO =\n",
        "{{\n",
    ];
    foreach my $entry (@UBX_CLASSES)
    {
        my $name = "UBX-$entry->{name}";
        my $clsid = "UBX_$entry->{name}_CLSID";
        my $strid = "UBX_$entry->{name}_STRID";
        # print(Dumper($entry);)
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_CLASSES}},
            sprintf("static constexpr uint16_t    %-30s = 0x%02x;                     //!< $name class ID\n", $clsid, $entry->{clsid}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name class name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP}},
             sprintf("    { %-30s 0x00,                          %-30s },\n", "$clsid,", $strid),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP}},
        "}};\n",
    );

    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MESSAGES} = [];
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MSGINFO_HPP} = [
        "using UbxClassesInfo = std::array<UbxMsgInfo, " . ($#UBX_CLASSES + 1) .">;    //!< UBX classes lookup table\n",
        "using UbxMessagesInfo = std::array<UbxMsgInfo, " . ($#UBX_MESSAGES + 1) . ">;  //!< UBX messages lookup table\n",
    ];
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP}},
        "static const UbxMessagesInfo MSG_INFO =\n",
        "{{\n",
    );
    foreach my $entry (@UBX_MESSAGES)
    {
        my $name = "UBX-$entry->{class}-$entry->{name}";
        my $clsid = "UBX_$entry->{class}_CLSID";
        my $msgid = "UBX_$entry->{class}_$entry->{name}_MSGID";
        my $strid = "UBX_$entry->{class}_$entry->{name}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MESSAGES}},
            ($entry->{msgid} =~ m{^UBX} ?
                sprintf("static constexpr uint16_t    %-30s = %-24s  //!< $name message ID\n", $msgid, "$entry->{msgid};") :
                sprintf("static constexpr uint16_t    %-30s = 0x%02x;                     //!< $name message ID\n", $msgid, $entry->{msgid})
            ),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", "UBX_$entry->{class}_$entry->{name}_STRID", "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP}},
             sprintf("    { %-30s %-30s %-30s },\n", "$clsid,", "$msgid,", $strid),
     );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP}},
        "}};\n",
    );
};


########################################################################################################################
# Generate code for RTCM3
do
{
    push(@CODEGEN_FILES,
        path("$FPSDK_COMMON_DIR/include/fpsdk_common/parser/rtcm3.hpp"),
        path("$FPSDK_COMMON_DIR/src/parser/rtcm3.cpp"));
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MESSAGES} = [];
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MSGINFO} =
    [
        "static constexpr std::array<MsgInfo, " . ($#RTCM3_MESSAGES + 1) . "> MSG_INFO =\n",
        "{{\n",
    ];
    foreach my $entry (@RTCM3_MESSAGES)
    {
        my $type = sprintf("TYPE%04d", $entry->{type});
        my $name  = "RTCM3-${type}";
        my $msgid = "RTCM3_${type}_MSGID";
        my $strid = "RTCM3_${type}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MESSAGES}},
            sprintf("static constexpr uint16_t    %-30s = %4d;                     //!< $name message ID\n", $msgid, $entry->{type}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MSGINFO}},
            sprintf("    { %s, 0, %s, \"%s\" },\n", $msgid, $strid, $entry->{desc}),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MSGINFO}},
        "}};\n",
        "static constexpr std::array<MsgInfo, " . ($#RTCM3_SUBTYPES + 1) . "> SUB_INFO =\n",
        "{{\n");
    foreach my $entry (@RTCM3_SUBTYPES)
    {
        my $type = sprintf("TYPE%04d", $entry->{type});
        my $name  = "RTCM3-${type}_$entry->{subtype}";
        my $msgid = "RTCM3_${type}_MSGID";
        my $subid = "RTCM3_${type}_$entry->{subtype}_SUBID";
        my $strid = "RTCM3_${type}_$entry->{subtype}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MESSAGES}},
            sprintf("static constexpr uint16_t    %-30s = %d;                        //!< $name message ID\n", $subid, $entry->{subtype}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MSGINFO}},
            sprintf("    { %s, %s, %s, \"%s\" },\n", $msgid, $subid, $strid, $entry->{desc}),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_RTCM3_MSGINFO}},
        "}};\n");
};

########################################################################################################################
# Generate code for SPARTN
do
{
    push(@CODEGEN_FILES,
        path("$FPSDK_COMMON_DIR/include/fpsdk_common/parser/spartn.hpp"),
        path("$FPSDK_COMMON_DIR/src/parser/spartn.cpp"));
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MESSAGES} = [];
    $CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MSGINFO} =
    [
        "static constexpr std::array<MsgInfo, " . ($#SPARTN_MESSAGES + 1) . "> MSG_INFO =\n",
        "{{\n",
    ];
    foreach my $entry (@SPARTN_MESSAGES)
    {
        my $name  = "SPARTN-$entry->{name}";
        my $msgid = "SPARTN_$entry->{name}_MSGID";
        my $strid = "SPARTN_$entry->{name}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MESSAGES}},
            sprintf("static constexpr uint16_t    %-30s = %4d;                     //!< $name message ID\n", $msgid, $entry->{type}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MSGINFO}},
            sprintf("    { %-30s 0, %-30s \"%s\" },\n", "$msgid,", "$strid,", $entry->{desc}),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MSGINFO}},
        "}};\n",
        "static constexpr std::array<MsgInfo, " . ($#SPARTN_SUBTYPES + 1) . "> SUB_INFO =\n",
        "{{\n");
    foreach my $entry (@SPARTN_SUBTYPES)
    {
        my $name  = "SPARTN-$entry->{type}-$entry->{name}";
        my $msgid = "SPARTN_$entry->{type}_MSGID";
        my $subid = "SPARTN_$entry->{type}_$entry->{name}_SUBID";
        my $strid = "SPARTN_$entry->{type}_$entry->{name}_STRID";
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MESSAGES}},
            sprintf("static constexpr uint16_t    %-30s = %d;                        //!< $name message ID\n", $subid, $entry->{subtype}),
            sprintf("static constexpr const char* %-30s = %-25s //!< $name message name\n", $strid, "\"${name}\";"),
        );
        push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MSGINFO}},
            sprintf("    { %s, %s, %s, \"%s\" },\n", $msgid, $subid, $strid, $entry->{desc}),
        );
    }
    push(@{$CODEGEN_SECTIONS{FPSDK_COMMON_PARSER_SPARTN_MSGINFO}},
        "}};\n");
};

########################################################################################################################
# Update files as necessary

# printf("CODEGEN_FILES=%s\n", Dumper(\@CODEGEN_FILES));
# printf("CODEGEN_SECTIONS=%s\n", Dumper(\%CODEGEN_SECTIONS));

foreach my $file (@CODEGEN_FILES)
{
    my @lines = $file->lines();
    my $sha1 = Digest::SHA::sha1_hex(@lines);

    foreach my $section (sort keys %CODEGEN_SECTIONS)
    {
        my $in_block = 0;
        my @new_lines = ();
        foreach my $line (@lines)
        {
            if ($line =~ m{\@fp_codegen_end\{$section\}})
            {
                $in_block = 0;
            }
            push(@new_lines, $line) unless ($in_block);
            if ($line =~ m{\@fp_codegen_begin\{$section\}}) {
                $in_block = 1;
                push(@new_lines, @{$CODEGEN_SECTIONS{$section}});
            }
        }
        @lines = @new_lines;
    }
    my $new_sha1 = Digest::SHA::sha1_hex(@lines);

    if ($sha1 eq $new_sha1)
    {
        printf("up to date: %s\n", $file->relative($FPSDK_COMMON_DIR));
    }
    else
    {
        printf("updating:   %s\n", $file->relative($FPSDK_COMMON_DIR));
        $file->spew(@lines);
    }
}

########################################################################################################################
