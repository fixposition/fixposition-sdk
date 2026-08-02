/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::parser::ubx messages
 */
#ifndef __FPSDK_COMMON_TO_JSON_PARSER_UBX_HPP__
#define __FPSDK_COMMON_TO_JSON_PARSER_UBX_HPP__

/* LIBC/STL */
#include <cstring>
#include <string>
#include <vector>

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../parser/ubx.hpp"
#include "../string.hpp"
#include "../types.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::parser::ubx {

// UBX-ACK-ACK (ack_or_nak = true) and UBX-ACK-NAK (ack_or_nak = false), which have the same payload
inline void to_json_UBX_ACK(nlohmann::json& j, const std::vector<uint8_t>& data, const bool ack_or_nak)
{
    if (data.size() != (ack_or_nak ? UBX_ACK_ACK_V0_SIZE : UBX_ACK_NAK_V0_SIZE)) {
        return;
    }
    UBX_ACK_ACK_V0_GROUP0 ack;
    std::memcpy(&ack, &data[UBX_HEAD_SIZE], sizeof(ack));
    j = nlohmann::json::object({
        { "clsId", ack.clsId },
        { "msgId", ack.msgId },
    });

    // Name of the ack'ed message
    const uint8_t msg[UBX_FRAME_SIZE] = { UBX_SYNC_1, UBX_SYNC_2, ack.clsId, ack.msgId, 0, 0, 0, 0 };
    char name[MAX_NAME_SIZE];
    if (UbxGetMessageName(name, sizeof(name), msg, sizeof(msg))) {
        j["msgName"] = name;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_CFG_CFG(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() != UBX_CFG_CFG_V0_MIN_SIZE) && (data.size() != UBX_CFG_CFG_V0_MAX_SIZE)) {
        return;
    }
    UBX_CFG_CFG_V0_GROUP0 cfg;
    std::memcpy(&cfg, &data[UBX_HEAD_SIZE], sizeof(cfg));
    j = nlohmann::json::object({
        { "clearMask", cfg.clearMask },
        { "saveMask", cfg.saveMask },
        { "loadMask", cfg.loadMask },
    });

    if (data.size() == UBX_CFG_CFG_V0_MAX_SIZE) {
        UBX_CFG_CFG_V0_GROUP1 dev;
        std::memcpy(&dev, &data[UBX_HEAD_SIZE + sizeof(cfg)], sizeof(dev));
        j["deviceBbr"] = (dev.deviceMask & UBX_CFG_CFG_V0_DEVICE_BBR) != 0;
        j["deviceFlash"] = (dev.deviceMask & UBX_CFG_CFG_V0_DEVICE_FLASH) != 0;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_CFG_RST(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_CFG_RST_V0_SIZE) {
        UBX_CFG_RST_V0_GROUP0 rst;
        std::memcpy(&rst, &data[UBX_HEAD_SIZE], sizeof(rst));
        j = nlohmann::json::object({
            { "navBbrMask", rst.navBbrMask },
            { "resetMode", UBX_CFG_RST_V0_RESETMODE_STR(rst.resetMode) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_CFG_VALDEL(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_CFG_VALDEL_V1_MIN_SIZE) ||
        (UBX_CFG_VALDEL_VERSION(data.data()) != UBX_CFG_VALDEL_V1_VERSION)) {
        return;
    }
    UBX_CFG_VALDEL_V1_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    const std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    const std::size_t size = data.size() - UBX_CFG_VALDEL_V1_MIN_SIZE;
    j = nlohmann::json::object({
        { "version", head.version },
        { "layerBbr", (head.layers & UBX_CFG_VALDEL_V1_LAYER_BBR) != 0 },
        { "layerFlash", (head.layers & UBX_CFG_VALDEL_V1_LAYER_FLASH) != 0 },
        { "transaction", UBX_CFG_VALDEL_V1_TRANSACTION_STR(head.transaction) },
        { "keys", string::Base64Enc(std::vector<uint8_t>(data.cbegin() + offs, data.cbegin() + offs + size)) },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

// UBX-CFG-VALGET version 0 is the poll request (a list of keys), version 1 is the response (a list of key-value pairs)
inline void to_json_UBX_CFG_VALGET(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() < UBX_CFG_VALGET_V0_MIN_SIZE) {
        return;
    }
    if (UBX_CFG_VALGET_VERSION(data.data()) == UBX_CFG_VALGET_V0_VERSION) {
        UBX_CFG_VALGET_V0_GROUP0 head;
        std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
        const std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
        const std::size_t size = data.size() - UBX_CFG_VALGET_V0_MIN_SIZE;
        j = nlohmann::json::object({
            { "version", head.version },
            { "layer", UBX_CFG_VALGET_V0_LAYER_STR(head.layer) },
            { "position", head.position },
            { "keys", string::Base64Enc(std::vector<uint8_t>(data.cbegin() + offs, data.cbegin() + offs + size)) },
        });
    } else if (UBX_CFG_VALGET_VERSION(data.data()) == UBX_CFG_VALGET_V1_VERSION) {
        UBX_CFG_VALGET_V1_GROUP0 head;
        std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
        const std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
        const std::size_t size = data.size() - UBX_CFG_VALGET_V1_MIN_SIZE;
        j = nlohmann::json::object({
            { "version", head.version },
            { "layer", UBX_CFG_VALGET_V1_LAYER_STR(head.layer) },
            { "position", head.position },
            { "cfgData", string::Base64Enc(std::vector<uint8_t>(data.cbegin() + offs, data.cbegin() + offs + size)) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_CFG_VALSET(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() < UBX_CFG_VALSET_V0_MIN_SIZE) {
        return;
    }
    if (UBX_CFG_VALSET_VERSION(data.data()) == UBX_CFG_VALSET_V0_VERSION) {
        UBX_CFG_VALSET_V0_GROUP0 head;
        std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
        const std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
        const std::size_t size = data.size() - UBX_CFG_VALSET_V0_MIN_SIZE;
        j = nlohmann::json::object({
            { "version", head.version },
            { "layerRam", (head.layers & UBX_CFG_VALSET_V0_LAYERS_RAM) != 0 },
            { "layerBbr", (head.layers & UBX_CFG_VALSET_V0_LAYERS_BBR) != 0 },
            { "layerFlash", (head.layers & UBX_CFG_VALSET_V0_LAYERS_FLASH) != 0 },
            { "cfgData", string::Base64Enc(std::vector<uint8_t>(data.cbegin() + offs, data.cbegin() + offs + size)) },
        });
    } else if (UBX_CFG_VALSET_VERSION(data.data()) == UBX_CFG_VALSET_V1_VERSION) {
        UBX_CFG_VALSET_V1_GROUP0 head;
        std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
        const std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
        const std::size_t size = data.size() - UBX_CFG_VALSET_V1_MIN_SIZE;
        j = nlohmann::json::object({
            { "version", head.version },
            { "layerRam", (head.layers & UBX_CFG_VALSET_V1_LAYER_RAM) != 0 },
            { "layerBbr", (head.layers & UBX_CFG_VALSET_V1_LAYER_BBR) != 0 },
            { "layerFlash", (head.layers & UBX_CFG_VALSET_V1_LAYER_FLASH) != 0 },
            { "transaction", UBX_CFG_VALSET_V1_TRANSACTION_STR(head.transaction) },
            { "cfgData", string::Base64Enc(std::vector<uint8_t>(data.cbegin() + offs, data.cbegin() + offs + size)) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_ESF_MEAS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_ESF_MEAS_V0_MIN_SIZE) || (data.size() != UBX_ESF_MEAS_V0_SIZE(data.data()))) {
        return;
    }
    UBX_ESF_MEAS_V0_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));

    const std::size_t num_meas = UBX_ESF_MEAS_V0_FLAGS_NUMMEAS(head.flags);
    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto meas = nlohmann::json::array();
    for (std::size_t ix = 0; ix < num_meas; ix++, offs += sizeof(UBX_ESF_MEAS_V0_GROUP1)) {
        UBX_ESF_MEAS_V0_GROUP1 m;
        std::memcpy(&m, &data[offs], sizeof(m));
        meas.push_back(nlohmann::json::object({
            { "dataType", UBX_ESF_MEAS_V0_DATA_DATATYPE(m.data) },
            { "dataField", UBX_ESF_MEAS_V0_DATA_DATAFIELD(m.data) },
        }));
    }

    j = nlohmann::json::object({
        { "timeTag", head.timeTag },
        { "id", head.id },
        { "timeMarkSent", UBX_ESF_MEAS_V0_FLAGS_TIMEMARKSENT_STR(head.flags) },
        { "calibTtagValid", UBX_ESF_MEAS_V0_FLAGS_CALIBTTAGVALID(head.flags) },
        { "numMeas", UBX_ESF_MEAS_V0_FLAGS_NUMMEAS(head.flags) },
        { "meas", meas },
    });

    if (UBX_ESF_MEAS_V0_FLAGS_CALIBTTAGVALID(head.flags)) {
        UBX_ESF_MEAS_V0_GROUP2 g2;
        std::memcpy(&g2, &data[offs], sizeof(g2));
        j["calibTtag"] = g2.calibTtag;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_ESF_STATUS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_ESF_STATUS_V2_MIN_SIZE) ||
        (UBX_ESF_STATUS_VERSION(data.data()) != UBX_ESF_STATUS_V2_VERSION)) {
        return;
    }
    UBX_ESF_STATUS_V2_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    if (data.size() != (UBX_ESF_STATUS_V2_MIN_SIZE + (head.numSens * sizeof(UBX_ESF_STATUS_V2_GROUP1)))) {
        return;
    }
    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto sens = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.numSens; ix++, offs += sizeof(UBX_ESF_STATUS_V2_GROUP1)) {
        UBX_ESF_STATUS_V2_GROUP1 s;
        std::memcpy(&s, &data[offs], sizeof(s));
        sens.push_back(nlohmann::json::object({
            { "type", UBX_ESF_STATUS_V2_SENSSTATUS1_TYPE(s.sensStatus1) },
            { "used", (s.sensStatus1 & UBX_ESF_STATUS_V2_SENSSTATUS1_USED) != 0 },
            { "ready", (s.sensStatus1 & UBX_ESF_STATUS_V2_SENSSTATUS1_READY) != 0 },
            { "calibStatus", UBX_ESF_STATUS_V2_SENSSTATUS2_CALIBSTATUS_STR(s.sensStatus2) },
            { "timeStatus", UBX_ESF_STATUS_V2_SENSSTATUS2_TIMESTATUS_STR(s.sensStatus2) },
            { "freq", s.freq },
            { "badMeas", (s.faults & UBX_ESF_STATUS_V2_FAULTS_BADMEAS) != 0 },
            { "badTtag", (s.faults & UBX_ESF_STATUS_V2_FAULTS_BADTTAG) != 0 },
            { "missingMeas", (s.faults & UBX_ESF_STATUS_V2_FAULTS_MISSINGMEAS) != 0 },
            { "noisyMeas", (s.faults & UBX_ESF_STATUS_V2_FAULTS_NOISYMEAS) != 0 },
        }));
    }

    j = nlohmann::json::object({
        { "iTOW", head.iTOW },
        { "version", head.version },
        { "numSens", head.numSens },
        { "wtInitStatus", UBX_ESF_STATUS_V2_INITSTATUS1_WTINITSTATUS_STR(head.initStatus1) },
        { "mntAlgStatus", UBX_ESF_STATUS_V2_INITSTATUS1_MNTALGSTATUS_STR(head.initStatus1) },
        { "insInitStatus", UBX_ESF_STATUS_V2_INITSTATUS1_INSINITSTATUS_STR(head.initStatus1) },
        { "imuInitStatus", UBX_ESF_STATUS_V2_INITSTATUS2_IMUINITSTATUS_STR(head.initStatus2) },
        { "fusionMode", UBX_ESF_STATUS_V2_FUSIONMODE_STR(head.fusionMode) },
        { "sens", sens },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MGA_GAL_OSNMA_PUBKEY(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_MGA_GAL_OSNMA_PUBKEY_V0_SIZE) {
        UBX_MGA_GAL_OSNMA_PUBKEY_V0_GROUP0 pk;
        std::memcpy(&pk, &data[UBX_HEAD_SIZE], sizeof(pk));
        j = nlohmann::json::object({
            { "type", pk.type },
            { "version", pk.version },
            { "pubKeyId", UBX_MGA_GAL_OSNMA_PUBKEY_V0_BITFIELD0_PUBKEYID(pk.bitfield0) },
            { "pubKeyType", UBX_MGA_GAL_OSNMA_PUBKEY_V0_BITFIELD0_PUBKEYTYPE_STR(pk.bitfield0) },
            { "pubKeyPoint",
                string::Base64Enc(std::vector<uint8_t>(pk.pubKeyPoint, pk.pubKeyPoint + sizeof(pk.pubKeyPoint))) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MGA_GAL_OSNMA_MERKLE(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_MGA_GAL_OSNMA_MERKLE_V0_SIZE) {
        UBX_MGA_GAL_OSNMA_MERKLE_V0_GROUP0 mt;
        std::memcpy(&mt, &data[UBX_HEAD_SIZE], sizeof(mt));
        j = nlohmann::json::object({
            { "type", mt.type },
            { "version", mt.version },
            { "applicabilityTime", UBX_MGA_GAL_OSNMA_MERKLE_V0_BITFIELD0_APPLICABILITYTIME_STR(mt.bitfield0) },
            { "treeNode", string::Base64Enc(std::vector<uint8_t>(mt.treeNode, mt.treeNode + sizeof(mt.treeNode))) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MGA_INI_TIME_UTC(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_MGA_INI_TIME_UTC_V0_SIZE) {
        UBX_MGA_INI_TIME_UTC_V0_GROUP0 ini;
        std::memcpy(&ini, &data[UBX_HEAD_SIZE], sizeof(ini));
        j = nlohmann::json::object({
            { "type", ini.type },
            { "version", ini.version },
            { "leapSecs", ini.leapSecs },
            { "year", ini.year },
            { "month", ini.month },
            { "day", ini.day },
            { "hour", ini.hour },
            { "minute", ini.minute },
            { "second", ini.second },
            { "ns", ini.ns },
            { "tAccS", ini.tAccS },
            { "tAccNs", ini.tAccNs },
            { "source", UBX_MGA_INI_TIME_UTC_V0_REF_SOURCE_STR(ini.ref) },
            { "fall", UBX_MGA_INI_TIME_UTC_V0_REF_FALL(ini.ref) },
            { "last", UBX_MGA_INI_TIME_UTC_V0_REF_LAST(ini.ref) },
            { "trustedSource", UBX_MGA_INI_TIME_UTC_V0_BITFIELD0_TRUSTEDSOURCE(ini.bitfield0) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MGA_INI_TIME_GNSS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_MGA_INI_TIME_GNSS_V0_SIZE) {
        UBX_MGA_INI_TIME_GNSS_V0_GROUP0 ini;
        std::memcpy(&ini, &data[UBX_HEAD_SIZE], sizeof(ini));
        j = nlohmann::json::object({
            { "type", ini.type },
            { "version", ini.version },
            { "week", ini.week },
            { "tow", ini.tow },
            { "ns", ini.ns },
            { "tAccS", ini.tAccS },
            { "tAccNs", ini.tAccNs },
            { "gnssId", UBX_GNSSID_STR(ini.gnssId) },
            { "source", UBX_MGA_INI_TIME_GNSS_V0_REF_SOURCE_STR(ini.ref) },
            { "fall", UBX_MGA_INI_TIME_GNSS_V0_REF_FALL(ini.ref) },
            { "last", UBX_MGA_INI_TIME_GNSS_V0_REF_LAST(ini.ref) },
            { "trustedSource", UBX_MGA_INI_TIME_GNSS_V0_BITFIELD0_TRUSTEDSOURCE(ini.bitfield0) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_COMMS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_MON_COMMS_V0_MIN_SIZE) || (UBX_MON_COMMS_VERSION(data.data()) != UBX_MON_COMMS_V0_VERSION)) {
        return;
    }
    UBX_MON_COMMS_V0_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    if (data.size() != (UBX_MON_COMMS_V0_MIN_SIZE + (head.nPorts * sizeof(UBX_MON_COMMS_V0_GROUP1)))) {
        return;
    }
    auto prot_ids = nlohmann::json::array();
    for (std::size_t ix = 0; ix < types::NumOf(head.protIds); ix++) {
        prot_ids.push_back(UBX_MON_COMMS_V0_PROTIDS_STR(head.protIds[ix]));
    }

    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto ports = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.nPorts; ix++, offs += sizeof(UBX_MON_COMMS_V0_GROUP1)) {
        UBX_MON_COMMS_V0_GROUP1 p;
        std::memcpy(&p, &data[offs], sizeof(p));
        ports.push_back(nlohmann::json::object({
            { "portId", p.portId },
            { "txPending", p.txPending },
            { "txBytes", p.txBytes },
            { "txUsage", p.txUsage },
            { "txPeakUsage", p.txPeakUsage },
            { "rxPending", p.rxPending },
            { "rxBytes", p.rxBytes },
            { "rxUsage", p.rxUsage },
            { "rxPeakUsage", p.rxPeakUsage },
            { "overrunErrors", p.overrunErrors },
            { "msgs", nlohmann::json::array({ p.msgs[0], p.msgs[1], p.msgs[2], p.msgs[3] }) },
            { "skipped", p.skipped },
        }));
    }

    j = nlohmann::json::object({
        { "version", head.version },
        { "nPorts", head.nPorts },
        { "txErrorsMem", UBX_MON_COMMS_V0_TXERRORS_MEM(head.txErrors) },
        { "txErrorsAlloc", UBX_MON_COMMS_V0_TXERRORS_ALLOC(head.txErrors) },
        { "txErrorsOutputPort", UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT_STR(head.txErrors) },
        { "protIds", prot_ids },
        { "ports", ports },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_HW(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_MON_HW_V0_SIZE) {
        UBX_MON_HW_V0_GROUP0 hw;
        std::memcpy(&hw, &data[UBX_HEAD_SIZE], sizeof(hw));
        j = nlohmann::json::object({
            { "pinSel", hw.pinSel },
            { "pinBank", hw.pinBank },
            { "pinDir", hw.pinDir },
            { "pinVal", hw.pinVal },
            { "noisePerMS", hw.noisePerMS },
            { "agcCnt", hw.agcCnt },
            { "usedMask", hw.usedMask },
            { "jamInd", hw.jamInd },
            { "pinIrq", hw.pinIrq },
            { "pullH", hw.pullH },
            { "pullL", hw.pullL },
            { "aStatus", UBX_MON_HW_V0_ASTATUS_STR(hw.aStatus) },
            { "aPower", UBX_MON_HW_V0_APOWER_STR(hw.aPower) },
            { "rtcCalib", (hw.flags & UBX_MON_HW_V0_FLAGS_RTCCALIB) != 0 },
            { "safeBoot", (hw.flags & UBX_MON_HW_V0_FLAGS_SAFEBOOT) != 0 },
            { "jammingState", UBX_MON_HW_V0_FLAGS_JAMMINGSTATE_STR(hw.flags) },
            { "xtalAbsent", (hw.flags & UBX_MON_HW_V0_FLAGS_XTALABSENT) != 0 },
            { "VP", hw.VP },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_HW2(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_MON_HW2_V0_SIZE) {
        UBX_MON_HW2_V0_GROUP0 hw2;
        std::memcpy(&hw2, &data[UBX_HEAD_SIZE], sizeof(hw2));
        j = nlohmann::json::object({
            { "ofsI", hw2.ofsI },
            { "magI", hw2.magI },
            { "ofsQ", hw2.ofsQ },
            { "magQ", hw2.magQ },
            { "lowLevCfg", hw2.lowLevCfg },
            { "postStatus", hw2.postStatus },
            { "cfgSource", UBX_MON_HW2_V0_CFGSOURCE_STR(hw2.cfgSource) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_HW3(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_MON_HW3_V0_MIN_SIZE) || (UBX_MON_HW3_VERSION(data.data()) != UBX_MON_HW3_V0_VERSION)) {
        return;
    }
    UBX_MON_HW3_V0_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    if (data.size() != (UBX_MON_HW3_V0_MIN_SIZE + (head.nPins * sizeof(UBX_MON_HW3_V0_GROUP1)))) {
        return;
    }
    char hw_version[sizeof(head.hwVersion) + 1] = { 0 };
    std::memcpy(hw_version, head.hwVersion, sizeof(head.hwVersion));

    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto pins = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.nPins; ix++, offs += sizeof(UBX_MON_HW3_V0_GROUP1)) {
        UBX_MON_HW3_V0_GROUP1 p;
        std::memcpy(&p, &data[offs], sizeof(p));
        // The struct is packed, so we cannot use the fields directly
        const uint16_t pin_id = p.pinId;
        const uint16_t pin_mask = p.pinMask;
        const uint8_t vp = p.VP;
        pins.push_back(nlohmann::json::object({
            { "pinId", pin_id },
            { "periphPio", UBX_MON_HW3_V0_PINMASK_PERIPHPIO_STR(pin_mask) },
            { "pinBank", UBX_MON_HW3_V0_PINMASK_PINBANK_STR(pin_mask) },
            { "direction", UBX_MON_HW3_V0_PINMASK_DIRECTION_STR(pin_mask) },
            { "value", UBX_MON_HW3_V0_PINMASK_VALUE(pin_mask) },
            { "vpManager", UBX_MON_HW3_V0_PINMASK_VPMANAGER(pin_mask) },
            { "pioIrq", UBX_MON_HW3_V0_PINMASK_PIOIRQ(pin_mask) },
            { "pioPullHigh", UBX_MON_HW3_V0_PINMASK_PIOPULLHIGH(pin_mask) },
            { "pioPullLow", UBX_MON_HW3_V0_PINMASK_PIOPULLLOW(pin_mask) },
            { "VP", vp },
        }));
    }

    j = nlohmann::json::object({
        { "version", head.version },
        { "nPins", head.nPins },
        { "rtcCalib", UBX_MON_HW3_V0_FLAGS_RTCCALIB(head.flags) },
        { "safeBoot", UBX_MON_HW3_V0_FLAGS_SAFEBOOT(head.flags) },
        { "xtalAbsent", UBX_MON_HW3_V0_FLAGS_XTALABSENT(head.flags) },
        { "hwVersion", hw_version },
        { "pins", pins },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_RF(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_MON_RF_V0_MIN_SIZE) || (UBX_MON_RF_VERSION(data.data()) != UBX_MON_RF_V0_VERSION)) {
        return;
    }
    UBX_MON_RF_V0_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    if (data.size() != (UBX_MON_RF_V0_MIN_SIZE + (head.nBlocks * sizeof(UBX_MON_RF_V0_GROUP1)))) {
        return;
    }

    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto blocks = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.nBlocks; ix++, offs += sizeof(UBX_MON_RF_V0_GROUP1)) {
        UBX_MON_RF_V0_GROUP1 b;
        std::memcpy(&b, &data[offs], sizeof(b));
        blocks.push_back(nlohmann::json::object({
            { "blockId", b.blockId },
            { "jammingState", UBX_MON_RF_V0_FLAGS_JAMMINGSTATE_STR(b.flags) },
            { "antStatus", UBX_MON_RF_V0_ANTSTATUS_STR(b.antStatus) },
            { "antPower", UBX_MON_RF_V0_ANTPOWER_STR(b.antPower) },
            { "postStatus", b.postStatus },
            { "noisePerMS", b.noisePerMS },
            { "agcCnt", b.agcCnt },
            { "jamInd", b.jamInd },
            { "ofsI", b.ofsI },
            { "magI", b.magI },
            { "ofsQ", b.ofsQ },
            { "magQ", b.magQ },
        }));
    }

    j = nlohmann::json::object({
        { "version", head.version },
        { "nBlocks", head.nBlocks },
        { "blocks", blocks },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_SPAN(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_MON_SPAN_V0_MIN_SIZE) || (UBX_MON_SPAN_VERSION(data.data()) != UBX_MON_SPAN_V0_VERSION) ||
        (data.size() != UBX_MON_SPAN_V0_SIZE(data.data()))) {
        return;
    }
    UBX_MON_SPAN_V0_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));

    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto blocks = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.numRfBlocks; ix++, offs += sizeof(UBX_MON_SPAN_V0_GROUP1)) {
        UBX_MON_SPAN_V0_GROUP1 b;
        std::memcpy(&b, &data[offs], sizeof(b));
        auto spectrum = nlohmann::json::array();
        for (std::size_t bin = 0; bin < types::NumOf(b.spectrum); bin++) {
            spectrum.push_back(b.spectrum[bin]);
        }
        blocks.push_back(nlohmann::json::object({
            { "span", b.span },
            { "res", b.res },
            { "center", b.center },
            { "pga", b.pga },
            { "spectrum", spectrum },
        }));
    }

    j = nlohmann::json::object({
        { "version", head.version },
        { "numRfBlocks", head.numRfBlocks },
        { "blocks", blocks },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_SYS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_MON_SYS_V1_SIZE) && (UBX_MON_SYS_VERSION(data.data()) == UBX_MON_SYS_V1_VERSION)) {
        UBX_MON_SYS_V1_GROUP0 sys;
        std::memcpy(&sys, &data[UBX_HEAD_SIZE], sizeof(sys));
        j = nlohmann::json::object({
            { "msgVer", sys.msgVer },
            { "cpuLoad", sys.cpuLoad },
            { "cpuLoadMax", sys.cpuLoadMax },
            { "memUsage", sys.memUsage },
            { "memUsageMax", sys.memUsageMax },
            { "ioUsage", sys.ioUsage },
            { "ioUsageMax", sys.ioUsageMax },
            { "runTime", sys.runTime },
            { "noticeCount", sys.noticeCount },
            { "warnCount", sys.warnCount },
            { "errorCount", sys.errorCount },
            { "tempValue", sys.tempValue },
            { "bootType", UBX_MON_SYS_V1_BOOTTYPE_STR(sys.bootType) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_TEMP(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_MON_TEMP_V0_SIZE) && (UBX_MON_TEMP_VERSION(data.data()) == UBX_MON_TEMP_V0_VERSION)) {
        UBX_MON_TEMP_V0_GROUP0 temp;
        std::memcpy(&temp, &data[UBX_HEAD_SIZE], sizeof(temp));
        j = nlohmann::json::object({
            { "version", temp.version },
            { "temperature", temp.temperature },
            { "unknown", temp.unknown },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_MON_VER(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_MON_VER_V0_MIN_SIZE) ||
        (((data.size() - UBX_MON_VER_V0_MIN_SIZE) % sizeof(UBX_MON_VER_V0_GROUP1)) != 0)) {
        return;
    }
    UBX_MON_VER_V0_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    char sw_version[sizeof(head.swVersion) + 1] = { 0 };
    char hw_version[sizeof(head.hwVersion) + 1] = { 0 };
    std::memcpy(sw_version, head.swVersion, sizeof(head.swVersion));
    std::memcpy(hw_version, head.hwVersion, sizeof(head.hwVersion));

    const std::size_t num_ext = (data.size() - UBX_MON_VER_V0_MIN_SIZE) / sizeof(UBX_MON_VER_V0_GROUP1);
    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto extensions = nlohmann::json::array();
    for (std::size_t ix = 0; ix < num_ext; ix++, offs += sizeof(UBX_MON_VER_V0_GROUP1)) {
        UBX_MON_VER_V0_GROUP1 e;
        std::memcpy(&e, &data[offs], sizeof(e));
        char extension[sizeof(e.extension) + 1] = { 0 };
        std::memcpy(extension, e.extension, sizeof(e.extension));
        extensions.push_back(extension);
    }

    j = nlohmann::json::object({
        { "swVersion", sw_version },
        { "hwVersion", hw_version },
        { "extensions", extensions },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_ATT(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_NAV_ATT_V0_SIZE) && (UBX_NAV_ATT_VERSION(data.data()) == UBX_NAV_ATT_V0_VERSION)) {
        UBX_NAV_ATT_V0_GROUP0 att;
        std::memcpy(&att, &data[UBX_HEAD_SIZE], sizeof(att));
        j = nlohmann::json::object({
            { "iTOW", att.iTOW },
            { "version", att.version },
            { "roll", att.roll },
            { "pitch", att.pitch },
            { "heading", att.heading },
            { "accRoll", att.accRoll },
            { "accPitch", att.accPitch },
            { "accHeading", att.accHeading },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_CLOCK(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_CLOCK_V0_SIZE) {
        UBX_NAV_CLOCK_V0_GROUP0 clock;
        std::memcpy(&clock, &data[UBX_HEAD_SIZE], sizeof(clock));
        j = nlohmann::json::object({
            { "iTow", clock.iTow },
            { "clkB", clock.clkB },
            { "clkD", clock.clkD },
            { "tAcc", clock.tAcc },
            { "fAcc", clock.fAcc },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_COV(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_NAV_COV_V0_SIZE) && (UBX_NAV_COV_VERSION(data.data()) == UBX_NAV_COV_V0_VERSION)) {
        UBX_NAV_COV_V0_GROUP0 cov;
        std::memcpy(&cov, &data[UBX_HEAD_SIZE], sizeof(cov));
        j = nlohmann::json::object({
            { "iTOW", cov.iTOW },
            { "version", cov.version },
            { "posCovNN", cov.posCovNN },
            { "posCovNE", cov.posCovNE },
            { "posCovND", cov.posCovND },
            { "posCovEE", cov.posCovEE },
            { "posCovED", cov.posCovED },
            { "posCovDD", cov.posCovDD },
            { "velCovNN", cov.velCovNN },
            { "velCovNE", cov.velCovNE },
            { "velCovND", cov.velCovND },
            { "velCovEE", cov.velCovEE },
            { "velCovED", cov.velCovED },
            { "velCovDD", cov.velCovDD },
            { "posCovValid", cov.posCovValid != 0 },
            { "velCovValid", cov.velCovValid != 0 },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_DOP(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_DOP_V0_SIZE) {
        UBX_NAV_DOP_V0_GROUP0 dop;
        std::memcpy(&dop, &data[UBX_HEAD_SIZE], sizeof(dop));
        // The struct is packed, so we cannot use the fields directly
        j = nlohmann::json::object({
            { "iTOW", (uint32_t)dop.iTOW },
            { "gDOP", (uint16_t)dop.gDOP },
            { "pDOP", (uint16_t)dop.pDOP },
            { "tDOP", (uint16_t)dop.tDOP },
            { "vDOP", (uint16_t)dop.vDOP },
            { "hDOP", (uint16_t)dop.hDOP },
            { "nDOP", (uint16_t)dop.nDOP },
            { "eDOP", (uint16_t)dop.eDOP },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_EELL(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_NAV_EELL_V0_SIZE) && (UBX_NAV_EELL_VERSION(data.data()) == UBX_NAV_EELL_V0_VERSION)) {
        UBX_NAV_EELL_V0_GROUP0 eell;
        std::memcpy(&eell, &data[UBX_HEAD_SIZE], sizeof(eell));
        j = nlohmann::json::object({
            { "iTOW", eell.iTOW },
            { "version", eell.version },
            { "errEllipseOrient", eell.errEllipseOrient },
            { "errEllipseMajor", eell.errEllipseMajor },
            { "errEllipseMinor", eell.errEllipseMinor },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_EOE(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_EOE_V0_SIZE) {
        UBX_NAV_EOE_V0_GROUP0 eoe;
        std::memcpy(&eoe, &data[UBX_HEAD_SIZE], sizeof(eoe));
        j = nlohmann::json::object({
            { "iTOW", eoe.iTOW },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_HPPOSECEF(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_NAV_HPPOSECEF_V0_SIZE) &&
        (UBX_NAV_HPPOSECEF_VERSION(data.data()) == UBX_NAV_HPPOSECEF_V0_VERSION)) {
        UBX_NAV_HPPOSECEF_V0_GROUP0 pos;
        std::memcpy(&pos, &data[UBX_HEAD_SIZE], sizeof(pos));
        j = nlohmann::json::object({
            { "version", pos.version },
            { "iTOW", pos.iTOW },
            { "ecefX", pos.ecefX },
            { "ecefY", pos.ecefY },
            { "ecefZ", pos.ecefZ },
            { "ecefXHp", pos.ecefXHp },
            { "ecefYHp", pos.ecefYHp },
            { "ecefZHp", pos.ecefZHp },
            { "pAcc", pos.pAcc },
            { "invalidEcef", UBX_NAV_HPPOSECEF_V0_FLAGS_INVALIDECEF(pos.flags) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_HPPOSLLH(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_NAV_HPPOSLLH_V0_SIZE) &&
        (UBX_NAV_HPPOSLLH_VERSION(data.data()) == UBX_NAV_HPPOSLLH_V0_VERSION)) {
        UBX_NAV_HPPOSLLH_V0_GROUP0 pos;
        std::memcpy(&pos, &data[UBX_HEAD_SIZE], sizeof(pos));
        j = nlohmann::json::object({
            { "version", pos.version },
            { "iTOW", pos.iTOW },
            { "lon", pos.lon },
            { "lat", pos.lat },
            { "height", pos.height },
            { "hMSL", pos.hMSL },
            { "lonHp", pos.lonHp },
            { "latHp", pos.latHp },
            { "heightHp", pos.heightHp },
            { "hMSLHp", pos.hMSLHp },
            { "hAcc", pos.hAcc },
            { "vAcc", pos.vAcc },
            { "invalidLlh", (pos.flags & UBX_NAV_HPPOSLLH_V0_FLAGS_INVALIDLLH) != 0 },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_POSECEF(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_POSECEF_V0_SIZE) {
        UBX_NAV_POSECEF_V0_GROUP0 pos;
        std::memcpy(&pos, &data[UBX_HEAD_SIZE], sizeof(pos));
        j = nlohmann::json::object({
            { "iTOW", pos.iTOW },
            { "ecefX", pos.ecefX },
            { "ecefY", pos.ecefY },
            { "ecefZ", pos.ecefZ },
            { "pAcc", pos.pAcc },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_PVT(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_PVT_V1_SIZE) {
        UBX_NAV_PVT_V1_GROUP0 pvt;
        std::memcpy(&pvt, &data[UBX_HEAD_SIZE], sizeof(pvt));
        j = nlohmann::json::object({
            { "iTOW", pvt.iTOW },
            { "year", pvt.year },
            { "month", pvt.month },
            { "day", pvt.day },
            { "hour", pvt.hour },
            { "min", pvt.min },
            { "sec", pvt.sec },
            { "tAcc", pvt.tAcc },
            { "nano", pvt.nano },
            { "numSV", pvt.numSV },
            { "lon", pvt.lon },
            { "lat", pvt.lat },
            { "height", pvt.height },
            { "hMSL", pvt.hMSL },
            { "hAcc", pvt.hAcc },
            { "vAcc", pvt.vAcc },
            { "velN", pvt.velN },
            { "velE", pvt.velE },
            { "velD", pvt.velD },
            { "gSpeed", pvt.gSpeed },
            { "headMot", pvt.headMot },
            { "sAcc", pvt.sAcc },
            { "headAcc", pvt.headAcc },
            { "pDOP", pvt.pDOP },
            { "headVeh", pvt.headVeh },
            { "magDec", pvt.magDec },
            { "magAcc", pvt.magAcc },
            { "validDate", UBX_NAV_PVT_V1_VALID_VALIDDATE(pvt.valid) },
            { "validTime", UBX_NAV_PVT_V1_VALID_VALIDTIME(pvt.valid) },
            { "fullyResolved", UBX_NAV_PVT_V1_VALID_FULLYRESOLVED(pvt.valid) },
            { "validMag", UBX_NAV_PVT_V1_VALID_VALIDMAG(pvt.valid) },
            { "fixType", UBX_NAV_PVT_V1_FIXTYPE_STR(pvt.fixType) },
            { "gnssFixOk", UBX_NAV_PVT_V1_FLAGS_GNSSFIXOK(pvt.flags) },
            { "diffSoln", UBX_NAV_PVT_V1_FLAGS_DIFFSOLN(pvt.flags) },
            { "psmState", UBX_NAV_PVT_V1_FLAGS_PSMSTATE_STR(pvt.flags) },
            { "headVehValid", UBX_NAV_PVT_V1_FLAGS_HEADVEHVALID(pvt.flags) },
            { "carrSoln", UBX_NAV_PVT_V1_FLAGS_CARRSOLN_STR(pvt.flags) },
            { "confAvail", UBX_NAV_PVT_V1_FLAGS2_CONFAVAIL(pvt.flags2) },
            { "confDate", UBX_NAV_PVT_V1_FLAGS2_CONFDATE(pvt.flags2) },
            { "confTime", UBX_NAV_PVT_V1_FLAGS2_CONFTIME(pvt.flags2) },
            { "invalidLlh", UBX_NAV_PVT_V1_FLAGS3_INVALIDLLH(pvt.flags3) },
            { "lastCorrectionAge", UBX_NAV_PVT_V1_FLAGS3_LASTCORRECTIONAGE_STR(pvt.flags3) },
            { "authTime", UBX_NAV_PVT_V1_FLAGS3_AUTHTIME_STR(pvt.flags3) },
            { "nmaFixStatus", UBX_NAV_PVT_V1_FLAGS3_NMAFIXSTATUS_STR(pvt.flags3) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_RELPOSNED(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_NAV_RELPOSNED_V1_SIZE) &&
        (UBX_NAV_RELPOSNED_VERSION(data.data()) == UBX_NAV_RELPOSNED_V1_VERSION)) {
        UBX_NAV_RELPOSNED_V1_GROUP0 rel;
        std::memcpy(&rel, &data[UBX_HEAD_SIZE], sizeof(rel));
        j = nlohmann::json::object({
            { "version", rel.version },
            { "refStationId", rel.refStationId },
            { "iTOW", rel.iTOW },
            { "relPosN", rel.relPosN },
            { "relPosE", rel.relPosE },
            { "relPosD", rel.relPosD },
            { "relPosLength", rel.relPosLength },
            { "relPosHeading", rel.relPosHeading },
            { "relPosHPN", rel.relPosHPN },
            { "relPosHPE", rel.relPosHPE },
            { "relPosHPD", rel.relPosHPD },
            { "relPosHPLength", rel.relPosHPLength },
            { "accN", rel.accN },
            { "accE", rel.accE },
            { "accD", rel.accD },
            { "accLength", rel.accLength },
            { "accHeading", rel.accHeading },
            { "gnssFixOk", UBX_NAV_RELPOSNED_V1_FLAGS_GNSSFIXOK(rel.flags) },
            { "diffSoln", UBX_NAV_RELPOSNED_V1_FLAGS_DIFFSOLN(rel.flags) },
            { "relPosValid", UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSVALID(rel.flags) },
            { "carrSoln", UBX_NAV_RELPOSNED_V1_FLAGS_CARRSOLN_STR(rel.flags) },
            { "isMoving", UBX_NAV_RELPOSNED_V1_FLAGS_ISMOVING(rel.flags) },
            { "refPosMiss", UBX_NAV_RELPOSNED_V1_FLAGS_REFPOSMISS(rel.flags) },
            { "refObsMiss", UBX_NAV_RELPOSNED_V1_FLAGS_REFOBSMISS(rel.flags) },
            { "relPosHeadingValid", UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSHEADINGVALID(rel.flags) },
            { "relPosNormalized", UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSNORMALIZED(rel.flags) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_SAT(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_NAV_SAT_V1_MIN_SIZE) || (UBX_NAV_SAT_VERSION(data.data()) != UBX_NAV_SAT_V1_VERSION)) {
        return;
    }
    UBX_NAV_SAT_V1_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    if (data.size() != (UBX_NAV_SAT_V1_MIN_SIZE + (head.numSvs * sizeof(UBX_NAV_SAT_V1_GROUP1)))) {
        return;
    }

    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto svs = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.numSvs; ix++, offs += sizeof(UBX_NAV_SAT_V1_GROUP1)) {
        UBX_NAV_SAT_V1_GROUP1 s;
        std::memcpy(&s, &data[offs], sizeof(s));
        svs.push_back(nlohmann::json::object({
            { "gnssId", UBX_GNSSID_STR(s.gnssId) },
            { "svId", s.svId },
            { "cno", s.cno },
            { "elev", s.elev },
            { "azim", s.azim },
            { "prRes", s.prRes },
            { "orbitSource", UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_STR(s.flags) },
            { "ephAvail", UBX_NAV_SAT_V1_FLAGS_EPHAVAIL(s.flags) },
            { "almAvail", UBX_NAV_SAT_V1_FLAGS_ALMAVAIL(s.flags) },
            { "anoAvail", UBX_NAV_SAT_V1_FLAGS_ANOAVAIL(s.flags) },
            { "aopAvail", UBX_NAV_SAT_V1_FLAGS_AOPAVAIL(s.flags) },
        }));
    }

    j = nlohmann::json::object({
        { "iTOW", head.iTOW },
        { "version", head.version },
        { "numSvs", head.numSvs },
        { "svs", svs },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_SIG(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_NAV_SIG_V0_MIN_SIZE) || (UBX_NAV_SIG_VERSION(data.data()) != UBX_NAV_SIG_V0_VERSION)) {
        return;
    }
    UBX_NAV_SIG_V0_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    if (data.size() != (UBX_NAV_SIG_V0_MIN_SIZE + (head.numSigs * sizeof(UBX_NAV_SIG_V0_GROUP1)))) {
        return;
    }

    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto sigs = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.numSigs; ix++, offs += sizeof(UBX_NAV_SIG_V0_GROUP1)) {
        UBX_NAV_SIG_V0_GROUP1 s;
        std::memcpy(&s, &data[offs], sizeof(s));
        sigs.push_back(nlohmann::json::object({
            { "gnssId", UBX_GNSSID_STR(s.gnssId) },
            { "svId", s.svId },
            { "sigId", UBX_SIGID_STR(s.gnssId, s.sigId) },
            { "freqId", (int)s.freqId - (int)UBX_NAV_SIG_V0_FREQID_OFFS },
            { "prRes", s.prRes },
            { "cno", s.cno },
            { "qualityInd", UBX_NAV_SIG_V0_QUALITYIND_STR(s.qualityInd) },
            { "corrSource", UBX_NAV_SIG_V0_CORRSOURCE_STR(s.corrSource) },
            { "ionoModel", UBX_NAV_SIG_V0_IONOMODEL_STR(s.ionoModel) },
            { "health", UBX_NAV_SIG_V0_SIGFLAGS_HEALTH_STR(s.sigFlags) },
            { "prSmoothed", UBX_NAV_SIG_V0_SIGFLAGS_PR_SMOOTHED(s.sigFlags) },
            { "prUsed", UBX_NAV_SIG_V0_SIGFLAGS_PR_USED(s.sigFlags) },
            { "crUsed", UBX_NAV_SIG_V0_SIGFLAGS_CR_USED(s.sigFlags) },
            { "doUsed", UBX_NAV_SIG_V0_SIGFLAGS_DO_USED(s.sigFlags) },
            { "prCorrUsed", UBX_NAV_SIG_V0_SIGFLAGS_PR_CORR_USED(s.sigFlags) },
            { "crCorrUsed", UBX_NAV_SIG_V0_SIGFLAGS_CR_CORR_USED(s.sigFlags) },
            { "doCorrUsed", UBX_NAV_SIG_V0_SIGFLAGS_DO_CORR_USED(s.sigFlags) },
            { "authStatus", UBX_NAV_SIG_V0_SIGFLAGS_AUTH_STATUS(s.sigFlags) },
        }));
    }

    j = nlohmann::json::object({
        { "iTOW", head.iTOW },
        { "version", head.version },
        { "numSigs", head.numSigs },
        { "sigs", sigs },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_STATUS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_STATUS_V0_SIZE) {
        UBX_NAV_STATUS_V0_GROUP0 sta;
        std::memcpy(&sta, &data[UBX_HEAD_SIZE], sizeof(sta));
        j = nlohmann::json::object({
            { "iTow", sta.iTow },
            { "ttff", sta.ttff },
            { "msss", sta.msss },
            { "gpsFix", UBX_NAV_STATUS_V0_GPSFIX_STR(sta.gpsFix) },
            { "gpsFixOk", UBX_NAV_STATUS_V0_FLAGS_GPSFIXOK(sta.flags) },
            { "diffSoln", UBX_NAV_STATUS_V0_FLAGS_DIFFSOLN(sta.flags) },
            { "wknSet", UBX_NAV_STATUS_V0_FLAGS_WKNSET(sta.flags) },
            { "towSet", UBX_NAV_STATUS_V0_FLAGS_TOWSET(sta.flags) },
            { "diffCorr", UBX_NAV_STATUS_V0_FIXSTAT_DIFFCORR(sta.fixStat) },
            { "carrSolnValid", UBX_NAV_STATUS_V0_FIXSTAT_CARRSOLNVALID(sta.fixStat) },
            { "psmState", UBX_NAV_STATUS_V0_FLAGS2_PSMSTATE_STR(sta.flags2) },
            { "spoofDetState", UBX_NAV_STATUS_V0_FLAGS2_SPOOFDETSTATE_STR(sta.flags2) },
            { "carrSoln", UBX_NAV_STATUS_V0_FLAGS2_CARRSOLN_STR(sta.flags2) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_TIMEBDS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_TIMEBDS_V0_SIZE) {
        UBX_NAV_TIMEBDS_V0_GROUP0 time;
        std::memcpy(&time, &data[UBX_HEAD_SIZE], sizeof(time));
        j = nlohmann::json::object({
            { "iTow", time.iTow },
            { "SOW", time.SOW },
            { "fSOW", time.fSOW },
            { "week", time.week },
            { "leapS", time.leapS },
            { "tAcc", time.tAcc },
            { "sowValid", UBX_NAV_TIMEBDS_V0_VALID_SOWVALID(time.valid) },
            { "weekValid", UBX_NAV_TIMEBDS_V0_VALID_WEEKVALID(time.valid) },
            { "leapSValid", UBX_NAV_TIMEBDS_V0_VALID_LEAPSVALID(time.valid) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_TIMEGAL(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_TIMEGAL_V0_SIZE) {
        UBX_NAV_TIMEGAL_V0_GROUP0 time;
        std::memcpy(&time, &data[UBX_HEAD_SIZE], sizeof(time));
        j = nlohmann::json::object({
            { "iTow", time.iTow },
            { "galTow", time.galTow },
            { "fGalTow", time.fGalTow },
            { "galWno", time.galWno },
            { "leapS", time.leapS },
            { "tAcc", time.tAcc },
            { "galTowValid", UBX_NAV_TIMEGAL_V0_VALID_GALTOWVALID(time.valid) },
            { "galWnoValid", UBX_NAV_TIMEGAL_V0_VALID_GALWNOVALID(time.valid) },
            { "leapSValid", UBX_NAV_TIMEGAL_V0_VALID_LEAPSVALID(time.valid) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_TIMEGLO(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_TIMEGLO_V0_SIZE) {
        UBX_NAV_TIMEGLO_V0_GROUP0 time;
        std::memcpy(&time, &data[UBX_HEAD_SIZE], sizeof(time));
        j = nlohmann::json::object({
            { "iTow", time.iTow },
            { "TOD", time.TOD },
            { "fTOD", time.fTOD },
            { "Nt", time.Nt },
            { "N4", time.N4 },
            { "tAcc", time.tAcc },
            { "todValid", UBX_NAV_TIMEGLO_V0_VALID_TODVALID(time.valid) },
            { "dateValid", UBX_NAV_TIMEGLO_V0_VALID_DATEVALID(time.valid) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_TIMEGPS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_TIMEGPS_V0_SIZE) {
        UBX_NAV_TIMEGPS_V0_GROUP0 time;
        std::memcpy(&time, &data[UBX_HEAD_SIZE], sizeof(time));
        j = nlohmann::json::object({
            { "iTow", time.iTow },
            { "fTOW", time.fTOW },
            { "week", time.week },
            { "leapS", time.leapS },
            { "tAcc", time.tAcc },
            { "towValid", UBX_NAV_TIMEGPS_V0_VALID_TOWVALID(time.valid) },
            { "weekValid", UBX_NAV_TIMEGPS_V0_VALID_WEEKVALID(time.valid) },
            { "leapSValid", UBX_NAV_TIMEGPS_V0_VALID_LEAPSVALID(time.valid) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_TIMETRUSTED(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_NAV_TIMETRUSTED_V1_SIZE) &&
        (UBX_NAV_TIMETRUSTED_VERSION(data.data()) == UBX_NAV_TIMETRUSTED_V1_VERSION)) {
        UBX_NAV_TIMETRUSTED_V1_GROUP0 tt;
        std::memcpy(&tt, &data[UBX_HEAD_SIZE], sizeof(tt));
        j = nlohmann::json::object({
            { "version", tt.version },
            { "iTOW", tt.iTOW },
            { "iniWno", tt.iniWno },
            { "propWno", tt.propWno },
            { "iniTow", tt.iniTow },
            { "propTow", tt.propTow },
            { "iniTAcc", tt.iniTAcc },
            { "propTAcc", tt.propTAcc },
            { "deltaS", tt.deltaS },
            { "deltaMs", tt.deltaMs },
            { "refSys", UBX_NAV_TIMETRUSTED_V1_REFSYS_STR(tt.refSys) },
            { "trustedTimeValid", UBX_NAV_TIMETRUSTED_V1_VALID_TRUSTEDTIMEVALID(tt.valid) },
            { "deltaTimeValid", UBX_NAV_TIMETRUSTED_V1_VALID_DELTATIMEVALID(tt.valid) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_TIMEUTC(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_TIMEUTC_V0_SIZE) {
        UBX_NAV_TIMEUTC_V0_GROUP0 utc;
        std::memcpy(&utc, &data[UBX_HEAD_SIZE], sizeof(utc));
        j = nlohmann::json::object({
            { "iTow", utc.iTow },
            { "tAcc", utc.tAcc },
            { "nano", utc.nano },
            { "year", utc.year },
            { "month", utc.month },
            { "day", utc.day },
            { "hour", utc.hour },
            { "min", utc.min },
            { "sec", utc.sec },
            { "validTow", UBX_NAV_TIMEUTC_V0_VALID_VALIDTOW(utc.valid) != 0 },
            { "validWkn", UBX_NAV_TIMEUTC_V0_VALID_VALIDWKN(utc.valid) != 0 },
            { "validUtc", UBX_NAV_TIMEUTC_V0_VALID_VALIDUTC(utc.valid) != 0 },
            { "authStatus", UBX_NAV_TIMEUTC_V0_VALID_AUTHSTATUS(utc.valid) },
            { "utcStandard", UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_STR(utc.valid) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_TIMELS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_TIMELS_V0_SIZE) {
        UBX_NAV_TIMELS_V0_GROUP0 ls;
        std::memcpy(&ls, &data[UBX_HEAD_SIZE], sizeof(ls));
        j = nlohmann::json::object({
            { "iTOW", ls.iTOW },
            { "version", ls.version },
            { "currLs", ls.currLs },
            { "lsChange", ls.lsChange },
            { "timeToLsEvent", ls.timeToLsEvent },
            { "dateOfLsGpsWn", ls.dateOfLsGpsWn },
            { "dateOfLsGpsDn", ls.dateOfLsGpsDn },
            { "srcOfCurrLs", UBX_NAV_TIMELS_V0_SRCOFCURRLS_STR(ls.srcOfCurrLs) },
            { "srcOfLsChange", UBX_NAV_TIMELS_V0_SRCOFCURRLSCHANGE_STR(ls.srcOfLsChange) },
            { "currLsValid", UBX_NAV_TIMELS_V0_VALID_CURRLSVALID(ls.valid) },
            { "timeToLsEventValid", UBX_NAV_TIMELS_V0_VALID_TIMETOLSEVENTVALID(ls.valid) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_NAV_VELECEF(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_NAV_VELECEF_V0_SIZE) {
        UBX_NAV_VELECEF_V0_GROUP0 vel;
        std::memcpy(&vel, &data[UBX_HEAD_SIZE], sizeof(vel));
        j = nlohmann::json::object({
            { "iTOW", vel.iTOW },
            { "ecefVX", vel.ecefVX },
            { "ecefVY", vel.ecefVY },
            { "ecefVZ", vel.ecefVZ },
            { "sAcc", vel.sAcc },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_RXM_RAWX(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_RXM_RAWX_V1_MIN_SIZE) || (UBX_RXM_RAWX_VERSION(data.data()) != UBX_RXM_RAWX_V1_VERSION) ||
        (data.size() != UBX_RXM_RAWX_V1_SIZE(data.data()))) {
        return;
    }
    UBX_RXM_RAWX_V1_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));

    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto meas = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.numMeas; ix++, offs += sizeof(UBX_RXM_RAWX_V1_GROUP1)) {
        UBX_RXM_RAWX_V1_GROUP1 m;
        std::memcpy(&m, &data[offs], sizeof(m));
        meas.push_back(nlohmann::json::object({
            { "prMeas", m.prMeas },
            { "cpMeas", m.cpMeas },
            { "doMeas", m.doMeas },
            { "gnssId", UBX_GNSSID_STR(m.gnssId) },
            { "svId", m.svId },
            { "sigId", UBX_SIGID_STR(m.gnssId, m.sigId) },
            { "freqId", UBX_RXM_RAWX_V1_GROUP1_FREQID_TO_SLOT(m.freqId) },
            { "locktime", m.locktime },
            { "cno", m.cno },
            { "prStdev", UBX_RXM_RAWX_V1_PRSTDEV_PRSTD(m.prStdev) },
            { "cpStdev", UBX_RXM_RAWX_V1_CPSTDEV_CPSTD(m.cpStdev) },
            { "doStdev", UBX_RXM_RAWX_V1_DOSTDEV_DOSTD(m.doStdev) },
            { "prValid", UBX_RXM_RAWX_V1_TRKSTAT_PRVALID(m.trkStat) },
            { "cpValid", UBX_RXM_RAWX_V1_TRKSTAT_CPVALID(m.trkStat) },
            { "halfCyc", UBX_RXM_RAWX_V1_TRKSTAT_HALFCYC(m.trkStat) },
            { "subHalfCyc", UBX_RXM_RAWX_V1_TRKSTAT_SUBHALFCYC(m.trkStat) },
        }));
    }

    j = nlohmann::json::object({
        { "rcvTow", head.rcvTow },
        { "week", head.week },
        { "leapS", head.leapS },
        { "numMeas", head.numMeas },
        { "version", head.version },
        { "leapSec", UBX_RXM_RAWX_V1_RECSTAT_LEAPSEC(head.recStat) },
        { "clkReset", UBX_RXM_RAWX_V1_RECSTAT_CLKRESET(head.recStat) },
        { "meas", meas },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_RXM_RTCM(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_RXM_RTCM_V2_SIZE) && (UBX_RXM_RTCM_VERSION(data.data()) == UBX_RXM_RTCM_V2_VERSION)) {
        UBX_RXM_RTCM_V2_GROUP0 rtcm;
        std::memcpy(&rtcm, &data[UBX_HEAD_SIZE], sizeof(rtcm));
        j = nlohmann::json::object({
            { "version", rtcm.version },
            { "subType", rtcm.subType },
            { "refStation", rtcm.refStation },
            { "msgType", rtcm.msgType },
            { "crcFailed", UBX_RXM_RTCM_V2_FLAGS_CRCFAILED(rtcm.flags) },
            { "msgUsed", UBX_RXM_RTCM_V2_FLAGS_MSGUSED_STR(rtcm.flags) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_RXM_SFRBX(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_RXM_SFRBX_V2_MIN_SIZE) || (UBX_RXM_SFRBX_VERSION(data.data()) != UBX_RXM_SFRBX_V2_VERSION)) {
        return;
    }
    UBX_RXM_SFRBX_V2_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    if (data.size() != (UBX_RXM_SFRBX_V2_MIN_SIZE + (head.numWords * sizeof(UBX_RXM_SFRBX_V2_GROUP1)))) {
        return;
    }
    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto dwrds = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.numWords; ix++, offs += sizeof(UBX_RXM_SFRBX_V2_GROUP1)) {
        UBX_RXM_SFRBX_V2_GROUP1 w;
        std::memcpy(&w, &data[offs], sizeof(w));
        dwrds.push_back(string::Sprintf("%08x", w.dwrd));
    }

    j = nlohmann::json::object({
        { "gnssId", UBX_GNSSID_STR(head.gnssId) },
        { "svId", head.svId },
        { "sigId", UBX_SIGID_STR(head.gnssId, head.sigId) },
        { "freqId", UBX_RXM_SFRBX_V2_GROUP0_FREQID_TO_SLOT(head.freqId) },
        { "numWords", head.numWords },
        { "chn", head.chn },
        { "version", head.version },
        { "dwrds", dwrds },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_RXM_SPARTN(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() == UBX_RXM_SPARTN_V1_SIZE) && (UBX_RXM_SPARTN_VERSION(data.data()) == UBX_RXM_SPARTN_V1_VERSION)) {
        UBX_RXM_SPARTN_V1_GROUP0 spartn;
        std::memcpy(&spartn, &data[UBX_HEAD_SIZE], sizeof(spartn));
        j = nlohmann::json::object({
            { "version", spartn.version },
            { "subType", spartn.subType },
            { "msgType", spartn.msgType },
            { "msgUsed", UBX_RXM_SPARTN_V1_FLAGS_MSGUSED_STR(spartn.flags) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_SEC_OSNMA(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if ((data.size() < UBX_SEC_OSNMA_V3_MIN_SIZE) || (UBX_SEC_OSNMA_VERSION(data.data()) != UBX_SEC_OSNMA_V3_VERSION)) {
        return;
    }
    UBX_SEC_OSNMA_V3_GROUP0 head;
    std::memcpy(&head, &data[UBX_HEAD_SIZE], sizeof(head));
    const std::size_t num_svs = UBX_SEC_OSNMA_V3_GENERALANDTIMING_AUTHSVS(head.generalAndTiming);
    if (data.size() != (UBX_SEC_OSNMA_V3_MIN_SIZE + (num_svs * sizeof(UBX_SEC_OSNMA_V3_GROUP1)))) {
        return;
    }
    std::size_t offs = UBX_HEAD_SIZE + sizeof(head);
    auto svs = nlohmann::json::array();
    for (std::size_t ix = 0; ix < num_svs; ix++, offs += sizeof(UBX_SEC_OSNMA_V3_GROUP1)) {
        UBX_SEC_OSNMA_V3_GROUP1 s;
        std::memcpy(&s, &data[offs], sizeof(s));
        svs.push_back(nlohmann::json::object({
            { "svId", s.svId },
            { "iode", UBX_SEC_OSNMA_V3_GROUP1_BITFIELD1_IODE(s.bitfield1) },
            { "authNum", UBX_SEC_OSNMA_V3_GROUP1_BITFIELD1_AUTHNUM(s.bitfield1) },
            { "authStatus", UBX_SEC_OSNMA_V3_GROUP1_BITFIELD1_AUTHSTATUS(s.bitfield1) },
        }));
    }

    j = nlohmann::json::object({
        { "version", head.version },
        { "headerAuthStatus", UBX_SEC_OSNMA_V3_NMAHEADER_HEADERAUTHSTATUS(head.nmaHeader) },
        { "nmaStatus", UBX_SEC_OSNMA_V3_NMAHEADER_NMASTATUS_STR(head.nmaHeader) },
        { "chainInForce", UBX_SEC_OSNMA_V3_NMAHEADER_CHAININFORCE(head.nmaHeader) },
        { "cpks", UBX_SEC_OSNMA_V3_NMAHEADER_CPKS_STR(head.nmaHeader) },
        { "osnmaEnabled", UBX_SEC_OSNMA_V3_OSNMAMONITORING_OSNMAENABLED(head.osnmaMonitoring) },
        { "numberSvs", UBX_SEC_OSNMA_V3_OSNMAMONITORING_NUMBERSVS(head.osnmaMonitoring) },
        { "nmaHeaderUpdate", UBX_SEC_OSNMA_V3_OSNMAMONITORING_NMAHEADERUPDATE_STR(head.osnmaMonitoring) },
        { "noData", UBX_SEC_OSNMA_V3_OSNMAMONITORING_NODATA(head.osnmaMonitoring) },
        { "wrongData", UBX_SEC_OSNMA_V3_OSNMAMONITORING_WRONGDATA(head.osnmaMonitoring) },
        { "wrongFlxMac", UBX_SEC_OSNMA_V3_OSNMAMONITORING_WRONGFLXMAC(head.osnmaMonitoring) },
        { "wrongMacLt", UBX_SEC_OSNMA_V3_OSNMAMONITORING_WRONGMACLT(head.osnmaMonitoring) },
        { "timSyncEnabled", UBX_SEC_OSNMA_V3_TIMSYNCREQ_TIMSYNCENABLED(head.timSyncReq) },
        { "timSyncStatus", UBX_SEC_OSNMA_V3_TIMSYNCREQ_TIMSYNCSTATUS_STR(head.timSyncReq) },
        { "timSyncReqDiff", head.timSyncReqDiff },
        { "dsmAuthenticationStatus",
            UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_DSMAUTHENTICATIONSTATUS_STR(head.dsmAuthentication) },
        { "hashFunction", UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_HASHFUNCTION(head.dsmAuthentication) },
        { "macFunction", UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_MACFUNCTION(head.dsmAuthentication) },
        { "pubKeyId", UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_PUBKEYID(head.dsmAuthentication) },
        { "macLookupTable", UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_MACLOOKUPTABLE(head.dsmAuthentication) },
        { "keySize", UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_KEYSIZE(head.dsmAuthentication) },
        { "macSize", UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_MACSIZE(head.dsmAuthentication) },
        { "fromNvs", UBX_SEC_OSNMA_V3_DSMAUTHENTICATION_FROMNVS(head.dsmAuthentication) },
        { "teslaKeyAuthStatus", UBX_SEC_OSNMA_V3_TESLAKEY_TESLAKEYAUTHSTATUS_STR(head.teslaKey) },
        { "wnSf", UBX_SEC_OSNMA_V3_TESLAKEY_WNSF(head.teslaKey) },
        { "towSf", UBX_SEC_OSNMA_V3_TESLAKEY_TOWSF(head.teslaKey) },
        { "chainId", UBX_SEC_OSNMA_V3_TESLAKEY_CHAINID(head.teslaKey) },
        { "authSvs", UBX_SEC_OSNMA_V3_GENERALANDTIMING_AUTHSVS(head.generalAndTiming) },
        { "authNumTim", UBX_SEC_OSNMA_V3_GENERALANDTIMING_AUTHNUMTIM(head.generalAndTiming) },
        { "timingAuthResult", UBX_SEC_OSNMA_V3_GENERALANDTIMING_TIMINGAUTHRESULT_STR(head.generalAndTiming) },
        { "macAdkdType", UBX_SEC_OSNMA_V3_GENERALANDTIMING_MACADKDTYPE_STR(head.generalAndTiming) },
        { "pubKeySrc", UBX_SEC_OSNMA_V3_GENERALANDTIMING_PUBKEYSRC_STR(head.generalAndTiming) },
        { "merkleRootSrc", UBX_SEC_OSNMA_V3_GENERALANDTIMING_MERKLEROOTSRC_STR(head.generalAndTiming) },
        { "merkleRootVal", UBX_SEC_OSNMA_V3_GENERALANDTIMING_MERKLEROOTVAL(head.generalAndTiming) },
        { "futureMerkleRootSrc", UBX_SEC_OSNMA_V3_GENERALANDTIMING_FUTUREMERKLEROOTSRC_STR(head.generalAndTiming) },
        { "futureMerkleRootVal", UBX_SEC_OSNMA_V3_GENERALANDTIMING_FUTUREMERKLEROOTVAL(head.generalAndTiming) },
        { "pubKeyVal", UBX_SEC_OSNMA_V3_GENERALANDTIMING_PUBKEYVAL(head.generalAndTiming) },
        { "futurePubKeyVal", UBX_SEC_OSNMA_V3_GENERALANDTIMING_FUTUREPUBKEYVAL(head.generalAndTiming) },
        { "futurePubKeySrc", UBX_SEC_OSNMA_V3_GENERALANDTIMING_FUTUREPUBKEYSRC_STR(head.generalAndTiming) },
        { "futurePubKeyId", UBX_SEC_OSNMA_V3_GENERALANDTIMING_FUTUREPUBKEYID(head.generalAndTiming) },
        { "svs", svs },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_TIM_SVIN(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_TIM_SVIN_V0_SIZE) {
        UBX_TIM_SVIN_V0_GROUP0 svin;
        std::memcpy(&svin, &data[UBX_HEAD_SIZE], sizeof(svin));
        j = nlohmann::json::object({
            { "dur", svin.dur },
            { "meanX", svin.meanX },
            { "meanY", svin.meanY },
            { "meanZ", svin.meanZ },
            { "meanV", svin.meanV },
            { "obs", svin.obs },
            { "valid", svin.valid != 0 },
            { "active", svin.active != 0 },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_TIM_TM2(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_TIM_TM2_V0_SIZE) {
        UBX_TIM_TM2_V0_GROUP0 tm2;
        std::memcpy(&tm2, &data[UBX_HEAD_SIZE], sizeof(tm2));
        j = nlohmann::json::object({
            { "ch", tm2.ch },
            { "count", tm2.count },
            { "wnR", tm2.wnR },
            { "wnF", tm2.wnF },
            { "towMsR", tm2.towMsR },
            { "towSubMsR", tm2.towSubMsR },
            { "towMsF", tm2.towMsF },
            { "towSubMsF", tm2.towSubMsF },
            { "accEst", tm2.accEst },
            { "mode", UBX_TIM_TM2_V0_FLAGS_MODE_STR(tm2.flags) },
            { "run", UBX_TIM_TM2_V0_FLAGS_RUN_STR(tm2.flags) },
            { "newFallingEdge", UBX_TIM_TM2_V0_FLAGS_NEWFALLINGEDGE(tm2.flags) },
            { "timeBase", UBX_TIM_TM2_V0_FLAGS_TIMEBASE_STR(tm2.flags) },
            { "utcAcAvail", UBX_TIM_TM2_V0_FLAGS_UTCACAVAIL(tm2.flags) },
            { "timeValid", UBX_TIM_TM2_V0_FLAGS_TIMEVALID(tm2.flags) },
            { "newRisingEdge", UBX_TIM_TM2_V0_FLAGS_NEWRISINGEDGE(tm2.flags) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_UBX_TIM_TP(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() == UBX_TIM_TP_V0_SIZE) {
        UBX_TIM_TP_V0_GROUP0 tp;
        std::memcpy(&tp, &data[UBX_HEAD_SIZE], sizeof(tp));
        j = nlohmann::json::object({
            { "towMs", tp.towMs },
            { "towSubMs", tp.towSubMs },
            { "qErr", tp.qErr },
            { "week", tp.week },
            { "timeBase", UBX_TIM_TP_V0_FLAGS_TIMEBASE_STR(tp.flags) },
            { "utc", UBX_TIM_TP_V0_FLAGS_UTC(tp.flags) },
            { "raim", UBX_TIM_TP_V0_FLAGS_RAIM_STR(tp.flags) },
            { "qErrInvalid", UBX_TIM_TP_V0_FLAGS_QERRINVALID(tp.flags) },
            { "tpNotLocked", UBX_TIM_TP_V0_FLAGS_TPNOTLOCKED(tp.flags) },
            { "timeRefGnss", UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS_STR(tp.refInfo) },
            { "utcStandard", UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_STR(tp.refInfo) },
        });
    }
}

// ---------------------------------------------------------------------------------------------------------------------

// UBX-MGA-GAL and UBX-MGA-INI are multiplexed on the payload type byte
inline void to_json_UBX_MGA_GAL(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() < (UBX_HEAD_SIZE + 1)) {
        return;
    }
    switch (data[UBX_HEAD_SIZE]) {  // clang-format off
        case UBX_MGA_GAL_OSNMA_PUBKEY_TYPE: to_json_UBX_MGA_GAL_OSNMA_PUBKEY(j, data); break;
        case UBX_MGA_GAL_OSNMA_MERKLE_TYPE: to_json_UBX_MGA_GAL_OSNMA_MERKLE(j, data); break;
    }  // clang-format on
}

inline void to_json_UBX_MGA_INI(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() < (UBX_HEAD_SIZE + 1)) {
        return;
    }
    switch (data[UBX_HEAD_SIZE]) {  // clang-format off
        case UBX_MGA_INI_TIME_UTC_TYPE:  to_json_UBX_MGA_INI_TIME_UTC(j, data);  break;
        case UBX_MGA_INI_TIME_GNSS_TYPE: to_json_UBX_MGA_INI_TIME_GNSS(j, data); break;
    }  // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

inline nlohmann::json to_json_UBX(const ParserMsg& msg)
{
    auto j = nlohmann::json::object();
    switch (UbxClsId(msg.Data())) {  // clang-format off
        case UBX_ACK_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_ACK_ACK_MSGID:          to_json_UBX_ACK(j, msg.data_, true);      break;
            case UBX_ACK_NAK_MSGID:          to_json_UBX_ACK(j, msg.data_, false);     break; } break;
        case UBX_CFG_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_CFG_CFG_MSGID:          to_json_UBX_CFG_CFG(j, msg.data_);        break;
            case UBX_CFG_RST_MSGID:          to_json_UBX_CFG_RST(j, msg.data_);        break;
            case UBX_CFG_VALDEL_MSGID:       to_json_UBX_CFG_VALDEL(j, msg.data_);     break;
            case UBX_CFG_VALGET_MSGID:       to_json_UBX_CFG_VALGET(j, msg.data_);     break;
            case UBX_CFG_VALSET_MSGID:       to_json_UBX_CFG_VALSET(j, msg.data_);     break; } break;
        case UBX_ESF_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_ESF_MEAS_MSGID:         to_json_UBX_ESF_MEAS(j, msg.data_);       break;
            case UBX_ESF_STATUS_MSGID:       to_json_UBX_ESF_STATUS(j, msg.data_);     break; } break;
        case UBX_MGA_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_MGA_GAL_MSGID:          to_json_UBX_MGA_GAL(j, msg.data_);        break;
            case UBX_MGA_INI_MSGID:          to_json_UBX_MGA_INI(j, msg.data_);        break; } break;
        case UBX_MON_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_MON_COMMS_MSGID:        to_json_UBX_MON_COMMS(j, msg.data_);      break;
            case UBX_MON_HW_MSGID:           to_json_UBX_MON_HW(j, msg.data_);         break;
            case UBX_MON_HW2_MSGID:          to_json_UBX_MON_HW2(j, msg.data_);        break;
            case UBX_MON_HW3_MSGID:          to_json_UBX_MON_HW3(j, msg.data_);        break;
            case UBX_MON_RF_MSGID:           to_json_UBX_MON_RF(j, msg.data_);         break;
            case UBX_MON_SPAN_MSGID:         to_json_UBX_MON_SPAN(j, msg.data_);       break;
            case UBX_MON_SYS_MSGID:          to_json_UBX_MON_SYS(j, msg.data_);        break;
            case UBX_MON_TEMP_MSGID:         to_json_UBX_MON_TEMP(j, msg.data_);       break;
            case UBX_MON_VER_MSGID:          to_json_UBX_MON_VER(j, msg.data_);        break; } break;
        case UBX_NAV_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_NAV_ATT_MSGID:          to_json_UBX_NAV_ATT(j, msg.data_);        break;
            case UBX_NAV_CLOCK_MSGID:        to_json_UBX_NAV_CLOCK(j, msg.data_);      break;
            case UBX_NAV_COV_MSGID:          to_json_UBX_NAV_COV(j, msg.data_);        break;
            case UBX_NAV_DOP_MSGID:          to_json_UBX_NAV_DOP(j, msg.data_);        break;
            case UBX_NAV_EELL_MSGID:         to_json_UBX_NAV_EELL(j, msg.data_);       break;
            case UBX_NAV_EOE_MSGID:          to_json_UBX_NAV_EOE(j, msg.data_);        break;
            case UBX_NAV_HPPOSECEF_MSGID:    to_json_UBX_NAV_HPPOSECEF(j, msg.data_);  break;
            case UBX_NAV_HPPOSLLH_MSGID:     to_json_UBX_NAV_HPPOSLLH(j, msg.data_);   break;
            case UBX_NAV_POSECEF_MSGID:      to_json_UBX_NAV_POSECEF(j, msg.data_);    break;
            case UBX_NAV_PVT_MSGID:          to_json_UBX_NAV_PVT(j, msg.data_);        break;
            case UBX_NAV_RELPOSNED_MSGID:    to_json_UBX_NAV_RELPOSNED(j, msg.data_);  break;
            case UBX_NAV_SAT_MSGID:          to_json_UBX_NAV_SAT(j, msg.data_);        break;
            case UBX_NAV_SIG_MSGID:          to_json_UBX_NAV_SIG(j, msg.data_);        break;
            case UBX_NAV_STATUS_MSGID:       to_json_UBX_NAV_STATUS(j, msg.data_);     break;
            case UBX_NAV_TIMEBDS_MSGID:      to_json_UBX_NAV_TIMEBDS(j, msg.data_);    break;
            case UBX_NAV_TIMEGAL_MSGID:      to_json_UBX_NAV_TIMEGAL(j, msg.data_);    break;
            case UBX_NAV_TIMEGLO_MSGID:      to_json_UBX_NAV_TIMEGLO(j, msg.data_);    break;
            case UBX_NAV_TIMEGPS_MSGID:      to_json_UBX_NAV_TIMEGPS(j, msg.data_);    break;
            case UBX_NAV_TIMELS_MSGID:       to_json_UBX_NAV_TIMELS(j, msg.data_);     break;
            case UBX_NAV_TIMETRUSTED_MSGID:  to_json_UBX_NAV_TIMETRUSTED(j, msg.data_); break;
            case UBX_NAV_TIMEUTC_MSGID:      to_json_UBX_NAV_TIMEUTC(j, msg.data_);    break;
            case UBX_NAV_VELECEF_MSGID:      to_json_UBX_NAV_VELECEF(j, msg.data_);    break; } break;
        case UBX_NAV2_CLSID:             switch (UbxMsgId(msg.Data())) {
            case UBX_NAV2_CLOCK_MSGID:       to_json_UBX_NAV_CLOCK(j, msg.data_);      break;
            case UBX_NAV2_COV_MSGID:         to_json_UBX_NAV_COV(j, msg.data_);        break;
            case UBX_NAV2_DOP_MSGID:         to_json_UBX_NAV_DOP(j, msg.data_);        break;
            case UBX_NAV2_EOE_MSGID:         to_json_UBX_NAV_EOE(j, msg.data_);        break;
            case UBX_NAV2_POSECEF_MSGID:     to_json_UBX_NAV_POSECEF(j, msg.data_);    break;
            case UBX_NAV2_PVT_MSGID:         to_json_UBX_NAV_PVT(j, msg.data_);        break;
            case UBX_NAV2_SAT_MSGID:         to_json_UBX_NAV_SAT(j, msg.data_);        break;
            case UBX_NAV2_SIG_MSGID:         to_json_UBX_NAV_SIG(j, msg.data_);        break;
            case UBX_NAV2_STATUS_MSGID:      to_json_UBX_NAV_STATUS(j, msg.data_);     break;
            case UBX_NAV2_TIMEBDS_MSGID:     to_json_UBX_NAV_TIMEBDS(j, msg.data_);    break;
            case UBX_NAV2_TIMEGAL_MSGID:     to_json_UBX_NAV_TIMEGAL(j, msg.data_);    break;
            case UBX_NAV2_TIMEGLO_MSGID:     to_json_UBX_NAV_TIMEGLO(j, msg.data_);    break;
            case UBX_NAV2_TIMEGPS_MSGID:     to_json_UBX_NAV_TIMEGPS(j, msg.data_);    break;
            case UBX_NAV2_TIMELS_MSGID:      to_json_UBX_NAV_TIMELS(j, msg.data_);     break;
            case UBX_NAV2_TIMEUTC_MSGID:     to_json_UBX_NAV_TIMEUTC(j, msg.data_);    break;
            case UBX_NAV2_VELECEF_MSGID:     to_json_UBX_NAV_VELECEF(j, msg.data_);    break; } break;
        case UBX_RXM_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_RXM_RAWX_MSGID:         to_json_UBX_RXM_RAWX(j, msg.data_);       break;
            case UBX_RXM_RTCM_MSGID:         to_json_UBX_RXM_RTCM(j, msg.data_);       break;
            case UBX_RXM_SFRBX_MSGID:        to_json_UBX_RXM_SFRBX(j, msg.data_);      break;
            case UBX_RXM_SPARTN_MSGID:       to_json_UBX_RXM_SPARTN(j, msg.data_);     break; } break;
        case UBX_SEC_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_SEC_OSNMA_MSGID:        to_json_UBX_SEC_OSNMA(j, msg.data_);      break; } break;
        case UBX_TIM_CLSID:              switch (UbxMsgId(msg.Data())) {
            case UBX_TIM_SVIN_MSGID:         to_json_UBX_TIM_SVIN(j, msg.data_);       break;
            case UBX_TIM_TM2_MSGID:          to_json_UBX_TIM_TM2(j, msg.data_);        break;
            case UBX_TIM_TP_MSGID:           to_json_UBX_TIM_TP(j, msg.data_);         break; } break;
    }  // clang-format on
    return j;
}

}  // namespace fpsdk::common::parser::ubx
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_PARSER_UBX_HPP__
