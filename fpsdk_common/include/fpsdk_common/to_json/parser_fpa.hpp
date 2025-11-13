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
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::parser::fpa messages
 */
#ifndef __FPSDK_COMMON_TO_JSON_PARSER_FPA_HPP__
#define __FPSDK_COMMON_TO_JSON_PARSER_FPA_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../parser/fpa.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::parser::fpa {

inline void to_json(nlohmann::json& j, const FpaInitStatus e)
{
    j = FpaInitStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaFusionStatusLegacy e)
{
    j = FpaFusionStatusLegacyStr(e);
}

inline void to_json(nlohmann::json& j, const FpaMeasStatus e)
{
    j = FpaMeasStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaImuStatus e)
{
    j = FpaImuStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaImuStatusLegacy e)
{
    j = FpaImuStatusLegacyStr(e);
}

inline void to_json(nlohmann::json& j, const FpaImuNoise e)
{
    j = FpaImuNoiseStr(e);
}

inline void to_json(nlohmann::json& j, const FpaImuConv e)
{
    j = FpaImuConvStr(e);
}

inline void to_json(nlohmann::json& j, const FpaGnssStatus e)
{
    j = FpaGnssStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaCorrStatus e)
{
    j = FpaCorrStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaBaselineStatus e)
{
    j = FpaBaselineStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaCamStatus e)
{
    j = FpaCamStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaWsStatus e)
{
    j = FpaWsStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaWsStatusLegacy e)
{
    j = FpaWsStatusLegacyStr(e);
}

inline void to_json(nlohmann::json& j, const FpaWsConv e)
{
    j = FpaWsConvStr(e);
}

inline void to_json(nlohmann::json& j, const FpaMarkersStatus e)
{
    j = FpaMarkersStatusStr(e);
}

inline void to_json(nlohmann::json& j, const FpaMarkersConv e)
{
    j = FpaMarkersConvStr(e);
}

inline void to_json(nlohmann::json& j, const FpaGnssFix e)
{
    j = FpaGnssFixStr(e);
}

inline void to_json(nlohmann::json& j, const FpaEpoch e)
{
    j = FpaEpochStr(e);
}

inline void to_json(nlohmann::json& j, const FpaAntState e)
{
    j = FpaAntStateStr(e);
}

inline void to_json(nlohmann::json& j, const FpaAntPower e)
{
    j = FpaAntPowerStr(e);
}

inline void to_json(nlohmann::json& j, const FpaTextLevel e)
{
    j = FpaTextLevelStr(e);
}

inline void to_json(nlohmann::json& j, const FpaTimebase e)
{
    j = FpaTimebaseStr(e);
}

inline void to_json(nlohmann::json& j, const FpaTimeref e)
{
    j = FpaTimerefStr(e);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaInt& m)
{
    if (m.valid) {
        j = m.value;
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const FpaFloat& m)
{
    if (m.valid) {
        j = m.value;
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const FpaFloat3& m)
{
    if (m.valid) {
        j = m.values;
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const FpaFloat4& m)
{
    if (m.valid) {
        j = m.values;
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const FpaFloat6& m)
{
    if (m.valid) {
        j = m.values;
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const FpaGpsTime& m)
{
    j = nlohmann::json::object({
        { "gps_week", m.week },
        { "gps_tow", m.tow },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaEoePayload& m)
{
    j = nlohmann::json::object({
        { "epoch", m.epoch },
    });
    j.update(m.gps_time);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaGnssantPayload& m)
{
    j = nlohmann::json::object({
        { "gnss1_state", m.gnss1_state },
        { "gnss1_power", m.gnss1_power },
        { "gnss1_age", m.gnss1_age },
        { "gnss2_state", m.gnss2_state },
        { "gnss2_power", m.gnss2_power },
        { "gnss2_age", m.gnss2_age },
    });
    j.update(m.gps_time);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaGnsscorrPayload& m)
{
    j = nlohmann::json::object({
        { "gnss1_fix", m.gnss1_fix },
        { "gnss1_nsig_l1", m.gnss1_nsig_l1 },
        { "gnss1_nsig_l2", m.gnss1_nsig_l2 },
        { "gnss2_fix", m.gnss2_fix },
        { "gnss2_nsig_l2", m.gnss2_nsig_l1 },
        { "gnss2_nsig_l2", m.gnss2_nsig_l2 },
        { "corr_latency", m.corr_latency },
        { "corr_update_rate", m.corr_update_rate },
        { "corr_data_rate", m.corr_data_rate },
        { "corr_msg_rate", m.corr_msg_rate },
        { "sta_id", m.sta_id },
        { "sta_llh", m.sta_llh },
        { "sta_dist", m.sta_dist },
    });
    j.update(m.gps_time);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaImuPayload& m)
{
    j = nlohmann::json::object({
        { "acc", m.acc },
        { "rot", m.rot },
        { "bias_comp", m.bias_comp },
        { "imu_status", m.imu_status },
    });
    j.update(m.gps_time);
}

inline void to_json(nlohmann::json& j, const FpaRawimuPayload& m)
{
    to_json(j, dynamic_cast<const FpaImuPayload&>(m));
}

inline void to_json(nlohmann::json& j, const FpaCorrimuPayload& m)
{
    to_json(j, dynamic_cast<const FpaImuPayload&>(m));
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaImubiasPayload& m)
{
    j = nlohmann::json::object({
        { "fusion_imu", m.fusion_imu },
        { "imu_status", m.imu_status },
        { "imu_noise", m.imu_noise },
        { "imu_conv", m.imu_conv },
        { "bias_acc", m.bias_acc },
        { "bias_gyr", m.bias_gyr },
        { "bias_cov_acc", m.bias_cov_acc },
        { "bias_cov_gyr", m.bias_cov_gyr },
    });
    j.update(m.gps_time);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaLlhPayload& m)
{
    j = nlohmann::json::object({
        { "llh", m.llh },
        { "longitude", nullptr },
        { "height", nullptr },
        { "cov_enu", m.cov_enu },
    });
    j.update(m.gps_time);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaOdomPayload& m)
{
    j = nlohmann::json::object({
        { "pos", m.pos },
        { "orientation", m.orientation },
        { "vel", m.vel },
        { "rot", m.rot },
        { "acc", m.acc },
        { "fusion_status", m.fusion_status },
        { "imu_bias_status", m.imu_bias_status },
        { "gnss1_fix", m.gnss1_fix },
        { "gnss2_fix", m.gnss2_fix },
        { "wheelspeed_status", m.wheelspeed_status },
        { "pos_cov", m.pos_cov },
        { "orientation_cov", m.orientation_cov },
        { "vel_cov", m.vel_cov },
    });
    j.update(m.gps_time);
}

inline void to_json(nlohmann::json& j, const FpaOdometryPayload& m)
{
    if (m.valid_) {
        to_json(j, dynamic_cast<const FpaOdomPayload&>(m));
        j["version"] = m.version;
    }
}

inline void to_json(nlohmann::json& j, const FpaOdomenuPayload& m)
{
    to_json(j, dynamic_cast<const FpaOdomPayload&>(m));
}

inline void to_json(nlohmann::json& j, const FpaOdomshPayload& m)
{
    to_json(j, dynamic_cast<const FpaOdomPayload&>(m));
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaOdomstatusPayload& m)
{
    j = nlohmann::json::object({
        { "init_status", m.init_status },
        { "fusion_imu", m.fusion_imu },
        { "fusion_gnss1", m.fusion_gnss1 },
        { "fusion_gnss2", m.fusion_gnss2 },
        { "fusion_corr", m.fusion_corr },
        { "fusion_cam1", m.fusion_cam1 },
        { "fusion_ws", m.fusion_ws },
        { "fusion_markers", m.fusion_markers },
        { "imu_status", m.imu_status },
        { "imu_noise", m.imu_noise },
        { "imu_conv", m.imu_conv },
        { "gnss1_status", m.gnss1_status },
        { "gnss2_status", m.gnss2_status },
        { "baseline_status", m.baseline_status },
        { "corr_status", m.corr_status },
        { "cam1_status", m.cam1_status },
        { "ws_status", m.ws_status },
        { "ws_conv", m.ws_conv },
        { "markers_status", m.markers_status },
        { "markers_conv", m.markers_conv },
    });
    j.update(m.gps_time);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaTextPayload& m)
{
    j = nlohmann::json::object({
        { "level", m.level },
        { "text", m.text },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaTfPayload& m)
{
    j = nlohmann::json::object({
        { "frame_a", m.frame_a },
        { "frame_b", m.frame_b },
        { "translation", m.translation },
        { "orientation", m.orientation },
    });
    j.update(m.gps_time);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaTpPayload& m)
{
    j = nlohmann::json::object({
        { "tp_name", m.tp_name },
        { "timebase", m.timebase },
        { "timeref", m.timeref },
        { "tp_week", m.tp_week },
        { "tp_tow_sec", m.tp_tow_sec },
        { "tp_tow_psec", m.tp_tow_psec },
        { "gps_leaps", m.gps_leaps },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaVersionPayload& m)
{
    j = nlohmann::json::object({
        { "sw_version", m.sw_version },
        { "hw_name", m.hw_name },
        { "hw_ver", m.hw_ver },
        { "hw_uid", m.hw_uid },
        { "product_model", m.product_model },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FpaPayloadPtr& fpa)
{
    j = nlohmann::json::object();
    if (!fpa) {
        return;
    }
    j["_valid"] = fpa->valid_;
    if (fpa) {
        switch (fpa->msg_type_) {  // clang-format off
            case FpaMessageType::UNSPECIFIED: break;
            case FpaMessageType::EOE:         j.update(dynamic_cast<const FpaEoePayload&>(*fpa));        break;
            case FpaMessageType::GNSSANT:     j.update(dynamic_cast<const FpaGnssantPayload&>(*fpa));    break;
            case FpaMessageType::GNSSCORR:    j.update(dynamic_cast<const FpaGnsscorrPayload&>(*fpa));   break;
            case FpaMessageType::RAWIMU:      j.update(dynamic_cast<const FpaRawimuPayload&>(*fpa));     break;
            case FpaMessageType::CORRIMU:     j.update(dynamic_cast<const FpaCorrimuPayload&>(*fpa));    break;
            case FpaMessageType::IMUBIAS:     j.update(dynamic_cast<const FpaImubiasPayload&>(*fpa));    break;
            case FpaMessageType::LLH:         j.update(dynamic_cast<const FpaLlhPayload&>(*fpa));        break;
            case FpaMessageType::ODOMETRY:    j.update(dynamic_cast<const FpaOdometryPayload&>(*fpa));   break;
            case FpaMessageType::ODOMENU:     j.update(dynamic_cast<const FpaOdomenuPayload&>(*fpa));    break;
            case FpaMessageType::ODOMSH:      j.update(dynamic_cast<const FpaOdomshPayload&>(*fpa));     break;
            case FpaMessageType::ODOMSTATUS:  j.update(dynamic_cast<const FpaOdomstatusPayload&>(*fpa)); break;
            case FpaMessageType::TEXT:        j.update(dynamic_cast<const FpaTextPayload&>(*fpa));       break;
            case FpaMessageType::TF:          j.update(dynamic_cast<const FpaTfPayload&>(*fpa));         break;
            case FpaMessageType::TP:          j.update(dynamic_cast<const FpaTpPayload&>(*fpa));         break;
            case FpaMessageType::VERSION:     j.update(dynamic_cast<const FpaVersionPayload&>(*fpa));    break;
        }  // clang-format on
    }
}

}  // namespace fpsdk::common::parser::fpa
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_PARSER_FPA_HPP__
