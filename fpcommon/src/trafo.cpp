/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Transformation utilities
 */

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */
#include "fpcommon/trafo.hpp"

#include "fpcommon/math.hpp"

namespace fp {
namespace common {
namespace trafo {
/* ****************************************************************************************************************** */

using namespace fp::common::math;

/**
 * @details
 *
 * \f[
 *
 * \text{Rotate a vector from ECEF to ENU:} \\
 * V_{ENU} = M \cdot V_{ECEF} \\
 * \text{Rotate a covariance matrix from ECEF to ENU:} \\
 * Cov_{ENU} = M \cdot Cov_{ECEF} \cdot M^{T} \\
 *
 * \f]
 *
 * Reference https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
 */
Eigen::Matrix3d RotEnuEcef(const double lat, const double lon)
{
    const double s_lon = sin(lon);
    const double c_lon = cos(lon);
    const double s_lat = sin(lat);
    const double c_lat = cos(lat);
    const double m00 = -s_lon;
    const double m01 = c_lon;
    const double m02 = 0;
    const double m10 = -c_lon * s_lat;
    const double m11 = -s_lon * s_lat;
    const double m12 = c_lat;
    const double m20 = c_lon * c_lat;
    const double m21 = s_lon * c_lat;
    const double m22 = s_lat;

    Eigen::Matrix3d res;
    // This initializes the matrix row by row
    res << m00, m01, m02, m10, m11, m12, m20, m21, m22;

    return res;
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @details
 *
 * \f[
 *
 * \text{Rotate a vector from ECEF to ENU:} \\
 * V_{ENU} = M \cdot V_{ECEF} \\
 * \text{Rotate a covariance matrix from ECEF to ENU:} \\
 * Cov_{ENU} = M \cdot Cov_{ECEF} \cdot M^{T} \\
 *
 * \f]
 */
Eigen::Matrix3d RotEnuEcef(const Eigen::Vector3d& ecef)
{
    const Eigen::Vector3d wgs84llh = TfWgs84LlhEcef(ecef);
    return RotEnuEcef(wgs84llh.x(), wgs84llh.y());
}

Eigen::Matrix3d RotNedEnu()
{
    // | 0, 1, 0 |
    // | 1, 0, 0 |
    // | 0, 0,-1 |
    Eigen::Matrix3d rot_ned_enu;
    rot_ned_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    return rot_ned_enu;
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Matrix3d RotNedEcef(const double lat, const double lon)
{
    return RotNedEnu() * RotEnuEcef(lat, lon);
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Matrix3d RotNedEcef(const Eigen::Vector3d& ecef)
{
    const Eigen::Vector3d wgs84llh = TfWgs84LlhEcef(ecef);
    return RotNedEcef(wgs84llh.x(), wgs84llh.y());
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d TfEnuEcef(const Eigen::Vector3d& ecef, const Eigen::Vector3d& wgs84llh_ref)
{
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return RotEnuEcef(wgs84llh_ref.x(), wgs84llh_ref.y()) * (ecef - ecef_ref);
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d TfEcefEnu(const Eigen::Vector3d& enu, const Eigen::Vector3d& wgs84llh_ref)
{
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return ecef_ref + RotEnuEcef(wgs84llh_ref.x(), wgs84llh_ref.y()).transpose() * enu;
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d TfNedEcef(const Eigen::Vector3d& ecef, const Eigen::Vector3d& wgs84llh_ref)
{
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return RotNedEcef(wgs84llh_ref.x(), wgs84llh_ref.y()) * (ecef - ecef_ref);
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d TfEcefNed(const Eigen::Vector3d& ned, const Eigen::Vector3d& wgs84llh_ref)
{
    const Eigen::Vector3d ecef_ref = TfEcefWgs84Llh(wgs84llh_ref);
    return ecef_ref + RotNedEcef(wgs84llh_ref.x(), wgs84llh_ref.y()).transpose() * ned;
}

// ---------------------------------------------------------------------------------------------------------------------

// Implementation based on paper
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#The_application_of_Ferrari's_solution
Eigen::Vector3d TfWgs84LlhEcef(const Eigen::Vector3d& ecef)
{
    const double x = ecef.x();
    const double y = ecef.y();
    const double z = ecef.z();

    const double x_2 = x * x;  //!< x^2
    const double y_2 = y * y;  //!< y^2
    const double z_2 = z * z;  //!< z^2

    const double r_2 = (x_2 + y_2);  //!< r^2
    const double r = sqrt(r_2);      //!< r

    const double F = 54.0 * WGS84_B2 * z_2;
    const double G = r_2 + (1 - WGS84_E2) * z_2 - WGS84_E2 * (WGS84_A2 - WGS84_B2);
    const double c = WGS84_E2 * WGS84_E2 * F * r_2 / (G * G * G);
    const double s = cbrt(1 + c + sqrt(c * c + 2 * c));
    const double P = F / (3.0 * (s + 1.0 + 1.0 / s) * (s + 1.0 + 1.0 / s) * G * G);
    const double Q = sqrt(1 + 2 * WGS84_E2 * WGS84_E2 * P);
    const double r0 = -P * WGS84_E2 * r / (1 + Q) +
                      sqrt(0.5 * WGS84_A2 * (1.0 + 1.0 / Q) - (P * (1 - WGS84_E2) * z_2 / (Q + Q * Q)) - 0.5 * P * r_2);
    const double t1 = (r - WGS84_E2 * r0);
    const double t1_2 = t1 * t1;
    const double U = sqrt(t1_2 + z_2);
    const double V = sqrt(t1_2 + (1 - WGS84_E2) * z_2);
    const double a_V = WGS84_A * V;
    const double z0 = WGS84_B2 * z / a_V;

    const double h = U * (1 - WGS84_B2 / a_V);
    const double lat = atan((z + WGS84_EE2 * z0) / r);
    const double lon = atan2(y, x);

    return Eigen::Vector3d(lat, lon, h);
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d TfEcefWgs84Llh(const Eigen::Vector3d& wgs84llh)
{
    const double lat = wgs84llh.x();
    const double lon = wgs84llh.y();
    const double height = wgs84llh.z();
    const double s_lat = sin(lat);
    const double c_lat = cos(lat);
    const double s_lon = sin(lon);
    const double c_lon = cos(lon);
    // N is in meters
    const double n = WGS84_A / sqrt(1.0 - WGS84_E2 * s_lat * s_lat);
    const double n_plus_height = n + height;

    Eigen::Vector3d ecef;
    ecef.x() = n_plus_height * c_lat * c_lon;
    ecef.y() = n_plus_height * c_lat * s_lon;
    ecef.z() = (n * (1 - WGS84_E2) + height) * s_lat;

    return ecef;
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d EcefPoseToEnuEul(const Eigen::Vector3d& ecef_p, const Eigen::Matrix3d& ecef_r)
{
    //! Rotation Matrix to convert to ENU frame
    const Eigen::Matrix3d rot_enu_ecef = RotEnuEcef(ecef_p);
    //! Convert the Pose into ENU frame
    const Eigen::Matrix3d rot_enu_body = rot_enu_ecef * ecef_r;
    //! Convert Rotation Matrix into Yaw-Pitch-Roll
    return RotToEul(rot_enu_body);
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d QuatToEul(const Eigen::Quaterniond& q)
{
    auto qw = q.w();
    auto qx = q.x();
    auto qy = q.y();
    auto qz = q.z();
    auto eul0 = atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
    auto eul1 = asin(-2.0 * (qx * qz - qw * qy));
    auto eul2 = atan2(2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);

    Eigen::Vector3d eul;
    eul << eul0, eul1, eul2;

    return eul;
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d RotToEul(const Eigen::Matrix3d& rot)
{
    const Eigen::Quaterniond quat(rot);
    return QuatToEul(quat);
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d LlhDegToRad(const Eigen::Vector3d& llh_deg)
{
    return Eigen::Vector3d(DegToRad(llh_deg.x()), DegToRad(llh_deg.y()), llh_deg.z());
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d LlhRadToDeg(const Eigen::Vector3d& llh_rad)
{
    return Eigen::Vector3d(RadToDeg(llh_rad.x()), RadToDeg(llh_rad.y()), llh_rad.z());
}

/* ****************************************************************************************************************** */
}  // namespace trafo
}  // namespace common
}  // namespace fp
