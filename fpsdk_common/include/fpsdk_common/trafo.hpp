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
 * @brief Fixposition SDK: Transformation utilities
 *
 * @page FPSDK_COMMON_TRAFO Transformation utilities
 *
 * **API**: fpsdk_common/trafo.hpp and fpsdk::common::trafo
 *
 */
#ifndef __FPSDK_COMMON_TRAFO_HPP__
#define __FPSDK_COMMON_TRAFO_HPP__

/* LIBC/STL */
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

/* EXTERNAL */
#include "fpsdk_common/ext/eigen_core.hpp"
#include "fpsdk_common/ext/eigen_geometry.hpp"

/* PACKAGE */
#include "types.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief Transformation utilities
 */
namespace trafo {
/* ****************************************************************************************************************** */

/**
 * @name WGS-84 constants
 *
 * https://en.wikipedia.org/wiki/World_Geodetic_System#Defining_Parameters
 *
 * @{
 */
static constexpr double WGS84_A = 6378137.0;                      //!< WGS-84 Earth radius major axis [m]
static constexpr double WGS84_B = 6356752.314245;                 //!< WGS-84 Earth radius minor axis [m]
static constexpr double WGS84_C = 40075016.6855785;               //!< Circumference at equator (= 2π * WGS84_A) [m]
static constexpr double WGS84_1F = 298.257223563;                 //!< WGS-84 1/f inverse of flattening parameter
static constexpr double WGS84_E2 = 6.69437999014e-3;              //!< WGS-84 first eccentricity squared
static constexpr double WGS84_A2 = WGS84_A * WGS84_A;             //!< WGS-84 a^2
static constexpr double WGS84_B2 = WGS84_B * WGS84_B;             //!< WGS-84 b^2
static constexpr double WGS84_EE2 = (WGS84_A2 / WGS84_B2) - 1.0;  //!< WGS-84 e'^2 second eccentricity squared
///@}

/**
 * @brief Calculate the rotation matrix from ECEF to ENU for a given reference latitude/longitude
 *
 * @param[in]  lat  Reference latitude [rad]
 * @param[in]  lon  Reference longitude [rad]
 *
 * @returns the rotation matrix from ECEF to ENU
 *
 * Reference https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
 */
Eigen::Matrix3d RotEnuEcef(const double lat, const double lon);

/**
 * @brief Calculate the rotation matrix from ECEF to ENU for a given reference position
 *
 * @note The reference position is projected to the WGS84 ellipsoid
 *
 * @param[in]  ecef  Reference position in ECEF [m]
 *
 * @returns the rotation matrix from ECEF to ENU
 */
Eigen::Matrix3d RotEnuEcef(const Eigen::Vector3d& ecef);

/**
 * @brief Returns rotation matrix between NED and ENU
 *
 *     | 0,  1,  0 |
 *     | 1,  0,  0 |
 *     | 0,  0, -1 |
 *
 * @returns the rotation matrix between NED and ENU
 */
Eigen::Matrix3d RotNedEnu();

/**
 * @brief Calculate the rotation matrix from ECEF to NED for a given reference latitude/longitude
 *
 * @param[in]  lat  Reference latitude [rad]
 * @param[in]  lon  Reference longitude [rad]
 *
 * @returns the rotation matrix from ECEF to NED
 */
Eigen::Matrix3d RotNedEcef(const double lat, const double lon);

/**
 * @brief Calculate the rotation matrix from ECEF to NED for a given reference origin
 *
 * @note The reference position is projected to the WGS84 ellipsoid
 *
 * @param[in]  ecef  Reference position in ECEF [m]
 *
 * @returns the rRotation matrix from ECEF to NED
 */
Eigen::Matrix3d RotNedEcef(const Eigen::Vector3d& ecef);

/**
 * @brief Transform ECEF coordinate to ENU with specified ENU-origin
 *
 * @param[in]  ecef          ECEF position to be transformed [m]
 * @param[in]  wgs84llh_ref  ENU-origin in geodetic coordinates (lat [rad], lon [rad], height [m])
 *
 * @returns the position in ENU coordinates
 */
Eigen::Vector3d TfEnuEcef(const Eigen::Vector3d& ecef, const Eigen::Vector3d& wgs84llh_ref);

/**
 * @brief Transform ENU coordinate to ECEF with specified ENU-origin
 *
 * @param[in]  enu           ENU position to be transformed [m]
 * @param[in]  wgs84llh_ref  ENU-origin in geodetic coordinates (lat [rad], lon [rad], height [m])
 *
 * @returns @todo documentation
 */
Eigen::Vector3d TfEcefEnu(const Eigen::Vector3d& enu, const Eigen::Vector3d& wgs84llh_ref);

/**
 * @brief Convert ECEF coordinates to ENU with specified ENU origin
 *
 * @param[in]  ecef          ECEF position to be converted
 * @param[in]  wgs84llh_ref  ENU-origin input given in geodetic coordinates
 *
 * @returns the position in ENU coordinates
 */
Eigen::Vector3d TfNedEcef(const Eigen::Vector3d& ecef, const Eigen::Vector3d& wgs84llh_ref);

/**
 * @todo documentation
 */
Eigen::Vector3d TfEcefNed(const Eigen::Vector3d& ned, const Eigen::Vector3d& wgs84llh_ref);

/**
 * @brief Convert geodetic coordinates (latitude, longitude, height) to ECEF (x, y, z).
 *
 * @param[in]  wgs84llh  Geodetic coordinates (lat [rad], lon [rad], height [m])
 *
 * @returns the ECEF coordinates [m]
 */
Eigen::Vector3d TfEcefWgs84Llh(const Eigen::Vector3d& wgs84llh);

/**
 * @brief Convert ECEF (x, y, z) coordinates to geodetic coordinates (latitude, longitude, height)
 *
 * @param[in] ecef ECEF coordinates [m]
 *
 * @returns the geodetic coordinates (lat [rad], lon [rad], height [m])
 */
Eigen::Vector3d TfWgs84LlhEcef(const Eigen::Vector3d& ecef);

/**
 * @brief Calculate yaw, pitch and roll in ENU from a given pose in ECEF
 *
 * Yaw is -Pi/2 when X points North, because ENU starts with East
 *
 * @param[in]  ecef_p  3D position vector in ECEF
 * @param[in]  ecef_r  3x3 rotation matrix representing rotation from body to ECEF
 *
 * @returns yaw, pitch and roll in ENU
 */
Eigen::Vector3d EcefPoseToEnuEul(const Eigen::Vector3d& ecef_p, const Eigen::Matrix3d& ecef_r);

/**
 * @brief Vector4 quaternion to intrinsic Euler angles in ZYX (yaw, pitch, roll)
 *
 * @param[in]  quat  Quaternion (w, i, j, k)
 *
 * @returns the intrinsic Euler angles in ZYX (yaw, pitch, roll) order
 */
Eigen::Vector3d QuatToEul(const Eigen::Quaterniond& quat);

/**
 * @brief Rotation Matrix to intrinsic Euler angles in ZYX (yaw, pitch, roll) order in radian
 *
 * @param[in]  rot  Eigen::Matrix3d
 *
 * @returns intrinsic euler angle in ZYX (Yaw-Pitch-Roll) order in radian
 */
Eigen::Vector3d RotToEul(const Eigen::Matrix3d& rot);

/**
 * @brief Convert llh from deg to rad, height component unchanged
 *
 * @param[in]  llh_deg  llh in degrees
 *
 * @returns llh in radian
 */
Eigen::Vector3d LlhDegToRad(const Eigen::Vector3d& llh_deg);

/**
 * @brief Convert llh from rad to deg, height component unchanged
 *
 * @param[in]  llh_rad  llh in radian
 *
 * @returns llh in degrees
 */
Eigen::Vector3d LlhRadToDeg(const Eigen::Vector3d& llh_rad);

#if FPSDK_USE_PROJ || defined(_DOXYGEN_)
/**
 * @brief Coordinate reference system (CRS) types
 *
 * (Like PROJ's PJ_TYPE_GEOCENTRIC_CRS etc., see their docu for details)
 */
enum class CrsType : int
{  // clang-format off
    UNKNOWN       = 0,  //!< Unknown (or not initialised)
    GEOCENTRIC    = 1,  //!< Geocentric (ECEF x/y/z), e.g. EPSG:4978
    GEOGRAPHIC_2D = 2,  //!< Geographic 2D (lat/lon), e.g. EPSG:4326
    GEOGRAPHIC_3D = 3,  //!< Geographic 3D (lat/lon/height), e.g. EPSG:4979
    PROJECTED     = 4,  //!< Projected (east/north), e.g. EPSG:2056
    VERTICAL      = 5,  //!< Vertical (height), e.g. EPSG:5703
    COMPOUND      = 6,  //!< Compound (horizontal + vertical), e.g. EPSG:4258+5703
    OTHER         = 7,  //!< Other type (engineering, temporal, ...)
};  // clang-format on

/**
 * @brief Stringify CRS type
 *
 * @param[in]  type  The CRS type
 *
 * @returns a unique identifier string for the CRS type
 */
const char* CrsTypeStr(const CrsType type);

/**
 * @brief Coordinate reference system (CRS) information
 */
struct CrsInfo
{
    CrsType type_ = CrsType::UNKNOWN;  //!< CRS type
    std::string name_;                 //!< CRS name, e.g. "WGS 84"
    std::vector<CrsType> sub_types_;   //!< Types of the parts of a COMPOUND CRS, e.g. { GEOGRAPHIC_2D, VERTICAL }
};

/**
 * @brief Transformer options
 *
 * The defaults select the most suitable operation PROJ can find, which may be an approximate ("ballpark") one.
 */
struct TransformerOpts
{
    /**
     * @brief Require a precise transformation
     *
     * Do not allow approximate ("ballpark") operations (ALLOW_BALLPARK=NO), fail if the best operation is not
     * available, e.g. because of missing grids (ONLY_BEST=YES), and check that the operation can be instantiated.
     */
    bool precise_ = false;

    /**
     * @brief Minimal accuracy [m] of the operation (ACCURACY=x), 0.0 = any accuracy
     *
     * Operations with unknown accuracy, such as ballpark operations, are not used if this is set.
     */
    double min_accuracy_ = 0.0;

    /**
     * @brief Use "traditional GIS" axis order
     *
     * Use longitude/latitude (and east/north) order for all CRS, regardless of the axis order of the CRS (see
     * proj_normalize_for_visualization()). For example, EPSG:4979 natively uses latitude/longitude order.
     */
    bool normalize_axes_ = false;

    /**
     * @brief Allow network access
     *
     * Allow PROJ to download missing grids from the PROJ content delivery network (see
     * https://proj.org/en/9.4/usage/network.html). This overrides the PROJ configuration (proj.ini, PROJ_NETWORK).
     */
    bool allow_network_ = false;

    /**
     * @brief Area of interest (bounding box)
     */
    struct Area
    {
        double west_ = 0.0;   //!< Western-most longitude [deg]
        double south_ = 0.0;  //!< Southern-most latitude [deg]
        double east_ = 0.0;   //!< Eastern-most longitude [deg]
        double north_ = 0.0;  //!< Northern-most latitude [deg]
    };

    /**
     * @brief Area of interest, used to select the operation
     *
     * If unset PROJ may select a different operation for each coordinate, depending on the coordinate's position.
     */
    std::optional<Area> area_;
};

/**
 * @brief "Universal" coordinate transformer, backed by PROJ
 *
 * All methods are thread-safe. Only one thread at a time can transform, other threads wait.
 *
 * @note This is only available if compiled with PROJ, see @ref FPSDK_BUILD_DEPS.
 */
class Transformer : private types::NoCopyNoMove
{
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  name   Optional name, for debugging
     */
    explicit Transformer(const std::string& name = "");

    ~Transformer();

    /**
     * @brief Initialise transformer
     *
     * The source and target CRS specification can be anything that proj_create() understands (see
     * https://proj.org/en/9.4/development/reference/functions.html#c.proj_create). You may have to change the PROJ
     * configuration (proj.ini) and/or update some PROJ data yourself to support some CRS. See the PROJ documentation
     * for details (see https://proj.org/en/9.4/resource_files.html).
     *
     * @param[in]  source_crs  Source coordinate reference system, e.g. "EPSG:4326"
     * @param[in]  target_crs  Target coordinate reference system, e.g. "EPSG:2056"
     * @param[in]  opts        Options
     *
     * @returns true on success, false otherwise (bad params, missing data, already initialised, no PROJ, ...)
     */
    bool Init(const std::string& source_crs, const std::string& target_crs, const TransformerOpts& opts = {});

    /**
     * @brief Get source CRS information
     *
     * @returns the source CRS information (type UNKNOWN if not initialised)
     */
    CrsInfo SourceCrs() const;

    /**
     * @brief Get target CRS information
     *
     * @returns the target CRS information (type UNKNOWN if not initialised)
     */
    CrsInfo TargetCrs() const;

    /**
     * @brief Transform coordinates
     *
     * @param[in,out]  inout  Coordinates to transform, replaced with result
     * @param[in]      inv    Do the inverse transformation (true), default is forward (false)
     *
     * @returns true on success, false otherwise (not initialised, transformation failed)
     */
    bool Transform(Eigen::Vector3d& inout, const bool inv = false) const;

    /**
     * @brief Transform coordinates
     *
     * @param[in]   in   Coordinates to transform
     * @param[out]  out  Transformed coordinates
     * @param[in]   inv  Do the inverse transformation (true), default is forward (false)
     *
     * @returns true on success, false otherwise (not initialised, transformation failed)
     */
    bool Transform(const Eigen::Vector3d& in, Eigen::Vector3d& out, const bool inv = false) const;

    /**
     * @brief Transform coordinates (with time)
     *
     * @param[in]   in   Coordinates to transform (with time)
     * @param[out]  out  Transformed coordinates (with time)
     * @param[in]   inv  Do the inverse transformation (true), default is forward (false)
     *
     * @returns true on success, false otherwise (not initialised, transformation failed)
     */
    bool Transform(const Eigen::Vector4d& in, Eigen::Vector4d& out, const bool inv = false) const;

   private:
    const std::string name_;    //!< Name, for debugging
    mutable std::mutex mutex_;  //!< One thread at a time uses PROJ
    bool pj_init_;              //!< Proj is initialised
    void* pj_ctx_;              //!< Proj context
    void* pj_tf_;               //!< Proj transformation object
    CrsInfo source_crs_;        //!< Source CRS information
    CrsInfo target_crs_;        //!< Target CRS information

    bool _Transform(const Eigen::Vector4d& in, Eigen::Vector4d& out, const bool inv) const;
};
#endif  // FPSDK_USE_PROJ

/* ****************************************************************************************************************** */
}  // namespace trafo
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_TRAFO_HPP__
