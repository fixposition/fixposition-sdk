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
 * @brief Fixposition SDK: tests for fpsdk::common::trafo
 */

/* LIBC/STL */
#include <cmath>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/math.hpp>
#include <fpsdk_common/trafo.hpp>
#include <fpsdk_common/yaml.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::trafo;

static const char* TEST_DATA_YAML = R"yaml(
LLH_ECEF:
  - { LLH: [47.3, 8.5, 420.0], ECEF: [4285920.9089, 640535.1715, 4664755.5879] }
  - { LLH: [1.0, 1.0, 1.0], ECEF: [6376201.8059, 111297.0165, 110568.7923] }
  - { LLH: [-1.0, -1.0, -1.0], ECEF: [6376199.8065, -111296.9816, -110568.7574] }
  - { LLH: [45.0, 45.0, 0], ECEF: [3194419.1451, 3194419.1451, 4487348.4088] }
  - { LLH: [45.0, -135.0, 0], ECEF: [-3194419.1451, -3194419.1451, 4487348.4088] }
  - { LLH: [-25.1, 15.0, 7100.0], ECEF: [5588608.632, 1497463.170, -2692121.689] } # From https://tool-online.com/en/coordinate-converter.php
  - { LLH: [0.0, 0.0, 0.0], ECEF: [6378137.0, 0.0, 0.0] }

ENU_ECEF:
  - {
      LLH: [0.0, 0.0, 0.0],
      ECEF: [6.3762018059274e+6, 1.11297017e+5, 1.1056879228e+5],
      ENU: [1.112970165178818e+5, 1.105687922769731e+5, -1.935194072552025e+03],
    }
  - {
      LLH: [47.0, 8.0, 0.0],
      ECEF: [4278996.8972, 642733.9702, 4670938.7017],
      ENU: [4.095766020273372e+04, 4.248254463537285e+04, 2.746784580117637e+02],
    }

NED_ECEF:
  - {
      LLH: [0.0, 0.0, 0.0],
      ECEF: [6.3762018059274e+6, 1.11297017e+5, 1.1056879228e+5],
      NED: [1.105687922769731e+5, 1.112970165178818e+5, 1.935194072552025e+03],
    }
  - {
      LLH: [47.0, 8.0, 0.0],
      ECEF: [4278996.8972, 642733.9702, 4670938.7017],
      NED: [4.248254463537285e+04, 4.095766020273372e+04, -2.746784580117637e+02],
    }

ROTATIONS:
  - { ROTMAT: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], Q: [1.0, 0.0, 0.0, 0.0], EUL: [0.0, 0.0, 0.0] }
  - { ROTMAT: [-1, 0, 0, 0, -1, 0, 0, 0, 1], Q: [0, 0, 0, 1], EUL: [3.1415927, 0, 0] }
  - {
      ROTMAT: [0.0, -0.9848077, 0.1736482, 0.7071068, -0.1227878, -0.6963642, 0.7071068, 0.1227878, 0.6963642],
      Q: [0.6272114, 0.3265056, -0.2126311, 0.6743797],
      EUL: [1.5707963, -0.7853981, 0.1745329],
    }
)yaml";

YAML::Node TEST_DATA;

static bool LoadTestData()
{
    return (TEST_DATA.size() > 0 ? true : fpsdk::common::yaml::StringToYaml(TEST_DATA_YAML, TEST_DATA));
}

#define EXPECT_NEAR_EIGEN_VECTOR3D(v1, v2, e)   \
    do {                                        \
        EXPECT_NEAR((v1).x(), (v2).x(), e.x()); \
        EXPECT_NEAR((v1).y(), (v2).y(), e.y()); \
        EXPECT_NEAR((v1).z(), (v2).z(), e.z()); \
    } while (0)

static const Eigen::Vector3d ECEF_ERR = { 5e-4, 5e-4, 5e-4 };
static const Eigen::Vector3d LLH_ERR = { 1e-8, 1e-8, 5e-4 };

#define DEBUG_EIGEN_VECTOR3D(n, v) DEBUG("%s %.10g %.10g %.10g", n, (v).x(), (v).y(), (v).z())

// ---------------------------------------------------------------------------------------------------------------------

TEST(TrafoTest, LlhEcef)
{
    EXPECT_TRUE(LoadTestData());
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;
    for (auto elem : TEST_DATA["LLH_ECEF"]) {
        Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
        Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
        llh_vec_.push_back(llh);
        ecef_vec_.push_back(ecef);
    }
    const std::size_t num_tests = llh_vec_.size();
    EXPECT_GT(num_tests, 0);

    for (std::size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resllh = TfWgs84LlhEcef(ecef);
        const Eigen::Vector3d resecef = TfEcefWgs84Llh(llh);

        DEBUG_EIGEN_VECTOR3D("Res LLH ", LlhRadToDeg(resllh));
        DEBUG_EIGEN_VECTOR3D("Res ECEF", resecef);
        EXPECT_NEAR_EIGEN_VECTOR3D(llh, resllh, LLH_ERR);
        EXPECT_NEAR_EIGEN_VECTOR3D(ecef, resecef, ECEF_ERR);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

class EnuEcefTest : public ::testing::Test
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST(TrafoTest, EnuEcef)
{
    EXPECT_TRUE(LoadTestData());
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> enu_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;
    for (auto elem : TEST_DATA["ENU_ECEF"]) {
        Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
        Eigen::Vector3d enu(elem["ENU"].as<std::vector<double>>().data());
        Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
        llh_vec_.push_back(llh);
        enu_vec_.push_back(enu);
        ecef_vec_.push_back(ecef);
    }
    const std::size_t num_tests = llh_vec_.size();
    EXPECT_GT(num_tests, 0);

    for (std::size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d enu = enu_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resenu = TfEnuEcef(ecef, llh);

        DEBUG_EIGEN_VECTOR3D("Res ENU ", resenu);
        EXPECT_NEAR_EIGEN_VECTOR3D(enu, resenu, ECEF_ERR);
    }

    for (std::size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d enu = enu_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resecef = TfEcefEnu(enu, llh);

        DEBUG_EIGEN_VECTOR3D("Res ECEF", resecef);
        EXPECT_NEAR_EIGEN_VECTOR3D(ecef, resecef, ECEF_ERR);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

class NedEcefTest : public ::testing::Test
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST(TrafoTest, NedEcef)
{
    EXPECT_TRUE(LoadTestData());
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> ned_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;
    for (auto elem : TEST_DATA["NED_ECEF"]) {
        Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
        Eigen::Vector3d ned(elem["NED"].as<std::vector<double>>().data());
        Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
        llh_vec_.push_back(llh);
        ned_vec_.push_back(ned);
        ecef_vec_.push_back(ecef);
    }
    const std::size_t num_tests = llh_vec_.size();
    EXPECT_GT(num_tests, 0);

    for (std::size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d ned = ned_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resned = TfNedEcef(ecef, llh);

        DEBUG_EIGEN_VECTOR3D("Res NED ", LlhRadToDeg(resned));
        EXPECT_NEAR_EIGEN_VECTOR3D(ned, resned, ECEF_ERR);
    }

    for (std::size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d ned = ned_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resecef = TfEcefNed(ned, llh);

        DEBUG_EIGEN_VECTOR3D("Res ECEF", resecef);
        EXPECT_NEAR_EIGEN_VECTOR3D(ecef, resecef, ECEF_ERR);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

class RotationConversionTest : public ::testing::Test
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST(TrafoTest, QuatEul)
{
    EXPECT_TRUE(LoadTestData());
    std::vector<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotmat_vec_;
    std::vector<Eigen::Vector4d> q_vec_;
    std::vector<Eigen::Vector3d> eul_vec_;
    std::vector<Eigen::Vector3d> eul_deg_vec_;
    for (auto elem : TEST_DATA["ROTATIONS"]) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotmat(elem["ROTMAT"].as<std::vector<double>>().data());
        Eigen::Vector4d q(elem["Q"].as<std::vector<double>>().data());
        Eigen::Vector3d eul(elem["EUL"].as<std::vector<double>>().data());
        rotmat_vec_.push_back(rotmat);
        q_vec_.push_back(q);
        eul_vec_.push_back(eul);
    }
    const std::size_t num_tests = rotmat_vec_.size();
    EXPECT_GT(num_tests, 0);

    for (std::size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector4d q = q_vec_.at(i);
        const Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
        const Eigen::Vector3d eul = eul_vec_.at(i);

        const Eigen::Vector3d eul_test = QuatToEul(quat);

        DEBUG_EIGEN_VECTOR3D("Eul     ", eul);
        DEBUG_EIGEN_VECTOR3D("Eul_Test", eul_test);

        // compare quat, might be 1 0 0 0 or -1 0 0 0
        EXPECT_LE((eul_test - eul).norm(), 1e-5);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
#if FPSDK_USE_PROJ

TEST(TrafoTest, Trafo)
{
    const Eigen::Vector3d p1_wgs84_llh = { 47.4002984167, 8.45036571389, 459.410 };
    const Eigen::Vector3d p1_wgs84_xyz = { 4278387.47, 635620.71, 4672340.04 };
    const Eigen::Vector3d p1_ch1903_enu = { 2676373.211, 1250433.955,
        459.410 };  // ell height!, orthometric height should be ~412
    const Eigen::Vector3d p1_etrs89_xyz = { 4278387.47, 635620.71,
        4672340.04 };  // same as wgs84, at this position and accuracy
    const Eigen::Vector3d err = { 1e-3, 1e-3, 1e-3 };
    DEBUG_EIGEN_VECTOR3D("p1_wgs84_llh", p1_wgs84_llh);
    DEBUG_EIGEN_VECTOR3D("p1_wgs84_xyz", p1_wgs84_xyz);
    DEBUG_EIGEN_VECTOR3D("p1_ch1903_enu", p1_ch1903_enu);
    DEBUG_EIGEN_VECTOR3D("p1_etrs89_xyz", p1_etrs89_xyz);
    DEBUG_EIGEN_VECTOR3D("err", err);

    // https://epsg.io/4326 WGS 84 lat/lon/height
    // https://epsg.io/2056 Swiss CH1903+ / LV95 east/north/up
    {
        Transformer trafo("wgs84_llh_to_ch1903_enu");
        EXPECT_TRUE(trafo.Init("EPSG:4326", "EPSG:2056"));

        Eigen::Vector3d p;
        EXPECT_TRUE(trafo.Transform(p1_wgs84_llh, p));
        EXPECT_NEAR_EIGEN_VECTOR3D(p1_ch1903_enu, p, err);
        p = p1_wgs84_llh;
        EXPECT_TRUE(trafo.Transform(p));
        EXPECT_NEAR_EIGEN_VECTOR3D(p1_ch1903_enu, p, err);
        DEBUG_EIGEN_VECTOR3D("p1_wgs84_llh -> ch1903_enu", p);
    }

    // https://epsg.io/4326 WGS 84 lat/lon/height
    // https://epsg.io/4978 WGS 84 ECEF x/y/z
    {
        Transformer trafo("wgs84_llh_to_wgs84_xyz");
        EXPECT_TRUE(trafo.Init("EPSG:4326", "EPSG:4978"));

        Eigen::Vector3d p;
        EXPECT_TRUE(trafo.Transform(p1_wgs84_llh, p));
        EXPECT_NEAR_EIGEN_VECTOR3D(p1_wgs84_xyz, p, err);
        DEBUG_EIGEN_VECTOR3D("p1_wgs84_llh -> wgs84_xyz", p);
    }

    // https://epsg.io/4326 WGS 84 lat/lon/height
    // https://epsg.io/4936 ETRS89 x/y/z
    {
        Transformer trafo("wgs84_llh_to_etrs89_xyz");
        EXPECT_TRUE(trafo.Init("EPSG:4326", "EPSG:4936"));

        Eigen::Vector3d p;
        EXPECT_TRUE(trafo.Transform(p1_wgs84_llh, p));
        EXPECT_NEAR_EIGEN_VECTOR3D(p1_etrs89_xyz, p, err);
        DEBUG_EIGEN_VECTOR3D("p1_wgs84_llh -> etrs89_xyz", p);
    }

    // TODO: more tests, trafo with time, etc.
    // https://epsg.io/4936 ETRS89 ECEF xyz
    // https://epsg.io/4978 WGS 84 ECEF xyz
    // https://epsg.io/7789 ITRF2014 xyz
    // https://epsg.io/9988 ITRF2020 xyz
    // https://epsg.io/8401 ETRF2014 xyz
    // https://epsg.io/7930 ETRF2000 xyz
    // https://epsg.io/7922 ETRF93 xyz
}

#  if 0
#    define INFO_VV(n, v1, v2)                                                                                  \
        INFO("%-20s v1: %.3f %.3f %.3f  v2: %.3f %.3f %.3f  d=%.3f", n, (v1).x(), (v1).y(), (v1).z(), (v2).x(), \
            (v2).y(), (v2).z(), (v1 - v2).norm())
#    define INFO_V(n, v1) INFO("%-20s v1: %.3f %.3f %.3f  %.3f", n, (v1).x(), (v1).y(), (v1).z(), (v1).norm());

#    define INFO_V4V4(n, v1, v2)                                                                                  \
        INFO("%-20s  v1: %.3f %.3f %.3f (%.1f)  v2: %.3f %.3f %.3f (%.1f)  d=%.3f (%.1f)", n, (v1).x(), (v1).y(), \
            (v1).z(), (v1).w(), (v2).x(), (v2).y(), (v2).z(), (v2).w(), (v1 - v2).head<3>().norm(),               \
            (v2).w() - (v1).w())

#    define INFO_V4(n, v1) \
        INFO("%-20s v1: %.3f %.3f %.3f  %.3f (%.1f)", n, (v1).x(), (v1).y(), (v1).z(), (v1).head<3>().norm(), (v1).w());
#  endif

#endif  // FPSDK_USE_PROJ

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
