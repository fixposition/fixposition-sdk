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
#include <limits>
#include <thread>
#include <vector>

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

TEST(TrafoTest, TrafoCrsInfo)
{
    {
        Transformer trafo("uninitialised");
        EXPECT_EQ(trafo.SourceCrs().type_, CrsType::UNKNOWN);
        EXPECT_EQ(trafo.TargetCrs().type_, CrsType::UNKNOWN);
    }
    {
        Transformer trafo("ecef_to_llh");
        ASSERT_TRUE(trafo.Init("EPSG:4978", "EPSG:4979"));
        EXPECT_EQ(trafo.SourceCrs().type_, CrsType::GEOCENTRIC);
        EXPECT_EQ(trafo.SourceCrs().name_, "WGS 84");
        EXPECT_TRUE(trafo.SourceCrs().sub_types_.empty());
        EXPECT_EQ(trafo.TargetCrs().type_, CrsType::GEOGRAPHIC_3D);
    }
    {
        Transformer trafo("llh_to_ch1903");
        ASSERT_TRUE(trafo.Init("EPSG:4326", "EPSG:2056"));
        EXPECT_EQ(trafo.SourceCrs().type_, CrsType::GEOGRAPHIC_2D);
        EXPECT_EQ(trafo.TargetCrs().type_, CrsType::PROJECTED);
    }
    {
        // https://epsg.io/4936 ETRS89 x/y/z, https://epsg.io/4258 ETRS89 lat/lon, https://epsg.io/5730 EVRF2000 height
        Transformer trafo("etrs89_to_compound");
        ASSERT_TRUE(trafo.Init("EPSG:4936", "EPSG:4258+5730"));
        EXPECT_EQ(trafo.TargetCrs().type_, CrsType::COMPOUND);
        EXPECT_EQ(trafo.TargetCrs().sub_types_, std::vector<CrsType>({ CrsType::GEOGRAPHIC_2D, CrsType::VERTICAL }));
    }
    EXPECT_STREQ(CrsTypeStr(CrsType::GEOCENTRIC), "GEOCENTRIC");
}

TEST(TrafoTest, TrafoInitFail)
{
    {
        Transformer trafo("bad_crs");
        EXPECT_FALSE(trafo.Init("EPSG:4978", "not a CRS"));
        EXPECT_FALSE(trafo.Init("not a CRS", "EPSG:4979"));
        EXPECT_FALSE(trafo.Init("EPSG:4978", "EPSG:4979+3855"));  // Geographic 3D + vertical is not allowed
    }
    {
        Transformer trafo("init_twice");
        EXPECT_TRUE(trafo.Init("EPSG:4978", "EPSG:4979"));
        EXPECT_FALSE(trafo.Init("EPSG:4978", "EPSG:4979"));
    }
}

TEST(TrafoTest, TrafoOpts)
{
    const Eigen::Vector3d llh_deg = { 47.4002984167, 8.45036571389, 459.410 };
    const Eigen::Vector3d xyz = TfEcefWgs84Llh(LlhDegToRad(llh_deg));
    const Eigen::Vector3d err = { 1e-8, 1e-8, 1e-3 };

    // Native axis order of EPSG:4979 is lat/lon
    {
        Transformer trafo("native");
        ASSERT_TRUE(trafo.Init("EPSG:4978", "EPSG:4979"));
        Eigen::Vector3d p;
        ASSERT_TRUE(trafo.Transform(xyz, p));
        EXPECT_NEAR_EIGEN_VECTOR3D(llh_deg, p, err);
    }
    // Normalized axis order is lon/lat, for EPSG:4979 as well as for OGC:CRS84h (which natively is lon/lat)
    const Eigen::Vector3d lonlath_deg = { llh_deg.y(), llh_deg.x(), llh_deg.z() };
    for (const char* target_crs : { "EPSG:4979", "OGC:CRS84h" }) {
        Transformer trafo("normalized");
        TransformerOpts opts;
        opts.normalize_axes_ = true;
        ASSERT_TRUE(trafo.Init("EPSG:4978", target_crs, opts));
        Eigen::Vector3d p;
        ASSERT_TRUE(trafo.Transform(xyz, p));
        EXPECT_NEAR_EIGEN_VECTOR3D(lonlath_deg, p, err);
        Eigen::Vector3d q;
        ASSERT_TRUE(trafo.Transform(p, q, true));
        EXPECT_NEAR_EIGEN_VECTOR3D(xyz, q, Eigen::Vector3d(1e-3, 1e-3, 1e-3));
    }
    // Precise
    {
        Transformer trafo("precise");
        TransformerOpts opts;
        opts.precise_ = true;
        EXPECT_TRUE(trafo.Init("EPSG:4978", "EPSG:4979", opts));
    }
    // Network (no grids are required, so no network access happens)
    {
        Transformer trafo("network");
        TransformerOpts opts;
        opts.allow_network_ = true;
        EXPECT_TRUE(trafo.Init("EPSG:4978", "EPSG:4979", opts));
    }
    // ETRS89 to WGS84 has no operation with an accuracy better than 1m
    {
        Transformer trafo("min_accuracy_ok");
        TransformerOpts opts;
        opts.min_accuracy_ = 2.0;
        EXPECT_TRUE(trafo.Init("EPSG:4936", "EPSG:4978", opts));
    }
    {
        Transformer trafo("min_accuracy_fail");
        TransformerOpts opts;
        opts.min_accuracy_ = 0.01;
        EXPECT_FALSE(trafo.Init("EPSG:4936", "EPSG:4978", opts));
    }
    // Area of interest
    {
        Transformer trafo("area");
        TransformerOpts opts;
        opts.area_ = TransformerOpts::Area{ 5.9, 45.8, 10.5, 47.8 };  // Switzerland
        ASSERT_TRUE(trafo.Init("EPSG:4326", "EPSG:2056", opts));
        Eigen::Vector3d p;
        ASSERT_TRUE(trafo.Transform(llh_deg, p));
        EXPECT_NEAR_EIGEN_VECTOR3D(
            Eigen::Vector3d(2676373.211, 1250433.955, 459.410), p, Eigen::Vector3d(1e-3, 1e-3, 1e-3));
    }
}

TEST(TrafoTest, TrafoTransformFail)
{
    {
        Transformer trafo("uninitialised");
        Eigen::Vector3d p = { 1.0, 2.0, 3.0 };
        EXPECT_FALSE(trafo.Transform(p));
    }
    {
        Transformer trafo("nan");
        ASSERT_TRUE(trafo.Init("EPSG:4978", "EPSG:4979"));
        const double nan = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector3d p = { nan, 1.0, 1.0 };
        EXPECT_FALSE(trafo.Transform(p));
        Eigen::Vector4d q;
        EXPECT_FALSE(trafo.Transform(Eigen::Vector4d(nan, 1.0, 1.0, 0.0), q));
    }
}

TEST(TrafoTest, TrafoThreads)
{
    const Eigen::Vector3d llh_deg = { 47.4002984167, 8.45036571389, 459.410 };
    const Eigen::Vector3d xyz = TfEcefWgs84Llh(LlhDegToRad(llh_deg));
    const Eigen::Vector3d err = { 1e-8, 1e-8, 1e-3 };

    Transformer trafo_init("threads");
    ASSERT_TRUE(trafo_init.Init("EPSG:4978", "EPSG:4979"));
    const Transformer& trafo = trafo_init;

    // Transform in threads different from the one that initialised the transformer, concurrently
    constexpr int NUM_THREADS = 4;
    constexpr int NUM_TRAFOS = 1000;
    std::vector<int> num_ok(NUM_THREADS, 0);
    std::vector<std::thread> threads;
    for (int t_ix = 0; t_ix < NUM_THREADS; t_ix++) {
        threads.emplace_back([&, t_ix]() {
            for (int ix = 0; ix < NUM_TRAFOS; ix++) {
                Eigen::Vector3d p;
                if (trafo.Transform(xyz, p) && ((llh_deg - p).cwiseAbs().array() <= err.array()).all()) {
                    num_ok[t_ix]++;
                }
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }
    for (int t_ix = 0; t_ix < NUM_THREADS; t_ix++) {
        EXPECT_EQ(num_ok[t_ix], NUM_TRAFOS);
    }
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
