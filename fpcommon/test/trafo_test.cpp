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
 * @brief Fixposition SDK: tests for fp::common::trafo
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpcommon/logging.hpp>
#include <fpcommon/math.hpp>
#include <fpcommon/trafo.hpp>
#include <fpcommon/yaml.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fp::common::trafo;

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

TEST(TrafoTest, LoadYaml)
{
    EXPECT_TRUE(fp::common::yaml::StringToYaml(TEST_DATA_YAML, TEST_DATA));
}

// ---------------------------------------------------------------------------------------------------------------------

static constexpr double ECEF_ERR = 5e-4;
static constexpr double RAD_ERR = 1e-8;

static void CompareEigenVec(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2, const double err = ECEF_ERR)
{
    EXPECT_EQ(vec1.size(), vec2.size());
    for (int ix = 0; ix < vec1.size(); ix++) {
        EXPECT_NEAR(vec1[ix], vec2[ix], err);
    }
}

static void CompareLlh(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2)
{
    EXPECT_EQ(vec1.size(), vec2.size());
    EXPECT_NEAR(vec1.x(), vec2.x(), RAD_ERR);
    EXPECT_NEAR(vec1.y(), vec2.y(), RAD_ERR);
    EXPECT_NEAR(vec1.z(), vec2.z(), ECEF_ERR);
}

// ---------------------------------------------------------------------------------------------------------------------

class LlhEcefTest : public ::testing::Test
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;

    virtual void SetUp() final
    {
        for (auto elem : TEST_DATA["LLH_ECEF"]) {
            Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
            Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
            llh_vec_.push_back(llh);
            ecef_vec_.push_back(ecef);
        }
    }
};

TEST_F(LlhEcefTest, LlhEcef)
{
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resllh = TfWgs84LlhEcef(ecef);
        const Eigen::Vector3d resecef = TfEcefWgs84Llh(llh);

        DEBUG_S("Res LLH  " << LlhRadToDeg(resllh).transpose().format(Eigen::IOFormat(15)));
        DEBUG_S("Res ECEF " << resecef.transpose().format(Eigen::IOFormat(15)));
        CompareLlh(llh, resllh);
        CompareEigenVec(ecef, resecef);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

class EnuEcefTest : public ::testing::Test
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> enu_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;

    virtual void SetUp() final
    {
        YAML::Node ENU_ECEF = TEST_DATA["ENU_ECEF"];
        for (auto elem : ENU_ECEF) {
            Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
            Eigen::Vector3d enu(elem["ENU"].as<std::vector<double>>().data());
            Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
            llh_vec_.push_back(llh);
            enu_vec_.push_back(enu);
            ecef_vec_.push_back(ecef);
        }
    }
};

TEST_F(EnuEcefTest, TfEnuEcef)
{
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d enu = enu_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resenu = TfEnuEcef(ecef, llh);

        DEBUG_S("Res ENU  " << resenu.transpose().format(Eigen::IOFormat(15)));
        CompareEigenVec(enu, resenu);
    }
}

TEST_F(EnuEcefTest, TfEcefEnu)
{
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d enu = enu_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resecef = TfEcefEnu(enu, llh);

        DEBUG_S("Res ECEF  " << resecef.transpose().format(Eigen::IOFormat(15)));
        CompareEigenVec(ecef, resecef);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

class NedEcefTest : public ::testing::Test
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> ned_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;

    virtual void SetUp() override
    {
        YAML::Node NED_ECEF = TEST_DATA["NED_ECEF"];
        for (auto elem : NED_ECEF) {
            Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
            Eigen::Vector3d ned(elem["NED"].as<std::vector<double>>().data());
            Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
            llh_vec_.push_back(llh);
            ned_vec_.push_back(ned);
            ecef_vec_.push_back(ecef);
        }
    }
};

TEST_F(NedEcefTest, TestNedEcef)
{
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d ned = ned_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resned = TfNedEcef(ecef, llh);

        DEBUG_S("Res NED  " << LlhRadToDeg(resned).transpose().format(Eigen::IOFormat(15)));
        CompareEigenVec(ned, resned);
    }
}

TEST_F(NedEcefTest, TfEcefNed)
{
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d ned = ned_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resecef = TfEcefNed(ned, llh);

        DEBUG_S("Res ECEF  " << resecef.transpose().format(Eigen::IOFormat(15)));
        CompareEigenVec(ecef, resecef);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

class RotationConversionTest : public ::testing::Test
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotmat_vec_;
    std::vector<Eigen::Vector4d> q_vec_;
    std::vector<Eigen::Vector3d> eul_vec_;
    std::vector<Eigen::Vector3d> eul_deg_vec_;

    virtual void SetUp() override
    {
        YAML::Node ROTATIONS = TEST_DATA["ROTATIONS"];
        for (auto elem : ROTATIONS) {
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotmat(elem["ROTMAT"].as<std::vector<double>>().data());
            Eigen::Vector4d q(elem["Q"].as<std::vector<double>>().data());
            Eigen::Vector3d eul(elem["EUL"].as<std::vector<double>>().data());
            rotmat_vec_.push_back(rotmat);
            q_vec_.push_back(q);
            eul_vec_.push_back(eul);
        }
    }
};

TEST_F(RotationConversionTest, TestQuatEul)
{
    const size_t num_tests = rotmat_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector4d q = q_vec_.at(i);
        const Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
        const Eigen::Vector3d eul = eul_vec_.at(i);

        const Eigen::Vector3d eul_test = QuatToEul(quat);

        DEBUG_S("Eul " << eul.transpose() << " Eul_Test " << eul_test.transpose());

        // compare quat, might be 1 0 0 0 or -1 0 0 0
        EXPECT_LE((eul_test - eul).norm(), 1e-5);
    }
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fp::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fp::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
