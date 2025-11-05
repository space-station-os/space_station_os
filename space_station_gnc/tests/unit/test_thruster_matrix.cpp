// Copyright 2025 Space Station OS
//
// Unit tests for ThrusterMatrix (URDF + properties.yaml + table.yaml path)
// - English comments
// - Focus on: one-thruster behavior (non-negativity projection),
//             multi-thruster nominal case,
//             zero-W mode error,
//             accepted name prefixes.
//
// NOTE: These tests build tiny URDFs and YAMLs inline and write them to temp files.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>

#include "space_station_gnc/thruster_matrix.hpp"

namespace
{

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------

static std::string writeTextFile(const std::string & path, const std::string & text)
{
  std::ofstream ofs(path);
  ofs << text;
  ofs.close();
  return path;
}

// 1-thruster URDF (child link name uses accepted "thr_" prefix)
static std::string makeURDF_OneThruster()
{
  return
    R"(
<robot name="one_thr">
  <link name="base"/>
  <joint name="j_thr" type="continuous">
    <parent link="base"/>
    <child  link="thr_one"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>
  <link name="thr_one"/>
</robot>
)";
}

// 6-thruster URDF: +/-X, +/-Y, +/-Z (names with "th_" prefix)
static std::string makeURDF_Eagle6()
{
  return
    R"(
<robot name="eagle6">
  <link name="base"/>

  <joint name="j_th_xp" type="continuous">
    <parent link="base"/><child link="th_xp"/>
    <origin xyz=" 1 0 0" rpy="0 0 0"/><axis xyz=" 1 0 0"/>
  </joint><link name="th_xp"/>

  <joint name="j_th_xn" type="continuous">
    <parent link="base"/><child link="th_xn"/>
    <origin xyz="-1 0 0" rpy="0 0 0"/><axis xyz="-1 0 0"/>
  </joint><link name="th_xn"/>

  <joint name="j_th_yp" type="continuous">
    <parent link="base"/><child link="th_yp"/>
    <origin xyz="0  1 0" rpy="0 0 0"/><axis xyz="0  1 0"/>
  </joint><link name="th_yp"/>

  <joint name="j_th_yn" type="continuous">
    <parent link="base"/><child link="th_yn"/>
    <origin xyz="0 -1 0" rpy="0 0 0"/><axis xyz="0 -1 0"/>
  </joint><link name="th_yn"/>

  <joint name="j_th_zp" type="continuous">
    <parent link="base"/><child link="th_zp"/>
    <origin xyz="0 0  1" rpy="0 0 0"/><axis xyz="0 0  1"/>
  </joint><link name="th_zp"/>

  <joint name="j_th_zn" type="continuous">
    <parent link="base"/><child link="th_zn"/>
    <origin xyz="0 0 -1" rpy="0 0 0"/><axis xyz="0 0 -1"/>
  </joint><link name="th_zn"/>
</robot>
)";
}

// URDF with 3 thrusters using different accepted prefixes: th_, thr_, thruster_
static std::string makeURDF_NamePrefixes()
{
  return
    R"(
<robot name="prefixes">
  <link name="base"/>

  <joint name="j1" type="continuous">
    <parent link="base"/><child link="th_a"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
  </joint><link name="th_a"/>

  <joint name="j2" type="continuous">
    <parent link="base"/><child link="thr_b"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint><link name="thr_b"/>

  <joint name="j3" type="continuous">
    <parent link="base"/><child link="thruster_c"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint><link name="thruster_c"/>
</robot>
)";
}

} // namespace


// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

TEST(ThrusterMatrixTest, OneThruster_NonNegativeProjection_Works)
{
  // URDF with 1 thruster oriented +X
  ThrusterMatrix tm;
  printf("Testing   tm.initialize(makeURDF_OneThruster()) with one thruster...\n");
  tm.initialize(makeURDF_OneThruster());
  printf("Fisish testing tm.initialize(makeURDF_OneThruster()) with one thruster...\n");
  ASSERT_TRUE(tm.isReady());
  ASSERT_EQ(tm.getNumThr(), static_cast<std::size_t>(1));

  // properties.yaml (optional; keep defaults)
  const std::string props =
    R"(
thrusters:
  thr_one: { max_force: 50.0, isp: 300.0, efficiency: 0.7 }
)";
  printf("Testing ThrusterMatrix with one thruster...\n");
  tm.loadProperties(writeTextFile("one_thr_props.yaml", props));

  // table.yaml: choose torque around X (Tx) only, force all zero
  // This is artificial: in practice a single col cannot span 6D, but we
  // rely on LS + non-negativity projection behavior (u >= 0).
  const std::string table_yaml =
    R"(
tables:
  mode_tx_only:
    thr_one:
      torque: [1, 0, 0]
      force:  [0, 0, 0]
)";
  printf("Testing ThrusterMatrix with one thruster and table...\n");
  tm.loadTable(writeTextFile("one_thr_table.yaml", table_yaml));
  printf("Setting thruster table to 'mode_tx_only'...\n");
  tm.setThrusterTable("mode_tx_only");

  // Case 1: desired Tx positive -> u should be non-negative
  {
    Eigen::VectorXd wrench(6); wrench << 1, 0, 0, 0, 0, 0; // [Tx Ty Tz Fx Fy Fz]
    Eigen::VectorXd u;
    tm.generateCommandFromTable(wrench, u);
    ASSERT_EQ(u.size(), 1);
    EXPECT_GE(u(0), 0.0);
    EXPECT_TRUE(u.allFinite());
  }

  // Case 2: desired Tx negative -> unconstrained LS may be negative
  //         projection should clamp to zero since u>=0 only
  {
    Eigen::VectorXd wrench(6); wrench << -1, 0, 0, 0, 0, 0;
    Eigen::VectorXd u;
    tm.generateCommandFromTable(wrench, u);
    ASSERT_EQ(u.size(), 1);
    EXPECT_NEAR(u(0), 0.0, 1e-12);
  }
}

TEST(ThrusterMatrixTest, Eagle6_Nominal_CommandIsFiniteAndNonNegative)
{
  ThrusterMatrix tm;
  tm.initialize(makeURDF_Eagle6());
  ASSERT_TRUE(tm.isReady());
  ASSERT_EQ(tm.getNumThr(), static_cast<std::size_t>(6));

  // Optional properties (override some)
  const std::string props =
    R"(
thrusters:
  th_xp: { max_force: 100.0 }
  th_xn: { max_force: 100.0 }
  th_yp: { max_force: 100.0 }
  th_yn: { max_force: 100.0 }
  th_zp: { max_force: 100.0 }
  th_zn: { max_force: 100.0 }
)";
  tm.loadProperties(writeTextFile("eagle6_props.yaml", props));

  // table: simple weights that allow contributing along each axis.
  // (Not a physically exact model; enough to exercise LS + projection.)
  const std::string table_yaml =
    R"(
tables:
  nominal:
    th_xp: { torque: [0,0,0], force: [ 1, 0, 0] }
    th_xn: { torque: [0,0,0], force: [-1, 0, 0] }
    th_yp: { torque: [0,0,0], force: [ 0, 1, 0] }
    th_yn: { torque: [0,0,0], force: [ 0,-1, 0] }
    th_zp: { torque: [0,0,0], force: [ 0, 0, 1] }
    th_zn: { torque: [0,0,0], force: [ 0, 0,-1] }
)";
  tm.loadTable(writeTextFile("eagle6_table.yaml", table_yaml));
  tm.setThrusterTable("nominal");

  // Few test wrenches
  const std::vector<Eigen::VectorXd> wrenches = {
    (Eigen::VectorXd(6) << 0, 0, 0, 5, 0, 0).finished(),  // +Fx
    (Eigen::VectorXd(6) << 0, 0, 0, 0, 5, 0).finished(),  // +Fy
    (Eigen::VectorXd(6) << 0, 0, 0, 0, 0, 5).finished(),  // +Fz
    (Eigen::VectorXd(6) << 0, 0, 0, 3, 4, 5).finished()   // mixed forces
  };

  for (const auto & w : wrenches) {
    Eigen::VectorXd u;
    tm.generateCommandFromTable(w, u);
    ASSERT_EQ(u.size(), 6);
    EXPECT_TRUE(u.allFinite());
    for (int i = 0; i < u.size(); ++i) {
      EXPECT_GE(u(i), -1e-12); // numerical tolerance, should be clamped to >=0
    }
  }
}

TEST(ThrusterMatrixTest, ZeroWMode_ShouldThrow)
{
  // URDF with 1 thruster but table weights all zero -> W is all-zero, must throw
  ThrusterMatrix tm;
  tm.initialize(makeURDF_OneThruster());
  ASSERT_TRUE(tm.isReady());
  ASSERT_EQ(tm.getNumThr(), static_cast<std::size_t>(1));

  const std::string table_yaml =
    R"(
tables:
  zero_mode:
    thr_one:
      torque: [0, 0, 0]
      force:  [0, 0, 0]
)";
  tm.loadTable(writeTextFile("zeroW_table.yaml", table_yaml));
  tm.setThrusterTable("zero_mode");

  Eigen::VectorXd wrench(6); wrench.setZero();
  Eigen::VectorXd u;
  EXPECT_THROW(tm.generateCommandFromTable(wrench, u), std::runtime_error);
}

TEST(ThrusterMatrixTest, NamePrefixes_AreRecognized)
{
  // Ensure that th_ / thr_ / thruster_ are all recognized
  ThrusterMatrix tm;
  tm.initialize(makeURDF_NamePrefixes());
  ASSERT_TRUE(tm.isReady());

  // We only care the count >= 3 (three joints/links meet the rule)
  EXPECT_GE(tm.getNumThr(), static_cast<std::size_t>(3));

  // Minimal table enabling all three (with simple force weights) so command works
  const std::string table_yaml =
    R"(
tables:
  all_on:
    th_a:        { torque: [0,0,0], force: [1,0,0] }
    thr_b:       { torque: [0,0,0], force: [0,1,0] }
    thruster_c:  { torque: [0,0,0], force: [0,0,1] }
)";
  tm.loadTable(writeTextFile("prefix_table.yaml", table_yaml));
  tm.setThrusterTable("all_on");

  Eigen::VectorXd wrench(6); wrench << 0, 0, 0, 1, 1, 1;
  Eigen::VectorXd u;
  tm.generateCommandFromTable(wrench, u);
  ASSERT_EQ(u.size(), tm.getNumThr());
  EXPECT_TRUE(u.allFinite());
  for (int i = 0; i < u.size(); ++i) {
    EXPECT_GE(u(i), -1e-12);
  }
}
