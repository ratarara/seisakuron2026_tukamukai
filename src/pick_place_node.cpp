// Copyright 2022 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Based on crane_x7_examples/src/pick_and_place.cpp
// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tukamukai/board_geometry.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// 置く位置の補正の設定（correct_position）
struct CompConfig
{
  bool enabled = true;
  double tol = 0.001;        // これより小さいずれは補正しない [m]
  int max_iter = 2;          // 動き直す回数の上限
  double max_total = 0.010;  // 指令をずらす量の上限 [m]。超えたら補正しない
  bool use_z = false;        // 高さも補正するか（つかむ高さは実機で決めた指令値なので、初期値は補正しない）
  // 試験用：測ったずれに、横（y）にこの量を足す [m]。Mock は腕が指令どおりに止まり、
  // ずれが 0 なので、補正の動きを確かめるときに使う。実機では 0 のままにする
  double fake_error_y = 0.0;
};

// 指令のずらし量 [m]
struct Offset
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

class PickAndPlace
{
public:
  // グリッパを閉じきったときの角度
  inline static const double GRIPPER_CLOSE = 0.0;

  // 関節の名前
  inline static const char * const kPanJoint = "crane_x7_shoulder_fixed_part_pan_joint";  // 根本の回転
  inline static const char * const kTwistJoint =
    "crane_x7_upper_arm_revolute_part_twist_joint";  // 上腕のねじり
  inline static const char * const kElbowJoint =
    "crane_x7_upper_arm_revolute_part_rotate_joint";  // 肘
  inline static const char * const kForearmTwistJoint =
    "crane_x7_lower_arm_fixed_part_joint";  // 前腕のねじり

  // ノードを受け取り、アーム・グリッパのMoveGroupInterfaceを初期化する
  explicit PickAndPlace(rclcpp::Node::SharedPtr node)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(node, "gripper");
  }

  // グリッパの開く角度・つかむ角度 [deg] を設定する。取っ手の大きさに合わせて実機で調整する
  void set_gripper_angles(const double open_deg, const double grasp_deg)
  {
    gripper_open_ = angles::from_degrees(open_deg);
    gripper_grasp_ = angles::from_degrees(grasp_deg);
  }

  // グリッパを開く（離す）。成功したら true
  bool open_gripper()
  {
    return move_gripper_angle(gripper_open_);
  }

  // グリッパでつかむ。成功したら true
  bool grasp_gripper()
  {
    return move_gripper_angle(gripper_grasp_);
  }

  // ステップ実行モードの切り替え。true のとき、動作の前に毎回 Enter で確認する
  void set_step_by_step(const bool enable)
  {
    step_by_step_ = enable;
  }

  // ステップ実行で q が押された（または入力が閉じた）ら true。以後、動作は実行されない
  bool aborted() const
  {
    return aborted_;
  }

  // 速度・加速度の倍率を設定する（0.0 ~ 1.0）。実機では小さい値から始める
  void set_speed_scale(const double scale)
  {
    move_group_arm_->setMaxVelocityScalingFactor(scale);
    move_group_arm_->setMaxAccelerationScalingFactor(scale);
  }

  // アームを目標位置・姿勢（Pose）に動かす。成功したら true
  // （ステップ実行の確認は、これを呼ぶ control_arm 側で行う）
  bool move_arm_to_pose(const geometry_msgs::msg::Pose & pose)
  {
    move_group_arm_->setPoseTarget(pose);
    return static_cast<bool>(move_group_arm_->move());
  }

  // アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす。成功したら true
  bool control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    char text[160];
    std::snprintf(
      text, sizeof(text),
      "Move hand to x=%.3f y=%.3f z=%.3f [m]  roll=%.0f pitch=%.0f yaw=%.0f [deg]",
      x, y, z, roll, pitch, yaw);
    if (!confirm_step(text)) {return false;}

    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);
    return move_arm_to_pose(target_pose);
  }

  // SRDFに定義された姿勢名でアームを動かす。成功したら true
  bool move_arm_to_named_pose(const std::string & name)
  {
    if (!confirm_step("Move arm to named pose '" + name + "'")) {return false;}
    move_group_arm_->setNamedTarget(name);
    return static_cast<bool>(move_group_arm_->move());
  }

  // グリッパを角度[rad]を指定して開閉する。成功したら true
  bool move_gripper_angle(const double angle)
  {
    char text[160];
    std::snprintf(
      text, sizeof(text), "Move gripper to %.0f [deg]  (open=%.0f, grasp=%.0f, close=0)",
      angles::to_degrees(angle), angles::to_degrees(gripper_open_),
      angles::to_degrees(gripper_grasp_));
    if (!confirm_step(text)) {return false;}
    return move_gripper_angle_now(angle);
  }

  // 今のグリッパの角度 [rad]（実際の指の角度。命令した角度とはずれることがある）
  // （角度を読めなかったときは NaN）
  double current_gripper_angle()
  {
    const auto values = move_group_gripper_->getCurrentJointValues();
    return values.empty() ? std::nan("") : values[0];
  }

  // 関節の角度を受け取れるまで待つ（最大 timeout_sec 秒）。受け取れたら true。
  // 起動した直後は、MoveIt がまだ関節の角度を受け取っていないことがあり、
  // そのまま動かそうとすると、空のデータを読んでプログラムが落ちる
  bool wait_for_robot_state(const double timeout_sec)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start <
      std::chrono::duration<double>(timeout_sec))
    {
      if (move_group_arm_->getCurrentState(1.0) &&
        !move_group_gripper_->getCurrentJointValues().empty())
      {
        return true;
      }
    }
    return false;
  }

  // グリッパを角度[rad]に動かす（ステップ実行の確認をしない）。成功したら true
  bool move_gripper_angle_now(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    if (joint_values.empty()) {return false;}  // 今の角度を読めなかった
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    if (!static_cast<bool>(move_group_gripper_->move())) {return false;}
    wait_gripper_settled(angle);
    return true;
  }

  // グリッパの指が止まるまで待つ。
  // グリッパの制御は、目標との差が 0.1 rad（約 5.7°）以内なら、指が動いている途中でも
  // 完了を返す。すぐ次の動作を計画すると、出発点が実際の指の位置とずれて中止されるので、
  // 「目標に着いた」か「動かなくなった（取っ手をつかんだなど）」まで待つ（最大 3 秒）
  void wait_gripper_settled(const double target)
  {
    const auto start = std::chrono::steady_clock::now();
    double prev = current_gripper_angle();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      const double now = current_gripper_angle();
      // 動き始める前を「止まった」と誤判定しないよう、最初の 0.3 秒は着いたかだけを見る
      const bool started = std::chrono::steady_clock::now() - start >
        std::chrono::milliseconds(300);
      if (std::abs(now - target) < angles::from_degrees(0.5) ||
        (started && std::abs(now - prev) < angles::from_degrees(0.1)))
      {
        return;
      }
      prev = now;
    }
  }

  // アームの関節の一部に可動制限を設定する
  void set_constraints()
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "crane_x7_lower_arm_fixed_part_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(30);
    joint_constraint.tolerance_below = angles::from_degrees(30);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(30);
    joint_constraint.tolerance_below = angles::from_degrees(30);
    joint_constraint.weight = 0.8;
    constraints.joint_constraints.push_back(joint_constraint);

    // 肘の角度の範囲（set_elbow_limit で指定したときだけ）
    if (elbow_limit_enabled_) {
      joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_rotate_joint";
      joint_constraint.position = (elbow_min_ + elbow_max_) / 2.0;
      joint_constraint.tolerance_above = (elbow_max_ - elbow_min_) / 2.0;
      joint_constraint.tolerance_below = (elbow_max_ - elbow_min_) / 2.0;
      joint_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(joint_constraint);
    }

    path_constraints_ = constraints;
    move_group_arm_->setPathConstraints(constraints);
  }

  // 今の手先（crane_x7_gripper_base_link）の高さ [m]
  double current_hand_z()
  {
    return move_group_arm_->getCurrentPose().pose.position.z;
  }

  // 手先を waypoints の順にまっすぐ（直線でつないで）、speed [m/s] の一定の速さで動かす。
  // 小さな動作を何回も計画すると、7 軸のアームは途中で腕の形を変えることがあるので、
  // 直線の経路（Cartesian path）を一度に計算する。経路にも関節の可動制限（肘など）をかける。
  // 実機の腕は重さで指令より少し下がるので、waypoints には「測った位置」ではなく
  // 「本来あるべき位置と向き」を渡す（測った値から作ると、ずれが動くたびにたまる）。
  // 実行中に *stop が true になったら、その場で止める（stopped_by_user = true）。
  // stop が nullptr のときは止めない。
  // allow_partial が false のとき、経路を最後まで計算できなければ動かさずに false を返す
  bool move_hand_cartesian(
    const std::vector<geometry_msgs::msg::Pose> & waypoints, const double speed,
    const std::shared_ptr<std::atomic<bool>> & stop, const rclcpp::Logger & logger,
    bool & stopped_by_user, const std::string & what, const bool allow_partial = false,
    const bool log_error = true)
  {
    stopped_by_user = false;
    if (waypoints.empty() || speed <= 0.0) {
      RCLCPP_ERROR(logger, "[cartesian] invalid waypoints or speed=%.3f", speed);
      return false;
    }
    if (!confirm_step(what)) {return false;}

    // 経路の長さ（今の手の位置から、waypoints を順にたどった距離）
    geometry_msgs::msg::Point prev = move_group_arm_->getCurrentPose().pose.position;
    double length = 0.0;
    for (const auto & w : waypoints) {
      length += std::hypot(
        w.position.x - prev.x, w.position.y - prev.y, w.position.z - prev.z);
      prev = w.position;
    }
    // もう着いている（0.5 mm 未満）ときは動かない。短すぎる経路は計算できないため
    if (length < 0.0005) {return true;}

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.001;  // 1 mm ごとに経路を計算する
    const double fraction = move_group_arm_->computeCartesianPath(
      waypoints, eef_step, trajectory, path_constraints_);
    auto & points = trajectory.joint_trajectory.points;
    if (fraction <= 0.0 || points.size() < 2) {
      RCLCPP_ERROR(logger, "[cartesian] %s: could not compute a straight path", what.c_str());
      return false;
    }
    if (fraction < 0.999) {
      RCLCPP_WARN(
        logger, "[cartesian] %s: can follow only %.0f %% of the path (joint or elbow limit)",
        what.c_str(), fraction * 100.0);
      // 最後に計算できた点の関節の角度（どの関節が制限に当たったかを見るため）
      const auto & names = trajectory.joint_trajectory.joint_names;
      const auto & last = points.back().positions;
      std::string joints;
      for (size_t i = 0; i < names.size() && i < last.size(); ++i) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), " %s=%.1f", names[i].c_str(), angles::to_degrees(last[i]));
        joints += buf;
      }
      RCLCPP_WARN(logger, "[cartesian] %s: last joints [deg]:%s", what.c_str(), joints.c_str());
      if (!allow_partial) {return false;}
    }

    // 一定の速さで動くよう、各点の時刻を付け直す（点は経路上にほぼ等間隔に並ぶ）
    const double duration = std::max(length * fraction / speed, 0.1);
    for (size_t i = 0; i < points.size(); ++i) {
      const double t = duration * static_cast<double>(i) / static_cast<double>(points.size() - 1);
      points[i].time_from_start = rclcpp::Duration::from_seconds(t);
      points[i].velocities.clear();
      points[i].accelerations.clear();
    }

    // 実行は別スレッドで行い、そのあいだ Enter（*stop）を見張る
    auto result = std::async(
      std::launch::async, [this, &trajectory]() {
        return move_group_arm_->execute(trajectory);
      });
    while (result.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
      if (stop && *stop && !stopped_by_user) {
        move_group_arm_->stop();
        stopped_by_user = true;
      }
    }
    const auto code = result.get();

    // 目標（waypoints の最後）と、モーターのエンコーダが読んだ実際の手の位置の差を表示する。
    // 腕が目標に着いているか、重さでたわんでいないかの確認に使う（止められたときは除く）
    if (log_error && !stopped_by_user && static_cast<bool>(code)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 止まるのを待つ
      const auto actual = move_group_arm_->getCurrentPose().pose.position;
      const auto & goal = waypoints.back().position;
      RCLCPP_INFO(
        logger, "[cartesian] %s: error (actual - target) dx=%+.1f dy=%+.1f dz=%+.1f [mm]",
        what.c_str(), (actual.x - goal.x) * 1000.0, (actual.y - goal.y) * 1000.0,
        (actual.z - goal.z) * 1000.0);
    }
    return stopped_by_user || static_cast<bool>(code);
  }

  // 手先を今の向きのまま、真上または真下へまっすぐ target_z [m] まで動かす
  // （今の位置と向きを出発点にする。lower・lift_place モード用。
  //   届かないときは、届くところまで動かす）
  bool move_hand_vertical(
    const double target_z, const double speed, const std::shared_ptr<std::atomic<bool>> & stop,
    const rclcpp::Logger & logger, bool & stopped_by_user)
  {
    geometry_msgs::msg::Pose goal = move_group_arm_->getCurrentPose().pose;
    const double z0 = goal.position.z;
    goal.position.z = target_z;
    return move_hand_cartesian(
      {goal}, speed, stop, logger, stopped_by_user,
      "Move hand straight from z=" + std::to_string(z0) + " to z=" +
      std::to_string(target_z) + " [m]", true);
  }

  // 今いる位置で、target（本来あるべき位置）とのずれを測り、ずれの分だけ指令をずらして
  // まっすぐ動き直す。ずれが comp.tol より小さくなるか、comp.max_iter 回動き直したら終わる。
  //
  // なぜ直るか：モーターは「指令と今の角度の差」に比例した力しか出さない（P 制御）ので、
  // 腕の重さを支えるぶん、いつも目標の手前で止まる。このずれは同じ姿勢ならほぼ同じ
  // （実機で ±1〜1.5 mm）。そこで target − ずれ を指令すると、同じだけ手前で止まった位置が
  // ちょうど target になる。target そのものは変えず、指令（offset）だけをずらすので、
  // ずれがたまることはない。
  //
  // offset：指令のずらし量 [m]。前の場所で決まった値を持ち越し、この中で更新する
  // （上で横の位置を合わせたあと、下ろすときも同じだけずらすため）
  bool correct_position(
    const geometry_msgs::msg::Pose & target, const double speed, const CompConfig & comp,
    Offset & offset, const rclcpp::Logger & logger, const std::string & what)
  {
    for (int k = 0; ; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 止まるのを待つ
      const auto actual = move_group_arm_->getCurrentPose().pose.position;
      const double ex = actual.x - target.position.x;
      const double ey = actual.y - target.position.y + comp.fake_error_y;
      const double ez = comp.use_z ? actual.z - target.position.z : 0.0;
      RCLCPP_INFO(
        logger, "[comp] %s: try %d error dx=%+.1f dy=%+.1f dz=%+.1f [mm]%s", what.c_str(), k,
        ex * 1000.0, ey * 1000.0, (actual.z - target.position.z) * 1000.0,
        comp.use_z ? "" : " (z not corrected)");

      if (std::hypot(ex, ey) < comp.tol && std::abs(ez) < comp.tol) {return true;}
      if (k >= comp.max_iter) {
        RCLCPP_WARN(logger, "[comp] %s: still %.1f mm off after %d tries", what.c_str(),
          std::hypot(ex, ey) * 1000.0, k);
        return true;  // ずれが残っても、動作は続ける
      }

      Offset next{offset.x - ex, offset.y - ey, offset.z - ez};
      if (std::hypot(next.x, next.y, next.z) > comp.max_total) {
        // 測り間違いなどで大きく動くのを防ぐ
        RCLCPP_WARN(
          logger, "[comp] %s: correction %.1f mm is over the limit %.1f mm. Not corrected.",
          what.c_str(), std::hypot(next.x, next.y, next.z) * 1000.0, comp.max_total * 1000.0);
        return true;
      }
      offset = next;
      geometry_msgs::msg::Pose command = target;
      command.position.x += offset.x;
      command.position.y += offset.y;
      command.position.z += offset.z;
      bool stopped = false;
      if (!move_hand_cartesian(
          {command}, speed, nullptr, logger, stopped,
          what + " (correction " + std::to_string(k + 1) + ")", false, false))
      {
        return false;
      }
    }
  }

  // target へまっすぐ動き、comp.enabled なら、着いたところでずれを補正する。
  // 指令は、持ち越した offset だけ最初からずらしておく
  bool move_hand_compensated(
    const geometry_msgs::msg::Pose & target, const double speed, const CompConfig & comp,
    Offset & offset, const rclcpp::Logger & logger, const std::string & what)
  {
    geometry_msgs::msg::Pose command = target;
    if (comp.enabled) {
      command.position.x += offset.x;
      command.position.y += offset.y;
      command.position.z += offset.z;
    }
    bool stopped = false;
    if (!move_hand_cartesian(
        {command}, speed, nullptr, logger, stopped, what, false, !comp.enabled))
    {
      return false;
    }
    return !comp.enabled || correct_position(target, speed, comp, offset, logger, what);
  }

  // 手を pose にして、腕を「平らな形」にする：上腕のねじり（twist）と前腕のねじり
  // （lower_arm_fixed）が ±tol_deg 以内で、根本の回転（pan）を手の方向に向けた形。
  //
  // なぜ：7 軸のアームは、同じ手の位置・向きでも腕の形がいくつもある。MoveIt に手の位置だけ
  // を渡すと、上腕がねじれた形を選ぶことがあり、そこから左右へまっすぐ動くと、ねじりの関節が
  // 制限（±30°）の端に当たって途中で止まる。ねじりがほぼ 0 の形から始めれば、左右の動きは
  // 根本の回転が受け持つので、制限に当たりにくい。
  //
  // 関節の角度は自分で逆運動学（IK）を解いて決め、その角度へ動かす（関節の目標で動く）
  bool move_arm_planar(
    const geometry_msgs::msg::Pose & pose, const double tol_deg, const rclcpp::Logger & logger,
    const std::string & what)
  {
    if (!confirm_step(what)) {return false;}
    auto state = move_group_arm_->getCurrentState(2.0);
    if (!state) {
      RCLCPP_ERROR(logger, "[planar] could not get the current robot state");
      return false;
    }
    const auto * group = state->getJointModelGroup("arm");

    // IK の出発点：ねじりを 0、根本を手の方向に向けた形（解がこの近くで見つかりやすい）
    state->setVariablePosition(kPanJoint, std::atan2(pose.position.y, pose.position.x));
    state->setVariablePosition(kTwistJoint, 0.0);
    state->setVariablePosition(kForearmTwistJoint, 0.0);

    // 解の条件：ねじり 2 つが ±tol_deg 以内、肘が set_elbow_limit の範囲内
    const double tol = angles::from_degrees(tol_deg);
    const auto acceptable = [this, tol](
      moveit::core::RobotState * s, const moveit::core::JointModelGroup * g,
      const double * values) {
        s->setJointGroupPositions(g, values);
        const double elbow = s->getVariablePosition(kElbowJoint);
        return std::abs(s->getVariablePosition(kTwistJoint)) <= tol &&
               std::abs(s->getVariablePosition(kForearmTwistJoint)) <= tol &&
               (!elbow_limit_enabled_ || (elbow >= elbow_min_ && elbow <= elbow_max_));
      };
    if (!state->setFromIK(group, pose, 1.0, acceptable)) {
      RCLCPP_ERROR(
        logger, "[planar] %s: no IK solution with twists within %.0f deg", what.c_str(), tol_deg);
      return false;
    }
    std::vector<double> joints;
    state->copyJointGroupPositions(group, joints);
    RCLCPP_INFO(
      logger, "[planar] %s: pan=%.1f twist=%.1f elbow=%.1f forearm_twist=%.1f [deg]",
      what.c_str(), angles::to_degrees(state->getVariablePosition(kPanJoint)),
      angles::to_degrees(state->getVariablePosition(kTwistJoint)),
      angles::to_degrees(state->getVariablePosition(kElbowJoint)),
      angles::to_degrees(state->getVariablePosition(kForearmTwistJoint)));
    move_group_arm_->setJointValueTarget(joints);
    return static_cast<bool>(move_group_arm_->move());
  }

  // 今の上腕のねじり（twist）[deg]。左右へ運ぶ前に、腕が平らな形か確かめるため
  double current_twist_deg()
  {
    const auto state = move_group_arm_->getCurrentState(2.0);
    return state ? angles::to_degrees(state->getVariablePosition(kTwistJoint)) : std::nan("");
  }

  // 肘の角度の範囲 [deg] を set_constraints に加える。
  // 肘を折りたたみすぎて、手が根本にぶつかるのを防ぐ（0 に近いほど腕が伸びる）
  void set_elbow_limit(const double min_deg, const double max_deg)
  {
    elbow_limit_enabled_ = true;
    elbow_min_ = angles::from_degrees(min_deg);
    elbow_max_ = angles::from_degrees(max_deg);
  }

  // 設定された関節可動制限をクリアする
  void clear_constraints()
  {
    move_group_arm_->clearPathConstraints();
  }

private:
  // ステップ実行の確認。実行してよければ true
  //   Enter      : 実行する
  //   q + Enter  : 中止する（以後の動作はすべて実行されない）
  // 入力を受け取れない場合（端末につながっていないとき）も、安全のため中止する
  bool confirm_step(const std::string & description)
  {
    if (aborted_) {return false;}
    if (!step_by_step_) {return true;}

    ++step_count_;
    std::cout << "\n[STEP " << step_count_ << "] " << description
              << "\n  Enter = run / q + Enter = abort > " << std::flush;

    std::string line;
    if (!std::getline(std::cin, line) || line == "q" || line == "Q") {
      aborted_ = true;
      std::cout << "Aborted. No further motion will be executed." << std::endl;
      return false;
    }
    return true;
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  double gripper_open_ = angles::from_degrees(20.0);
  double gripper_grasp_ = angles::from_degrees(10.0);
  moveit_msgs::msg::Constraints path_constraints_;
  bool elbow_limit_enabled_ = false;
  double elbow_min_ = 0.0;
  double elbow_max_ = 0.0;
  bool step_by_step_ = false;
  bool aborted_ = false;
  int step_count_ = 0;
};

// 手の位置 [m] と姿勢 [deg]
struct HandPose
{
  double x, y, z;
  double roll, pitch, yaw;
};

// ケースの設定。値は launch の引数で変えられる（未決定なので仮の値から始める）
// 行が増える向き = ロボットの +X（前）、列が増える向き = ロボットの +Y（左）
struct CaseConfig
{
  double origin_x;   // マス(行0, 列0)の中心のx [m]
  double origin_y;   // マス(行0, 列0)の中心のy [m]
  double cell_size;  // 1マスの一辺 [m]
};

// ソルバーの回転角 → 手の向き(yaw) の変換設定
struct YawConfig
{
  double at_rot0;  // ソルバーの回転角が0のときの手の向き [deg]
  double sign;     // ソルバーの回転(時計回り)に対する yaw の増減の向き。+1 か -1
};

// 角度を (-180, 180] に収める
double normalize_deg(double a)
{
  while (a > 180.0) {a -= 360.0;}
  while (a <= -180.0) {a += 360.0;}
  return a;
}

// HandPose（位置 [m]、姿勢 [deg]）→ geometry_msgs の Pose
geometry_msgs::msg::Pose to_pose(const HandPose & h)
{
  geometry_msgs::msg::Pose p;
  p.position.x = h.x;
  p.position.y = h.y;
  p.position.z = h.z;
  tf2::Quaternion q;
  q.setRPY(angles::from_degrees(h.roll), angles::from_degrees(h.pitch),
    angles::from_degrees(h.yaw));
  p.orientation = tf2::toMsg(q);
  return p;
}

// (行, 列, ソルバーの回転角) → 手の位置と向き
// 行・列は小数でもよい（4隅や中央を調べるため）
HandPose make_place_pose(
  const CaseConfig & c, const YawConfig & yc,
  const double row, const double col, const double rotation_deg, const double z)
{
  HandPose p;
  p.x = c.origin_x + c.cell_size * row;
  p.y = c.origin_y + c.cell_size * col;
  p.z = z;
  p.roll = -180.0;
  p.pitch = 0.0;
  p.yaw = normalize_deg(yc.at_rot0 + yc.sign * rotation_deg);
  return p;
}

// 1個を掴んで置く。途中で失敗したら、そこで止めて false を返す
bool pick_and_place_once(
  PickAndPlace & robot, const HandPose & pick, const HandPose & place, const double lift)
{
  // 指定した位置の、高さ z へ手を動かす
  auto go = [&robot](const HandPose & p, const double z) {
      return robot.control_arm(p.x, p.y, z, p.roll, p.pitch, p.yaw);
    };

  // ピック動作
  if (!go(pick, lift)) {return false;}                                         // 物体の上へ
  if (!go(pick, pick.z)) {return false;}                                       // 降りる
  if (!robot.grasp_gripper()) {return false;}                                  // 掴む
  if (!go(pick, lift)) {return false;}                                         // 持ち上げる

  // プレース動作
  if (!go(place, lift)) {return false;}                                        // 置く位置の上へ
  if (!go(place, place.z)) {return false;}                                     // 下ろす
  if (!robot.open_gripper()) {return false;}                                   // 離す
  if (!go(place, lift)) {return false;}                                        // 持ち上げる
  return true;
}

// Enter が押されたら true になるフラグを返す（別スレッドで入力を待つ）。
// 入力を受け取れない場合（端末につながっていないとき）も、安全のためすぐ true にする。
// スレッドは呼び出し元の関数が終わっても残るので、フラグは shared_ptr で共有する
std::shared_ptr<std::atomic<bool>> start_enter_watcher()
{
  auto stop = std::make_shared<std::atomic<bool>>(false);
  std::thread([stop]() {
      std::string line;
      std::getline(std::cin, line);
      *stop = true;
    }).detach();
  return stop;
}

// テスト5：手を真下に向けた開始位置から、まっすぐゆっくり下ろし、Enter で止める。
// 止めた高さを表示するので、つかむ高さ（pick_z）を決めるのに使う
bool run_lower_test(
  PickAndPlace & robot, const rclcpp::Logger & logger, const HandPose & start,
  const double min_z, const double speed)
{
  if (!robot.control_arm(start.x, start.y, start.z, start.roll, start.pitch, start.yaw)) {
    RCLCPP_ERROR(logger, "[lower] could not move to the start pose");
    return false;
  }
  std::cout << "\n[lower] The hand goes down from z=" << start.z << " to z=" << min_z
            << " [m] at " << speed * 100.0 << " [cm/s].\n"
            << "  Press Enter to stop." << std::endl;
  const auto stop = start_enter_watcher();

  bool stopped_by_user = false;
  const bool ok = robot.move_hand_vertical(min_z, speed, stop, logger, stopped_by_user);
  const double z = robot.current_hand_z();
  if (stopped_by_user) {
    RCLCPP_INFO(logger, "[lower] stopped at z=%.3f [m]  -> try pick_z:=%.3f", z, z);
  } else {
    RCLCPP_INFO(logger, "[lower] %s at z=%.3f [m] (without Enter)", ok ? "reached" : "FAILED", z);
  }
  return ok;
}

// テスト6：今の位置から、グリッパはそのままで手をまっすぐ dz [m] 持ち上げ、
// 同じ道をまっすぐ下ろして元の位置に戻し、グリッパを開いて離す
bool run_lift_place(
  PickAndPlace & robot, const rclcpp::Logger & logger, const double dz, const double speed,
  const double release_deg)
{
  const double z0 = robot.current_hand_z();
  RCLCPP_INFO(logger, "[lift_place] start z=%.3f -> lift to z=%.3f [m]", z0, z0 + dz);
  bool stopped = false;
  if (!robot.move_hand_vertical(z0 + dz, speed, nullptr, logger, stopped)) {
    RCLCPP_ERROR(logger, "[lift_place] lifting failed");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  if (!robot.move_hand_vertical(z0, speed, nullptr, logger, stopped)) {
    RCLCPP_ERROR(logger, "[lift_place] going back down failed");
    return false;
  }
  if (!robot.move_gripper_angle(angles::from_degrees(release_deg))) {
    RCLCPP_ERROR(logger, "[lift_place] opening the gripper failed");
    return false;
  }
  RCLCPP_INFO(
    logger, "[lift_place] placed at z=%.3f [m], gripper -> %.1f [deg]",
    robot.current_hand_z(), release_deg);
  return true;
}

// テスト8：指先を、ロボットの高さ 0（base_link、土台の底の高さ）の tip_z [m] に合わせて止める。
// 机の面がロボットの高さ 0 からどれだけ低いかを、指先から机までを定規で測って調べるため。
// 指を閉じ（0°）、(x, y) の真上から手を真下に向けてまっすぐ下ろす。腕の重さで下がる分は、
// 高さも含めて補正する（comp.use_z を true にする）。指先は手首の付け根（gripper_base_link）
// から finger_length [m] 下にあるとして計算する
bool run_fingertip_zero(
  PickAndPlace & robot, const rclcpp::Logger & logger, const double x, const double y,
  const double tip_z, const double finger_length, const double speed, CompConfig comp,
  const double planar_tol_deg)
{
  comp.enabled = true;
  comp.use_z = true;
  // 高さ（10〜17 mm）と前後（約 10 mm）のずれが合わさるので、補正の上限を広げる。
  // 机は高さ 0 より約 3 cm 低いので、25 mm 動き直しても机には届かない
  comp.max_total = std::max(comp.max_total, 0.025);
  comp.max_iter = std::max(comp.max_iter, 3);
  HandPose hand{x, y, tip_z + finger_length, -180.0, 0.0, -90.0};
  HandPose above = hand;
  above.z += 0.05;
  Offset offset;
  RCLCPP_INFO(
    logger, "[fingertip_zero] fingertip -> (%.3f, %.3f, %.3f) [m], hand (gripper_base_link) z=%.3f",
    x, y, tip_z, hand.z);
  const auto step = [&logger](const bool ok, const char * what) {
      RCLCPP_INFO(logger, "[fingertip_zero] %s : %s", what, ok ? "OK" : "FAILED");
      return ok;
    };
  const bool ok =
    step(robot.move_gripper_angle(PickAndPlace::GRIPPER_CLOSE), "close the fingers") &&
    step(robot.move_arm_planar(to_pose(above), planar_tol_deg, logger, "move above"),
    "move above") &&
    step(robot.move_hand_compensated(to_pose(hand), speed, comp, offset, logger, "go down"),
    "go down");
  if (ok) {
    RCLCPP_INFO(
      logger, "[fingertip_zero] The fingertip is now at height %.3f m. Measure from the "
      "fingertip to the table with a ruler.", tip_z);
  }
  return ok;
}

// テスト7：home から、つかむ位置の上へ移動し、真下へ下ろしてつかむ → まっすぐ持ち上げる →
// 高さと手の向きを保ったまま、(dx, dy) [m] だけ水平にまっすぐ動く → まっすぐ下ろす →
// 離す → まっすぐ持ち上げる。手は常に真下を向け、グリッパ（指）は水平にする。
// 目標はすべて「本来あるべき位置と向き」から計算する（測った値を使うと、ずれがたまる）。
// comp.enabled なら、つかむ場所・置く場所で、腕が手前で止まるぶんを補正する（correct_position）
bool run_pick_move(
  PickAndPlace & robot, const rclcpp::Logger & logger, const HandPose & pick,
  const double grasp_deg, const double release_deg, const double dz, const double speed,
  const double dx, const double dy, const CompConfig & comp, const double planar_tol_deg)
{
  HandPose above = pick;
  above.z += dz;
  HandPose place = pick;
  place.x += dx;
  place.y += dy;
  HandPose place_above = place;
  place_above.z += dz;

  RCLCPP_INFO(
    logger, "[pick_move] pick (%.3f, %.3f, %.3f) -> place (%.3f, %.3f, %.3f) [m], "
    "compensation %s", pick.x, pick.y, pick.z, place.x, place.y, place.z,
    comp.enabled ? "ON" : "OFF");
  bool stopped = false;
  const auto step = [&logger](const bool ok, const char * what) {
      RCLCPP_INFO(logger, "[pick_move] %s : %s", what, ok ? "OK" : "FAILED");
      return ok;
    };
  const auto straight = [&](const HandPose & to, const char * what) {
      return step(
        robot.move_hand_cartesian({to_pose(to)}, speed, nullptr, logger, stopped, what), what);
    };
  // 補正は、つかむ場所・置く場所のそれぞれで、まず真上（周りに当たらない高さ）で横の位置を
  // 合わせ、同じずらし量のまま下ろし、下でもう一度合わせる。場所ごとにずらし量を測り直す
  Offset at_pick;
  Offset at_place;
  // 左右へ運ぶ直前の、上腕のねじり（平らな形を保てているかの確認用）
  const auto log_twist = [&]() {
      RCLCPP_INFO(logger, "[pick_move] twist before moving sideways: %.1f [deg]",
        robot.current_twist_deg());
      return true;
    };
  const auto compensated = [&](const HandPose & to, Offset & offset, const char * what) {
      return step(robot.move_hand_compensated(to_pose(to), speed, comp, offset, logger, what),
      what);
    };
  return
    // 真上へは、腕を「平らな形」（ねじりがほぼ 0）にして移動する。左右へ運ぶとき、
    // ねじりの関節が制限に当たって途中で止まらないように（move_arm_planar）
    step(robot.move_arm_planar(to_pose(above), planar_tol_deg, logger, "move above the piece"),
    "move above the piece") &&
    step(
    !comp.enabled ||
    robot.correct_position(to_pose(above), speed, comp, at_pick, logger, "above the piece"),
    "correct above the piece") &&
    step(robot.move_gripper_angle(angles::from_degrees(release_deg)), "open") &&
    compensated(pick, at_pick, "go down") &&
    step(robot.move_gripper_angle(angles::from_degrees(grasp_deg)), "grasp") &&
    straight(above, "lift") &&
    log_twist() &&
    compensated(place_above, at_place, "move sideways") &&
    compensated(place, at_place, "put down") &&
    step(robot.move_gripper_angle(angles::from_degrees(release_deg)), "release") &&
    straight(place_above, "move away");
}

// テスト3：グリッパを開く角度から少しずつ閉じていき、Enter が押されたら止める
// 止まった角度を表示するので、取っ手をつかむ角度（gripper_grasp_deg）を決めるのに使う
// グリッパの制御は、目標との差が 0.1 rad（約 5.7°）以内なら「着いた」とみなすので、
// 命令するたびに interval_sec だけ待ち、実際の指の角度を読んで表示する
// Enter で止めたら true（そのあと指を閉じきらないようにするため）
bool run_gripper_sweep(
  PickAndPlace & robot, const rclcpp::Logger & logger,
  const double open_deg, const double step_deg, const double interval_sec, bool & stopped)
{
  stopped = false;
  if (step_deg <= 0.0) {
    RCLCPP_ERROR(logger, "[sweep] sweep_step_deg must be positive: %.1f", step_deg);
    return false;
  }

  // Enter を待つのは別スレッドで行い、そのあいだもグリッパを動かし続ける
  std::cout << "\n[sweep] The gripper closes slowly from " << open_deg << " [deg].\n"
            << "  Press Enter to stop." << std::endl;
  const auto stop = start_enter_watcher();

  double angle = open_deg;
  while (!*stop && angle > 0.0) {
    angle = std::max(angle - step_deg, 0.0);
    if (!robot.move_gripper_angle_now(angles::from_degrees(angle))) {
      RCLCPP_ERROR(logger, "[sweep] gripper motion failed at %.1f [deg]", angle);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(interval_sec));
    RCLCPP_INFO(
      logger, "[sweep] command = %.1f [deg]  actual = %.1f [deg]",
      angle, angles::to_degrees(robot.current_gripper_angle()));
  }

  if (*stop) {
    stopped = true;
    const double actual = angles::to_degrees(robot.current_gripper_angle());
    RCLCPP_INFO(
      logger, "[sweep] stopped. command = %.1f [deg]  actual = %.1f [deg]  "
      "-> try gripper_grasp_deg:=%.1f", angle, actual, angle);
  } else {
    RCLCPP_INFO(logger, "[sweep] reached 0 [deg] (fully closed) without Enter");
  }
  return true;
}

// テスト4：グリッパを最大 → 最小と動かすのを count 回くり返す（動作の確認用）
// 指の間には何も入れないこと（最小は閉じきった状態）
bool run_gripper_cycle(
  PickAndPlace & robot, const rclcpp::Logger & logger,
  const double max_deg, const double min_deg, const int count)
{
  for (int i = 1; i <= count; ++i) {
    if (robot.aborted()) {return false;}
    const bool ok =
      robot.move_gripper_angle(angles::from_degrees(max_deg)) &&
      robot.move_gripper_angle(angles::from_degrees(min_deg));
    RCLCPP_INFO(
      logger, "[cycle] %d / %d : max=%.0f -> min=%.0f [deg] : %s",
      i, count, max_deg, min_deg, ok ? "OK" : "FAILED");
    if (!ok) {return false;}
  }
  return true;
}

// テスト1：ケースの4隅と中央に、手が届くか調べる（上空 → 降下 → 上空）
// 失敗した箇所があっても、残りも調べる（ステップ実行で中止した場合は、そこで終わる）
// 全部成功したら true
bool run_corner_test(
  PickAndPlace & robot, const rclcpp::Logger & logger, const CaseConfig & c,
  const YawConfig & yc, const double lift, const double z)
{
  const double points[5][2] = {{0.0, 0.0}, {0.0, 7.0}, {7.0, 0.0}, {7.0, 7.0}, {3.5, 3.5}};
  int failed = 0;
  for (const auto & pt : points) {
    if (robot.aborted()) {break;}
    const HandPose p = make_place_pose(c, yc, pt[0], pt[1], 0.0, z);
    const bool ok =
      robot.control_arm(p.x, p.y, lift, p.roll, p.pitch, p.yaw) &&
      robot.control_arm(p.x, p.y, p.z, p.roll, p.pitch, p.yaw) &&
      robot.control_arm(p.x, p.y, lift, p.roll, p.pitch, p.yaw);
    RCLCPP_INFO(
      logger, "[corner] row=%.1f col=%.1f -> x=%.3f y=%.3f : %s",
      pt[0], pt[1], p.x, p.y, ok ? "OK" : "FAILED");
    if (!ok) {++failed;}
  }
  RCLCPP_INFO(logger, "[corner] failed points: %d / 5", failed);
  return failed == 0 && !robot.aborted();
}

// テスト2：1つのマスの上で、回転角を 0, 90, 180, 270 度と変えて、手の向きを確認する
bool run_rotation_test(
  PickAndPlace & robot, const rclcpp::Logger & logger, const CaseConfig & c,
  const YawConfig & yc, const double row, const double col, const double lift)
{
  bool all_ok = true;
  for (const int rot : {0, 90, 180, 270}) {
    if (robot.aborted()) {break;}
    const HandPose p = make_place_pose(c, yc, row, col, static_cast<double>(rot), lift);
    const bool ok = robot.control_arm(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
    RCLCPP_INFO(
      logger, "[rotation] solver_rotation=%d -> hand yaw=%.1f : %s",
      rot, p.yaw, ok ? "OK" : "FAILED");
    if (!ok) {all_ok = false;}
    std::this_thread::sleep_for(std::chrono::seconds(1));  // RVizで見えるように少し待つ
  }
  return all_ok && !robot.aborted();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("tukamukai_pick_place", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  PickAndPlace controller(node);
  const auto logger = node->get_logger();
  if (!controller.wait_for_robot_state(10.0)) {
    RCLCPP_ERROR(logger, "Could not receive the robot joint states. Is the robot running?");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  // launch の引数から設定を読む（指定が無ければ、右の値を使う）
  const std::string mode = node->get_parameter_or<std::string>("mode", "single");
  const double speed_scale = node->get_parameter_or<double>("speed_scale", 1.0);
  const bool step_by_step = node->get_parameter_or<bool>("step_by_step", false);
  const CaseConfig case_cfg{
    node->get_parameter_or<double>("origin_x", 0.19),
    // 1マスは 20mm（#31、hardware/dimensions.md）。8×8 のケースが左右の真ん中に来るよう、
    // マス(行0, 列0)の中心は y = -3.5 マス
    node->get_parameter_or<double>("origin_y", -3.5 * tukamukai::kDefaultCellSize),
    node->get_parameter_or<double>("cell_size", tukamukai::kDefaultCellSize)};
  const YawConfig yaw_cfg{
    node->get_parameter_or<double>("yaw_at_rot0", -90.0),
    node->get_parameter_or<double>("yaw_sign", -1.0)};
  const double row = node->get_parameter_or<double>("row", 0.0);
  const double col = node->get_parameter_or<double>("col", 0.0);
  const double rotation = node->get_parameter_or<double>("rotation", 0.0);
  const double place_z = node->get_parameter_or<double>("place_z", 0.13);
  const double gripper_open_deg = node->get_parameter_or<double>("gripper_open_deg", 20.0);
  const double gripper_grasp_deg = node->get_parameter_or<double>("gripper_grasp_deg", 10.0);
  const double sweep_step_deg = node->get_parameter_or<double>("sweep_step_deg", 1.0);
  const double sweep_interval_sec = node->get_parameter_or<double>("sweep_interval_sec", 0.5);
  const double cycle_max_deg = node->get_parameter_or<double>("cycle_max_deg", 90.0);
  const double cycle_min_deg = node->get_parameter_or<double>("cycle_min_deg", -2.0);
  const int cycle_count = node->get_parameter_or<int>("cycle_count", 1);
  const HandPose hand_down_pose{
    node->get_parameter_or<double>("down_x", 0.25),
    node->get_parameter_or<double>("down_y", 0.0),
    node->get_parameter_or<double>("down_z", 0.2),
    -180.0, 0.0, -90.0};  // roll=-180, pitch=0 で手が真下を向く
  const double down_gripper_deg = node->get_parameter_or<double>("down_gripper_deg", -2.0);
  const double down_end_gripper_deg =
    node->get_parameter_or<double>("down_end_gripper_deg", 10.0);
  const double lower_start_z = node->get_parameter_or<double>("lower_start_z", 0.2);
  const double lower_min_z = node->get_parameter_or<double>("lower_min_z", 0.09);
  const double lower_speed = node->get_parameter_or<double>("lower_speed", 0.01);
  const double lift_dz = node->get_parameter_or<double>("lift_dz", 0.05);
  const double lift_speed = node->get_parameter_or<double>("lift_speed", 0.02);
  const double place_dx = node->get_parameter_or<double>("place_dx", 0.0);
  const double place_dy = node->get_parameter_or<double>("place_dy", 0.15);
  const double pick_grasp_deg = node->get_parameter_or<double>("pick_grasp_deg", -2.0);
  CompConfig comp;
  comp.enabled = node->get_parameter_or<bool>("comp_enabled", true);
  comp.tol = node->get_parameter_or<double>("comp_tol_mm", 1.0) / 1000.0;
  comp.max_iter = node->get_parameter_or<int>("comp_max_iter", 2);
  comp.max_total = node->get_parameter_or<double>("comp_max_mm", 10.0) / 1000.0;
  comp.use_z = node->get_parameter_or<bool>("comp_use_z", false);
  comp.fake_error_y = node->get_parameter_or<double>("comp_fake_error_mm", 0.0) / 1000.0;
  const double planar_tol_deg = node->get_parameter_or<double>("planar_tol_deg", 5.0);
  const double tip_x = node->get_parameter_or<double>("tip_x", 0.25);
  const double tip_y = node->get_parameter_or<double>("tip_y", 0.0);
  const double tip_z = node->get_parameter_or<double>("tip_z", 0.0);
  // 手首の付け根から指先まで [m]（ロボットのモデル：指の付け根 0.024 ＋ 指の長さ 0.060）
  const double finger_length = node->get_parameter_or<double>("finger_length", 0.084);
  const double down_elbow_min_deg = node->get_parameter_or<double>("down_elbow_min_deg", -135.0);
  const double down_elbow_max_deg = node->get_parameter_or<double>("down_elbow_max_deg", -45.0);

  controller.set_speed_scale(speed_scale);
  controller.set_step_by_step(step_by_step);
  controller.set_gripper_angles(gripper_open_deg, gripper_grasp_deg);

  RCLCPP_INFO(
    logger, "mode=%s speed=%.2f step_by_step=%s origin=(%.3f, %.3f) cell=%.3f yaw0=%.1f sign=%.0f",
    mode.c_str(), speed_scale, step_by_step ? "true" : "false",
    case_cfg.origin_x, case_cfg.origin_y, case_cfg.cell_size, yaw_cfg.at_rot0, yaw_cfg.sign);
  RCLCPP_INFO(
    logger, "gripper open=%.1f grasp=%.1f [deg]", gripper_open_deg, gripper_grasp_deg);
  if (step_by_step) {
    std::cout << "STEP-BY-STEP MODE: confirm every motion with Enter. "
              << "Type q + Enter to abort.\n"
              << "(Keep the emergency stop within reach.)" << std::endl;
  }

  // アプローチ・退避時の高さ [m]
  const double LIFTING_HEIGHT = node->get_parameter_or<double>("lift_z", 0.3);

  // 掴む位置（今は固定。あとでカメラの認識結果に置き換える）
  // z は指先ではなく、手首の付け根（crane_x7_gripper_base_link）の高さ
  const HandPose PICK{
    node->get_parameter_or<double>("pick_x", 0.25),
    node->get_parameter_or<double>("pick_y", 0.0),
    node->get_parameter_or<double>("pick_z", 0.105),
    -180.0, 0.0,
    node->get_parameter_or<double>("pick_yaw", -90.0)};

  // hand_down モードでは、グリッパを一度開いてから down_gripper_deg まで閉じる
  const bool is_hand_down = (mode == "hand_down");

  // lift_place モードは、今の姿勢とグリッパのまま始める（home に戻らない）
  const bool keep_start = (mode == "lift_place");

  // 初期化動作：失敗したら、先へ進まずに止める
  const bool init_ok = keep_start || (
    controller.move_arm_to_named_pose("home") &&
    controller.open_gripper() &&  // 何かを掴んでいた時のために開く
    (!is_hand_down || controller.move_gripper_angle(angles::from_degrees(down_gripper_deg))));
  if (!init_ok) {
    RCLCPP_ERROR(logger, "Initialization failed or aborted. Stopping.");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  // 手を真下に向けるモードでは、肘を折りたたみすぎて根本にぶつからないよう制限する
  if (is_hand_down || mode == "lower" || keep_start || mode == "pick_move" ||
    mode == "fingertip_zero")
  {
    controller.set_elbow_limit(down_elbow_min_deg, down_elbow_max_deg);
  }
  controller.set_constraints();

  bool ok = true;
  bool keep_gripper = false;  // true のとき、終了時にグリッパを閉じない
  bool keep_pose = false;     // true のとき、終了時に home へ戻らない
  if (mode == "home") {
    // 初期化で home に移動済み。終了動作で、もう一度 home を確認してグリッパを閉じる
    RCLCPP_INFO(logger, "[home] OK");
  } else if (mode == "gripper") {
    // アームは home のまま、グリッパだけを「開く → つかむ → 開く」と動かす。
    // 取っ手を指の間に手で持って、開き具合・つかみ具合を確かめる
    ok = controller.grasp_gripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ok = ok && controller.open_gripper();
    RCLCPP_INFO(logger, "[gripper] %s", ok ? "OK" : "FAILED");
  } else if (mode == "gripper_sweep") {
    ok = run_gripper_sweep(
      controller, logger, gripper_open_deg, sweep_step_deg, sweep_interval_sec, keep_gripper);
  } else if (mode == "gripper_cycle") {
    ok = run_gripper_cycle(controller, logger, cycle_max_deg, cycle_min_deg, cycle_count);
    keep_gripper = true;  // 終了時も、最小の角度のままにする
  } else if (is_hand_down) {
    // 手を真下に向けた、つかむときの姿勢で止まる（グリッパは down_gripper_deg のまま）
    const HandPose & p = hand_down_pose;
    ok = controller.control_arm(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
    RCLCPP_INFO(
      logger, "[hand_down] x=%.3f y=%.3f z=%.3f gripper=%.1f [deg] : %s",
      p.x, p.y, p.z, down_gripper_deg, ok ? "OK" : "FAILED");
    // 動作が終わったら、指を押しつけたままにしないよう、少し開いて止める
    if (!controller.aborted()) {
      controller.move_gripper_angle(angles::from_degrees(down_end_gripper_deg));
      RCLCPP_INFO(logger, "[hand_down] gripper -> %.1f [deg]", down_end_gripper_deg);
    }
    keep_gripper = true;
    keep_pose = true;
  } else if (mode == "lower") {
    HandPose start = hand_down_pose;  // 位置（down_x, down_y）と向きは hand_down と同じ
    start.z = lower_start_z;
    ok = run_lower_test(controller, logger, start, lower_min_z, lower_speed);
    keep_pose = true;     // 止めた高さのままにする
    keep_gripper = true;
  } else if (mode == "fingertip_zero") {
    ok = run_fingertip_zero(
      controller, logger, tip_x, tip_y, tip_z, finger_length, lift_speed, comp, planar_tol_deg);
    keep_pose = true;     // 定規で測れるよう、その位置のままにする
    keep_gripper = true;
  } else if (mode == "pick_move") {
    ok = run_pick_move(
      controller, logger, PICK, pick_grasp_deg, gripper_open_deg, lift_dz, lift_speed,
      place_dx, place_dy, comp, planar_tol_deg);
    keep_gripper = true;  // 終わったら home に戻り、グリッパは開いたままにする
  } else if (keep_start) {
    ok = run_lift_place(controller, logger, lift_dz, lift_speed, gripper_open_deg);
    keep_pose = true;     // 置いた位置のままにする
    keep_gripper = true;
  } else if (mode == "corners") {
    ok = run_corner_test(controller, logger, case_cfg, yaw_cfg, LIFTING_HEIGHT, place_z);
  } else if (mode == "rotations") {
    ok = run_rotation_test(controller, logger, case_cfg, yaw_cfg, row, col, LIFTING_HEIGHT);
  } else if (mode == "single") {
    const HandPose place = make_place_pose(case_cfg, yaw_cfg, row, col, rotation, place_z);
    RCLCPP_INFO(
      logger, "[single] row=%.1f col=%.1f rotation=%.0f -> x=%.3f y=%.3f yaw=%.1f",
      row, col, rotation, place.x, place.y, place.yaw);
    ok = pick_and_place_once(controller, PICK, place, LIFTING_HEIGHT);
    if (!ok) {
      // 失敗したときは、勝手に動かさず、その場で止める（実機での安全のため）
      RCLCPP_ERROR(logger, "Motion failed or aborted. Stopping here.");
      rclcpp::shutdown();
      spin_thread.join();
      return 1;
    }
    RCLCPP_INFO(logger, "Pick and place finished.");
  } else {
    RCLCPP_ERROR(
      logger, "Unknown mode: %s (home / single / corners / rotations / gripper / gripper_sweep / "
      "gripper_cycle / hand_down / lower / lift_place / pick_move / fingertip_zero)",
      mode.c_str());
    ok = false;
  }

  // 終了動作（ステップ実行で中止した場合と、姿勢を保つモードでは、動かさずに終わる）
  if (!controller.aborted() && !keep_pose) {
    controller.clear_constraints();
    controller.move_arm_to_named_pose("home");
    if (!keep_gripper) {
      controller.move_gripper_angle(PickAndPlace::GRIPPER_CLOSE);
    }
  }

  rclcpp::shutdown();
  spin_thread.join();
  return ok ? 0 : 1;
}
