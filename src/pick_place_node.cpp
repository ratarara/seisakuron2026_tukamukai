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
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tukamukai/board_geometry.hpp"
#include "tukamukai/grasp_yaw.hpp"
#include "tukamukai/piece_assignment.hpp"
#include "tukamukai/solver.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// 置く位置の補正の設定（correct_position）
struct CompConfig
{
  bool enabled = true;
  double tol = 0.001;        // これより小さいずれは補正しない [m]
  int max_iter = 2;          // 動き直す回数の上限
  double max_total = 0.015;  // 指令をずらす量の上限 [m]。超えたら補正しない
  bool use_z = false;        // 高さも補正するか（つかむ高さは実機で決めた指令値なので、初期値は補正しない）
  // 試験用：測ったずれに、横（y）にこの量を足す [m]。Mock は腕が指令どおりに止まり、
  // ずれが 0 なので、補正の動きを確かめるときに使う。実機では 0 のままにする
  double fake_error_y = 0.0;
  // 手の傾きも補正するか。関節のたわみで手が真下を向かないと、指先が目標から大きくずれる
  // （手首の付け根から指先まで約 90 mm あるので、6° の傾きで約 9 mm）
  bool use_tilt = true;
  double tilt_tol_deg = 0.5;   // これより小さい傾きは補正しない [deg]
  double max_tilt_deg = 12.0;  // 向きをずらす量の上限 [deg]。超えたら補正しない
  // 試験用：測った手の向きを、指先が前（+x）へこの角度だけ傾いたものとして扱う [deg]。
  // fake_error_y と同じく、Mock で補正の動きを確かめるときに使う。実機では 0 のままにする
  double fake_tilt_deg = 0.0;
};

// 手首の付け根（gripper_base_link）から実際の指先まで [m]。fingertip_zero の実測（ログに出す目安）
constexpr double REAL_FINGER_LENGTH = 0.0896;

// 指令のずらし量 [m]
struct Offset
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  tf2::Quaternion rot{0.0, 0.0, 0.0, 1.0};  // 指令の向きをずらす回転（base_link から見て）
};

// target に offset を足した指令を作る
geometry_msgs::msg::Pose with_offset(const geometry_msgs::msg::Pose & target, const Offset & offset)
{
  geometry_msgs::msg::Pose command = target;
  command.position.x += offset.x;
  command.position.y += offset.y;
  command.position.z += offset.z;
  tf2::Quaternion q;
  tf2::fromMsg(target.orientation, q);
  command.orientation = tf2::toMsg((offset.rot * q).normalized());
  return command;
}

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
  inline static const char * const kWristJoint = "crane_x7_wrist_joint";  // 手首の回転（関節 7）
  // 手首の関節の可動範囲に残す余裕 [deg]。補正や運ぶ途中で少し回っても、範囲を超えないように
  inline static const double kWristMarginDeg = 5.0;

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
    // と、手の向きを変える角度
    const auto start = move_group_arm_->getCurrentPose().pose;
    geometry_msgs::msg::Point prev = start.position;
    tf2::Quaternion prev_q;
    tf2::fromMsg(start.orientation, prev_q);
    double length = 0.0;
    double turn = 0.0;
    for (const auto & w : waypoints) {
      length += std::hypot(
        w.position.x - prev.x, w.position.y - prev.y, w.position.z - prev.z);
      prev = w.position;
      tf2::Quaternion q;
      tf2::fromMsg(w.orientation, q);
      turn += prev_q.angleShortestPath(q);
      prev_q = q;
    }
    // もう着いている（0.5 mm 未満、0.2° 未満）ときは動かない。短すぎる経路は計算できないため
    if (length < 0.0005 && turn < angles::from_degrees(0.2)) {return true;}

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
    // 向きを変えるだけの動きが速くなりすぎないよう、向きは 1 秒に 5° までにする
    const double max_turn_speed = angles::from_degrees(5.0);
    const double duration =
      std::max({length * fraction / speed, turn * fraction / max_turn_speed, 0.1});
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
      const auto pose = move_group_arm_->getCurrentPose().pose;
      const auto & actual = pose.position;
      const double ex = actual.x - target.position.x;
      const double ey = actual.y - target.position.y + comp.fake_error_y;
      const double ez = comp.use_z ? actual.z - target.position.z : 0.0;

      // 手の向きのずれ：目標の向きから、実際の向きへの回転（base_link から見て）
      tf2::Quaternion actual_q;
      tf2::Quaternion target_q;
      tf2::fromMsg(pose.orientation, actual_q);
      tf2::fromMsg(target.orientation, target_q);
      actual_q = tf2::Quaternion(
        tf2::Vector3(0.0, 1.0, 0.0), angles::from_degrees(-comp.fake_tilt_deg)) * actual_q;
      const tf2::Quaternion rot_error = (actual_q * target_q.inverse()).normalized();
      const double rot_error_deg = angles::to_degrees(rot_error.getAngleShortestPath());
      // 指先の向きと、指先の位置の目安。tilt は、指先が真下から前（+x）・左（+y）へ傾いた角度
      const tf2::Vector3 finger = tf2::quatRotate(actual_q, tf2::Vector3(0.0, 0.0, 1.0));
      RCLCPP_INFO(
        logger, "[comp] %s: try %d error dx=%+.1f dy=%+.1f dz=%+.1f [mm]%s, "
        "tilt front=%+.1f left=%+.1f [deg], fingertip about (%.1f, %.1f, %.1f) [mm]",
        what.c_str(), k, ex * 1000.0, ey * 1000.0, (actual.z - target.position.z) * 1000.0,
        comp.use_z ? "" : " (z not corrected)",
        angles::to_degrees(std::atan2(finger.x(), -finger.z())),
        angles::to_degrees(std::atan2(finger.y(), -finger.z())),
        (actual.x + finger.x() * REAL_FINGER_LENGTH) * 1000.0,
        (actual.y + finger.y() * REAL_FINGER_LENGTH) * 1000.0,
        (actual.z + finger.z() * REAL_FINGER_LENGTH) * 1000.0);

      const bool tilt_ok = !comp.use_tilt || rot_error_deg < comp.tilt_tol_deg;
      if (std::hypot(ex, ey) < comp.tol && std::abs(ez) < comp.tol && tilt_ok) {return true;}
      if (k >= comp.max_iter) {
        RCLCPP_WARN(logger, "[comp] %s: still %.1f mm, %.1f deg off after %d tries", what.c_str(),
          std::hypot(ex, ey) * 1000.0, rot_error_deg, k);
        return true;  // ずれが残っても、動作は続ける
      }

      Offset next{offset.x - ex, offset.y - ey, offset.z - ez, offset.rot};
      if (comp.use_tilt) {
        next.rot = (offset.rot * rot_error.inverse()).normalized();
        const double total_deg = angles::to_degrees(next.rot.getAngleShortestPath());
        if (total_deg > comp.max_tilt_deg) {
          RCLCPP_WARN(
            logger,
            "[comp] %s: tilt correction %.1f deg is over the limit %.1f deg. Not corrected.",
            what.c_str(), total_deg, comp.max_tilt_deg);
          return true;
        }
      }
      if (std::hypot(next.x, next.y, next.z) > comp.max_total) {
        // 測り間違いなどで大きく動くのを防ぐ
        RCLCPP_WARN(
          logger, "[comp] %s: correction %.1f mm is over the limit %.1f mm. Not corrected.",
          what.c_str(), std::hypot(next.x, next.y, next.z) * 1000.0, comp.max_total * 1000.0);
        return true;
      }
      offset = next;
      bool stopped = false;
      if (!move_hand_cartesian(
          {with_offset(target, offset)}, speed, nullptr, logger, stopped,
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
    const geometry_msgs::msg::Pose command = comp.enabled ? with_offset(target, offset) : target;
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
    const auto state = solve_planar_ik(pose, tol_deg);
    if (!state) {
      RCLCPP_ERROR(
        logger, "[planar] %s: no IK solution with twists within %.0f deg", what.c_str(), tol_deg);
      return false;
    }
    // 目標の手の向きへ、手首の関節だけを回す（手の位置と、腕の平らな形は変わらない）
    double turn = 0.0;
    if (!turn_wrist_in_state(*state, yaw_deg_of(pose), logger, turn)) {return false;}
    std::vector<double> joints;
    state->copyJointGroupPositions("arm", joints);
    RCLCPP_INFO(
      logger, "[planar] %s: pan=%.1f twist=%.1f elbow=%.1f forearm_twist=%.1f wrist=%.1f "
      "[deg]", what.c_str(), angles::to_degrees(state->getVariablePosition(kPanJoint)),
      angles::to_degrees(state->getVariablePosition(kTwistJoint)),
      angles::to_degrees(state->getVariablePosition(kElbowJoint)),
      angles::to_degrees(state->getVariablePosition(kForearmTwistJoint)),
      angles::to_degrees(state->getVariablePosition(kWristJoint)));
    move_group_arm_->setJointValueTarget(joints);
    return static_cast<bool>(move_group_arm_->move());
  }

  // pose の位置に、手の向きを届くことが分かっている -90° にした、平らな形の腕の状態を IK で
  // 求める（腕は動かさない）。解がなければ nullptr。
  // ねじりの条件が厳しいので、ほかの手の向きのままだと、ランダムに探す IK が時間内に解を
  // 見つけられないことがある。目標の向きへは、あとで手首の関節だけを回して合わせる
  moveit::core::RobotStatePtr solve_planar_ik(
    const geometry_msgs::msg::Pose & pose, const double tol_deg)
  {
    auto state = move_group_arm_->getCurrentState(2.0);
    if (!state) {return nullptr;}

    // IK の出発点：ねじりを 0、根本を手の方向に向けた形（解がこの近くで見つかりやすい）
    state->setVariablePosition(kPanJoint, std::atan2(pose.position.y, pose.position.x));
    state->setVariablePosition(kTwistJoint, 0.0);
    state->setVariablePosition(kForearmTwistJoint, 0.0);

    geometry_msgs::msg::Pose preferred = pose;
    tf2::Quaternion q_preferred;
    q_preferred.setRPY(
      angles::from_degrees(-180.0), 0.0, angles::from_degrees(tukamukai::kPreferredHandDeg));
    preferred.orientation = tf2::toMsg(q_preferred);

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
    if (!state->setFromIK(state->getJointModelGroup("arm"), preferred, 1.0, acceptable)) {
      return nullptr;
    }
    return state;
  }

  // 手の向きの候補のうち、手が届く最初のものを選ぶ（腕は動かさない）。
  // つかむ場所の真上（つかむ手の向き）、置く場所の真上（運んできた手の向き、置く手の向き）の
  // 3 か所で、手首の関節が可動範囲に収まるかを確かめる。届く候補がなければ false
  bool choose_reachable_yaw(
    const std::vector<tukamukai::GraspYaw> & candidates,
    const geometry_msgs::msg::Pose & pick_above, const geometry_msgs::msg::Pose & place_above,
    const double tol_deg, const rclcpp::Logger & logger, tukamukai::GraspYaw & chosen)
  {
    const auto pick_state = solve_planar_ik(pick_above, tol_deg);
    const auto place_state = solve_planar_ik(place_above, tol_deg);
    if (!pick_state || !place_state) {
      RCLCPP_ERROR(logger, "[yaw] no IK solution above the %s", pick_state ? "place" : "piece");
      return false;
    }
    const auto reachable = [](const moveit::core::RobotState & base, const double yaw) {
        moveit::core::RobotState state(base);
        return wrist_target_for_yaw(state, yaw).has_value();
      };
    for (const auto & g : candidates) {
      const bool ok = reachable(*pick_state, g.pick_hand) &&
        reachable(*place_state, g.pick_hand) && reachable(*place_state, g.place_hand);
      RCLCPP_INFO(
        logger, "[yaw] hand yaw %.1f -> %.1f (wrist turn %.1f%s) [deg] : %s", g.pick_hand,
        g.place_hand, g.wrist_turn, g.flipped ? ", handle from the other side" : "",
        ok ? "reachable" : "out of the wrist range");
      if (ok) {
        chosen = g;
        return true;
      }
    }
    RCLCPP_ERROR(logger, "[yaw] no reachable hand yaw");
    return false;
  }

  // 今の手の向き（手首の付け根の yaw）[deg]。手を真下に向けているときの値
  double current_hand_yaw_deg()
  {
    const auto q = move_group_arm_->getCurrentPose().pose.orientation;
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    return angles::to_degrees(yaw);
  }

  // Pose の手の向き（yaw）[deg]
  static double yaw_deg_of(const geometry_msgs::msg::Pose & pose)
  {
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    .getRPY(roll, pitch, yaw);
    return angles::to_degrees(yaw);
  }

  // state の腕の形のまま、手の向きを target_yaw_deg にする手首の関節（関節 7）の角度 [rad] を
  // 求める。可動範囲（余裕 kWristMarginDeg つき）に収まらなければ、値なし。
  // 手首の関節を回す向きと、手の向きが変わる向きの関係は腕の形で変わるので、
  // ロボットのモデルで関節を少し回したときの手の向きの変化から、回す向きを決める。
  // 手の向きは 360° で同じなので、回す向きは短いほうから試し、だめなら反対回りも試す
  static std::optional<double> wrist_target_for_yaw(
    moveit::core::RobotState & state, const double target_yaw_deg)
  {
    state.update();
    const auto * link = state.getLinkModel("crane_x7_gripper_base_link");
    const auto yaw_of = [&]() {
        const Eigen::Matrix3d r = state.getGlobalLinkTransform(link).rotation();
        return angles::to_degrees(std::atan2(r(1, 0), r(0, 0)));
      };
    const double wrist0 = state.getVariablePosition(kWristJoint);
    const double yaw0 = yaw_of();
    state.setVariablePosition(kWristJoint, wrist0 + angles::from_degrees(1.0));
    state.update();
    const double sign = tukamukai::normalize_deg(yaw_of() - yaw0) > 0.0 ? 1.0 : -1.0;
    state.setVariablePosition(kWristJoint, wrist0);
    state.update();

    const double turn = tukamukai::normalize_deg(target_yaw_deg - yaw0);
    const auto & bounds = state.getRobotModel()->getVariableBounds(kWristJoint);
    const double margin = angles::from_degrees(kWristMarginDeg);
    for (const double t : {turn, turn > 0.0 ? turn - 360.0 : turn + 360.0}) {
      const double wrist = wrist0 + sign * angles::from_degrees(t);
      if (wrist >= bounds.min_position_ + margin && wrist <= bounds.max_position_ - margin) {
        return wrist;
      }
    }
    return std::nullopt;
  }

  // state の手首の関節（関節 7）だけを回して、手の向きを target_yaw_deg にする
  // （手首の付け根は手首の関節の軸の上にあるので、手の位置は変わらない）。
  // turn_deg：手首の関節を回す量（手の向きの変化とは符号が逆のことがある）。
  // 可動範囲を超えるときは false（state は変えない）
  bool turn_wrist_in_state(
    moveit::core::RobotState & state, const double target_yaw_deg,
    const rclcpp::Logger & logger, double & turn_deg)
  {
    const auto wrist = wrist_target_for_yaw(state, target_yaw_deg);
    if (!wrist) {
      RCLCPP_ERROR(
        logger, "[wrist] hand yaw %.1f deg is out of the wrist range (wrist joint now %.1f deg)",
        target_yaw_deg, angles::to_degrees(state.getVariablePosition(kWristJoint)));
      return false;
    }
    turn_deg = angles::to_degrees(*wrist - state.getVariablePosition(kWristJoint));
    state.setVariablePosition(kWristJoint, *wrist);
    state.update();
    return true;
  }

  // 手首の関節（関節 7）だけを回して、手の向きを target_yaw_deg にする（手の位置は変わらない）
  bool rotate_wrist_to(const double target_yaw_deg, const rclcpp::Logger & logger)
  {
    char text[120];
    std::snprintf(text, sizeof(text), "Rotate the wrist to hand yaw %.1f [deg]", target_yaw_deg);
    if (!confirm_step(text)) {return false;}

    auto state = move_group_arm_->getCurrentState(2.0);
    if (!state) {return false;}
    const double yaw0 = current_hand_yaw_deg();
    double turn = 0.0;
    if (!turn_wrist_in_state(*state, target_yaw_deg, logger, turn)) {return false;}
    std::vector<double> joints;
    state->copyJointGroupPositions("arm", joints);
    move_group_arm_->setJointValueTarget(joints);
    if (!static_cast<bool>(move_group_arm_->move())) {return false;}
    RCLCPP_INFO(
      logger, "[wrist] wrist joint turned %.1f: hand yaw %.1f -> %.1f (target %.1f) [deg]", turn,
      yaw0,
      current_hand_yaw_deg(), target_yaw_deg);
    return true;
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

// 1 個のピースをつかんで置くのに必要な値
struct PieceMove
{
  char kind;               // ピースの種類
  double pick_x, pick_y;   // つかむ位置（取っ手）[m]
  double pick_piece_deg;   // つかむときのピースの向き [deg]
  double place_x, place_y;   // 置く位置（取っ手）[m]
  double place_piece_deg;  // 置くときのピースの向き [deg]
};

// つかみ方・置き方の、ピースによらない設定
struct MotionConfig
{
  double pick_z;           // つかむ高さ（手首の付け根）[m]
  double release_dz;       // 置くとき、つかむ高さよりこれだけ上で離す [m]
  double dz;               // 持ち上げる高さ [m]
  double speed;            // 上下・横の移動の速さ [m/s]
  double grasp_deg;        // つかむ指の角度 [deg]
  double pick_open_deg;    // つかむ前に開く指の角度 [deg]
  double place_open_deg;   // 置いて離すときの指の角度 [deg]
  double hand_offset_deg;  // 手の向き - ピースの向き [deg]
  double planar_tol_deg;   // 腕を平らな形にするときの、ねじりの許容 [deg]
  CompConfig comp;
};

// 手の向きを選ぶ（腕は動かさない）。届く候補がなければ false
bool choose_yaw_for(
  PickAndPlace & robot, const rclcpp::Logger & logger, const PieceMove & m,
  const MotionConfig & c, tukamukai::GraspYaw & yaw)
{
  const HandPose pick_above{m.pick_x, m.pick_y, c.pick_z + c.dz, -180.0, 0.0, -90.0};
  const HandPose place_above{m.place_x, m.place_y, c.pick_z + c.dz, -180.0, 0.0, -90.0};
  return robot.choose_reachable_yaw(
    tukamukai::grasp_yaw_candidates(
      m.kind, m.pick_piece_deg, m.place_piece_deg, c.hand_offset_deg),
    to_pose(pick_above), to_pose(place_above), c.planar_tol_deg, logger, yaw);
}

// 1 個のピースを運ぶ：つかむ場所の真上へ移動（腕は平らな形）→ 指を開く → まっすぐ下ろす →
// つかむ → まっすぐ持ち上げる → 手の向きを変えずに、置く場所の真上へまっすぐ運ぶ →
// 手首を回して向きを合わせる → まっすぐ下ろす → 指を開いて離す → まっすぐ持ち上げる。
// 手は常に真下を向く。目標はすべて「本来あるべき位置と向き」から計算する（測った値を使うと、
// ずれがたまる）。comp.enabled なら、腕が手前で止まるぶんを補正する（correct_position）
bool pick_and_place_piece(
  PickAndPlace & robot, const rclcpp::Logger & logger, const PieceMove & m,
  const MotionConfig & c, const tukamukai::GraspYaw & yaw)
{
  const HandPose pick{m.pick_x, m.pick_y, c.pick_z, -180.0, 0.0, yaw.pick_hand};
  HandPose above = pick;
  above.z += c.dz;
  const HandPose place{
    m.place_x, m.place_y, c.pick_z + c.release_dz, -180.0, 0.0, yaw.place_hand};
  HandPose place_above = place;
  place_above.z = above.z;
  // 横に運ぶあいだは、つかんだときの手の向きのまま。置く場所の真上で手首を回す
  HandPose place_above_before_turn = place_above;
  place_above_before_turn.yaw = yaw.pick_hand;

  RCLCPP_INFO(
    logger, "[pick_move] pick (%.3f, %.3f, %.3f) -> place (%.3f, %.3f, %.3f) [m], "
    "compensation %s", pick.x, pick.y, pick.z, place.x, place.y, place.z,
    c.comp.enabled ? "ON" : "OFF");
  RCLCPP_INFO(
    logger, "[pick_move] piece %c: piece yaw %.1f -> %.1f, hand yaw %.1f -> %.1f "
    "(wrist turn %.1f%s) [deg]", m.kind, m.pick_piece_deg, m.place_piece_deg, yaw.pick_hand,
    yaw.place_hand, yaw.wrist_turn, yaw.flipped ? ", handle grasped from the other side" : "");

  const auto step = [&logger](const bool ok, const char * what) {
      RCLCPP_INFO(logger, "[pick_move] %s : %s", what, ok ? "OK" : "FAILED");
      return ok;
    };
  // 補正は、つかむ場所・置く場所のそれぞれで、まず真上（周りに当たらない高さ）で横の位置を
  // 合わせ、同じずらし量のまま下ろし、下でもう一度合わせる。場所ごとにずらし量を測り直す
  Offset at_pick;
  Offset at_place;
  // つかんだあとの実際の指の角度と、爪の間の幅の目安（指令との差に比例した力で挟む）。
  // 幅は実測（5° で 13.5 mm）から 2 × 77.4 mm × sin(角度) で計算する
  const auto log_grip = [&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      const double actual = angles::to_degrees(robot.current_gripper_angle());
      RCLCPP_INFO(
        logger, "[pick_move] grip: command %.1f, actual %.1f [deg], gap about %.1f [mm]",
        c.grasp_deg, actual, 2.0 * 77.4 * std::sin(angles::from_degrees(actual)));
      return true;
    };
  const auto compensated = [&](const HandPose & to, Offset & offset, const char * what) {
      return step(
        robot.move_hand_compensated(to_pose(to), c.speed, c.comp, offset, logger, what), what);
    };
  return
    // 真上へは、腕を「平らな形」（ねじりがほぼ 0）にして移動する。左右へ運ぶとき、
    // ねじりの関節が制限に当たって途中で止まらないように（move_arm_planar）
    step(
    robot.move_arm_planar(to_pose(above), c.planar_tol_deg, logger, "move above the piece"),
    "move above the piece") &&
    step(
    !c.comp.enabled ||
    robot.correct_position(to_pose(above), c.speed, c.comp, at_pick, logger, "above the piece"),
    "correct above the piece") &&
    step(robot.move_gripper_angle(angles::from_degrees(c.pick_open_deg)), "open") &&
    compensated(pick, at_pick, "go down") &&
    step(robot.move_gripper_angle(angles::from_degrees(c.grasp_deg)), "grasp") &&
    log_grip() &&
    // 下で測ったずらし量を保ったまま上げる。戻すと、挟んだまま手が前後に動いて傾いてしまう
    compensated(above, at_pick, "lift") &&
    compensated(place_above_before_turn, at_place, "move sideways") &&
    (std::abs(yaw.wrist_turn) < 0.5 ||
    step(robot.rotate_wrist_to(yaw.place_hand, logger), "rotate the wrist")) &&
    compensated(place, at_place, "put down") &&
    step(robot.move_gripper_angle(angles::from_degrees(c.place_open_deg)), "release") &&
    // ここも同じ。戻すと、開いた指が置いたピースの取っ手に当たるおそれがある
    compensated(place_above, at_place, "move away");
}

// テスト7：1 個のピースをつかんで、(dx, dy) [m] だけ横に運んで置く
bool run_pick_move(
  PickAndPlace & robot, const rclcpp::Logger & logger, const PieceMove & m,
  const MotionConfig & c)
{
  tukamukai::GraspYaw yaw{};
  return choose_yaw_for(robot, logger, m, c, yaw) &&
         pick_and_place_piece(robot, logger, m, c, yaw);
}

// テスト9：決まった位置（置き場）に置いた 4 ピースを、4×4 のケースに敷き詰める（カメラなし）。
// 置き場は tray_x, tray_y を先頭にした 2×2 の格子で、ピースはすべて同じ向き（tray_piece_deg）。
// ソルバーの答えから置く位置・向きを計算する（board_geometry）。
// 途中で届かないと分かるとピースを持ったまま止まるので、動き出す前に全部のピースの手の向きを
// 確かめる。手首が 1 回も回らない組み合わせなら、動かさずに false を返す
// つかむ位置：取っ手の位置 [m] とピースの向き [deg]（上から見て反時計回り）
struct PickSpot
{
  double x, y, piece_deg;
  std::string from;  // ログ用（"tray 1"、"piece_L_0" など）
};

// 決め打ちの置き場：答えの i 番目のピースは、置き場の i 番目（2 列に並べる）で待っている
std::vector<PickSpot> tray_spots(
  const size_t count, const double tray_x, const double tray_y, const double tray_dx,
  const double tray_dy, const double tray_piece_deg)
{
  std::vector<PickSpot> spots;
  for (size_t i = 0; i < count; ++i) {
    spots.push_back(
      {tray_x + static_cast<double>(i % 2) * tray_dx,
        tray_y + static_cast<double>(i / 2) * tray_dy, tray_piece_deg,
        "tray " + std::to_string(i + 1)});
  }
  return spots;
}

// 認識のノードが出す tf "piece_<種類>_<番号>"（base_link から見た取っ手の位置と向き）を読み、
// 答えの各ピースに割り当てる。必要なピースがそろうまで wait_sec 秒待ち、そろわなければ空を返す。
// 動いている途中の画像で作った古い結果を使わないよう、max_age_sec 秒より古い tf は使わない
// （static_transform_publisher のように時刻 0 で出している tf は、古さを見ない）。
// 認識のノードは 1 枚の画像のピースを同じ時刻で出す。画像ごとに番号（piece_L_0 と piece_L_1）が
// 入れ替わることがあるので、違う画像の tf を混ぜると 2 つが同じピースを指すおそれがある。
// そのため、同じ時刻の tf だけをまとめて使う（新しい時刻から順に、そろうものを探す）
std::optional<std::vector<PickSpot>> tf_spots(
  const rclcpp::Node::SharedPtr & node, const tf2_ros::Buffer & buffer,
  const std::vector<tukamukai::Placement> & solution, const std::string & base_frame,
  const double wait_sec, const double max_age_sec, const rclcpp::Logger & logger)
{
  const auto deadline = node->now() + rclcpp::Duration::from_seconds(wait_sec);
  std::string error;
  for (;; ) {
    std::map<int64_t, std::vector<tukamukai::DetectedPiece>> by_stamp;
    for (const auto & name : buffer.getAllFrameNames()) {
      const auto kind = tukamukai::kind_from_frame(name);
      if (!kind) {continue;}
      geometry_msgs::msg::TransformStamped t;
      try {
        t = buffer.lookupTransform(base_frame, name, tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        continue;
      }
      const rclcpp::Time stamp(t.header.stamp, RCL_ROS_TIME);
      if (stamp.nanoseconds() != 0 && (node->now() - stamp).seconds() > max_age_sec) {
        continue;
      }
      by_stamp[stamp.nanoseconds()].push_back(
        {*kind, t.transform.translation.x, t.transform.translation.y,
          tf2::getYaw(t.transform.rotation), name});
    }
    if (by_stamp.empty()) {
      tukamukai::assign_pieces(solution, {}, error);  // 足りないものを error に書くため
    }
    for (auto it = by_stamp.rbegin(); it != by_stamp.rend(); ++it) {
      const auto & detected = it->second;
      const auto order = tukamukai::assign_pieces(solution, detected, error);
      if (!order) {continue;}
      std::vector<PickSpot> spots;
      for (const size_t i : *order) {
        const auto & d = detected[i];
        spots.push_back({d.x, d.y, angles::to_degrees(d.yaw), d.frame});
      }
      return spots;
    }
    if (node->now() > deadline) {
      RCLCPP_ERROR(
        logger, "[fill] pieces not found in tf within %.1f s (%s). Is the recognizer running?",
        wait_sec, error.c_str());
      return std::nullopt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

// solution の i 番目のピースを、spots[i] からつかんで置く。
// require_turn：手首が 1 回も回らないなら動かさない（決め打ちの置き場で、デモで回転を見せるため）
bool run_fill_4x4(
  PickAndPlace & robot, const rclcpp::Logger & logger, const tukamukai::CaseFrame & frame,
  const std::vector<tukamukai::Placement> & solution, const std::vector<PickSpot> & spots,
  const bool require_turn, const MotionConfig & c)
{
  RCLCPP_INFO(logger, "[fill] solution:\n%s", tukamukai::render(4, 4, solution).c_str());

  std::vector<PieceMove> moves;
  std::vector<tukamukai::GraspYaw> yaws;
  int turns = 0;
  for (size_t i = 0; i < solution.size(); ++i) {
    const auto & p = solution[i];
    const auto target = tukamukai::placement_in_robot(p, frame);
    const PieceMove m{
      p.kind, spots[i].x, spots[i].y, spots[i].piece_deg,
      target.x, target.y, angles::to_degrees(target.yaw)};
    tukamukai::GraspYaw yaw{};
    if (!choose_yaw_for(robot, logger, m, c, yaw)) {
      RCLCPP_ERROR(logger, "[fill] piece %zu (%c) is out of reach. Nothing moved.", i + 1, p.kind);
      return false;
    }
    if (std::abs(yaw.wrist_turn) >= 0.5) {++turns;}
    RCLCPP_INFO(
      logger, "[fill] %zu: %c  %s at (%.3f, %.3f) yaw %.0f  ->  place (%.3f, %.3f) yaw %.0f "
      "(wrist turn %.0f)", i + 1, p.kind, spots[i].from.c_str(), m.pick_x, m.pick_y,
      m.pick_piece_deg, m.place_x, m.place_y, m.place_piece_deg, yaw.wrist_turn);
    moves.push_back(m);
    yaws.push_back(yaw);
  }
  if (turns == 0 && require_turn) {
    RCLCPP_ERROR(logger, "[fill] the wrist never turns. Change tray_piece_deg. Nothing moved.");
    return false;
  }
  RCLCPP_INFO(logger, "[fill] the wrist turns for %d of %zu pieces", turns, moves.size());

  for (size_t i = 0; i < moves.size(); ++i) {
    RCLCPP_INFO(logger, "[fill] ===== piece %zu / %zu : %c =====", i + 1, moves.size(),
      moves[i].kind);
    if (!pick_and_place_piece(robot, logger, moves[i], c, yaws[i])) {
      RCLCPP_ERROR(logger, "[fill] stopped at piece %zu (%c)", i + 1, moves[i].kind);
      return false;
    }
  }
  RCLCPP_INFO(logger, "[fill] finished: all %zu pieces placed", moves.size());
  return true;
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
  const double gripper_open_deg = node->get_parameter_or<double>("gripper_open_deg", 10.0);
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
  const double pick_grasp_deg = node->get_parameter_or<double>("pick_grasp_deg", -4.0);
  // ピースの種類と向き [deg]（ソルバー・認識と同じ、上から見て反時計回り）
  const std::string piece_kind = node->get_parameter_or<std::string>("piece_kind", "L");
  const double pick_piece_deg = node->get_parameter_or<double>("pick_piece_deg", 0.0);
  const double place_piece_deg = node->get_parameter_or<double>("place_piece_deg", 0.0);
  const double hand_offset_deg =
    node->get_parameter_or<double>("hand_offset_deg", tukamukai::kDefaultHandOffsetDeg);
  // fill_4x4：ケースの角（マス (0, 0) の外側の角）。根本に近いと肘の制限内で届かないので前 22 cm から
  const double case_x = node->get_parameter_or<double>("case_x", 0.22);
  const double case_y = node->get_parameter_or<double>("case_y", -0.04);
  const double case_yaw_deg = node->get_parameter_or<double>("case_yaw_deg", 0.0);
  // fill_4x4：置き場（ピースが待っている位置。取っ手の位置）
  const double tray_x = node->get_parameter_or<double>("tray_x", 0.20);
  const double tray_y = node->get_parameter_or<double>("tray_y", 0.12);
  const double tray_dx = node->get_parameter_or<double>("tray_dx", 0.08);
  const double tray_dy = node->get_parameter_or<double>("tray_dy", 0.09);
  const double tray_piece_deg = node->get_parameter_or<double>("tray_piece_deg", 0.0);
  // fill_4x4：ピースの位置をどこから得るか。tray = 決め打ちの置き場、tf = 認識のノードが出す tf
  const std::string piece_source = node->get_parameter_or<std::string>("piece_source", "tray");
  const double tf_wait_sec = node->get_parameter_or<double>("tf_wait_sec", 5.0);
  const double tf_max_age_sec = node->get_parameter_or<double>("tf_max_age_sec", 5.0);
  // tf を受け取り始めるのは時間がかかるので、最初に作っておく
  const auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  const auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  const double fill_lift_dz = node->get_parameter_or<double>("fill_lift_dz", 0.08);
  // 置いて離すときの指の角度。4×4 ではとなりの取っ手まで約 25 mm しかないので、大きく開かない
  // （20° だと片側約 24 mm 開き、当たる。5° なら片側約 6 mm）
  const double fill_place_open_deg = node->get_parameter_or<double>("fill_place_open_deg", 5.0);
  const double fill_release_dz = node->get_parameter_or<double>("fill_release_dz", 0.003);
  CompConfig comp;
  comp.enabled = node->get_parameter_or<bool>("comp_enabled", true);
  comp.tol = node->get_parameter_or<double>("comp_tol_mm", 1.0) / 1000.0;
  comp.max_iter = node->get_parameter_or<int>("comp_max_iter", 2);
  comp.max_total = node->get_parameter_or<double>("comp_max_mm", 15.0) / 1000.0;
  comp.use_z = node->get_parameter_or<bool>("comp_use_z", false);
  comp.fake_error_y = node->get_parameter_or<double>("comp_fake_error_mm", 0.0) / 1000.0;
  comp.use_tilt = node->get_parameter_or<bool>("comp_use_tilt", true);
  comp.tilt_tol_deg = node->get_parameter_or<double>("comp_tilt_tol_deg", 0.5);
  comp.max_tilt_deg = node->get_parameter_or<double>("comp_max_tilt_deg", 12.0);
  comp.fake_tilt_deg = node->get_parameter_or<double>("comp_fake_tilt_deg", 0.0);
  const double planar_tol_deg = node->get_parameter_or<double>("planar_tol_deg", 5.0);
  const double tip_x = node->get_parameter_or<double>("tip_x", 0.25);
  const double tip_y = node->get_parameter_or<double>("tip_y", 0.0);
  const double tip_z = node->get_parameter_or<double>("tip_z", 0.0);
  // 手首の付け根から指先まで [m]（ロボットのモデル：指の付け根 0.024 ＋ 指の長さ 0.060）
  const double finger_length = node->get_parameter_or<double>("finger_length", 0.084);
  // 指先を下ろす速さ [m/s]。机との間を見ながら止められるよう、ゆっくりにする
  const double tip_speed = node->get_parameter_or<double>("tip_speed", 0.005);
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
    node->get_parameter_or<double>("pick_z", 0.108),
    -180.0, 0.0,
    node->get_parameter_or<double>("pick_yaw", -90.0)};

  // hand_down モードでは、グリッパを一度開いてから down_gripper_deg まで閉じる
  const bool is_hand_down = (mode == "hand_down");

  // pick_move・fill_4x4 で共通の設定
  const MotionConfig motion{
    PICK.z, fill_release_dz, fill_lift_dz, lift_speed, pick_grasp_deg, gripper_open_deg,
    fill_place_open_deg, hand_offset_deg, planar_tol_deg, comp};

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
    mode == "fingertip_zero" || mode == "fill_4x4")
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
      controller, logger, tip_x, tip_y, tip_z, finger_length, tip_speed, comp, planar_tol_deg);
    keep_pose = true;     // 定規で測れるよう、その位置のままにする
    keep_gripper = true;
  } else if (mode == "pick_move") {
    MotionConfig c = motion;
    c.place_open_deg = gripper_open_deg;
    c.release_dz = 0.0;
    c.dz = lift_dz;
    const PieceMove m{
      piece_kind.empty() ? 'L' : piece_kind[0], PICK.x, PICK.y, pick_piece_deg,
      PICK.x + place_dx, PICK.y + place_dy, place_piece_deg};
    ok = run_pick_move(controller, logger, m, c);
    keep_gripper = true;  // 終わったら home に戻り、グリッパは開いたままにする
  } else if (mode == "fill_4x4") {
    const tukamukai::CaseFrame frame{
      case_x, case_y, 0.0, angles::from_degrees(case_yaw_deg), tukamukai::kDefaultCellSize};
    const auto solution = tukamukai::solve(4, 4, {{'I', 1}, {'S', 1}, {'L', 2}});
    std::optional<std::vector<PickSpot>> spots;
    if (!solution) {
      RCLCPP_ERROR(logger, "[fill] no solution for 4x4 with I, S, L, L");
    } else if (piece_source == "tray") {
      spots = tray_spots(solution->size(), tray_x, tray_y, tray_dx, tray_dy, tray_piece_deg);
    } else if (piece_source == "tf") {
      spots = tf_spots(
        node, *tf_buffer, *solution, "base_link", tf_wait_sec, tf_max_age_sec, logger);
    } else {
      RCLCPP_ERROR(logger, "[fill] unknown piece_source: %s (tray / tf)", piece_source.c_str());
    }
    ok = spots.has_value() &&
      run_fill_4x4(controller, logger, frame, *solution, *spots, piece_source == "tray", motion);
    keep_gripper = true;
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
      "gripper_cycle / hand_down / lower / lift_place / pick_move / fingertip_zero / fill_4x4)",
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
