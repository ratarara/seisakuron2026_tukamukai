// ソルバーの配置（マスと回転角）を、ケース座標・ロボット座標（base_link）の位置と向きに直す（#31）
// ROS に依存しない。座標の決まりは docs/interfaces.md と同じ。
//
// - ケース座標：原点はマス (0, 0) の外側の角。x 軸は行が増える向き、y 軸は列が増える向き、z 軸は上向き
// - マス (r, c) の中心：x = (r + 0.5) × cell_size、y = (c + 0.5) × cell_size
// - 向き（yaw）：上から見て反時計回りが正。ソルバーの回転角は時計回りなので、ケース座標の yaw = −回転角
// - ピースに固定した座標軸：基準の向き（shape() の形）で、x 軸は行が増える向き、y 軸は列が増える向き
//
// 使い方:
//   tukamukai::CaseFrame frame{0.16, -0.04, 0.0, 0.0, 0.020};  // ケースの原点と向き（base_link）
//   tukamukai::Pose2D pose = tukamukai::placement_in_robot(placement, frame);
//   // pose.x, pose.y にピースの重心（取っ手の位置）、pose.yaw にピースの x 軸の向き

#ifndef TUKAMUKAI__BOARD_GEOMETRY_HPP_
#define TUKAMUKAI__BOARD_GEOMETRY_HPP_

#include <array>

#include "tukamukai/solver.hpp"

namespace tukamukai
{

inline constexpr double kDefaultCellSize = 0.020;  // 1マス [m]

// 平面上の位置 [m] と向き [rad]
struct Pose2D
{
  double x;
  double y;
  double yaw;
};

// ケースの置き場所（base_link から見たケース座標の原点と向き）。後で LocateCase（#20）の結果を入れる
struct CaseFrame
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;   // ケースの底面の高さ
  double yaw = 0.0;
  double cell_size = kDefaultCellSize;
};

// 角度を (-π, π] にそろえる
double normalize_angle(double angle);

// ソルバーの回転角 [deg]（時計回り）を、ケース座標での yaw [rad]（反時計回り）にする
double rotation_to_yaw(int rotation_deg);

// マス (row, col) の中心のケース座標。row, col は Placement::center() のような小数でもよい
Pose2D cell_center_in_case(double row, double col, double cell_size = kDefaultCellSize);

// ピースの重心の位置と向き（ケース座標）
Pose2D placement_in_case(const Placement & placement, double cell_size = kDefaultCellSize);

// ケース座標の位置と向きを base_link に直す
Pose2D case_to_robot(const Pose2D & in_case, const CaseFrame & frame);

// ピースの重心の位置と向き（base_link）。ロボットはこの位置・向きでピースを置く
Pose2D placement_in_robot(const Placement & placement, const CaseFrame & frame);

// 基準の向きで、重心から4マスそれぞれの中心までのずれ（ピース座標 [m]）
// ピースの位置と向きからマスの位置を描くのに使う
std::array<std::array<double, 2>, 4> cell_offsets(char kind, double cell_size = kDefaultCellSize);

}  // namespace tukamukai

#endif  // TUKAMUKAI__BOARD_GEOMETRY_HPP_
