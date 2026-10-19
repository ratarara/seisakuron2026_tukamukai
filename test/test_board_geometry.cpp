#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tukamukai/board_geometry.hpp"

using tukamukai::CaseFrame;
using tukamukai::Cell;
using tukamukai::Placement;
using tukamukai::Pose2D;

namespace
{

constexpr double kEps = 1e-9;
constexpr double kCell = tukamukai::kDefaultCellSize;

// 基準の向きの形を (row0, col0) だけずらして置いた配置を作る
Placement make_placement(
  char kind, int rotation, const tukamukai::Shape & shape, int row0,
  int col0)
{
  Placement p{kind, rotation, {}};
  for (size_t i = 0; i < shape.size(); ++i) {
    p.cells[i] = Cell{shape[i].row + row0, shape[i].col + col0};
  }
  return p;
}

}  // namespace

TEST(BoardGeometry, CellCenter)
{
  // マス (0, 0) の中心は、ケースの角から半マスずつ内側
  const Pose2D p = tukamukai::cell_center_in_case(0, 0);
  EXPECT_NEAR(p.x, 0.010, kEps);
  EXPECT_NEAR(p.y, 0.010, kEps);

  const Pose2D q = tukamukai::cell_center_in_case(3, 2);
  EXPECT_NEAR(q.x, 0.070, kEps);
  EXPECT_NEAR(q.y, 0.050, kEps);
}

TEST(BoardGeometry, PlacementCenterIsShiftedByHalfCell)
{
  // 0 行目に横向きに置いた I。重心は (0, 1.5) マスなので、ケース座標では (0.010, 0.040)
  // center() をそのまま 0.020 倍すると (0, 0.030) になり、半マスずれてケースからはみ出す
  const Placement i_piece{'I', 0, {Cell{0, 0}, Cell{0, 1}, Cell{0, 2}, Cell{0, 3}}};
  const Pose2D p = tukamukai::placement_in_case(i_piece);
  EXPECT_NEAR(p.x, 0.010, kEps);
  EXPECT_NEAR(p.y, 0.040, kEps);
}

TEST(BoardGeometry, RotationToYaw)
{
  // ソルバーは時計回り、ROS は反時計回りが正なので符号が逆になる
  EXPECT_NEAR(tukamukai::rotation_to_yaw(0), 0.0, kEps);
  EXPECT_NEAR(tukamukai::rotation_to_yaw(90), -M_PI / 2, kEps);
  EXPECT_NEAR(tukamukai::rotation_to_yaw(180), M_PI, kEps);
  EXPECT_NEAR(tukamukai::rotation_to_yaw(270), M_PI / 2, kEps);
}

TEST(BoardGeometry, YawReproducesSolverCells)
{
  // すべての種類・向きで、「重心の位置 + yaw でピース座標のマスを回したもの」が
  // ソルバーの置いたマスの中心と一致する。yaw の符号や軸の取り方を間違えると失敗する
  for (char kind : tukamukai::kKinds) {
    const auto offsets = tukamukai::cell_offsets(kind);
    for (const auto & [rotation, shape] : tukamukai::orientations(kind)) {
      const Placement p = make_placement(kind, rotation, shape, 2, 3);
      const Pose2D pose = tukamukai::placement_in_case(p);
      const double c = std::cos(pose.yaw);
      const double s = std::sin(pose.yaw);

      for (const auto & [dx, dy] : offsets) {
        const double x = pose.x + c * dx - s * dy;
        const double y = pose.y + s * dx + c * dy;
        bool found = false;
        for (const auto & cell : p.cells) {
          const Pose2D center = tukamukai::cell_center_in_case(cell.row, cell.col);
          found |= std::abs(center.x - x) < kEps && std::abs(center.y - y) < kEps;
        }
        EXPECT_TRUE(found) << kind << " rotation=" << rotation;
      }
    }
  }
}

TEST(BoardGeometry, CaseToRobot)
{
  // ケースを (0.2, 0.1) に、90° 回して置いた場合。ケースの x 軸はロボットの y 軸の向きになる
  const CaseFrame frame{0.2, 0.1, 0.0, M_PI / 2, kCell};
  const Pose2D p = tukamukai::case_to_robot({0.01, 0.03, 0.0}, frame);
  EXPECT_NEAR(p.x, 0.2 - 0.03, kEps);
  EXPECT_NEAR(p.y, 0.1 + 0.01, kEps);
  EXPECT_NEAR(p.yaw, M_PI / 2, kEps);

  // yaw は (-π, π] にそろう
  const Pose2D q = tukamukai::case_to_robot({0.0, 0.0, M_PI}, frame);
  EXPECT_NEAR(q.yaw, -M_PI / 2, kEps);
}

TEST(BoardGeometry, SolvedPiecesStayInsideCase)
{
  // 中間発表の 4×4（I・S・L・L）を解き、全ピースの重心がケースの内側に入る
  const auto result = tukamukai::solve(4, 4, {{'I', 1}, {'S', 1}, {'L', 2}});
  ASSERT_TRUE(result.has_value());
  const CaseFrame frame{0.16, -0.04, 0.0, 0.0, kCell};
  for (const auto & p : *result) {
    const Pose2D pose = tukamukai::placement_in_robot(p, frame);
    EXPECT_GT(pose.x, 0.16);
    EXPECT_LT(pose.x, 0.16 + 4 * kCell);
    EXPECT_GT(pose.y, -0.04);
    EXPECT_LT(pose.y, -0.04 + 4 * kCell);
  }
}

TEST(BoardGeometry, CellOffsetsAreCentered)
{
  for (char kind : tukamukai::kKinds) {
    double sx = 0.0;
    double sy = 0.0;
    for (const auto & [dx, dy] : tukamukai::cell_offsets(kind)) {
      sx += dx;
      sy += dy;
    }
    EXPECT_NEAR(sx, 0.0, kEps) << kind;
    EXPECT_NEAR(sy, 0.0, kEps) << kind;
  }
}
