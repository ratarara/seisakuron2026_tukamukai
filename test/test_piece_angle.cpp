#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <vector>

#include "tukamukai/piece_angle.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

// 向き yaw の形を描き、真ん中に取っ手の白い穴（1/3 マス角）をあけたマスク
cv::Mat make_mask(char kind, double yaw, double cell_px, const cv::Point2d & center)
{
  cv::Mat mask = cv::Mat::zeros(400, 400, CV_8U);
  for (const auto & cell : tukamukai::piece_cells_in_image(kind, yaw, cell_px, center)) {
    std::vector<cv::Point> pts;
    for (const auto & p : cell) {
      pts.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    cv::fillConvexPoly(mask, pts, 255);
  }
  const int h = static_cast<int>(cell_px / 6);
  cv::rectangle(
    mask, cv::Point(cvRound(center.x) - h, cvRound(center.y) - h),
    cv::Point(cvRound(center.x) + h, cvRound(center.y) + h), 0, cv::FILLED);
  return mask;
}

// 対称な形は、周期だけ違う角度を同じとみなして差を求める [deg]
double angle_error_deg(double a, double b, char kind)
{
  const double period = tukamukai::symmetry_period(kind);
  return std::abs(std::remainder(a - b, period)) * 180.0 / kPi;
}

}  // namespace

TEST(PieceAngle, FindsAngleForAllKindsAndAngles)
{
  for (char kind : {'I', 'O', 'T', 'S', 'Z', 'J', 'L'}) {
    for (double deg = -180.0; deg < 180.0; deg += 17.0) {
      const double yaw = deg * kPi / 180.0;
      const cv::Point2d center(201.3, 198.7);
      const auto r = tukamukai::estimate_piece_angle(make_mask(kind, yaw, 30.0, center), kind);
      ASSERT_TRUE(r.has_value()) << kind << " " << deg;
      EXPECT_LT(angle_error_deg(r->yaw, yaw, kind), 1.5) << kind << " " << deg;
      // 取っ手の穴は真ん中なので、重心はずれない
      EXPECT_LT(cv::norm(r->center - center), 1.0) << kind << " " << deg;
      EXPECT_GT(r->score, 0.9) << kind << " " << deg;
    }
  }
}

TEST(PieceAngle, DistinguishesFullTurnForL)
{
  // L は 180° 回すと別の見た目なので、170° と -10° を取り違えない
  for (double deg : {170.0, -10.0, 95.0, -85.0}) {
    const double yaw = deg * kPi / 180.0;
    const auto r = tukamukai::estimate_piece_angle(make_mask('L', yaw, 25.0, {200, 200}), 'L');
    ASSERT_TRUE(r.has_value());
    EXPECT_LT(std::abs(std::remainder(r->yaw - yaw, 2 * kPi)) * 180.0 / kPi, 1.5) << deg;
  }
}

TEST(PieceAngle, CounterClockwiseInImageIsPositive)
{
  // I を 90° 反時計回りに回すと縦長になる（画像の x 方向の幅が 1 マス）
  const auto r = tukamukai::estimate_piece_angle(
    make_mask('I', kPi / 2, 30.0, {200, 200}), 'I');
  ASSERT_TRUE(r.has_value());
  EXPECT_LT(angle_error_deg(r->yaw, kPi / 2, 'I'), 1.5);
  // L の基準の向き：下の行が 3 マス、右上に 1 マス。90° 反時計回りで、右の列が 3 マス・左下に 1 マス
  const auto cells = tukamukai::piece_cells_in_image('L', kPi / 2, 10.0, {0, 0});
  int right = 0;
  for (const auto & cell : cells) {
    double x = 0.0;
    for (const auto & p : cell) {
      x += p.x / 4.0;
    }
    if (x > 0.0) {++right;}
  }
  EXPECT_EQ(right, 3);
}

TEST(PieceAngle, RejectsWrongShapes)
{
  // 円は、どの向きの L ともよく重ならない
  cv::Mat mask = cv::Mat::zeros(300, 300, CV_8U);
  cv::circle(mask, {150, 150}, 40, 255, cv::FILLED);
  EXPECT_FALSE(tukamukai::estimate_piece_angle(mask, 'L').has_value());
  EXPECT_FALSE(tukamukai::estimate_piece_angle(cv::Mat::zeros(100, 100, CV_8U), 'L').has_value());
}

TEST(PieceAngle, ImageDownYaw)
{
  // 画像の下向き（光軸のフレームの y 軸）が base_link の -x を向いているとき
  EXPECT_NEAR(std::abs(tukamukai::image_down_yaw(-1.0, 0.0)), kPi, 1e-9);
  // 画像の下向きが base_link の +y（ロボットの左）を向いているとき
  EXPECT_NEAR(tukamukai::image_down_yaw(0.0, 0.7), kPi / 2, 1e-9);
}

TEST(PieceAngle, ImageYawToRobotYaw)
{
  // 画像の下向きが base_link の -x（ロボットに向かう向き）のとき、画像で 0 は robot yaw π
  EXPECT_NEAR(std::abs(tukamukai::image_yaw_to_robot_yaw(0.0, kPi)), kPi, 1e-9);
  EXPECT_NEAR(tukamukai::image_yaw_to_robot_yaw(kPi / 2, kPi), -kPi / 2, 1e-9);
  EXPECT_NEAR(tukamukai::image_yaw_to_robot_yaw(0.3, 0.2), 0.5, 1e-9);
}
