// 1 つのピースの領域（マスク）と種類から、ピースの向き（角度）を求める（ROS に依存しない。OpenCV を使う）
//
// 色の切り出し（どの画素がどのピースか）は認識の担当（B）のコードで行い、その結果をここに渡す。
// ソルバーと同じ形（tukamukai::shape）を少しずつ回して領域と重ね、一番よく重なる（IoU が最大の）
// 角度を向きとする。L・J・T は左右非対称なので 360° 区別できる。
//
// 画像の向きの決まり：
//   画像は x が右、y が下。ソルバーの形の (行, 列) を画像の (下, 右) に重ねた向きを 0 とし、
//   画像を見て **反時計回り** を正とする（yaw [rad]、(-π, π]）。
//   カメラが真上から見下ろしているとき、ロボット座標でのピースの yaw（docs/interfaces.md）は
//     robot_yaw = （画像の下向きが base_link で向いている角度） + image_yaw
//   になる（image_yaw_to_robot_yaw）。
//
// 使い方:
//   // mask：そのピースの画素が 255、それ以外が 0 の画像（CV_8U）。白い取っ手の穴はあってもよい
//   auto r = tukamukai::estimate_piece_angle(mask, 'L');
//   if (r) { r->yaw; r->center; r->score; }

#ifndef TUKAMUKAI__PIECE_ANGLE_HPP_
#define TUKAMUKAI__PIECE_ANGLE_HPP_

#include <opencv2/core.hpp>

#include <optional>
#include <vector>

namespace tukamukai
{

struct AngleConfig
{
  double coarse_step_deg = 2.0;  // 角度を探す刻み（粗く）
  double fine_step_deg = 0.5;    // 角度を探す刻み（細かく）
  // 形の重なり（IoU）がこれより小さければ「分からない」とする。2026-10-26 の録画では、全体が写った
  // ピースは 0.91 以上、画像の端で切れて角度を間違えたものは 0.65 だった
  double min_score = 0.8;
};

struct PieceAngle
{
  double yaw;            // 向き [rad]。画像を見て反時計回りが正（上の決まり）
  cv::Point2d center;    // 領域の重心 [px]（取っ手の位置）
  double cell_px;        // 1 マスの大きさ [px]（面積から求めたもの）
  double score;          // 形の重なり（IoU、0〜1）
};

// マスクの中の一番大きい領域を 1 つのピースとみなし、向きを求める。
// 領域がない、または形がよく重ならないときは std::nullopt
std::optional<PieceAngle> estimate_piece_angle(
  const cv::Mat & mask, char kind, const AngleConfig & config = AngleConfig{});

// 画像上の向き [rad] を、ロボット座標の向き [rad] にする。
// image_down_yaw：画像の下向き（+y）が base_link で向いている角度 [rad]（カメラが真上から見下ろすとき）
double image_yaw_to_robot_yaw(double image_yaw, double image_down_yaw);

// カメラの光軸のフレーム（camera_color_optical_frame）の y 軸（画像の下向き）を base_link で表した
// ベクトルから、image_yaw_to_robot_yaw に渡す image_down_yaw [rad] を求める。
// カメラがほぼ真下を向いているときに使う（y 軸の水平な成分の向き）
double image_down_yaw(double optical_y_x, double optical_y_y);

// 形が同じに見える回転の周期 [rad]（O は 90°、I・S・Z は 180°、ほかは 360°）
double symmetry_period(char kind);

// 種類 kind の形を、向き yaw・1 マス cell_px で center に置いたときの、4 マスそれぞれの四角形の角
std::vector<std::vector<cv::Point2d>> piece_cells_in_image(
  char kind, double yaw, double cell_px, const cv::Point2d & center);

// 求めた向きを画像に描く（当てはめた形・向きの矢印・角度の文字）
void draw_piece_angle(cv::Mat & bgr, char kind, const PieceAngle & angle);

}  // namespace tukamukai

#endif  // TUKAMUKAI__PIECE_ANGLE_HPP_
