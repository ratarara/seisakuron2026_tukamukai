#include "tukamukai/piece_angle.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <utility>

#include "tukamukai/solver.hpp"

namespace tukamukai
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double deg2rad(double deg) {return deg * kPi / 180.0;}

// 画像を見て反時計回りに yaw 回す（画像は y が下向きなので、数学の回転とは y の符号が逆）
cv::Point2d rotate(const cv::Point2d & p, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {p.x * c + p.y * s, -p.x * s + p.y * c};
}

// 領域と、向き yaw の形との重なり（IoU）
double overlap(
  const cv::Mat & region, const cv::Point2d & offset, char kind, double yaw, double cell_px,
  const cv::Point2d & center)
{
  cv::Mat shape_mask = cv::Mat::zeros(region.size(), CV_8U);
  for (const auto & cell : piece_cells_in_image(kind, yaw, cell_px, center)) {
    std::vector<cv::Point> pts;
    for (const auto & p : cell) {
      pts.emplace_back(cvRound(p.x - offset.x), cvRound(p.y - offset.y));
    }
    cv::fillConvexPoly(shape_mask, pts, 255);
  }
  cv::Mat both;
  cv::Mat either;
  cv::bitwise_and(region, shape_mask, both);
  cv::bitwise_or(region, shape_mask, either);
  const int u = cv::countNonZero(either);
  return u > 0 ? static_cast<double>(cv::countNonZero(both)) / u : 0.0;
}

}  // namespace

double symmetry_period(char kind)
{
  switch (kind) {
    case 'O': return kPi / 2;
    case 'I':
    case 'S':
    case 'Z': return kPi;
    default: return 2 * kPi;
  }
}

std::vector<std::vector<cv::Point2d>> piece_cells_in_image(
  char kind, double yaw, double cell_px, const cv::Point2d & center)
{
  const Shape cells = shape(kind);
  // 形の重心（マス単位）。マス (r, c) の中心を (r, c) とする
  double cr = 0.0;
  double cc = 0.0;
  for (const auto & cell : cells) {
    cr += cell.row;
    cc += cell.col;
  }
  cr /= 4.0;
  cc /= 4.0;
  std::vector<std::vector<cv::Point2d>> result;
  for (const auto & cell : cells) {
    std::vector<cv::Point2d> corners;
    for (const auto & [dr, dc] : {std::pair{-0.5, -0.5}, {-0.5, 0.5}, {0.5, 0.5}, {0.5, -0.5}}) {
      // 画像の x ＝ 列、y ＝ 行
      const cv::Point2d local{(cell.col + dc - cc) * cell_px, (cell.row + dr - cr) * cell_px};
      corners.push_back(center + rotate(local, yaw));
    }
    result.push_back(corners);
  }
  return result;
}

std::optional<PieceAngle> estimate_piece_angle(
  const cv::Mat & mask, char kind, const AngleConfig & config)
{
  CV_Assert(mask.type() == CV_8U);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {return std::nullopt;}
  // 一番大きい領域を使い、内側を全部塗る（白い取っ手の穴を埋める）
  size_t largest = 0;
  for (size_t i = 1; i < contours.size(); ++i) {
    if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest])) {largest = i;}
  }
  const cv::Moments m = cv::moments(contours[largest]);
  if (m.m00 <= 0.0) {return std::nullopt;}
  const cv::Point2d center(m.m10 / m.m00, m.m01 / m.m00);
  const double cell_px = std::sqrt(m.m00 / 4.0);

  // 形を回しても入るよう、領域の周りを 1 マス分広げて切り出す
  const int pad = static_cast<int>(std::ceil(cell_px));
  const cv::Rect box = cv::boundingRect(contours[largest]);
  const cv::Rect roi(box.x - pad, box.y - pad, box.width + 2 * pad, box.height + 2 * pad);
  cv::Mat region = cv::Mat::zeros(roi.size(), CV_8U);
  cv::drawContours(
    region, contours, static_cast<int>(largest), 255, cv::FILLED, cv::LINE_8, cv::noArray(), 0,
    -roi.tl());
  const cv::Point2d offset(roi.x, roi.y);

  // 粗く探してから、その近くを細かく探す
  const double period = symmetry_period(kind);
  double best_yaw = 0.0;
  double best = -1.0;
  for (double a = 0.0; a < period; a += deg2rad(config.coarse_step_deg)) {
    const double s = overlap(region, offset, kind, a, cell_px, center);
    if (s > best) {best = s; best_yaw = a;}
  }
  const double coarse = deg2rad(config.coarse_step_deg);
  const double fine = deg2rad(config.fine_step_deg);
  const double start = best_yaw;
  for (double a = start - coarse; a <= start + coarse + 1e-9; a += fine) {
    const double s = overlap(region, offset, kind, a, cell_px, center);
    if (s > best) {best = s; best_yaw = a;}
  }
  if (best < config.min_score) {return std::nullopt;}
  return PieceAngle{std::remainder(best_yaw, 2 * kPi), center, cell_px, best};
}

double image_down_yaw(double optical_y_x, double optical_y_y)
{
  return std::atan2(optical_y_y, optical_y_x);
}

double image_yaw_to_robot_yaw(double image_yaw, double image_down_yaw)
{
  return std::remainder(image_down_yaw + image_yaw, 2 * kPi);
}

void draw_piece_angle(cv::Mat & bgr, char kind, const PieceAngle & a)
{
  for (const auto & cell : piece_cells_in_image(kind, a.yaw, a.cell_px, a.center)) {
    std::vector<cv::Point> pts;
    for (const auto & c : cell) {
      pts.emplace_back(cvRound(c.x), cvRound(c.y));
    }
    cv::polylines(bgr, std::vector<std::vector<cv::Point>>{pts}, true, {0, 0, 0}, 2);
  }
  // ピースの x 軸（基準の向きで画像の下向き）を矢印で描く
  const cv::Point2d tip = a.center + rotate({0.0, 1.5 * a.cell_px}, a.yaw);
  cv::arrowedLine(bgr, a.center, tip, {0, 0, 255}, 2, cv::LINE_AA, 0, 0.25);
  char text[64];
  std::snprintf(text, sizeof(text), "%c %.0f deg (%.2f)", kind, a.yaw * 180.0 / kPi, a.score);
  const cv::Point2d at = a.center + cv::Point2d(-30, -0.9 * a.cell_px);
  cv::putText(bgr, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 3, cv::LINE_AA);
  cv::putText(bgr, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
}

}  // namespace tukamukai
