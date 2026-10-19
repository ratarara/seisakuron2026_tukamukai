#include "tukamukai/board_geometry.hpp"

#include <cmath>

namespace tukamukai
{

double normalize_angle(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  } else if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

double rotation_to_yaw(int rotation_deg)
{
  return normalize_angle(-rotation_deg * M_PI / 180.0);
}

Pose2D cell_center_in_case(double row, double col, double cell_size)
{
  return {(row + 0.5) * cell_size, (col + 0.5) * cell_size, 0.0};
}

Pose2D placement_in_case(const Placement & placement, double cell_size)
{
  // center() はマス (r, c) の中心を (r, c) とした単位なので、0.5 マスずらしてからメートルにする
  const auto [row, col] = placement.center();
  Pose2D pose = cell_center_in_case(row, col, cell_size);
  pose.yaw = rotation_to_yaw(placement.rotation);
  return pose;
}

Pose2D case_to_robot(const Pose2D & in_case, const CaseFrame & frame)
{
  const double c = std::cos(frame.yaw);
  const double s = std::sin(frame.yaw);
  return {
    frame.x + c * in_case.x - s * in_case.y,
    frame.y + s * in_case.x + c * in_case.y,
    normalize_angle(frame.yaw + in_case.yaw)};
}

Pose2D placement_in_robot(const Placement & placement, const CaseFrame & frame)
{
  return case_to_robot(placement_in_case(placement, frame.cell_size), frame);
}

std::array<std::array<double, 2>, 4> cell_offsets(char kind, double cell_size)
{
  const Shape cells = shape(kind);
  double mean_r = 0.0;
  double mean_c = 0.0;
  for (const auto & cell : cells) {
    mean_r += cell.row / 4.0;
    mean_c += cell.col / 4.0;
  }
  std::array<std::array<double, 2>, 4> offsets{};
  for (size_t i = 0; i < cells.size(); ++i) {
    offsets[i] = {(cells[i].row - mean_r) * cell_size, (cells[i].col - mean_c) * cell_size};
  }
  return offsets;
}

}  // namespace tukamukai
