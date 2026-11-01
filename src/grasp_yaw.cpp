#include "tukamukai/grasp_yaw.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace tukamukai
{

double normalize_deg(double deg)
{
  deg = std::fmod(deg, 360.0);
  if (deg > 180.0) {deg -= 360.0;}
  if (deg <= -180.0) {deg += 360.0;}
  return deg;
}

double piece_symmetry_deg(const char kind)
{
  switch (kind) {
    case 'O':
      return 90.0;
    case 'I':
    case 'S':
    case 'Z':
      return 180.0;
    default:  // T・J・L（と分からない種類）は、回して同じ形にならない
      return 360.0;
  }
}

namespace
{

// 2 つの角度の差の大きさ [deg]（0〜180）
double angle_distance(const double a, const double b)
{
  return std::abs(normalize_deg(a - b));
}

}  // namespace

std::vector<GraspYaw> grasp_yaw_candidates(
  const char kind, const double pick_piece_deg, const double place_piece_deg,
  const double hand_offset_deg, const bool handle_symmetric, const double preferred_hand_deg)
{
  const double period = piece_symmetry_deg(kind);
  std::vector<GraspYaw> candidates;
  for (const bool flipped : {false, true}) {
    if (flipped && !handle_symmetric) {continue;}
    // 取っ手を逆からつかんだら、手とピースの向きの関係は、置くときも 180° ずれたまま
    const double flip = flipped ? 180.0 : 0.0;
    const double pick_hand = normalize_deg(pick_piece_deg + hand_offset_deg + flip);
    for (double k = 0.0; k < 360.0 - 1e-9; k += period) {
      const double place_hand = normalize_deg(place_piece_deg + hand_offset_deg + flip + k);
      candidates.push_back(
        GraspYaw{pick_hand, place_hand, normalize_deg(place_hand - pick_hand), flipped});
    }
  }

  const auto rank = [preferred_hand_deg](const GraspYaw & g) {
      return std::array<double, 3>{
      std::abs(g.wrist_turn),
      angle_distance(g.pick_hand, preferred_hand_deg),
      angle_distance(g.place_hand, preferred_hand_deg)};
    };
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [&rank](const GraspYaw & a, const GraspYaw & b) {return rank(a) < rank(b);});
  return candidates;
}

}  // namespace tukamukai
