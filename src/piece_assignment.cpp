#include "tukamukai/piece_assignment.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <numeric>

namespace tukamukai
{

std::optional<char> kind_from_frame(const std::string & frame, const std::string & prefix)
{
  // prefix + 種類1文字 + '_' + 数字1文字以上
  if (frame.size() < prefix.size() + 3 || frame.compare(0, prefix.size(), prefix) != 0) {
    return std::nullopt;
  }
  const char kind = frame[prefix.size()];
  if (std::find(kKinds.begin(), kKinds.end(), kind) == kKinds.end() ||
    frame[prefix.size() + 1] != '_')
  {
    return std::nullopt;
  }
  const std::string number = frame.substr(prefix.size() + 2);
  if (!std::all_of(
      number.begin(), number.end(), [](unsigned char c) {return std::isdigit(c) != 0;}))
  {
    return std::nullopt;
  }
  return kind;
}

std::optional<std::vector<size_t>> assign_pieces(
  const std::vector<Placement> & solution, const std::vector<DetectedPiece> & detected,
  std::string & error)
{
  // 種類ごとに、見つけたピースの番号をフレーム名の順に並べる
  std::vector<size_t> sorted(detected.size());
  std::iota(sorted.begin(), sorted.end(), 0);
  std::sort(
    sorted.begin(), sorted.end(),
    [&detected](size_t a, size_t b) {return detected[a].frame < detected[b].frame;});
  std::map<char, std::vector<size_t>> by_kind;
  for (size_t i : sorted) {
    by_kind[detected[i].kind].push_back(i);
  }

  std::map<char, int> need;
  for (const auto & p : solution) {
    ++need[p.kind];
  }
  error.clear();
  for (const auto & [kind, n] : need) {
    const int have = static_cast<int>(by_kind[kind].size());
    if (have < n) {
      error += std::string(error.empty() ? "" : ", ") + kind + ": need " + std::to_string(n) +
        ", found " + std::to_string(have);
    }
  }
  if (!error.empty()) {
    return std::nullopt;
  }

  std::map<char, size_t> used;
  std::vector<size_t> order;
  for (const auto & p : solution) {
    order.push_back(by_kind[p.kind][used[p.kind]++]);
  }
  return order;
}

bool in_case(double x, double y, const CaseFrame & frame, int rows, int cols, double margin)
{
  // ケース座標（x 軸＝行が増える向き、y 軸＝列が増える向き）に直す
  const double dx = x - frame.x;
  const double dy = y - frame.y;
  const double c = std::cos(frame.yaw);
  const double s = std::sin(frame.yaw);
  const double u = c * dx + s * dy;
  const double v = -s * dx + c * dy;
  return u >= -margin && u <= rows * frame.cell_size + margin &&
         v >= -margin && v <= cols * frame.cell_size + margin;
}

std::optional<size_t> choose_piece(
  const std::vector<DetectedPiece> & detected, char kind, const CaseFrame & frame, int rows,
  int cols, double margin)
{
  std::optional<size_t> best;
  for (size_t i = 0; i < detected.size(); ++i) {
    const auto & d = detected[i];
    if (d.kind != kind || in_case(d.x, d.y, frame, rows, cols, margin)) {continue;}
    if (!best || d.frame < detected[*best].frame) {best = i;}
  }
  return best;
}

}  // namespace tukamukai
