#include "tukamukai/piece_assignment.hpp"

#include <algorithm>
#include <cctype>
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

}  // namespace tukamukai
