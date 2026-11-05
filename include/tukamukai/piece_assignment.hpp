// 認識で見つけたピースを、ソルバーの答えのどのピースとしてつかむかを決める（ROS に依存しない）
//
// 認識のノードは、見つけたピースごとに tf のフレーム "piece_<種類>_<番号>" を base_link からの
// 位置と向きで出す（例：piece_I_0、piece_S_0、piece_L_0、piece_L_1）。向きは docs/interfaces.md の
// ピースの yaw（上から見て反時計回り）。
//
// 使い方:
//   auto kind = tukamukai::kind_from_frame("piece_L_1");  // 'L'
//   std::string error;
//   auto order = tukamukai::assign_pieces(*solution, detected, error);
//   // (*order)[i] は、ソルバーの答えの i 番目のピースとしてつかむ detected の番号

#ifndef TUKAMUKAI__PIECE_ASSIGNMENT_HPP_
#define TUKAMUKAI__PIECE_ASSIGNMENT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "tukamukai/solver.hpp"

namespace tukamukai
{

inline constexpr char kPieceFramePrefix[] = "piece_";

// 認識で見つけたピース1個（base_link から見た取っ手の位置と、ピースの向き）
struct DetectedPiece
{
  char kind;
  double x;
  double y;
  double yaw;   // [rad]
  std::string frame;
};

// tf のフレーム名から種類を読む。"piece_<I O T S Z J L のどれか>_<番号>" でなければ std::nullopt
std::optional<char> kind_from_frame(
  const std::string & frame, const std::string & prefix = kPieceFramePrefix);

// ソルバーの答えの各ピースに、同じ種類の見つけたピースを1個ずつ割り当てる。
// 同じ種類が複数あるときは、フレーム名の順（piece_L_0 → piece_L_1）に先頭から使う。
// 足りない種類があれば std::nullopt を返し、error に足りないものを書く（余ったピースは使わない）
std::optional<std::vector<size_t>> assign_pieces(
  const std::vector<Placement> & solution, const std::vector<DetectedPiece> & detected,
  std::string & error);

}  // namespace tukamukai

#endif  // TUKAMUKAI__PIECE_ASSIGNMENT_HPP_
