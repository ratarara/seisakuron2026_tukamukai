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

#include "tukamukai/board_geometry.hpp"
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

// (x, y)（base_link）が、rows × cols マスのケースの中（周りに margin [m] 広げた範囲）にあるか
bool in_case(
  double x, double y, const CaseFrame & frame, int rows, int cols, double margin);

// 見つけたピースから、種類 kind で、ケースの外にあるものを 1 つ選ぶ（フレーム名の順で最初のもの）。
// ケースにすでに置いたピースを、もう一度つかまないようにするため。なければ std::nullopt
std::optional<size_t> choose_piece(
  const std::vector<DetectedPiece> & detected, char kind, const CaseFrame & frame, int rows,
  int cols, double margin);

}  // namespace tukamukai

#endif  // TUKAMUKAI__PIECE_ASSIGNMENT_HPP_
