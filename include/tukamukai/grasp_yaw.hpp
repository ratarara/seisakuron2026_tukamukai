// ピースの向きから、つかむとき・置くときの手の向き（yaw）を決める（#27）
// ROS に依存しない。角度はすべて度（deg）で、上から見て反時計回りが正（docs/interfaces.md と同じ）。
//
// - ピースの向き：ピースの基準の向き（ソルバーの形）からの回転角
// - 手の向き：手を真下に向けたときの、手首の付け根（crane_x7_gripper_base_link）の yaw
// - 手の向き = ピースの向き + hand_offset。実機の実験では、ピースの向き 0° を手の向き -90° で
//   つかんでいたので、hand_offset の初期値は -90°
// - 取っ手は 180° 回しても同じ形で、左右両方の指が動くので取っ手は指の中央に来る。
//   そのため、つかむ手の向きは 2 通りある
// - I・S・Z は 180°、O は 90° 回しても同じ形なので、置く手の向きも複数ある。T・J・L は 1 通り
// - ある手の向きに手が届くかは、手首の関節の可動範囲と腕の形で決まり、ここでは分からない。
//   そこで候補をすべて好ましい順に返し、使う側（ロボットのモデルを持つノード）が
//   届く最初の候補を選ぶ
//
// 使い方:
//   for (const tukamukai::GraspYaw & g : tukamukai::grasp_yaw_candidates('L', 0.0, 90.0)) {
//     // 届くなら、g.pick_hand でつかみ、g.wrist_turn だけ手首を回してから、g.place_hand で置く
//   }
#ifndef TUKAMUKAI__GRASP_YAW_HPP_
#define TUKAMUKAI__GRASP_YAW_HPP_

#include <vector>

namespace tukamukai
{

inline constexpr double kDefaultHandOffsetDeg = -90.0;  // 手の向き - ピースの向き
inline constexpr double kPreferredHandDeg = -90.0;      // 届くことが分かっている手の向き

// つかむ・置くときの手の向き [deg]
struct GraspYaw
{
  double pick_hand;   // つかむときの手の向き
  double place_hand;  // 置くときの手の向き
  double wrist_turn;  // 運ぶあいだに手首を回す量（place_hand - pick_hand、-180〜180）
  bool flipped;       // 取っ手を 180° 逆向きにつかんだか
};

// 角度を (-180, 180] にそろえる
double normalize_deg(double deg);

// ピースの形が、何度回すと同じ形に戻るか：O は 90、I・S・Z は 180、T・J・L は 360
double piece_symmetry_deg(char kind);

// ピースの種類と、つかむとき・置くときのピースの向きから、手の向きの候補をすべて返す。
// 好ましい順：手首を回す量が少ない → つかむ手の向きが preferred に近い → 置く手の向きが近い
std::vector<GraspYaw> grasp_yaw_candidates(
  char kind, double pick_piece_deg, double place_piece_deg,
  double hand_offset_deg = kDefaultHandOffsetDeg, bool handle_symmetric = true,
  double preferred_hand_deg = kPreferredHandDeg);

}  // namespace tukamukai

#endif  // TUKAMUKAI__GRASP_YAW_HPP_
