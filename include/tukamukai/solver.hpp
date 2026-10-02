// テトリミノで盤面を埋める配置を探すソルバー（ROS に依存しない）
//
// 座標は (行, 列)。左上が (0, 0) で、行は下向き、列は右向きに増える。
// 回転角は基準の向き（shape() の形）から上から見て時計回りに 0/90/180/270 度。
// ピースは裏返さない前提なので、鏡像（S と Z、L と J）は別の種類として扱う。
//
// 使い方:
//   tukamukai::PieceCounts pieces{{'I', 2}, {'O', 4}, {'T', 2}};
//   auto result = tukamukai::solve(8, 8, pieces, filled);  // 解がなければ std::nullopt
//   std::cout << tukamukai::render(8, 8, *result, filled);

#ifndef TUKAMUKAI__SOLVER_HPP_
#define TUKAMUKAI__SOLVER_HPP_

#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace tukamukai
{

// ピースの種類。テトリス標準色：I=水色、O=黄、T=紫、S=緑、Z=赤、J=青、L=オレンジ
inline constexpr std::array<char, 7> kKinds{'I', 'O', 'T', 'S', 'Z', 'J', 'L'};

struct Cell
{
  int row;
  int col;
  bool operator==(const Cell & other) const {return row == other.row && col == other.col;}
  bool operator<(const Cell & other) const
  {
    return row != other.row ? row < other.row : col < other.col;
  }
};

// 4 マスの形（左上を詰めて、行優先で並べたもの）
using Shape = std::array<Cell, 4>;

struct Placement
{
  char kind;                   // ピースの種類（'I', 'O', ...）
  int rotation;                // 回転角 [deg]（時計回り）
  std::array<Cell, 4> cells;   // 占めるマス

  // ピースの重心 (行, 列)（マス単位。マス (r, c) の中心を (r, c) とする）
  // 取っ手は重心の真上につくので、ロボットはこの位置をつかんで置く
  std::pair<double, double> center() const;
};

// {種類: 使える個数}
using PieceCounts = std::map<char, int>;

// 基準の向きでの形
Shape shape(char kind);

// 向きが異なる回転だけを (回転角, 形) で返す
// 対称な形は重複を除くので、O は 1 通り、I・S・Z は 2 通り、T・J・L は 4 通り
std::vector<std::pair<int, Shape>> orientations(char kind);

// rows × cols の盤面の、filled 以外のマスをすべて埋める配置を探す
// ピースは全部使い切らなくてよい（机の上に余ってよい）
// 解があれば Placement のリスト（探索で置いた順）、なければ std::nullopt を返す
// 盤面は 64 マスまで。知らない種類や大きすぎる盤面は std::invalid_argument を投げる
std::optional<std::vector<Placement>> solve(
  int rows, int cols, const PieceCounts & pieces, const std::set<Cell> & filled = {});

// 配置を文字の盤面にする（確認・デモ用）。埋めてあったマスは '#'、空きは '.'
std::string render(
  int rows, int cols, const std::vector<Placement> & placements,
  const std::set<Cell> & filled = {});

}  // namespace tukamukai

#endif  // TUKAMUKAI__SOLVER_HPP_
