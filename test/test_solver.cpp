#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>

#include "tukamukai/solver.hpp"

using tukamukai::Cell;
using tukamukai::PieceCounts;
using tukamukai::Placement;

namespace
{

// 解が盤面をちょうど埋めていて、ピースの個数を超えていないか確かめる
void expect_valid(
  int rows, int cols, const PieceCounts & pieces, const std::set<Cell> & filled,
  const std::optional<std::vector<Placement>> & result)
{
  ASSERT_TRUE(result.has_value());
  std::set<Cell> covered = filled;
  std::map<char, int> used;
  for (const auto & p : *result) {
    // 置いたマスの形が、その回転角の形と一致する
    std::array<Cell, 4> cells = p.cells;
    int min_r = cells[0].row;
    int min_c = cells[0].col;
    for (const auto & c : cells) {
      min_r = std::min(min_r, c.row);
      min_c = std::min(min_c, c.col);
    }
    for (auto & c : cells) {
      c = Cell{c.row - min_r, c.col - min_c};
    }
    std::sort(cells.begin(), cells.end());
    bool matched = false;
    for (const auto & [rotation, shape] : tukamukai::orientations(p.kind)) {
      matched |= rotation == p.rotation && shape == cells;
    }
    EXPECT_TRUE(matched);

    for (const auto & c : p.cells) {
      EXPECT_TRUE(0 <= c.row && c.row < rows && 0 <= c.col && c.col < cols);
      EXPECT_TRUE(covered.insert(c).second);  // 重なっていない
    }
    ++used[p.kind];
  }
  EXPECT_EQ(static_cast<int>(covered.size()), rows * cols);
  for (const auto & [kind, n] : used) {
    EXPECT_LE(n, pieces.at(kind));
  }
}

}  // namespace

TEST(Solver, ShapesHaveFourCells)
{
  for (char kind : tukamukai::kKinds) {
    const auto s = tukamukai::shape(kind);
    EXPECT_EQ(std::set<Cell>(s.begin(), s.end()).size(), 4u);
  }
}

TEST(Solver, OrientationCounts)
{
  const std::map<char, std::size_t> expected{
    {'I', 2}, {'O', 1}, {'T', 4}, {'S', 2}, {'Z', 2}, {'J', 4}, {'L', 4}};
  for (const auto & [kind, n] : expected) {
    EXPECT_EQ(tukamukai::orientations(kind).size(), n) << kind;
  }
}

TEST(Solver, RotationIsClockwise)
{
  // T の基準は下向きの出っ張り。時計回りに 90 度回すと左向きになる
  const auto o = tukamukai::orientations('T');
  ASSERT_EQ(o[1].first, 90);
  const tukamukai::Shape expected{{{0, 1}, {1, 0}, {1, 1}, {2, 1}}};
  EXPECT_EQ(o[1].second, expected);
}

TEST(Solver, CenterIsCentroid)
{
  const Placement p{'T', 0, {{{0, 0}, {0, 1}, {0, 2}, {1, 1}}}};
  EXPECT_DOUBLE_EQ(p.center().first, 0.25);
  EXPECT_DOUBLE_EQ(p.center().second, 1.0);
}

TEST(Solver, Solve8x8Mixed)
{
  const PieceCounts pieces{{'I', 2}, {'O', 2}, {'T', 4}, {'S', 2}, {'Z', 2}, {'J', 2}, {'L', 2}};
  expect_valid(8, 8, pieces, {}, tukamukai::solve(8, 8, pieces));
}

TEST(Solver, SolveWithFilledCells)
{
  std::set<Cell> filled;  // 上 2 行は最初から埋まっている
  for (int r = 0; r < 2; ++r) {
    for (int c = 0; c < 8; ++c) {
      filled.insert({r, c});
    }
  }
  const PieceCounts pieces{{'I', 2}, {'O', 2}, {'T', 2}, {'S', 2}, {'Z', 2}, {'L', 2}};
  expect_valid(8, 8, pieces, filled, tukamukai::solve(8, 8, pieces, filled));
}

TEST(Solver, ExtraPiecesAreAllowed)
{
  const PieceCounts pieces{{'O', 5}, {'I', 3}};
  const auto result = tukamukai::solve(4, 4, pieces);
  expect_valid(4, 4, pieces, {}, result);
  EXPECT_EQ(result->size(), 4u);
}

TEST(Solver, OddTCannotFill8x8)
{
  EXPECT_FALSE(tukamukai::solve(8, 8, {{'T', 1}, {'O', 15}}).has_value());
}

TEST(Solver, OddJLWithoutTCannotFill8x8)
{
  const PieceCounts pieces{{'I', 1}, {'O', 5}, {'S', 2}, {'Z', 5}, {'J', 1}, {'L', 2}};
  EXPECT_FALSE(tukamukai::solve(8, 8, pieces).has_value());
}

TEST(Solver, MirrorIsNotUsed)
{
  // S の形の穴は、Z を裏返せば埋まるが、回転だけでは埋まらない
  const std::set<Cell> filled{{0, 0}, {1, 2}};
  EXPECT_FALSE(tukamukai::solve(2, 3, {{'Z', 1}}, filled).has_value());
  EXPECT_TRUE(tukamukai::solve(2, 3, {{'S', 1}}, filled).has_value());
}

TEST(Solver, NotEnoughPieces)
{
  EXPECT_FALSE(tukamukai::solve(4, 4, {{'O', 3}}).has_value());
}

TEST(Solver, InvalidArguments)
{
  EXPECT_THROW(tukamukai::solve(4, 4, {{'X', 4}}), std::invalid_argument);
  EXPECT_THROW(tukamukai::solve(9, 8, {{'O', 18}}), std::invalid_argument);
}
TEST(Solver, Render)
{
  const std::set<Cell> filled{{0, 0}, {0, 1}, {0, 2}, {0, 3}};
  const auto result = tukamukai::solve(2, 4, {{'I', 1}}, filled);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(tukamukai::render(2, 4, *result, filled), "# # # #\nI I I I");
}
TEST(Solver, Solve4x4WithISLL)
{
  const PieceCounts pieces{{'I', 1}, {'S', 1}, {'L', 2}};
  expect_valid(4, 4, pieces, {}, tukamukai::solve(4, 4, pieces));
}

TEST(Solver, CannotSolve4x4WithISSL)
{
  const PieceCounts pieces{{'I', 1}, {'S', 2}, {'L', 1}};
  EXPECT_FALSE(tukamukai::solve(4, 4, pieces).has_value());
}
