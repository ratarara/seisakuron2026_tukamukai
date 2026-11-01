#include <gtest/gtest.h>

#include <cmath>

#include "tukamukai/grasp_yaw.hpp"

using tukamukai::grasp_yaw_candidates;
using tukamukai::GraspYaw;
using tukamukai::normalize_deg;
using tukamukai::piece_symmetry_deg;

namespace
{

constexpr double kEps = 1e-9;

// 置いたピースの向き = 置く手の向き - hand_offset（逆向きにつかんだら、さらに 180° ずれる）
double placed_piece_deg(const GraspYaw & g, const double hand_offset = -90.0)
{
  return normalize_deg(g.place_hand - hand_offset - (g.flipped ? 180.0 : 0.0));
}

}  // namespace

TEST(GraspYaw, NormalizeDeg)
{
  EXPECT_NEAR(normalize_deg(0.0), 0.0, kEps);
  EXPECT_NEAR(normalize_deg(180.0), 180.0, kEps);
  EXPECT_NEAR(normalize_deg(-180.0), 180.0, kEps);
  EXPECT_NEAR(normalize_deg(270.0), -90.0, kEps);
  EXPECT_NEAR(normalize_deg(-450.0), -90.0, kEps);
}

TEST(GraspYaw, Symmetry)
{
  EXPECT_EQ(piece_symmetry_deg('O'), 90.0);
  EXPECT_EQ(piece_symmetry_deg('I'), 180.0);
  EXPECT_EQ(piece_symmetry_deg('S'), 180.0);
  EXPECT_EQ(piece_symmetry_deg('Z'), 180.0);
  EXPECT_EQ(piece_symmetry_deg('T'), 360.0);
  EXPECT_EQ(piece_symmetry_deg('J'), 360.0);
  EXPECT_EQ(piece_symmetry_deg('L'), 360.0);
}

// 今までの実験と同じ：ピースの向き 0° は、手の向き -90° でつかんで、そのまま置く
TEST(GraspYaw, SameAsExperiment)
{
  const GraspYaw g = grasp_yaw_candidates('L', 0.0, 0.0).front();
  EXPECT_NEAR(g.pick_hand, -90.0, kEps);
  EXPECT_NEAR(g.place_hand, -90.0, kEps);
  EXPECT_NEAR(g.wrist_turn, 0.0, kEps);
  EXPECT_FALSE(g.flipped);
}

// 候補の数：取っ手の 2 通り × ピースの形の対称性
TEST(GraspYaw, NumberOfCandidates)
{
  EXPECT_EQ(grasp_yaw_candidates('L', 0.0, 0.0).size(), 2u);
  EXPECT_EQ(grasp_yaw_candidates('I', 0.0, 0.0).size(), 4u);
  EXPECT_EQ(grasp_yaw_candidates('O', 0.0, 0.0).size(), 8u);
  EXPECT_EQ(grasp_yaw_candidates('L', 0.0, 0.0, -90.0, false).size(), 1u);
}

// 手の向き 0° が届かないときのために、180° でつかむ候補も必ず入っている
TEST(GraspYaw, BothGraspsAreCandidates)
{
  const auto candidates = grasp_yaw_candidates('L', 90.0, 0.0);
  ASSERT_EQ(candidates.size(), 2u);
  EXPECT_NEAR(std::abs(normalize_deg(candidates[0].pick_hand - candidates[1].pick_hand)), 180.0,
    kEps);
  EXPECT_NE(candidates[0].flipped, candidates[1].flipped);
}

// 先頭の候補は、取っ手の向きを選べるなら -90° に近い（-180〜0° の範囲）
TEST(GraspYaw, FirstCandidateIsNearPreferred)
{
  for (const double piece : {0.0, 90.0, 180.0, 270.0, 45.0, -135.0}) {
    const GraspYaw g = grasp_yaw_candidates('L', piece, 0.0).front();
    EXPECT_LE(std::abs(normalize_deg(g.pick_hand + 90.0)), 90.0 + kEps) << "piece=" << piece;
  }
  // ピースの向き 180° は、手の向き 90° ではなく、取っ手を逆からつかんで -90°
  const GraspYaw g = grasp_yaw_candidates('L', 180.0, 0.0).front();
  EXPECT_NEAR(g.pick_hand, -90.0, kEps);
  EXPECT_TRUE(g.flipped);
}

// 向きが 1 通りのピース（L）は、どの候補でも、置いたピースの向きが目標どおりになる
TEST(GraspYaw, AsymmetricPieceIsPlacedExactly)
{
  for (const double pick : {0.0, 90.0, 180.0, 270.0}) {
    for (const double place : {0.0, 90.0, 180.0, 270.0}) {
      for (const GraspYaw & g : grasp_yaw_candidates('L', pick, place)) {
        EXPECT_NEAR(normalize_deg(placed_piece_deg(g) - place), 0.0, kEps)
          << "pick=" << pick << " place=" << place;
        EXPECT_NEAR(g.wrist_turn, normalize_deg(g.place_hand - g.pick_hand), kEps);
      }
    }
  }
}

// I は 180° 回しても同じ形なので、先頭の候補の手首を回す量は 90° 以内。
// どの候補でも、置いた向きは目標と同じか 180° 違う（I では同じ形）
TEST(GraspYaw, SymmetricPieceTurnsLess)
{
  for (const double pick : {0.0, 90.0, 180.0, 270.0}) {
    for (const double place : {0.0, 90.0, 180.0, 270.0}) {
      const auto candidates = grasp_yaw_candidates('I', pick, place);
      EXPECT_LE(std::abs(candidates.front().wrist_turn), 90.0 + kEps)
        << "pick=" << pick << " place=" << place;
      for (const GraspYaw & g : candidates) {
        const double diff = std::abs(normalize_deg(placed_piece_deg(g) - place));
        EXPECT_TRUE(diff < kEps || std::abs(diff - 180.0) < kEps)
          << "pick=" << pick << " place=" << place;
      }
    }
  }
  // O は 90° ごとに同じ形なので、手首は回さない
  EXPECT_NEAR(grasp_yaw_candidates('O', 0.0, 90.0).front().wrist_turn, 0.0, kEps);
}

// 候補は、手首を回す量が少ない順に並ぶ
TEST(GraspYaw, CandidatesAreSortedByWristTurn)
{
  const auto candidates = grasp_yaw_candidates('O', 0.0, 90.0);
  for (size_t i = 1; i < candidates.size(); ++i) {
    EXPECT_LE(std::abs(candidates[i - 1].wrist_turn), std::abs(candidates[i].wrist_turn) + kEps);
  }
}
