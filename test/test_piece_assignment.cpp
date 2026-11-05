#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "tukamukai/piece_assignment.hpp"
#include "tukamukai/solver.hpp"

using tukamukai::DetectedPiece;

TEST(KindFromFrame, ReadsKind)
{
  EXPECT_EQ(tukamukai::kind_from_frame("piece_I_0"), 'I');
  EXPECT_EQ(tukamukai::kind_from_frame("piece_L_12"), 'L');
}

TEST(KindFromFrame, RejectsOtherFrames)
{
  EXPECT_FALSE(tukamukai::kind_from_frame("base_link"));
  EXPECT_FALSE(tukamukai::kind_from_frame("piece_X_0"));   // 知らない種類
  EXPECT_FALSE(tukamukai::kind_from_frame("piece_L"));     // 番号がない
  EXPECT_FALSE(tukamukai::kind_from_frame("piece_LL_0"));
  EXPECT_FALSE(tukamukai::kind_from_frame("piece_L_a"));
  EXPECT_FALSE(tukamukai::kind_from_frame("target_0"));
}

TEST(AssignPieces, MatchesKindsForMidtermBoard)
{
  // 中間発表の盤面：4×4 を I・S・L・L で埋める
  const auto solution = tukamukai::solve(4, 4, {{'I', 1}, {'S', 1}, {'L', 2}});
  ASSERT_TRUE(solution.has_value());
  // 認識の結果は、ソルバーの答えとは関係ない順で届く
  const std::vector<DetectedPiece> detected{
    {'L', 0.1, 0.0, 0.0, "piece_L_1"},
    {'S', 0.2, 0.0, 0.0, "piece_S_0"},
    {'L', 0.3, 0.0, 0.0, "piece_L_0"},
    {'I', 0.4, 0.0, 0.0, "piece_I_0"},
  };
  std::string error;
  const auto order = tukamukai::assign_pieces(*solution, detected, error);
  ASSERT_TRUE(order.has_value()) << error;
  ASSERT_EQ(order->size(), solution->size());

  std::vector<bool> used(detected.size(), false);
  bool first_l = true;
  for (size_t i = 0; i < solution->size(); ++i) {
    const auto & d = detected[(*order)[i]];
    EXPECT_EQ(d.kind, (*solution)[i].kind);
    EXPECT_FALSE(used[(*order)[i]]);  // 同じピースを2回使わない
    used[(*order)[i]] = true;
    if (d.kind == 'L') {
      // L はフレーム名の順に使う
      EXPECT_EQ(d.frame, first_l ? "piece_L_0" : "piece_L_1");
      first_l = false;
    }
  }
}

TEST(AssignPieces, ReportsMissingKinds)
{
  const auto solution = tukamukai::solve(4, 4, {{'I', 1}, {'S', 1}, {'L', 2}});
  ASSERT_TRUE(solution.has_value());
  const std::vector<DetectedPiece> detected{
    {'L', 0.1, 0.0, 0.0, "piece_L_0"},
    {'I', 0.4, 0.0, 0.0, "piece_I_0"},
  };
  std::string error;
  EXPECT_FALSE(tukamukai::assign_pieces(*solution, detected, error).has_value());
  EXPECT_NE(error.find("L: need 2, found 1"), std::string::npos) << error;
  EXPECT_NE(error.find("S: need 1, found 0"), std::string::npos) << error;
}

TEST(AssignPieces, IgnoresExtraPieces)
{
  const auto solution = tukamukai::solve(2, 4, {{'I', 2}});
  ASSERT_TRUE(solution.has_value());
  const std::vector<DetectedPiece> detected{
    {'I', 0.1, 0.0, 0.0, "piece_I_0"},
    {'O', 0.2, 0.0, 0.0, "piece_O_0"},
    {'I', 0.3, 0.0, 0.0, "piece_I_1"},
    {'I', 0.4, 0.0, 0.0, "piece_I_2"},
  };
  std::string error;
  const auto order = tukamukai::assign_pieces(*solution, detected, error);
  ASSERT_TRUE(order.has_value()) << error;
  EXPECT_EQ(*order, (std::vector<size_t>{0, 2}));
}
