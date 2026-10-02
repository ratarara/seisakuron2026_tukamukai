// ソルバーの動作確認用。8×8 の例を解いて盤面と各ピースの配置を表示する
//   ros2 run tukamukai solver_demo

#include <chrono>
#include <cstdio>
#include <iostream>

#include "tukamukai/solver.hpp"

int main()
{
  const tukamukai::PieceCounts pieces{
    {'I', 2}, {'O', 2}, {'T', 4}, {'S', 2}, {'Z', 2}, {'J', 2}, {'L', 2}};

  const auto start = std::chrono::steady_clock::now();
  const auto result = tukamukai::solve(8, 8, pieces);
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;

  if (!result) {
    std::cout << "解なし" << std::endl;
    return 1;
  }
  std::cout << tukamukai::render(8, 8, *result) << "\n\n";
  for (const auto & p : *result) {
    const auto [r, c] = p.center();
    std::printf("%c rot=%3d center=(%.2f, %.2f)\n", p.kind, p.rotation, r, c);
  }
  std::printf("%.4f 秒\n", elapsed.count());
  return 0;
}
