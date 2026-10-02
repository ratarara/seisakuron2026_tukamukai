#include "tukamukai/solver.hpp"

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <unordered_set>

namespace tukamukai
{

namespace
{

// 盤面は 64 ビットのマスクで表す（マス (r, c) は r * cols + c ビット目）
using Mask = std::uint64_t;

int kind_index(char kind)
{
  const auto it = std::find(kKinds.begin(), kKinds.end(), kind);
  if (it == kKinds.end()) {
    throw std::invalid_argument(std::string("unknown piece kind: ") + kind);
  }
  return static_cast<int>(it - kKinds.begin());
}

Shape normalize(std::array<Cell, 4> cells)
{
  int min_r = cells[0].row;
  int min_c = cells[0].col;
  for (const auto & c : cells) {
    min_r = std::min(min_r, c.row);
    min_c = std::min(min_c, c.col);
  }
  for (auto & c : cells) {
    c.row -= min_r;
    c.col -= min_c;
  }
  std::sort(cells.begin(), cells.end());
  return cells;
}

int popcount(Mask m) {return static_cast<int>(std::bitset<64>(m).count());}

// 64 以上ずらすと未定義動作になるので 0 にする
Mask shift_left(Mask m, int n) {return n >= 64 ? 0 : m << n;}
Mask shift_right(Mask m, int n) {return n >= 64 ? 0 : m >> n;}

class Solver
{
public:
  Solver(int rows, int cols, const PieceCounts & pieces, const std::set<Cell> & filled)
  : rows_(rows), cols_(cols)
  {
    const int n = rows * cols;
    full_ = n == 64 ? ~Mask{0} : (Mask{1} << n) - 1;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const Mask bit = Mask{1} << (r * cols + c);
        if ((r + c) % 2 == 0) {black_ |= bit;}
        if (r % 2 == 0) {even_row_ |= bit;}
        if (c % 2 == 0) {even_col_ |= bit;}
        if (c == 0) {first_col_ |= bit;}
        if (c == cols - 1) {last_col_ |= bit;}
      }
    }
    empty_ = full_;
    for (const auto & cell : filled) {
      if (0 <= cell.row && cell.row < rows && 0 <= cell.col && cell.col < cols) {
        empty_ &= ~bit(cell);
      }
    }
    // 使える個数は必要なピース数で頭打ちにする（記録する状態を小さくするため）
    const int need = popcount(empty_) / 4;
    remaining_.fill(0);
    for (const auto & [kind, count] : pieces) {
      remaining_[kind_index(kind)] = std::clamp(count, 0, need);
    }

    // マスごとに、そのマスを覆う置き方の候補をすべて用意しておく
    covering_.resize(n);
    for (int k = 0; k < static_cast<int>(kKinds.size()); ++k) {
      if (remaining_[k] == 0) {continue;}
      for (const auto & [rotation, shape] : orientations(kKinds[k])) {
        int height = 0;
        int width = 0;
        for (const auto & c : shape) {
          height = std::max(height, c.row + 1);
          width = std::max(width, c.col + 1);
        }
        for (int dr = 0; dr + height <= rows; ++dr) {
          for (int dc = 0; dc + width <= cols; ++dc) {
            Candidate cand{k, Placement{kKinds[k], rotation, shape}, 0};
            for (auto & c : cand.placement.cells) {
              c.row += dr;
              c.col += dc;
              cand.mask |= bit(c);
            }
            if ((cand.mask & empty_) != cand.mask) {continue;}
            const int index = static_cast<int>(candidates_.size());
            candidates_.push_back(cand);
            for (const auto & c : cand.placement.cells) {
              covering_[c.row * cols + c.col].push_back(index);
            }
          }
        }
      }
    }
  }

  std::optional<std::vector<Placement>> run()
  {
    if (!search()) {return std::nullopt;}
    std::vector<Placement> result;
    for (int index : placed_) {
      result.push_back(candidates_[index].placement);
    }
    return result;
  }

private:
  struct Candidate
  {
    int kind;
    Placement placement;
    Mask mask;
  };

  struct State
  {
    Mask empty;
    std::uint64_t counts;
    bool operator==(const State & other) const
    {
      return empty == other.empty && counts == other.counts;
    }
  };

  struct StateHash
  {
    std::size_t operator()(const State & s) const
    {
      return std::hash<Mask>()(s.empty) ^ (std::hash<std::uint64_t>()(s.counts) * 31);
    }
  };

  Mask bit(const Cell & c) const {return Mask{1} << (c.row * cols_ + c.col);}

  // 盤面の (偶数側のマス数 - 奇数側のマス数)
  int diff(Mask side) const {return 2 * popcount(empty_ & side) - popcount(empty_);}

  // 明らかに解けない条件を弾く（探索の途中でも、残りの空きマスとピースで毎回確かめる）
  // - 空きマス数が 4 の倍数で、ピースが足りている
  // - 市松に塗ると T は 3:1、ほかは 2:2。T の使用数 t について
  //   |黒白の差| <= 2t かつ 黒白の差 ≡ 2t (mod 4)
  // - 行の縞で塗ると、J・L はどの向きでも差 ±2、T は横向きだけ ±2、I は横向きだけ ±4、
  //   S・Z・O は 0。列の縞では T・I の縦横が入れ替わる。よって
  //   (J+L の数) + (横向きの T の数) ≡ 行の差 / 2 (mod 2)
  //   (J+L の数) + (縦向きの T の数) ≡ 列の差 / 2 (mod 2)
  //   T を使うなら t ≡ (行の差 + 列の差) / 2、使わないなら J+L ≡ 行の差 / 2 ≡ 列の差 / 2。
  //   8×8 で T がなければ、J+L は偶数でないと埋まらない
  bool feasible() const
  {
    const int n_empty = popcount(empty_);
    const int need = n_empty / 4;
    int total = 0;
    for (int count : remaining_) {total += count;}
    if (n_empty % 4 != 0 || total < need) {return false;}
    const int row_diff = diff(even_row_);
    const int col_diff = diff(even_col_);
    if (row_diff % 2 != 0 || col_diff % 2 != 0) {return false;}
    const int row_parity = std::abs(row_diff / 2) % 2;
    const int col_parity = std::abs(col_diff / 2) % 2;
    const int imbalance = std::abs(diff(black_));
    const int n_t = remaining_[kind_index('T')];
    const int n_jl = remaining_[kind_index('J')] + remaining_[kind_index('L')];
    const int n_others = total - n_t - n_jl;
    for (int t = 0; t <= std::min(n_t, need); ++t) {
      if (imbalance > 2 * t || (imbalance - 2 * t) % 4 != 0) {continue;}
      if (t > 0 && t % 2 != (row_parity + col_parity) % 2) {continue;}
      // J・L の使用数 jl は、残りを I・O・S・Z で埋められる範囲
      for (int jl = std::max(0, need - t - n_others); jl <= std::min(n_jl, need - t); ++jl) {
        if (t > 0 || (jl % 2 == row_parity && jl % 2 == col_parity)) {return true;}
      }
    }
    return false;
  }

  // 空きマスの連結成分がすべて 4 の倍数かどうか（4 の倍数でない島は絶対に埋まらない）
  bool regions_ok() const
  {
    Mask rest = empty_;
    while (rest) {
      Mask region = rest & (~rest + 1);  // いちばん下のビット
      for (;; ) {
        const Mask grown = (region | shift_left(region, cols_) | shift_right(region, cols_) |
          ((region & ~last_col_) << 1) | ((region & ~first_col_) >> 1)) & rest;
        if (grown == region) {break;}
        region = grown;
      }
      if (popcount(region) % 4 != 0) {return false;}
      rest &= ~region;
    }
    return true;
  }

  bool usable(int index) const
  {
    const auto & cand = candidates_[index];
    return remaining_[cand.kind] > 0 && (cand.mask & empty_) == cand.mask;
  }

  bool search()
  {
    if (empty_ == 0) {return true;}
    std::uint64_t counts = 0;
    for (int count : remaining_) {counts = (counts << 8) | static_cast<std::uint64_t>(count);}
    const State state{empty_, counts};
    if (failed_.count(state) || !feasible()) {return false;}

    // 置ける候補がいちばん少ないマスから埋める（0 個のマスがあれば行き詰まり）
    int best_cell = -1;
    int best_count = 0;
    for (Mask rest = empty_; rest; rest &= rest - 1) {
      const int cell = __builtin_ctzll(rest);
      int count = 0;
      for (int index : covering_[cell]) {
        if (usable(index)) {++count;}
      }
      if (best_cell < 0 || count < best_count) {
        best_cell = cell;
        best_count = count;
        if (count == 0) {break;}
      }
    }

    for (int index : covering_[best_cell]) {
      if (!usable(index)) {continue;}
      const auto & cand = candidates_[index];
      empty_ &= ~cand.mask;
      --remaining_[cand.kind];
      placed_.push_back(index);
      if (regions_ok() && search()) {return true;}
      placed_.pop_back();
      ++remaining_[cand.kind];
      empty_ |= cand.mask;
    }
    failed_.insert(state);
    return false;
  }

  int rows_;
  int cols_;
  Mask full_ = 0;
  Mask black_ = 0;
  Mask even_row_ = 0;
  Mask even_col_ = 0;
  Mask first_col_ = 0;
  Mask last_col_ = 0;
  Mask empty_ = 0;
  std::array<int, kKinds.size()> remaining_{};
  std::vector<Candidate> candidates_;
  std::vector<std::vector<int>> covering_;
  std::vector<int> placed_;
  std::unordered_set<State, StateHash> failed_;  // 解けないと分かった状態
};

}  // namespace

std::pair<double, double> Placement::center() const
{
  double r = 0.0;
  double c = 0.0;
  for (const auto & cell : cells) {
    r += cell.row;
    c += cell.col;
  }
  return {r / cells.size(), c / cells.size()};
}

Shape shape(char kind)
{
  switch (kind) {
    case 'I': return {{{0, 0}, {0, 1}, {0, 2}, {0, 3}}};
    case 'O': return {{{0, 0}, {0, 1}, {1, 0}, {1, 1}}};
    case 'T': return {{{0, 0}, {0, 1}, {0, 2}, {1, 1}}};
    case 'S': return {{{0, 1}, {0, 2}, {1, 0}, {1, 1}}};
    case 'Z': return {{{0, 0}, {0, 1}, {1, 1}, {1, 2}}};
    case 'J': return {{{0, 0}, {1, 0}, {1, 1}, {1, 2}}};
    case 'L': return {{{0, 2}, {1, 0}, {1, 1}, {1, 2}}};
    default: throw std::invalid_argument(std::string("unknown piece kind: ") + kind);
  }
}

std::vector<std::pair<int, Shape>> orientations(char kind)
{
  std::vector<std::pair<int, Shape>> result;
  Shape cells = shape(kind);
  for (int k = 0; k < 4; ++k) {
    const Shape normalized = normalize(cells);
    const bool seen = std::any_of(
      result.begin(), result.end(), [&](const auto & o) {return o.second == normalized;});
    if (!seen) {result.emplace_back(90 * k, normalized);}
    for (auto & c : cells) {
      c = Cell{c.col, -c.row};  // 時計回りに 90 度
    }
  }
  return result;
}

std::optional<std::vector<Placement>> solve(
  int rows, int cols, const PieceCounts & pieces, const std::set<Cell> & filled)
{
  if (rows <= 0 || cols <= 0 || rows * cols > 64) {
    throw std::invalid_argument("board must have 1 to 64 cells");
  }
  Solver solver(rows, cols, pieces, filled);
  return solver.run();
}

std::string render(
  int rows, int cols, const std::vector<Placement> & placements, const std::set<Cell> & filled)
{
  std::vector<std::string> grid(rows, std::string(cols, '.'));
  for (const auto & c : filled) {
    if (0 <= c.row && c.row < rows && 0 <= c.col && c.col < cols) {
      grid[c.row][c.col] = '#';
    }
  }
  for (const auto & p : placements) {
    for (const auto & c : p.cells) {
      grid[c.row][c.col] = p.kind;
    }
  }
  std::string out;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (c > 0) {out += ' ';}
      out += grid[r][c];
    }
    if (r + 1 < rows) {out += '\n';}
  }
  return out;
}

}  // namespace tukamukai
