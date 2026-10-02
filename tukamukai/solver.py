# テトリミノで盤面を埋める配置を探すソルバー（ROS に依存しない）
#
# 座標は (行, 列)。左上が (0, 0) で、行は下向き、列は右向きに増える。
# 回転角は基準の向き（SHAPES の形）から上から見て時計回りに 0/90/180/270 度。
# ピースは裏返さない前提なので、鏡像（S と Z、L と J）は別の種類として扱う。
#
# 使い方:
#   placements = solve(8, 8, {'I': 2, 'O': 4, 'T': 2, ...}, filled={(0, 0), ...})
#   print(render(8, 8, placements, filled))

from dataclasses import dataclass

# 基準の向きでの形。テトリス標準色：I=水色、O=黄、T=紫、S=緑、Z=赤、J=青、L=オレンジ
SHAPES = {
    'I': ((0, 0), (0, 1), (0, 2), (0, 3)),
    'O': ((0, 0), (0, 1), (1, 0), (1, 1)),
    'T': ((0, 0), (0, 1), (0, 2), (1, 1)),
    'S': ((0, 1), (0, 2), (1, 0), (1, 1)),
    'Z': ((0, 0), (0, 1), (1, 1), (1, 2)),
    'J': ((0, 0), (1, 0), (1, 1), (1, 2)),
    'L': ((0, 2), (1, 0), (1, 1), (1, 2)),
}


@dataclass(frozen=True)
class Placement:
    kind: str                   # ピースの種類（'I', 'O', ...）
    rotation: int               # 回転角 [deg]（時計回り）
    cells: tuple                # 占めるマス ((行, 列), ...)

    # ピースの重心（マス単位。マス (r, c) の中心を (r, c) とする）
    # 取っ手は重心の真上につくので、ロボットはこの位置をつかんで置く
    @property
    def center(self):
        n = len(self.cells)
        return (sum(r for r, _ in self.cells) / n,
                sum(c for _, c in self.cells) / n)


def _normalize(cells):
    min_r = min(r for r, _ in cells)
    min_c = min(c for _, c in cells)
    return tuple(sorted((r - min_r, c - min_c) for r, c in cells))


# 種類ごとに、向きが異なる回転だけを (回転角, 形) のリストで返す
# 対称な形は重複を除くので、O は 1 通り、I・S・Z は 2 通り、T・J・L は 4 通り
def orientations(kind):
    result = []
    seen = set()
    cells = SHAPES[kind]
    for k in range(4):
        shape = _normalize(cells)
        if shape not in seen:
            seen.add(shape)
            result.append((90 * k, shape))
        cells = tuple((c, -r) for r, c in cells)  # 時計回りに 90 度
    return result


# 市松模様で塗ったときの (黒マス数 - 白マス数)
def _color_imbalance(cells):
    return sum(1 if (r + c) % 2 == 0 else -1 for r, c in cells)


# 行ごと・列ごとに縞模様で塗ったときの (偶数番目 - 奇数番目) のマス数
def _stripe_imbalance(cells, axis):
    return sum(1 if cell[axis] % 2 == 0 else -1 for cell in cells)


# 明らかに解けない条件を弾く（探索の途中でも、残りの空きマスとピースで毎回確かめる）
# - 空きマス数が 4 の倍数で、ピースが足りている
# - 市松に塗ると T は 3:1、ほかは 2:2。T の使用数 t について
#   |黒白の差| <= 2t かつ 黒白の差 ≡ 2t (mod 4)
# - 行の縞で塗ると、J・L はどの向きでも差 ±2、T は横向きだけ ±2、I は横向きだけ ±4、
#   S・Z・O は 0。列の縞では T・I の縦横が入れ替わる。よって
#   (J+L の数) + (横向きの T の数) ≡ 行の差 / 2 (mod 2)
#   (J+L の数) + (縦向きの T の数) ≡ 列の差 / 2 (mod 2)
#   T を使うなら t ≡ (行の差 + 列の差) / 2、使わないなら J+L ≡ 行の差 / 2 ≡ 列の差 / 2。
#   8×8 で T がなければ、J+L は偶数でないと埋まらない
def _feasible(empty, pieces):
    n_empty = len(empty)
    need = n_empty // 4
    if n_empty % 4 != 0 or sum(pieces.values()) < need:
        return False
    row_diff = _stripe_imbalance(empty, 0)
    col_diff = _stripe_imbalance(empty, 1)
    if row_diff % 2 != 0 or col_diff % 2 != 0:
        return False
    row_parity = row_diff // 2 % 2
    col_parity = col_diff // 2 % 2
    imbalance = abs(_color_imbalance(empty))
    n_t = pieces.get('T', 0)
    n_jl = pieces.get('J', 0) + pieces.get('L', 0)
    n_others = sum(pieces.values()) - n_t - n_jl
    for t in range(min(n_t, need) + 1):
        if imbalance > 2 * t or (imbalance - 2 * t) % 4 != 0:
            continue
        if t > 0 and t % 2 != (row_parity + col_parity) % 2:
            continue
        # J・L の使用数 jl は、残りを I・O・S・Z で埋められる範囲
        for jl in range(max(0, need - t - n_others), min(n_jl, need - t) + 1):
            if t > 0 or jl % 2 == row_parity == col_parity:
                return True
    return False


# 空きマスの連結成分がすべて 4 の倍数かどうか（4 の倍数でない島は絶対に埋まらない）
def _regions_ok(empty):
    unvisited = set(empty)
    while unvisited:
        stack = [unvisited.pop()]
        size = 0
        while stack:
            r, c = stack.pop()
            size += 1
            for nb in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if nb in unvisited:
                    unvisited.remove(nb)
                    stack.append(nb)
        if size % 4 != 0:
            return False
    return True


# rows × cols の盤面の、filled 以外のマスをすべて埋める配置を探す
# pieces は {種類: 使える個数}。全部使い切る必要はない（机の上に余ってよい）
# 解があれば Placement のリスト（置く順）、なければ None を返す
def solve(rows, cols, pieces, filled=frozenset()):
    for kind in pieces:
        if kind not in SHAPES:
            raise ValueError(f'unknown piece kind: {kind}')
    empty = {(r, c) for r in range(rows) for c in range(cols)} - set(filled)
    remaining = {k: v for k, v in pieces.items() if v > 0}
    if not _feasible(empty, remaining):
        return None

    # マスごとに、そのマスを覆う置き方の候補をすべて用意しておく
    covering = {cell: [] for cell in empty}
    for kind in remaining:
        for rotation, shape in orientations(kind):
            height = max(r for r, _ in shape) + 1
            width = max(c for _, c in shape) + 1
            for dr in range(rows - height + 1):
                for dc in range(cols - width + 1):
                    cells = tuple((r + dr, c + dc) for r, c in shape)
                    if all(cell in empty for cell in cells):
                        p = Placement(kind, rotation, cells)
                        for cell in cells:
                            covering[cell].append(p)

    placements = []
    failed = set()  # 解けないと分かった (空きマス, 残り個数) の状態

    def search():
        if not empty:
            return True
        state = (frozenset(empty), tuple(sorted(remaining.items())))
        if state in failed or not _feasible(empty, remaining):
            return False
        # 置ける候補がいちばん少ないマスから埋める（0 個のマスがあれば行き詰まり）
        best = None
        for cell in empty:
            options = [p for p in covering[cell]
                       if remaining[p.kind] > 0 and all(c in empty for c in p.cells)]
            if best is None or len(options) < len(best):
                best = options
                if not best:
                    break
        for p in best:
            empty.difference_update(p.cells)
            remaining[p.kind] -= 1
            placements.append(p)
            if _regions_ok(empty) and search():
                return True
            placements.pop()
            remaining[p.kind] += 1
            empty.update(p.cells)
        failed.add(state)
        return False

    return list(placements) if search() else None


# 配置を文字の盤面にする（確認・デモ用）。埋めてあったマスは '#'、空きは '.'
def render(rows, cols, placements, filled=frozenset()):
    grid = [['.'] * cols for _ in range(rows)]
    for r, c in filled:
        grid[r][c] = '#'
    for p in placements or []:
        for r, c in p.cells:
            grid[r][c] = p.kind
    return '\n'.join(' '.join(row) for row in grid)


if __name__ == '__main__':
    example = {'I': 2, 'O': 2, 'T': 4, 'S': 2, 'Z': 2, 'J': 2, 'L': 2}
    result = solve(8, 8, example)
    if result is None:
        print('解なし')
    else:
        print(render(8, 8, result))
        for p in result:
            print(f'{p.kind} rot={p.rotation:3d} center={p.center}')
