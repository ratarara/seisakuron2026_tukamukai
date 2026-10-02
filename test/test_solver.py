import pytest

from tukamukai.solver import orientations, Placement, render, SHAPES, solve


# 解が盤面をちょうど埋めていて、ピースの個数を超えていないか確かめる
def assert_valid(rows, cols, pieces, filled, placements):
    assert placements is not None
    covered = set(filled)
    used = {}
    for p in placements:
        min_r = min(r for r, _ in p.cells)
        min_c = min(c for _, c in p.cells)
        shape = tuple(sorted((r - min_r, c - min_c) for r, c in p.cells))
        assert shape == dict(orientations(p.kind))[p.rotation]
        for cell in p.cells:
            assert 0 <= cell[0] < rows and 0 <= cell[1] < cols
            assert cell not in covered
            covered.add(cell)
        used[p.kind] = used.get(p.kind, 0) + 1
    assert len(covered) == rows * cols
    for kind, n in used.items():
        assert n <= pieces[kind]


def test_shapes_have_four_cells():
    for shape in SHAPES.values():
        assert len(set(shape)) == 4


def test_orientation_counts():
    counts = {kind: len(orientations(kind)) for kind in SHAPES}
    assert counts == {'I': 2, 'O': 1, 'T': 4, 'S': 2, 'Z': 2, 'J': 4, 'L': 4}


def test_rotation_is_clockwise():
    # T の基準は下向きの出っ張り。時計回りに 90 度回すと左向きになる
    assert dict(orientations('T'))[90] == ((0, 1), (1, 0), (1, 1), (2, 1))


def test_center_is_centroid():
    p = Placement('T', 0, ((0, 0), (0, 1), (0, 2), (1, 1)))
    assert p.center == (0.25, 1.0)


def test_solve_8x8_mixed():
    pieces = {'I': 2, 'O': 2, 'T': 4, 'S': 2, 'Z': 2, 'J': 2, 'L': 2}
    assert_valid(8, 8, pieces, set(), solve(8, 8, pieces))


def test_solve_with_filled_cells():
    filled = {(r, c) for r in range(2) for c in range(8)}  # 上 2 行は最初から埋まっている
    pieces = {'I': 2, 'O': 2, 'T': 2, 'S': 2, 'Z': 2, 'L': 2}
    assert_valid(8, 8, pieces, filled, solve(8, 8, pieces, filled))


def test_extra_pieces_are_allowed():
    pieces = {'O': 5, 'I': 3}
    result = solve(4, 4, pieces)
    assert_valid(4, 4, pieces, set(), result)
    assert len(result) == 4


def test_odd_t_cannot_fill_8x8():
    assert solve(8, 8, {'T': 1, 'O': 15}) is None


def test_odd_jl_without_t_cannot_fill_8x8():
    assert solve(8, 8, {'I': 1, 'O': 5, 'S': 2, 'Z': 5, 'J': 1, 'L': 2}) is None


def test_mirror_is_not_used():
    # S の形の穴は、Z を裏返せば埋まるが、回転だけでは埋まらない
    filled = {(0, 0), (1, 2)}
    assert solve(2, 3, {'Z': 1}, filled) is None
    assert solve(2, 3, {'S': 1}, filled) is not None


def test_not_enough_pieces():
    assert solve(4, 4, {'O': 3}) is None


def test_unknown_kind():
    with pytest.raises(ValueError):
        solve(4, 4, {'X': 4})


def test_render():
    filled = {(0, 0), (0, 1), (0, 2), (0, 3)}
    result = solve(2, 4, {'I': 1}, filled)
    assert render(2, 4, result, filled) == '# # # #\nI I I I'
