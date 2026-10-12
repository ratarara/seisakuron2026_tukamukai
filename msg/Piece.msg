# 認識したピース1個。座標は base_link、単位は m と rad（docs/interfaces.md）

# ピースの種類：I O T S Z J L のどれか1文字
string kind

# 取っ手（ピースの重心の真上）の位置。z は取っ手の上面の高さ
geometry_msgs/Point position

# 向き：ピースの基準の向き（ソルバーの形）からの回転角。上から見て反時計回りが正
# 対称なピースは同じ見た目の角度が複数ある（O は 90°、I・S・Z は 180° ごと）。どれを返してもよい
float64 yaw

# 認識の確からしさ（0〜1）。分からなければ 1.0
float32 confidence
