# perception：認識（RealSense・OpenCV）の開発置き場

担当：堀さん（B：認識）。RealSense と OpenCV でピースとケースの位置を求めるために作ったコードを、**試作の途中のものも含めて全部** ここに置きます。

## フォルダの分け方

| フォルダ | 置くもの | 例 |
|---|---|---|
| `scripts/` | 試作のコード（Python・C++ どちらでもよい） | HSV のしきい値を調べるツール、輪郭を出す試し、ArUco の検出の試し、座標変換の確認 |
| `samples/` | 試すための小さいデータ | テスト用の写真（1枚数 MB まで）、しきい値の YAML の試作 |
| `notes/` | 調べたこと・試した結果のメモ | 照明を変えたときの色の見え方、うまくいかなかった方法とその理由 |

- 1つの試みごとにファイルを分け、ファイルの先頭に「何をするコードか」「どう動かすか」を2〜3行のコメントで書く
- うまくいかなかったコードも消さずに残してよい（何を試したかの記録になる）
- 動画・rosbag・大量の写真は Google Drive に置いて、`notes/` にリンクを書く（`.bag` `.mcap` `.mp4` などは Git に入らない設定）

## 試作から本番のノードへ

ここは **試作の置き場** です。チームで動かす完成版は、ROS 2 のノードとして `src/` に置きます。

1. ここで試して、やり方が決まる
2. 本番のノード（例：`src/piece_detector.cpp`）を作り、`CMakeLists.txt` に登録する。データの形式は [docs/interfaces.md](../docs/interfaces.md)（#29）に合わせる
3. HSV のしきい値など、決まった設定値は `config/` の YAML に移す

ここに `package.xml` を作る（ROS 2 のパッケージを作る）のはやめてください。パッケージの中にパッケージがあると、ビルドされません。

## このフォルダはテストの書き方チェックの対象外

`colcon test` の書き方チェック（flake8・uncrustify・pep257 など）は、`perception/` を対象から外しています。試作のコードの書き方は気にしなくて大丈夫です。`src/` に移すときに、チェックが通るように整えます。

## 上げ方

コマンドで上げる場合：

```bash
cd ~/ros2_ws/src/seisakuron2026_tukamukai
git switch main && git pull
git switch -c feature/perception-<内容>      # 例：feature/perception-hsv
# perception/ にファイルを置く
git add perception/
git commit -m "HSV のしきい値を調べるツールを追加"
git push -u origin feature/perception-<内容>
gh pr create --fill
```

ブラウザから上げる場合は、[hardware/README.md](../hardware/README.md) の「ブラウザから上げる手順」と同じです（`perception/scripts` などを開いて「Add file」→「Upload files」、**新しいブランチで PR にする**）。
