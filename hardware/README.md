# hardware：ハードのデータ置き場

取っ手・ピース・ケースの CAD データ、印刷用の STL、設計値、試作の記録を置きます。担当は A（ハード）ですが、寸法はソフトの担当（B・C）も使うので、決まったら必ずここに書いてください。

## フォルダの分け方

| フォルダ | 中身 | 関係する Issue |
|---|---|---|
| [`dimensions.md`](dimensions.md) | **設計値の一覧**（マスの大きさ、取っ手の寸法など）。ソフトの担当はここを見る | すべて |
| `handle/` | 取っ手 | #13 |
| `prototypes/` | 1マス分の試作（すり鉢・突起）と、ずらし実験の記録 | #14 |
| `pieces/` | ピース7種類（I・O・T・S・Z・J・L） | #15 |
| `case/` | ケース（8×8 の枠、マーカーの取り付け位置） | #16 |

カメラのマウンタは rt-net 公式のものを使います。RT 社の非商用使用許諾規約の対象なので、このリポジトリにはコピーせず、[rt-net/crane_x7_Hardware](https://github.com/rt-net/crane_x7_Hardware/blob/master/3d_print_parts/v1.0/CRANE-X7_HandA_RealSenseD435マウンタ.stl) から直接ダウンロードしてください（#45）。

## 置くファイル

部品ごとに、次の3種類をそろえて置きます。

| 種類 | 形式 | 何のため |
|---|---|---|
| CAD の元データ | 使っている CAD の形式（Fusion 360 なら `.f3d`） | あとで修正するため |
| STEP | `.step` | 別の CAD でも開けるようにするため |
| STL | `.stl` | 3D プリンタで印刷するため |

**ファイル名**：`<部品>_<内容>.<拡張子>` にします。英数字とアンダーバーだけを使ってください。

- 例：`handle/handle.step`、`pieces/piece_T.stl`、`case/case_8x8.step`、`prototypes/cell_cone45.stl`

**直したとき**：同じファイル名のまま上書きします（`_v2` などは付けない）。前の版は Git の履歴に残っていて、いつでも戻せます。何を変えたかは、コミットメッセージ（ブラウザの場合は「Commit changes」の説明欄）に書いてください。

**大きいファイル**：ブラウザから上げられるのは1ファイル 25MB まで、Git では 100MB までです。それより大きいものや、写真・動画は Google Drive に置いて、リンクを `dimensions.md` や各フォルダのメモに貼ってください。

## ブラウザから上げる手順（ターミナルを使わない方法）

`main` には直接入れず、ブランチを作って PR にします。

1. GitHub でリポジトリを開き、`hardware` → 上げたいフォルダ（例：`handle`）を開く
2. 右上の **「Add file」→「Upload files」** を押す
3. ファイルをドラッグして入れる
4. 下の「Commit changes」で：
   - 1行目に、何を上げたかを書く（例：`取っ手の初版を追加`）
   - **「Create a new branch for this commit and start a pull request」** を選ぶ
   - ブランチ名を `feature/hardware-<内容>` にする（例：`feature/hardware-handle`）
5. **「Propose changes」** を押すと PR の画面になるので、説明に `Closes #13` のように Issue 番号を書いて「Create pull request」
6. ほかのメンバーに見てもらってからマージする

寸法の数字だけを直すときは、`dimensions.md` を開いて右上の鉛筆マーク（Edit）から直し、同じように新しいブランチで PR にします。
