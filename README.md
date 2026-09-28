# つかむ会（seisakuron2026_tukamukai）

千葉工業大学 未来ロボティクス学科「ロボット設計製作論実習3（2026年度 知能コース）」のチーム「つかむ会」のリポジトリです。

## 概要

CRANE-X7 と RealSense を使って、〇〇をするロボットを作ります。

<!-- 例：カメラでブロックの色と形を認識し、CRANE-X7で実物のテトリスをプレイする -->

- デモ動画：（最終発表後に追加）
- 発表資料：[docs/](docs/)

## メンバー

| 名前 | GitHub | 担当 |
|------|--------|------|
| 比屋根佳歩 | [@ratarara](https://github.com/ratarara) | |
| 石川奈々美 | [@73lkj](https://github.com/73lkj) | |
| 堀円香 | [@](https://github.com/) | |
| キンジュンウ | [@](https://github.com/) | |

## 動作環境

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- CRANE-X7（7軸マニピュレータ）
- Intel RealSense（深度カメラ）

## セットアップ

ROS 2 Jazzy と GitHub の設定がまだの人は、先に [ROS 2 環境構築](docs/ros2_setup.md) を済ませてください。

このリポジトリと CRANE-X7 パッケージの取得・ビルド：

```bash
cd ~/ros2_ws/src
gh repo clone ratarara/seisakuron2026_tukamukai

# CRANE-X7 の ROS 2 パッケージ（rt-net）
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git

# 依存パッケージのインストールとビルド（時間がかかります）
cd ~/ros2_ws
rosdep install -r -y -i --from-paths src
colcon build --symlink-install
source ~/.bashrc
```

## 使い方

CRANE-X7 を USB で接続したら、ポートへのアクセス権限を付与します（接続するたびに必要）。

```bash
sudo chmod 666 /dev/ttyUSB0
```

```bash
# 実機の起動
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0

# 別ターミナルで本パッケージを起動
ros2 launch tukamukai main.launch.py
```

<!-- launchファイル名・引数は実装に合わせて書き換える -->

## ディレクトリ構成

```
seisakuron2026_tukamukai/
├── README.md
├── LICENSE
├── package.xml
├── setup.py
├── launch/        # 起動用launchファイル
├── tukamukai/    # ノード本体（認識・動作計画など）
├── config/        # カメラ・パラメータ設定
└── docs/          # 発表資料・議事録
```

## 進捗管理

進捗は [Issues](../../issues) と [Projects](../../projects) で管理します。
Milestone は3回の発表に対応しています。

| Milestone | 内容 | 日程 |
|-----------|------|------|
| プラン発表 | 作るものの計画 | |
| 中間発表 | デモ付き | |
| 最終発表 | デモ付き | |

## 開発ルール

- `main` には直接pushせず、作業ごとにブランチを切って Pull Request でマージする
- ブランチ名は `feature/<内容>`（例：`feature/block-detection`）
- 作業を始めるときは対応する Issue に自分を Assign する
- 詰まったら Issue にコメントして早めに相談する

## ライセンス・謝辞

- 本リポジトリは [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros)（Apache License 2.0）を利用しています。
- 先輩のコードを参考・流用した場合は、ここに元リポジトリとライセンスを記載する。

© 2026 つかむ会
