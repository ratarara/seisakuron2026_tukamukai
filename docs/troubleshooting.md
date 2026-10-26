# トラブルシューティング

つまずいたことと解決方法を書いていきます。新しく書くときは、**症状**（出たエラーをそのまま）・**原因**・**解決方法** の順で書いてください。

## セットアップ

### 複数行を貼り付けると、sudo のあとの行が実行されない

**症状**：sudo を含む複数行のコマンドをまとめてターミナルに貼り付けると、最初のほうしか実行されない。`sudo: 1 incorrect password attempt` などが出ることもある。

**原因**：sudo がパスワードを聞いている間に、貼り付けた次の行がパスワードとして読み込まれてしまう。

**解決方法**：sudo を含むコマンドは **1行ずつ** 貼り付けて、終わってから次を貼る。どこまで実行されたか分からなくなったら、`/var/log/auth.log` で sudo の実行記録を確認できる。

```bash
grep "sudo:.*COMMAND" /var/log/auth.log | tail
```

### `rosdep install` で「rosdep installation has not been initialized yet」と出る

**症状**：

```
ERROR: your rosdep installation has not been initialized yet.  Please run:

    sudo rosdep init
    rosdep update
```

**原因**：`sudo rosdep init` を実行していない（または失敗していた）。

**解決方法**：

```bash
sudo rosdep init
rosdep update
```

### `gh` や `git push` で認証エラーになる

**症状**：

```
fatal: could not read Username for 'https://github.com': そのようなデバイスやアドレスはありません
```

または `gh auth status` で `The token in ~/.config/gh/hosts.yml is invalid`。

**原因**：GitHub CLI のログイン（トークン）が切れている。

**解決方法**：ログインし直す。

```bash
gh auth login -h github.com
gh auth status   # "Logged in to github.com account ..." と出れば OK
```

## ビルド

### crane_x7_examples のビルドで `Policy CMP0144 is not set` という警告が出る

**症状**：`colcon build` の最後に `1 package had stderr output: crane_x7_examples` と出て、PCL（FindFLANN.cmake）の CMake 警告が表示される。

**原因**：PCL ライブラリ側の CMake の書き方に対する警告。

**解決方法**：対応不要。`Summary: ... packages finished` と出ていればビルドは成功している。

## シミュレーション（Gazebo・RViz）

### VSCode のターミナルから起動すると RViz が落ちる

**症状**：`ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py` で Gazebo は開くが、RViz が開かずに次のエラーで終了する。

```
[rviz2-5] /opt/ros/jazzy/lib/rviz2/rviz2: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
[ERROR] [rviz2-5]: process has died [pid ..., exit code 127, ...]
```

**原因**：VSCode を snap 版（`sudo snap install code`）で入れていると、VSCode の中のターミナルに snap 用の環境変数（`GTK_PATH` など）が引き継がれる。そのため RViz が snap 側の古いライブラリを読み込んでしまう。

**解決方法**：Gazebo・RViz などの画面を使うものは、VSCode の中ではなく **普通のターミナル（`Ctrl + Alt + T`）** から起動する。VSCode の中で起動したいときは、次のように snap の環境変数を外す。

```bash
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

### ウィンドウを閉じても、裏でプログラムが動き続ける

**症状**：Gazebo と RViz のウィンドウを閉じたあと、もう一度起動したりサンプルを動かしたりすると、`Publisher already registered for node name` などの警告が出る。動きもおかしくなる。

**原因**：ウィンドウを閉じても、MoveIt（`move_group`）などの画面のないプログラムは終了しない。

**解決方法**：シミュレーションは、起動したターミナルで **`Ctrl + C`** を押して止める。残っていないかは次で確認できる（何も表示されなければ OK）。

```bash
pgrep -a -x "move_group|rviz2|ruby|robot_state_publisher|parameter_bridge"
```

### サンプルを動かしてもアームが動かない（`No state update received`）

**症状**：`ros2 launch crane_x7_examples example.launch.py example:=pick_and_place use_sim_time:=true` を実行すると `Plan and Execute request accepted` で止まり、シミュレーション側のターミナルに次の警告が出続ける。

```
[move_group-4] [WARN] ... No state update received within 100ms of system clock.
```

**原因**：Gazebo が終了している（ウィンドウを閉じた、落ちたなど）ため、関節の状態が MoveIt に届いていない。

**解決方法**：シミュレーションを `Ctrl + C` で止めて起動し直す。シミュレーション側のターミナルに `You can start planning now!` と `Configured and activated crane_x7_arm_controller` が出てから、サンプルを実行する。サンプルにはシミュレーションの時計を使う `use_sim_time:=true` を必ず付ける。

## 実機

実機の動かし方は [real_robot.md](real_robot.md) にあります。

### `/dev/ttyUSB0` が見つからない

**症状**：`ls -l /dev/ttyUSB0` や `demo.launch.py` で次のように出る。

```
ls: '/dev/ttyUSB0' にアクセスできません: そのようなファイルやディレクトリはありません
```

**原因**：ロボットが PC につながっていない、またはロボットの電源が入っていない。

**解決方法**：電源と USB ケーブルを確認する。正しくつながっていれば、次のコマンドで `Future Technology Devices International ... FT232H` という行が出る。

```bash
lsusb | grep -i ftdi
```

### 起動はできたのに、アームが動かない（`START_STATE_INVALID`）

**症状**：`demo.launch.py` は起動し、RViz にもロボットが表示されているのに、サンプルを実行してもアームが動かない。サンプル側のターミナルには次のように出る。

```
Plan and Execute request aborted
MoveGroupInterface::move() failed or timeout reached
```

`demo.launch.py` 側のターミナルには次のように出ている。

```
Joint 'crane_x7_shoulder_revolute_part_tilt_joint' from the starting state is outside bounds by: [-1.62142 ] should be in the range [-1.6057 ], [1.6057 ].
PlanningRequestAdapter 'CheckStartStateBounds' failed, because 'Start state out of bounds.'. Aborting planning pipeline.
START_STATE_INVALID
```

**原因**：肩を上下に振る関節（shoulder tilt）が、MoveIt で決められた動いてよい範囲（±1.6057 rad）から少し（1° ほど）はみ出している。電源を切ってアームを台に寝かせておくと、この位置になりやすい。MoveIt は、スタートの姿勢が範囲の外だと、安全のために動作の計画をしない。

**解決方法**：

1. アームを手で支えて、`demo.launch.py` を `Ctrl + C` で止める
2. 肩の関節を、アームが **起き上がる向き** に手で 5° ほど動かす
3. `demo.launch.py` を起動し直して、もう一度サンプルを実行する

ほかの関節が範囲の外にあるときも同じエラーが出る。エラーに出ている関節名を見て、その関節を範囲の内側に動かす。
