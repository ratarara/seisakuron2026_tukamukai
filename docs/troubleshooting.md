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
