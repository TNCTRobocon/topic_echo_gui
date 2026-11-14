# topic_echo_gui

Qt 5 + C++17で実装したROS 2 Topic Echo GUIです。

## 主な機能

- ROS 2グラフ上のTopicを一覧表示
- Topic名またはメッセージ型で絞り込み
- 表示領域を手動で左右・上下に分割し、表示枠を選択してトピックを割り当て
- 1つの表示枠に複数のTopicを表示
- 1つのTopicは必ず1つの表示枠だけに所属し、別の表示枠で名前をクリックすると割り当て先を自動移動
- 各表示枠は、割り当てられたTopicごとに名前と最新メッセージを順に表示
- Topic一覧のチェックマークは表示枠に選択中（割り当て済み）かどうかを表示するだけで、クリックでは変更できない
- 「割当TopicをEcho」で全表示枠に割り当てられたTopicを購読開始
- Echo中にTopicを表示枠へ割り当てると、そのTopicの購読を開始。すでに購読中のTopicを別枠へ移動しても購読を作り直さない
- Echo停止で全購読を解除
- 「すべての割当を解除」で全表示枠からTopic割り当てを解除
- 分割位置をドラッグで調整し、表示枠の削除にも対応
- 分割構造、分割比率、表示枠ごとのTopic割り当てを `~/.config/topic_echo_gui/settings.json` に保存し、次回起動時に復元
- 旧設定の「1表示枠につき1Topic」形式も読み込み可能。重複して割り当てられている設定は最初に見つかった表示枠を優先して整理
- 設定を復元しても購読は自動開始せず、「割当TopicをEcho」を押して開始
- 実行時に型を読み込むGeneric Subscriptionに対応
- `/joy` は `sensor_msgs/msg/Joy` の型付き購読で表示
- 最新メッセージだけを保持し、古い表示待ちを蓄積しない
- GUI更新は2ms間隔を基本とし、受信と画面更新を分離

## ビルド

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select topic_echo_gui
source install/setup.bash
```

## 実行

```bash
ros2 run topic_echo_gui topic_echo_gui
```

通常は ROS_DOMAIN_ID を必要な値に設定してから起動してください。

## 操作

1. 「更新」で現在のTopic一覧を取得します。
2. 表示したいTopicの名前をクリックします。未選択なら選択中の表示枠に割り当てられます。同じ表示枠で選択済みのTopicをもう一度クリックすると選択解除されます。ほかの表示枠に割り当て済みのTopicをクリックした場合は、選択中の表示枠へ移動します。
3. 表示枠内の「表示枠」ボタンで割り当て先を選びます。「左右に分割」または「上下に分割」で、選択中の表示枠を分割できます。
4. 「割当TopicをEcho」を押すと、表示枠に割り当てられたTopicの購読を開始します。Echo中に割り当てを追加すると、そのTopicも購読します。
5. Topic一覧のチェックマークは、そのTopicがどこかの表示枠に割り当てられているかを示します。表示専用であり、クリックやSpaceキーで切り替えることはできません。Echo停止中でも、割り当てられているTopicにはチェックが付きます。
6. 「Echo停止」で購読をすべて解除します。表示枠への割り当ては保持され、再度「割当TopicをEcho」を押すと購読を再開します。
7. 「すべての割当を解除」で全表示枠からTopicの割り当てを解除します。Echo中の場合、これに伴い購読も解除されます。
8. 表示枠の境界をドラッグすると分割比率を調整できます。「表示枠を削除」で選択した枠を削除できます。最後の1枠は削除されず、Topic割り当てだけが解除されます。
9. 設定は変更時に保存され、次回起動時に分割構造、分割比率、Topic割り当てが復元されます。購読自体は自動開始されません。

## QoS

購読は BEST_EFFORT / VOLATILE / KEEP_LAST(10) を使用します。

## 表示方式

`/joy` はC++の `sensor_msgs::msg::Joy` を直接受信して文字列化します。
その他のTopicはROS 2のGeneric Subscriptionと型サポートを使って実行時にデシリアライズします。

高周波Topicでは受信のたびにGUIウィジェットを更新せず、最新値だけを保持します。
GUI側は短い周期で最新値を反映するため、端末の受信処理と画面描画を分離しています。

Python版の `main.py` / `setup.py` / `setup.cfg` は削除済みです。 `package.xml` のbuild typeは `ament_cmake` で、`ros2 run topic_echo_gui topic_echo_gui` はC++実行ファイルを起動します。
