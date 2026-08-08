# robosuite-ros2 tidy-up

robosuite / MuJoCo の片付けタスク（机上の球を箱に入れる）を、ROS 2 ノードで知覚・制御するデモです。
シーンは robosuite の `Lift` を継承した自作環境 (`CustomTidyUpEnv`) です。
---

## アーキテクチャ

```
robosuite (CustomTidyUpEnv)
        ↕ step / obs
robosuite_bridge_node
  pub: /camera/image_raw, /camera/camera_info, /camera/extrinsic
       /world/table_height, /robot/eef_pose
       /gt/ball_poses, /gt/box_pose          ← デバッグ用真値
  sub: /robot/cmd_action, /robot/target_pose
        │
        │ image + camera geometry
        ▼
hsv_perceptor_node
  HSV 検出 + 机平面交差で 3D 推定
  pub: /percept/ball_poses, /percept/box_pose
        │
        ▼
task_manager_node
  状態機械で把持シーケンス
  pub: /robot/cmd_action, /robot/target_pose
```

| ファイル | 役割 |
|---|---|
| `robosuite_bridge_node.py` | メインのシミュレーション境界（tidy-up シーン） |
| `hsv_perceptor_node.py` | HSV 検出 + 机平面交差による 3D 推定 |
| `task_manager_node.py` | 把持・収納の状態機械 |
| `percept_error_debug_node.py` | `/percept/*` と `/gt/*` の誤差比較（任意） |
| `robosuite_bridge_lift_node.py` | 最小構成の Lift ブリッジ（レガシー／動作確認用） |

制御経路では Task Manager は **`/percept/*`（推定）のみ** を使います。真値は `/gt/*` に分離してあるため、HSV 知覚と同時起動してもトピックが衝突しません。

---

## 設定ファイル

シーン・知覚・制御の数値は `config/` 以下の JSON に分離しています（追加依存なし）。

| ファイル | 内容 |
|---|---|
| `config/env.json` | 机・球・箱・カメラ・シミュレータ設定 |
| `config/perception.json` | HSV 閾値・検出面積など |
| `config/control.json` | Task Manager のゲイン・クリアランスなど |

球の半径などは `env.json` を正とし、知覚ノードもそこから読みます。
レイアウトを変えたいときは Python を編集せず、`config/env.json` を変更してください。

---

## 環境構築 (macOS / Apple Silicon)

### 前提

- macOS arm64
- Anaconda または Miniconda がインストール済み

### 依存パッケージ

| 分類 | パッケージ |
|---|---|
| ROS 2 | `rclpy`, `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `std_msgs` |
| 物理シム | `robosuite`, `mujoco` |
| その他 | `opencv`, `numpy`, `glfw` |

### 1. ROS 2 環境の作成

macOS には ROS 2 の公式バイナリがないため、conda-forge 経由の [RoboStack](https://robostack.github.io/) を使います。

```bash
conda create -y -n ros2_libero --solver=libmamba --override-channels \
  -c conda-forge -c robostack-staging \
  python=3.11 "numpy=1.26" \
  ros-humble-ros-base ros-humble-cv-bridge \
  numba scipy pillow py-opencv glfw
```

- `--solver=libmamba` は必須（classic solver では ROS の依存が解けない）
- `--override-channels` も必須（`defaults` が混ざると RoboStack が壊れる）
- Python は **3.11**（RoboStack osx-arm64 は 3.10 / 3.11 / 3.12 のみ）

### 2. robosuite / mujoco の追加

```bash
conda activate ros2_libero
pip install "mujoco==3.1.2" pynput termcolor
pip install --no-deps "robosuite==1.4.1"
```

- robosuite は **1.4.1 を固定**（1.5 系では `load_controller_config` が削除されている）
- `--no-deps` は必須（pip の `opencv-python` が入り、conda の `py-opencv` / `cv_bridge` と衝突してクラッシュしうる）

### 3. 動作確認

```bash
conda activate ros2_libero
python -c "import rclpy, cv_bridge, robosuite, mujoco, cv2, numpy; print('ok')"
ros2 topic list
```

---

## 起動

ターミナルを **3 枚**開き、それぞれで `conda activate ros2_libero` してください。

```bash
# Terminal A — simulation
cd /path/to/this/repo
conda activate ros2_libero
python robosuite_bridge_node.py
```

```bash
# Terminal B — vision
cd /path/to/this/repo
conda activate ros2_libero
python hsv_perceptor_node.py
```

```bash
# Terminal C — control
cd /path/to/this/repo
conda activate ros2_libero
python task_manager_node.py
```

任意で誤差デバッグ:

```bash
python percept_error_debug_node.py
```

`source /opt/ros/humble/setup.bash` は **不要**です。`conda activate` が ROS 2 の環境変数設定も兼ねます。

### デバッグ用コマンド

```bash
conda activate ros2_libero
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /robot/eef_pose
ros2 topic echo /percept/ball_poses
ros2 topic echo /debug/ball_error_stats
```

---

## Ubuntu で構築する場合

Ubuntu 22.04 なら ROS 2 Humble を apt で入れます。

```bash
# https://docs.ros.org/en/humble/Installation.html
sudo apt install ros-humble-ros-base ros-humble-cv-bridge

pip install "mujoco==3.1.2" "robosuite==1.4.1" "numpy<2" opencv-python glfw pynput termcolor
```

```bash
source /opt/ros/humble/setup.bash
python robosuite_bridge_node.py
```

macOS と違い、`cv_bridge` は system の libopencv とリンクしているため、pip の `opencv-python` をそのまま入れて問題ありません（`--no-deps` は不要）。

---

## 注意点

### 知覚と真値のトピック分離

| トピック | 内容 | 配信元 |
|---|---|---|
| `/percept/ball_poses`, `/percept/box_pose` | 視覚推定 | `hsv_perceptor_node` |
| `/gt/ball_poses`, `/gt/box_pose` | 物理エンジン真値 | `robosuite_bridge_node` |
| `/robot/eef_pose` | 手先位置 | `robosuite_bridge_node` |
| `/robot/cmd_action` | 7 次元 OSC 指令 | `task_manager_node` |
| `/robot/target_pose` | 現在の制御目標（ログ用） | `task_manager_node` |

Task Manager は `/percept/*` のみを購読します。真値比較は `percept_error_debug_node` が `/gt/*` と突き合わせます。

### レンダリング (macOS)

`config/env.json` の `sim.mujoco_gl`（既定 `glfw`）を bridge 起動時に適用します。
macOS のオフスクリーン描画は glfw / cgl のみです。変更する場合もこの範囲に留め、
シミュレーションループを別スレッドに移さないでください（GLFW はメインスレッド専用）。

### LIBERO 本体との関係

本デモの実行に **LIBERO 本体は不要**です。LIBERO の学習・ベンチマークを別途行う場合は、依存（numpy ピンなど）が RoboStack の `cv_bridge` と衝突しやすいので、conda 環境を分けてください。

| 環境例 | python | 用途 |
|---|---|---|
| `ros2_libero` | 3.11 | 本リポジトリの ROS 2 ノード |
| （任意）学習用環境 | — | LIBERO / torch など |

---

## 検証済みバージョン

| | Ubuntu 22.04 | macOS arm64 |
|---|---|---|
| ROS 2 Humble | apt `/opt/ros/humble` | RoboStack (conda) |
| rclpy | 3.3.21 | 3.3.16 |
| cv_bridge | 3.2.1 | 3.2.1 |
| robosuite | 1.4.1 | 1.4.1 |
| mujoco | 3.1.2 | 3.1.2 |
| opencv | 4.11.x | 4.11.x |
| numpy | 1.24〜1.26 | 1.26.4 |
| python | 3.10 | 3.11 |

---

## トラブルシューティング

**`ModuleNotFoundError: No module named 'rclpy'`**  
→ そのターミナルで `conda activate ros2_libero` する。

**`AttributeError: ... load_controller_config`**  
→ robosuite 1.5 以降が入っている。`pip install --no-deps "robosuite==1.4.1"` で戻す。

**conda create が終わらない**  
→ `--solver=libmamba` を付けているか確認する。

**RoboStack の解決に失敗する**  
→ `--override-channels` と、`~/.condarc` に `defaults` が残っていないかを確認する。

**グリッパが動かない / 球を見ない**  
→ Bridge・HSV・Task Manager の 3 つが起動しているか、`ros2 topic hz /percept/ball_poses` で推定が出ているかを確認する。

**`FileNotFoundError: Config not found`**  
→ リポジトリ直下（`config/` があるディレクトリ）から `python` を実行する。
