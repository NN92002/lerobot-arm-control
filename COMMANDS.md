# SO-101 雙組主從系統：完整操作指令

硬體為兩組 SO-101，共四支手臂：左主臂、左從臂、右主臂、右從臂。UI、錄製、訓練與離線推論共用專案內 `.conda/` 環境。

## 1. 每次使用：本機錄製、伺服器保存與訓練

```bash
cd /home/itri2026-3090/Desktop/lerobot-arm-control

# 開啟實機 UI；完整 episode 會自動上傳並驗證後刪除本機檔案
./run.sh -m rgbd.gui --config configs/rgbd.json --output data/recordings

# 無硬體 UI 測試
./run.sh -m rgbd.gui --mock

# 第一次使用或程式更新後，同步程式到伺服器
./server.sh deploy

# 伺服器資料集名稱由 --output 的資料夾名稱決定，例如 recordings
# 在伺服器上訓練，checkpoint 寫入 /home/itri2026/lerobot_checkpoints
./server.sh train recordings pick_place_v1 \
  --steps 100 --batch-size 8 --device cuda
```

`run.sh` 使用 `.conda/bin/python`，清除繼承的 `PYTHONPATH`、`PYTHONHOME`，停用使用者 site-packages，並將工作目錄設為專案目錄。即使目前啟用 base 或其他 Conda 環境，也會使用這個專案的 Python。後續加裝套件請使用 `./run.sh -m pip ...`。

## 2. 確認四支手臂接口

```bash
./run.sh control.py ports
```

建議逐支接上 USB，在 UI 按「Rescan Ports」，根據新增路徑指定角色。優先用 `/dev/serial/by-id/` 固定路徑。UI 不會自動辨認主從或左右，請貼標籤確認。

| UI 角色 | JSON 欄位 |
|---|---|
| 左主臂 Leader | `teleop.ports.left` |
| 左從臂 Follower | `robot.ports.left` |
| 右主臂 Leader | `teleop.ports.right` |
| 右從臂 Follower | `robot.ports.right` |

按「Save Ports」只寫入目前載入的設定檔，預設為 `configs/rgbd.json`，不會同步 `configs/dual.json`。以下雙組校正與遙控指令統一使用 `configs/rgbd.json`，讓接口設定一致。

錄製前仍需在 `configs/rgbd.json` 填好實際 URDF、link 名稱、joint mapping 與相機設定；詳見 [RGBD_GUIDE.md](RGBD_GUIDE.md)。`python` 預設為 `.conda/bin/python`，`lerobot_path: null` 表示使用環境內安裝的 LeRobot。

## 3. 校正四支手臂

先預覽，不連接手臂：

```bash
./run.sh control.py calibrate-teleop --config configs/rgbd.json
./run.sh control.py calibrate-robot --config configs/rgbd.json
```

實際執行，依終端機指示完成主臂與從臂校正：

```bash
./run.sh control.py calibrate-teleop --config configs/rgbd.json --run
./run.sh control.py calibrate-robot --config configs/rgbd.json --run
```

校正資料存於 `calibration/dual/teleop/`、`calibration/dual/robot/`。請先確認馬達 ID 與接線已設定好。帶 `--run` 會連接硬體。

只校正左組（不要求右組接口），保存的校正 ID 與 UI 相同：

```bash
./run.sh control.py calibrate-teleop --config configs/rgbd.json --side left
./run.sh control.py calibrate-teleop --config configs/rgbd.json --side left --run
./run.sh control.py calibrate-robot --config configs/rgbd.json --side left --run
# 右組將 left 改成 right；獨立純遙控也可加 --side left/right
```

## 4. 純遙控，不錄製

```bash
# 預覽
./run.sh control.py teleoperate --config configs/rgbd.json

# 實際遙控，Ctrl+C 結束
./run.sh control.py teleoperate --config configs/rgbd.json --run
```

停止／斷線時可能關閉從臂扭力，請先支撐手臂。UI 停止按鈕與 Ctrl+C 都不是硬體急停。

保留的其他設定範本：

```bash
# 若使用獨立雙組設定，須自行填好該檔的四個接口
./run.sh control.py teleoperate --config configs/dual.json

# 單組範本，同樣預設僅預覽
./run.sh control.py teleoperate --config configs/single.json
./run.sh control.py calibrate-teleop --config configs/single.json
./run.sh control.py calibrate-robot --config configs/single.json
```

## 5. 相機檢查

```bash
# 讀取設定中的 RealSense，檢查 RGB、深度與有效比例；不連接手臂
./server.sh train recordings pick_place_v1 \
  --steps 100 --batch-size 8 --device cuda

## 6. UI 獨立硬體控制與錄製

```bash
./run.sh -m rgbd.gui --output data/pick_place
# 無硬體測試同一套獨立控制
./run.sh -m rgbd.gui --mock --output data/mock_ui
```

| 控制 | 行為 |
|---|---|
| Camera connections / Connect | 只啟用指定相機，持續預覽，不保存資料 |
| Left pair / Connect | 只連接左主臂與左從臂，開始關節回授 |
| Right pair / Connect | 只連接右主臂與右從臂，開始關節回授 |
| Start Teleop / Stop Teleop | 開始／停止該組主從跟隨，與連線分開 |
| Start Recording | 只保存目前已連線的手臂組及相機 |
| Stop Recording | 提前停止、標記未完成、停止全部遙控；連線與預覽持續 |
| Disconnect | 釋放對應硬體；錄製中需先停止 |

例如：只 Connect 左組，就只保存左組手臂資料；再 Connect 相機，會一起保存左組＋影像。左右組與所有相機都 Connect，才保存全部資料。無裝置連線時無法開始錄製。模擬與實機模式不可混用；切換前需斷開全部裝置。

連接手臂只讀取狀態，不會自動開始跟隨。要收集訓練用示範，先按已連線各組的 `Start Teleop`，再錄製。只讀手臂仍可錄製關節與 FK，但沒有送出的動作標籤，使用 `action_valid=false` 與 NaN 表示；此類資料不能直接用於模仿訓練。

錄製完成也會停止遙控，裝置保持連線。下一回合前如需跟隨，重新按 `Start Teleop`。關閉視窗會保存未完成回合並斷開全部裝置。連線斷開可能關閉從臂扭力，先支撐手臂；停止不是硬體急停。

只有已啟用組的接口、校正與 URDF 會被檢查。URDF 尚未設定時可連線查看關節，末端欄顯示 `FK not configured`；錄製手臂資料前需補齊。純相機錄製不需要手臂設定。

同一回合的装置組合固定；要增減裝置，先停止錄製再連接／斷開。每次開始建立新的 `episode_*`。不同手臂／相機組合建議用不同資料夾，訓練會拒絕混用。

## 7. 終端機錄製與資料檢查

```bash
# 實際連接手臂與相機；每回合按 Enter 開始
./run.sh -m rgbd.record --config configs/rgbd.json \
  --output data/pick_place --episodes 10 --seconds 30

# 檢查完整性、幀数、有效 FPS、最大時間間隔及深度有效比例
./run.sh -m rgbd.inspect data/pick_place
```

原始 RGB-D 存為無損、未壓縮 NPZ。單台 640×480、30 FPS 約 2.8 GB／分鐘，請安排磁碟空間。這是本專案資料格式，使用本專案訓練入口。

### 終端錄製指定裝置

```bash
# 左組 + 設定中的所有相機
./run.sh -m rgbd.record --arms left --output data/left_with_camera
# 右組，不開相機
./run.sh -m rgbd.record --arms right --cameras --output data/right_arm_only
# 純相機，不連接手臂
./run.sh -m rgbd.record --arms none --output data/camera_only
# 雙組 + 指定 front 相機
./run.sh -m rgbd.record --arms both --cameras front --output data/both_front
```

以上不加 `--mock` 會實際連接所選硬體；有手臂時終端录製會啟動所選組遙控。`--arms` 預設 both；未提供 `--cameras` 時使用設定中的全部相機，提供但不填名稱則停用相機。

## 8. 訓練 Diffusion Policy

### 伺服器資料與訓練

本機錄製完成後，使用 SSH 將資料同步到訓練伺服器。伺服器設定為
`itri2026@140.114.58.2`，資料根目錄為 `/home/itri2026/lerobot_datasets`：

```bash
# 將本機資料集同步到伺服器的 /home/itri2026/lerobot_datasets/pick_place
./server.sh sync data/pick_place pick_place
./server.sh train pick_place pick_place_v1 \
  --steps 100 --batch-size 8 --device cuda
./server.sh deploy

# 在伺服器上訓練，checkpoint 存到 /home/itri2026/lerobot_checkpoints/pick_place_v1
./server.sh train pick_place pick_place_v1 \
  --steps 100 --batch-size 8 --device cuda
```

`server.sh` 需要本機已安裝 `ssh`、`rsync`，並且 SSH 金鑰登入伺服器已可用。可用
`LEROBOT_SERVER_*` 環境變數覆寫伺服器位址或目錄；不要把 SSH 密碼寫入專案。

使用 `configs/rgbd.json` 啟動實機 GUI 時，完整 episode 會自動上傳到
`/home/itri2026/lerobot_datasets/<資料集名稱>/<episode>`。程式會先以 checksum 同步，
錄製完成後先選擇 `positive` 或 `negative`，接著背景上傳並比對遠端檔案清單與大小；只有驗證成功才刪除本機 episode。
上傳在背景執行，可以立即開始下一個 episode；關閉 GUI 時會等待背景上傳完成。

GUI 按 `Start Recording` 後會倒數 3 秒才建立 episode。`Stop Recording` 只停止目前回合並保持 Teleop，方便直接錄下一回合；遇到危險或需要立即停止命令時才按 `Emergency Stop`，它會停止 Teleop 並斷開手臂。
中斷錄製、mock 錄製或上傳失敗時，本機檔案會保留。訓練入口只會載入 `positive` episode，
`negative` 仍保留在伺服器供檢查。

`<資料集名稱>` 是 GUI 的 output 資料夾名稱，例如 `--output data/recordings` 會上傳到
`/home/itri2026/lerobot_datasets/recordings/`。因此同一個 output 資料夾應只保存同一種
手臂／相機組合與同一個任務。

如果確認某個已上傳 episode 錄壞，可在本機執行以下指令刪除伺服器副本；`--yes` 是必要的安全確認。刪除後，下一次錄製會優先使用最小的空缺編號：

```bash
./server.sh delete-episode recordings episode_000001 --yes
```

```bash
# 正式訓練請使用上面的 server.sh train，在伺服器直接讀取遠端資料。
# 以下只適用於資料仍在本機、且要做離線除錯時：
./run.sh -m rgbd.train --config configs/rgbd.json \
  --data data/pick_place --output outputs/pick_place_v1 \
  --steps 100 --batch-size 8 --device cuda
```

無可用 GPU 時可將 `--device cuda` 改為 `--device cpu`，速度較慢。測試可用 `--steps 100`，正式訓練再調大。`--output` 必須是新目錄；只保存 `checkpoint_best`（最低 training loss）與 `checkpoint_last`，不保存中間 checkpoint。

訓練維度會依資料自動選擇單組（action 6／state 15）或雙組（12／30）。左右單組即使維度相同也不能混用；推論會核對 checkpoint 的關節名稱。此影像條件模型要求至少一台相機與一組已啟動遙控的手臂；純相機、純手臂及未啟動遙控的觀測錄製仍可保存／檢查，但不適用此訓練入口。

目前沒有續訓入口或實機自主部署；訓練 loss 不代表實機成功率。

## 9. 離線推論

```bash
./run.sh -m rgbd.predict --config configs/rgbd.json \
  --checkpoint outputs/pick_place_v1/checkpoint_best \
  --data data/pick_place --output outputs/predicted_actions.json
```

此指令只產生預測目標 JSON，不會連接或驅動手臂。輸出檔必須尚不存在。

## 10. 無硬體完整流程測試

以下 `mock_check` 路徑以首次執行為例；重跑訓練／推論時請使用新的輸出名稱。

```bash
./run.sh -m unittest discover -s tests -v
./run.sh -m rgbd.record --mock --output data/mock_check --episodes 2 --seconds 0.6
./run.sh -m rgbd.inspect data/mock_check
./run.sh -m rgbd.train --data data/mock_check --output outputs/mock_check \
  --steps 2 --batch-size 2 --small --allow-mock --device cpu
./run.sh -m rgbd.predict --checkpoint outputs/mock_check/checkpoint_000002 \
  --data data/mock_check --output outputs/mock_prediction.json --allow-mock
```

合成資料標示 `synthetic`，不可混入正式示範資料。無螢幕時 GUI 測試會使用 Qt offscreen 模式。亦可明確指定 `QT_QPA_PLATFORM=offscreen`；本機有 Xvfb，可用以下指令驗證 X11 視窗：

```bash
xvfb-run -a ./run.sh -m unittest discover -s tests -v
```

## 11. 環境檢查與套件管理

```bash
# 確認 Python 與 LeRobot 都在專案 .conda 裡
./run.sh -c 'import sys, lerobot; print(sys.executable); print(lerobot.__file__)'
./run.sh -m pip check
./run.sh -m pip list
./run.sh -c 'import torch; print("CUDA available:", torch.cuda.is_available())'

# 查看 Conda 環境
/home/itri2026-3090/miniconda3/bin/conda env list
```

若需要手動啟用環境：

```bash
source /home/itri2026-3090/miniconda3/etc/profile.d/conda.sh
conda activate /home/itri2026-3090/Desktop/lerobot-arm-control/.conda
# 執行本專案仍建議使用 ./run.sh，以隔離 ROS 等外部 Python 路徑
conda deactivate
```

本次環境由已驗證的 `lerobot` 環境複製而成，保留其相依套件，並非最小套件集合。已移除複本中的 UR5e ROS editable 外部專案依賴，LeRobot 改由專案隨附 wheel 安裝，RealSense 套件也安裝在環境內。既有 base、lerobot 環境的套件未修改。

Conda 隔離 Python 套件與相關函式庫；作業系統、GPU 驅動、USB 權限與桌面顯示服務仍由本機提供。`.conda/` 是執行環境，不是可刪的快取；不要直接將這個目錄搬到其他路徑使用。

## 12. 在另一份專案目錄重建環境

`environment.yml` 鎖定本次 Conda 與 pip 套件版本，適用 Linux x86_64；`environment/lerobot-0.4.4-py3-none-any.whl` 是本機 LeRobot 原始碼建出的固定快照。保留兩者，無需原本 `/home/itri2026-3090/lerobot` 原始碼。其餘套件重建時需要網路及 Conda。

在新專案目錄（尚無 `.conda`）執行：

```bash
# 先 cd 到新專案目錄，conda 指令須已可用
# 清除外部 Python 路徑，避免環境安裝混入 ROS 或使用者套件
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 \
  conda env create --prefix "$PWD/.conda" --file environment.yml
./run.sh -m pip check
./run.sh -m unittest discover -s tests -v
./run.sh -m rgbd.gui --mock
```

硬體接口、URDF 絕對路徑、相機序號與校正資料仍需依目標機器調整。GPU 訓練也取決於目標機器驅動相容性。

## 13. 查看每個入口的全部參數

```bash
./run.sh control.py --help
./run.sh -m rgbd.gui --help
./run.sh -m rgbd.camera --help
./run.sh -m rgbd.record --help
./run.sh -m rgbd.inspect --help
./run.sh -m rgbd.train --help
./run.sh -m rgbd.predict --help
```

## 本次建置與驗證紀錄

- 環境：專案內 `.conda/`，約 7 GB，Python 3.10.19。
- LeRobot 0.4.4，來源版本 `0f392484458cb5ebca0310c0c4c47390a31c80ed`；wheel SHA256：`89c402c62f8d82e0efe4d75ee53c95717e15301eaa62466991c3664d1353eb0c`。
- `pip check`、28 項測試、全部入口 `--help`、模擬錄製 → CPU 兩步訓練 → 離線推論通過。
- Python 載入路徑已確認不包含舊 LeRobot 原始碼或 ROS 工作區。
- 本次 CPU 流程已驗證；測試程序回報 CUDA 不可用，未驗證 GPU 訓練或實機連線。
- `environment.yml` 由此環境匯出並改用隨附 wheel；尚未在另一台機器執行完整重建。
