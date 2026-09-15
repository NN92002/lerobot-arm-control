# LeRobot 手臂控制專案

完整操作與環境重建指令：[COMMANDS.md](COMMANDS.md)。套件已隔離至專案內 `.conda/`，使用 `./run.sh` 開啟介面。

**雙臂末端資訊、RGB＋深度錄製及 Diffusion Policy 訓練已加入：請看 [RGB-D 完整使用說明](RGBD_GUIDE.md)。**

## 目前正式流程

資料錄製在本機硬體上執行，伺服器負責保存資料與訓練。以下是目前建議的完整流程；除標示
「伺服器」的指令外，都在本機執行。

### 1. 初始化與確認接口

```bash
cd /home/itri2026-3090/Desktop/lerobot-arm-control
./run.sh control.py ports
```

在資料蒐集 GUI 的 `Rescan Ports` 找到接口並按 `Save Ports`。四個接口對應如下：

| 角色 | 設定欄位 |
|---|---|
| 左主臂 Leader | `teleop.ports.left` |
| 左從臂 Follower | `robot.ports.left` |
| 右主臂 Leader | `teleop.ports.right` |
| 右從臂 Follower | `robot.ports.right` |

確認 `configs/rgbd.json` 中的相機、URDF、link 與 joint mapping 都已設定。

### 2. 校正四支手臂

先預覽，再執行實際校正：

```bash
./run.sh control.py calibrate-teleop --config configs/rgbd.json
./run.sh control.py calibrate-robot --config configs/rgbd.json

./run.sh control.py calibrate-teleop --config configs/rgbd.json --run
./run.sh control.py calibrate-robot --config configs/rgbd.json --run
```

校正檔會存到 `calibration/dual/teleop/` 與 `calibration/dual/robot/`。

### 3. 資料蒐集

```bash
./run.sh -m rgbd.gui \
	--config configs/rgbd.json \
	--output data/recordings
```

GUI 操作順序：

1. 連接相機。
2. 連接左、右 Arm pair。
3. 左、右兩側按 `Start Teleop`。
4. 按 `Start Recording`，完成示範後按 `Stop Recording`。
5. 選擇 `positive` 或 `negative` 標籤。

完整 episode 會自動上傳至伺服器的
`/home/itri2026/lerobot_datasets/recordings`。只有 `positive` episode 會用於訓練。
上傳失敗或中斷的資料會保留在本機，不要直接刪除。

檢查本地資料：

```bash
./run.sh -m rgbd.inspect data/recordings
```

若使用終端錄製而非 GUI，需手動同步資料：

```bash
./run.sh -m rgbd.record --config configs/rgbd.json \
	--output data/recordings --episodes 10 --seconds 30
./server.sh sync data/recordings recordings
```

### 4. 訓練 Diffusion Policy（伺服器）

程式有更新時，先同步專案：

```bash
./server.sh deploy
```

使用伺服器上由 GUI 自動上傳的 `recordings` 資料集開始訓練：

```bash
./server.sh train recordings task1_dp_v5 \
	--steps 10000 --batch-size 8 --device cuda
```

訓練完成後，checkpoint 位於伺服器：

`/home/itri2026/lerobot_checkpoints/task1_dp_v5`

### 5. 下載 checkpoint 到本機

```bash
mkdir -p checkpoints/task1_dp_v5
rsync -av --partial --progress \
	itri2026@140.114.58.2:/home/itri2026/lerobot_checkpoints/task1_dp_v5/ \
	checkpoints/task1_dp_v5/
```

確認至少存在：

```text
checkpoints/task1_dp_v5/model/checkpoint_best/config.json
checkpoints/task1_dp_v5/model/checkpoint_best/model.safetensors
checkpoints/task1_dp_v5/model/checkpoint_best/preprocessing.json
```

### 6. 部署模型到手臂

```bash
./run.sh -m rgbd.policy_gui --config configs/rgbd.json
```

GUI 操作順序：

1. 選擇本地 checkpoint，按 `Load Checkpoint`。
2. 選擇左右 follower port，必要時按 `Rescan Ports`，再按 `Save Ports`。
3. 按 `Start Camera`。
4. 按 `Start Left Follower` 與 `Start Right Follower`。
5. 每支手臂會讓 `shoulder_pan` 小幅移動約 2 度後回位，用來確認左右接口沒有接錯。
6. 確認周圍安全後按 `Start Policy`。

`Stop Policy` 會停止送出動作但保持連線；`EMERGENCY STOP` 會停止推論並斷開相機與
手臂。軟體急停不是硬體電源級急停，實機操作仍須保留實體斷電措施。

### 7. 無硬體測試

```bash
QT_QPA_PLATFORM=offscreen ./run.sh -m unittest discover -s tests -v
./run.sh -m rgbd.gui --mock --output data/mock_ui
```


使用 `configs/rgbd.json` 啟動實機 GUI 時，每個完整 episode 錄完會先讓你選擇
`positive` 或 `negative`，接著在背景以 `rsync --checksum` 上傳並驗證遠端檔案，最後才刪除本機 episode。
上傳期間可以直接開始下一個 episode，不需要等待前一回合傳完。

按 `Start Recording` 後會倒數 3 秒才開始收資料；一般 `Stop Recording` 只結束目前回合，Teleop 會保持運作。
只有 `Emergency Stop` 會停止 Teleop 並斷開手臂。
上傳失敗或中斷錄製會保留本機檔案；訓練時會自動跳過 `negative` episode。

```bash
cd /home/itri2026-3090/Desktop/lerobot-arm-control
./run.sh -m rgbd.gui --config configs/rgbd.json --output data/recordings
```

第一次使用或程式更新後，先同步程式並在伺服器訓練：

```bash
./server.sh deploy
./server.sh train recordings pick_place_v1 --steps 100 --batch-size 8 --device cuda
```

正式錄製不需要手動執行 `server.sh sync`；該命令只用於手動補傳既有資料。

以下為原本的單組／雙組純遙控入口。

使用專案獨立環境內 LeRobot 的控制入口，支援 SO-100／SO-101 單組主從、雙組主從遙控。硬體已確認為兩組 SO-101 主從手臂。雙組需要兩支 leader（主臂）及兩支 follower（從臂），共四支手臂。此專案不是無主臂的自動軌跡控制。

找到的原始程式：

左右臂在同一迴圈依序通訊，並非硬體級同時觸發；設定 30 Hz 是目標頻率，實際速度取決於裝置。

## 即時錄製介面

```bash
cd /home/itri2026-3090/Desktop/lerobot-arm-control
./run.sh -m rgbd.gui --config configs/rgbd.json --output data/recordings
```

沒有硬體時才使用 `--mock`；mock 資料只用於測試，不會上傳伺服器。

介面可獨立啟動相機、左組與右組手臂：

1. 在 Camera connections 按該相機的 `Connect`，即可預覽，不必連接手臂。
2. 在 Arm connections 設定要用的那一組主從接口，按 `Connect`。此時只讀取狀態，不發送跟隨目標。
3. 要示範動作時，按該組 `Start Teleop`；左右組互相獨立。
4. 按 `Start Recording`，只保存已連線的硬體：左組、右組、雙組、純相機、純手臂皆可。
5. `Stop Recording` 保留未完成回合並停止遙控，裝置保持連線／預覽；每回合完成後需再次按 `Start Teleop` 才會恢復跟隨。`Disconnect` 或關閉視窗才釋放裝置。

單組資料只有 6 維 joints/action 與 15 維 state，雙組為 12／30 維。纯相機資料不包含手臂欄位。不會對未連線手臂補零。只讀狀態的手臂以 `action_valid=false` 和 NaN 表示未發送指令；訓練用示範需先啟動所有已連線手臂的遙控，並至少連接一台相機。

`Rescan Ports` 顯示新增／拔除的接口；`Save Ports` 只更新目前的 JSON，未使用的組可保持範本值。錄製期間鎖定裝置組合，需先停止錄製才能增減裝置。單組連線及錄製不檢查另一組的接口或 URDF。

實機手臂需先完成校正。僅連線與讀取關節時不要求 URDF；保存手臂資料時，啟用組的 URDF 與 FK 設定必須有效。末端座標為推算值。詳細校正與單組指令見 [COMMANDS.md](COMMANDS.md)。

介面採用英文 PyQt6，預覽約 10 Hz；儲存目標速率由設定 FPS 決定。`Stop Recording` 不是硬體急停。裝置讀取／寫入失敗會結束當次錄製、標記未完成並嘗試斷開所有已連線裝置。

## 使用

```bash
cd /home/itri2026-3090/Desktop/lerobot-arm-control
./run.sh control.py ports
```

編輯 `configs/dual.json` 的四個 `ports`，填入實際裝置路徑。建議使用 `/dev/serial/by-id/` 下固定名稱。`robot` 是從臂，`teleop` 是主臂；左右必須對應。若只有一組主從，改用 `configs/single.json`，並在每個指令加上 `--config configs/single.json`。單組可將 `model` 設為 `so100` 或 `so101`；雙組使用原始碼共用的 `bi_so_*` 類別。

設定中的 Python 指向專案 `.conda/bin/python`，`lerobot_path` 為 null，使用環境內安裝的 LeRobot 快照。

先預覽，再校正主臂及從臂：

```bash
./run.sh control.py calibrate-teleop
./run.sh control.py calibrate-teleop --run
./run.sh control.py calibrate-robot --run
```

依終端提示校正，校正資料存於專案的 `calibration/`。手臂應已完成原廠要求的馬達 ID 與接線設定。

開始遙控：

```bash
./run.sh control.py teleoperate
./run.sh control.py teleoperate --run
```

不加 `--run` 一律僅列印指令；Ctrl+C 結束。原始 LeRobot 正常結束時會斷線並關閉從臂扭力，請先支撐手臂。清空活動範圍，保持可切斷硬體電源；軟體中斷不等於硬體急停。

預設 `max_relative_target=5` 限制每次目標相對目前位置的變化，採 LeRobot 預設正規化單位，並非每秒角速度或固定 5 度。它不是避障機制。

## 驗證與限制

`./run.sh -m unittest discover -s tests -v` 驗證指令產生、校正角色隔離及重複連接埠拒絕。預覽與測試不會連接硬體。

目前未偵測到 USB 串列手臂裝置，尚未做實機校正與動作測試。依賴由專案 `.conda` 環境提供，重建方式請見 COMMANDS.md。

## 10. 實機 Diffusion Policy 控制

指定的伺服器 checkpoint 已下載至 `checkpoints/task1_dp_v4`。啟動推論介面：

```bash
./run.sh -m rgbd.policy_gui --config configs/rgbd.json
```

介面可從本地下拉選擇 checkpoint，也可從伺服器下載其他 checkpoint。按下左右
`Start Follower` 時，該手臂會先讓 `shoulder_pan` 以校正後的馬達正方向移動約 2 度，
停留後回到原位，方便確認 USB 接口與左右手臂沒有接錯；完成後才算連線成功。左右手臂
若為鏡像安裝，視覺上的順時針方向可能不同，請以實際手臂運動確認方向。

請依序載入模型、啟動相機、啟動左右 follower，再按 `Start Policy`。`Stop Policy` 只
停止送出動作並保持連線；`EMERGENCY STOP` 會停止推論並斷開相機與兩支 follower。
軟體急停不是硬體級電源急停，實機測試仍需保留可直接切斷馬達電源的措施。
