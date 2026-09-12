# LeRobot 手臂控制專案

完整操作與環境重建指令：[COMMANDS.md](COMMANDS.md)。套件已隔離至專案內 `.conda/`，使用 `./run.sh` 開啟介面。

**雙臂末端資訊、RGB＋深度錄製及 Diffusion Policy 訓練已加入：請看 [RGB-D 完整使用說明](RGBD_GUIDE.md)。**

## 目前正式流程

資料錄製在本機硬體上執行，伺服器負責保存資料與訓練：

- SSH：`itri2026@140.114.58.2`
- 資料：`/home/itri2026/lerobot_datasets`
- Checkpoint：`/home/itri2026/lerobot_checkpoints`
- 程式同步位置：`/home/itri2026/lerobot-arm-control`

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
./server.sh train recordings pick_place_v1 --steps 20000 --batch-size 8 --device cuda
```

正式錄製不需要手動執行 `server.sh sync`；該命令只用於手動補傳既有資料。

以下為原本的單組／雙組純遙控入口。

使用專案獨立環境內 LeRobot 的控制入口，支援 SO-100／SO-101 單組主從、雙組主從遙控。硬體已確認為兩組 SO-101 主從手臂。雙組需要兩支 leader（主臂）及兩支 follower（從臂），共四支手臂。此專案不是無主臂的自動軌跡控制。

找到的原始程式：
- `src/lerobot/scripts/lerobot_teleoperate.py`：控制迴圈與雙臂命令範例。
- `src/lerobot/robots/bi_so_follower/bi_so_follower.py`：拆分左右動作並依序送往兩支從臂。
- `src/lerobot/teleoperators/bi_so_leader/bi_so_leader.py`：讀取左右主臂。

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
