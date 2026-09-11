# 雙臂 RGB-D 示範資料 → Diffusion Policy

這個擴充專案提供雙 SO 主從遙控錄製、URDF 末端姿態估算、RGB＋原始深度保存，以及在伺服器執行的 LeRobot `DiffusionPolicy` 訓練入口。模型輸入包含深度；輸出依資料為單組 6 維或雙組 12 維絕對關節目標，末端姿態作為觀測輸入。

## 正式資料流程

本機只負責連接手臂／相機與錄製；伺服器設定如下：

```text
SSH       itri2026@140.114.58.2
資料      /home/itri2026/lerobot_datasets
Checkpoint /home/itri2026/lerobot_checkpoints
程式      /home/itri2026/lerobot-arm-control
```

實機 GUI 每完成一個 episode，就會自動以 checksum 上傳並驗證檔案清單；驗證成功才刪除本機 episode。上傳失敗、中斷錄製與 mock 錄製都會保留本機資料。資料集名稱取自 GUI 的 output 資料夾名稱。

已查到本機 RealSense D435I（序號 `944122072848`），已填入設定。手臂已確認為兩組 SO-101；四個串列埠與 URDF 尚待填寫；未執行實機手臂控制或正式示範訓練。

## 環境

完整指令與環境重建流程請見 [COMMANDS.md](COMMANDS.md)。

```bash
cd /home/itri2026-3090/Desktop/lerobot-arm-control
./run.sh -m rgbd.gui --mock
```

全部依賴使用專案 `.conda/` 環境，LeRobot 已封裝為固定版本 wheel；`run.sh` 隔離外部 Python 路徑。FK 使用本專案 URDF 解析器，不需要 placo。

## 1. 設定硬體與末端座標

修改 `configs/rgbd.json`：

- `robot.ports.left/right`：左右從臂。
- `teleop.ports.left/right`：左右主臂。
- `cameras`：每台 RealSense 的唯一名稱、序號、RGB／depth 共同支援的解析度和 FPS。預設一台前方 D435I，640×480、30 Hz，可新增相機。
- `kinematics.left/right.urdf`：實際機型與校正慣例相符的 URDF 絕對路徑。
- `base_link`、`tip_link`：URDF 中實際 link 名稱。
- `joint_map`：key 為 **URDF joint 名稱**；`motor` 為 LeRobot motor 名稱。預設名稱僅為範本，需依真實 URDF 修改。
- `scale`、`offset`：`URDF 關節值 = LeRobot 回授 × scale + offset`。轉動關節預設 degrees → radians；方向相反時需負號，零點不同時需 offset（radians）。移動關節必須指定 metres/unit。
- `world_from_base`：輸出座標系到手臂底座的關係，以 `T_output_base` 表示；平移單位公尺。預設 identity，`output_frame` 分別是 `left_base`、`right_base`。兩側預設不在同一世界座標；若要共用世界座標，先量測兩個底座外參並修改矩陣及 frame 名稱。
- `task`：此次示範的任務描述；目前模型不以文字作條件，建議同一資料目錄錄同一任務。

支援 URDF fixed、revolute、continuous、prismatic chain，遇 mimic 或不支援的 joint 類型會拒絕。末端姿態是由回授角度與模型計算的估計，沒有力感測，也不等同外部追蹤器測量。夾爪保存 0–100 正規化開合值，沒有宣稱毫米開口寬度。

本機 `BiSOLeader` 沒有傳遞 `use_degrees`；錄製程式以四支單臂介面建立雙臂控制，明確設定主從臂都使用 degrees。夾爪仍維持 LeRobot 的 0–100。左右通訊依序執行，不是硬體同步。

## 2. 校正、錄製、檢查

```bash
./run.sh control.py ports
./run.sh control.py calibrate-teleop --config configs/rgbd.json --run
./run.sh control.py calibrate-robot --config configs/rgbd.json --run
./run.sh -m rgbd.gui --config configs/rgbd.json --output data/recordings
```

錄製前請先完成校正。GUI 會實際連接相機和手臂；連接後按各組的 `Start Teleop`，確認示範動作正常，再按 `Start Recording`。每個完整 episode 會自動上傳到伺服器；不要在錄製期間手動刪除本機檔案。

第一次使用或程式更新後，執行：

```bash
./server.sh deploy
./server.sh train recordings pick_place_v1 \
  --steps 20000 --batch-size 8 --device cuda
```

`rgbd.record` 終端模式仍可用於進階除錯，但不會走 GUI 的自動逐 episode 上傳流程；正式收集資料請使用 GUI。

正常／異常退出均嘗試釋放已連接裝置；從臂斷線會關閉扭力，請先支撐手臂。軟體沒有碰撞規劃，保持硬體電源可切斷。預設 `max_relative_target=5` 在新錄製流程表示每次回授到目標的身體關節角度上限 5 度、夾爪 5 個正規化單位，並非每秒速度。

資料保存為每回合一個資料夾，每幀一個無損 `.npz`，完整回合才標示 `complete=true`；中斷回合留存但訓練會跳過。重跑錄製會建立下一個回合，不覆蓋資料。

| 資料欄位 | 內容 |
|---|---|
| `joints` | 每個啟用組 5 關節角度＋夾爪；單組 6、雙組 12 維 |
| `ee_transform` | 左右 4×4 末端變換矩陣，位置公尺 |
| `state` | 關節值＋每側 XYZ 與旋轉矩陣前兩欄；單組 15、雙組 30 維 |
| `requested_action` | 從主臂讀出的原始目標 |
| `action` | 實際送出的單組 6／雙組 12 維目標；未啟動遙控時為 NaN，並以 `action_valid=false` 標記 |
| `front__rgb` | 原始 RGB uint8 H×W×3 |
| `front__depth` | 對齊 RGB 的 uint16 H×W 原始深度，0 表示無效 |
| `host_time` / `action_host_time` | 主機 monotonic 回授完成／指令送出時間 |
| 相機附加欄位 | 主機收件時間、RGB／深度裝置時間、frame number、clock domain |

`episode.json` 保存深度比例尺（raw × scale = 公尺）、RGB 內參／畸變、手臂配置、URDF SHA256、frame 定義、有效 FPS 與最大時間間隔。相機外參目前為 null；要產生共同世界座標點雲，需另做相機外參校正。

RGB／深度從同一 RealSense frameset 取得，使用 SDK 對齊深度到 RGB。多台相機與手臂之間僅有軟體時間記錄，未設定硬體觸發同步。原始資料含不同時鐘的 timestamp，不能直接拿裝置 timestamp 相減当跨裝置延遲。

檔案使用無壓縮 NPZ，以避免深度精度損失。單台 640×480 RGB-D 原始影像約 1.54 MB/幀，30 Hz 約 2.8 GB/分鐘，另有少量狀態與檔案開銷。請依磁碟速度與容量調整錄製參數；`inspect` 可查看實際 FPS。若時間間隔過大，訓練會要求降低錄製 FPS 後重錄。

新錄製為格式版本 2，`episode.json` 記錄 `active_sides`、`teleop_sides` 及實際啟用相機。純相機回合沒有手臂陣列；未連接手臂不補零。UI 的相機預覽／手臂連線／遙控／錄製控制彼此分開，完整流程見 [COMMANDS.md](COMMANDS.md#6-ui-獨立硬體控制與錄製)。UI 停止或完成錄製會停止遙控，保持裝置連線與預覽；關閉 UI 才斷開全部裝置。

## 3. 伺服器訓練

```bash
./server.sh deploy
./server.sh train recordings pick_place_v1 \
  --steps 20000 --batch-size 8 --device cuda
```

伺服器訓練等同於在伺服器執行 `rgbd.train`，資料來源是
`/home/itri2026/lerobot_datasets/recordings`，checkpoint 寫入
`/home/itri2026/lerobot_checkpoints/pick_place_v1`。本機 `data/` 與 `outputs/` 只保留給離線除錯，不是正式資料路徑。

此訓練入口要求至少一組手臂及一台相機，且全部已錄製手臂都有遙控動作標籤；純影像、純手臂或只讀回授資料可錄製／檢查，但不適用本影像条件訓練入口。不同手臂組合不可混訓。

這是本專案 NPZ 格式＋PyTorch Dataset adapter，直接呼叫本機 LeRobot DiffusionPolicy，**不是**原生 LeRobotDataset；請使用此訓練入口，不可將資料路徑直接交給 `lerobot-train`。

模型觀測含過去兩幀；動作 horizon=16、每次預測執行區段 8 步。資料依實際主機時間找最近的 30 Hz 目標樣本（以設定 FPS 為準），只使用同回合內完整視窗，不跨回合填補。訓練拒絕非遞增 timestamp 與 >1.75 個 frame period 的缺口。

RGB 轉為 0–1，深度先由原始值轉公尺，再依固定 `--max-depth-m`（預設 3 m）縮放；有效像素為 `0.01 + 0.99 × clip(depth_m / max_depth_m, 0, 1)`，無效像素為 0。深度複製為 3 通道並使用獨立 encoder，和 RGB 共同作為 diffusion 條件。這是 RGB＋深度影像版本，未實作點雲 Diffusion Policy。原始深度不受訓練縮放影響，也不使用彩色深度視覺化來代替距離。

state/action 依訓練資料 min/max 正規化，預設影像縮為 96×96，不做隨機 crop。每 1000 步與最後一步保存 checkpoint、optimizer 狀態與 `preprocessing.json`；输出目錄必須是新路徑。目前未提供續訓 CLI、驗證集評估或實機自主部署，loss 只代表训练損失，不能作為任務成功率。

推論必须使用相同影像處理、state 順序、深度尺度及 action 反正規化；這些都記在 checkpoint 的 `preprocessing.json`。提供離線重載驗證：

```bash
./run.sh -m rgbd.predict \
  --checkpoint /home/itri2026/lerobot_checkpoints/pick_place_v1/checkpoint_020000 \
  --data /home/itri2026/lerobot_datasets/recordings --output predicted_actions.json
```

此指令只輸出預測的關節目標，不連接手臂。

## 4. 無硬體測試

```bash
./run.sh -m unittest discover -s tests -v
./run.sh -m rgbd.record --mock --output data/mock_check --episodes 2 --seconds 0.6
./run.sh -m rgbd.train --data data/mock_check --output outputs/mock_check \
  --steps 2 --batch-size 2 --small --allow-mock --device cpu
```

模擬資料標示 synthetic，正式訓練預設拒絕；只有明確 `--allow-mock` 才可作流程測試，不可與真實資料混合。

已在專案獨立環境驗證：28 項測試（包含 UI）、兩個模擬回合、RGB＋depth 條件下 2 次反向傳播、保存 checkpoint 並離線重載推論。測試覆蓋已知幾何的 FK、座標變換、深度保真與尺度、回合隔離、時間缺口、未完成回合跳過。

參考：本機 `src/lerobot/policies/diffusion/` 與 [LeRobot 官方訓練範例](https://github.com/huggingface/lerobot/blob/main/examples/tutorial/diffusion/diffusion_training_example.py)；RGB-D 對齊／深度比例尺依 [RealSense 官方範例](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)。實作以本機版本 API 為準。

## 即時觀測介面

執行 `./run.sh -m rgbd.gui` 開啟錄製視窗，或加 `--mock` 使用合成資料測試。啟動方式、顯示內容與停止行為請見 [README 的即時錄製介面](README.md#即時錄製介面)。原有 `./run.sh -m rgbd.record` 終端錄製方式仍可使用。
