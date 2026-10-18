# PersonTracking

使用 **AMB82-MINI**、內建 YOLOv4-tiny 模型與 **MG996R** 伺服馬達，進行人物偵測與單軸水平追蹤，並透過 RTSP 顯示人物框與影像中心標記。

完整套件清單、安裝步驟、硬體接線與 Arduino CLI 指令請見 [requirements.txt](requirements.txt)。

## 快速開始

1. 安裝 Arduino IDE 2.x 與 **Realtek Ameba Boards 4.0.9**（build20250805），選擇 **AMB82-MINI**。
2. 將專案資料夾命名為 `PersonTracking`，開啟 `PersonTracking.ino`。從本儲存庫下載 ZIP 後，請將 `ameba-PresonTracking-main` 重新命名為 `PersonTracking`；使用 Git 複製時也請將本機資料夾命名為 `PersonTracking`。
3. 修改程式中的 `ssid`、`pass`。伺服馬達訊號使用 `SERVO_PIN = 8` 對應腳位；馬達使用獨立電源，並與開發板共地。
4. 在「工具」選擇正確攝影機、序列埠，以及 **NN Model Load From: Flash**，編譯後上傳。
5. 開啟 **115200 baud** 序列埠監控視窗，等候 `READY`，把輸出的 RTSP URL 貼到 VLC 的網路串流。
6. 固定好攝影機與支架後，輸入小寫 `t` 啟動追蹤。

所有程式函式庫均由 AmebaPro2 開發板套件提供，無須另外安裝通用 Servo 或 YOLO 函式庫。

## 操作指令

| 指令 | 功能 |
| --- | --- |
| `t` | 啟動追蹤 |
| `s` | 停止追蹤並停止 PWM，馬達仍接著電源 |
| `c` | 回到 1500 μs 中心位置並關閉追蹤 |
| `i` | 反轉追蹤方向 |
| `u` | 再次印出 RTSP URL |

啟動後追蹤預設關閉。偵測到多位人物時，選擇符合信心門檻且面積最大的偵測框；人物離開時保持位置。初始追蹤行程為 1200–1800 μs，請依機構調整 `MIN_US`、`MAX_US` 與 `direction`。

## 檔案

- `PersonTracking.ino`：Arduino 主程式。
- `requirements.txt`：可分享給其他使用者的環境需求與安裝說明。
- `.gitignore`：排除編譯產物與本機工具。

## 編譯確認

2026-10-18 使用 Arduino CLI 1.5.1 與本機 AmebaPro2 4.0.9-build20250805 編譯通過，韌體使用 57% 儲存空間。Windows CLI 的語系相容設定請見 `requirements.txt`。此確認僅涵蓋編譯，未燒錄或測試實機。

## 官方文件

- [AmebaPro2 Arduino SDK](https://github.com/Ameba-AIoT/ameba-arduino-pro2)
- [AMB82-MINI 入門](https://ameba-doc-arduino-sdk.readthedocs-hosted.com/en/latest/ameba_pro2/amb82-mini/Getting_Started/Getting%20Started%20with%20AMB82%20MINI.html)
