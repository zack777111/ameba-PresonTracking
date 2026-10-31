# 智慧會議鏡頭自動追蹤系統

README 環境與操作說明

## 材料與接線

材料：AMB82-MINI 開發板 1 塊、相容攝影機 1 組、MG996R 伺服馬達 1 顆、旋轉支架與底座 1 組、USB 資料線及杜邦線。

電源紅線接 V_USB（5V）；接地棕線接開發板 GND。控制訊號線接 PF15，對應程式 SERVO_PIN = 8。PF15 是 PWM 訊號腳

## SDK 模型及相機設定

| 項目 | 使用設定 |
| --- | --- |
| 工具與 SDK | Arduino IDE 2.x；Realtek Ameba Boards 4.0.9（build20250805）；開發板選 AMB82-MINI。 |
| 辨識模型 | SDK 內建 DEFAULT_YOLOV4TINY；偵測 Person；NN Model Load From 選 Flash。 |
| 相機串流 | 通道 0；HD／20 FPS；H.264；透過 RTSP 觀看。 |
| 偵測影像 | 通道 3；576 × 320／10 FPS；RGB。 |
| 序列埠 | 115200 baud；供狀態訊息與操作指令使用。 |

## 燒錄與操作方式

1. 安裝上述 IDE 與開發板套件。專案資料夾為 PersonTracking，開啟 PersonTracking.ino，填入 WiFi 的 ssid 與 pass，選擇 AMB82-MINI 和正確序列埠。

2. 手動進入燒錄模式，按住 UART_DOWNLOAD，按下並放開 RESET，再放開 UART_DOWNLOAD；燒錄完成後按 RESET 執行。

3. 開啟 115200 baud 序列埠監控，等候 READY 與 RTSP URL。筆電和開發板連上同一個 WiFi，將完整 RTSP URL 貼入 VLC 的網路串流，查看人物框與 ID。

4. 固定鏡頭與支架後輸入 t 開始追蹤。s 停止追蹤與 PWM，c 回中並關閉追蹤，i 反轉方向，u 再次顯示串流網址。測試人物左右移動、離開畫面與 180 度搜尋。
