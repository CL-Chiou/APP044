更新日誌
===
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

[Unreleased]
---

[0.0.2] - 2021-04-15
---
### 臭蟲修復
- 修復傳送與接收RTCC當前時間的封包資料錯位問題 (resolved [#4])

[0.0.1] - 2021-04-09
---
### 新增功能
#### 顯示
 - LCD顯示器
	  - 第一行顯示 `Hello, World`
	  - 第二行顯示 `Son of San,Dick!`
 - 七段顯示器
	  - 上排顯示器顯示當前時間
	  - 下排顯示器顯示VR1的電壓值
 
#### 類比輸入
 - 4通道同時取樣；10-bit @ 1kHz取樣(基於Timer3)
 	- 取樣 AN0:VR1 並轉換結果
 	- 取樣 AN1:ANIN4 並轉換結果
 	- 取樣 AN2:ANIN3 並轉換結果
 	- 取樣 AN3:ANIN2 並轉換結果

#### 通訊

- CAN
	 - 透過按鍵觸／每20ms發連續傳送當前時間於CAN Bus
	 - 透過接收透定ID改寫RTCC時間並顯示於上排七段顯示器
	 - 每10ms傳送當前時間之"秒"
	 - 每100ms傳送讀取的VR1數值
	 - 每500ms傳送固定參數封包
- UART
	 - 每1ms傳送交替封包 @ 1kHz取樣(基於ADC取樣時機)
		  - 封包1 : UART1_TX_Protocol
		  - 封包2 : UART1_TX_BUFFER
- SPI
	 - 通過Protocol控制MCP4922輸出Sine Wave於**DAC_VoA**與**DAC_VoB**
- I2C
	 - i2c1
		- 通過Protocol控制MCP79410讀取／寫入RTCC晶片  
每60秒讀取一次RTCC時間至µC RAM並顯示於上排七段顯示器
	 - i2c2
		- 讀取VR1電壓值並通過Protocol控制MCP4551輸出於**I2C_DAC**

#### 按鍵
- DIP 開關 `SW-DIP`
	 - 當任意DSW被撥至**On**位置，即觸發傳送一次RTCC時間於CAN Bus
- 輕觸開關 `TACT-SW`
	 - 當任意BT被**按壓**，即觸發傳送一次RTCC時間於CAN Bus

當任意DSW被撥至**On**位置且任意BT被**按壓**超過1秒鐘，  
即開啟連續傳送RTCC時間至線上（頻率為50Hz），  
直到所有DSW撥至**Off**且所有BT回到未被**按壓**時狀態

#### 蜂鳴器
 - 當按下BT5鍵發出**Beep**聲，放開即停止**Beep**聲
 - 當按下BT3鍵時觸發**美妙旋律**
 - 開機時發出兩短促聲表示開機成功
 - 當時間來到**11:25**與**17:00**時發出**美妙旋律**


### 修改功能
- None

### 棄用功能 
- None

### 移除功能
- None

### 臭蟲修復
- None

### 安全性更新 
- None

[#4]: https://github.com/liohord/APP044/issues/4

[Unreleased]: https://github.com/liohord/APP044/compare/v0.0.1...HEAD
[0.0.2]: https://github.com/liohord/APP044/compare/0.0.1...0.0.2
[0.0.1]: https://github.com/liohord/APP044/releases/tag/0.0.1
