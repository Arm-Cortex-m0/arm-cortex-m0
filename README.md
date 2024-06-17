# arm-cortex-m0
## 簡介
* 馮紐曼架構
* 32 位元精簡指令集 ( RISC ) 處理器
* Armv6-M 微架構
* 32個物理中斷
* 睡眠模式
* Hardware single-cycle ( 32x32 ) multiply
## Program Rom 生成方式
1. 撰寫 Arm 組合語言
2. 透過 Keil uVision5 Debugger 產生 機器碼
3. 使用 asm2sv_.py 即可將其轉換成 Program Rom
