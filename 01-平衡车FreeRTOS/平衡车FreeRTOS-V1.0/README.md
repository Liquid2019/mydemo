# 平衡车 FreeRTOS（运行精简版）

路径：`01-平衡车FreeRTOS/平衡车FreeRTOS-V1.0`

只保留站立与遥控行驶，已去掉调试/校准/调参/Flash 存参。

## 保留

- TaskControl：姿态 + 三环 PID + 电机（TIM1 1ms 唤醒）
- TaskComm：NRF / 蓝牙遥控
- TaskUI：K1 启停、K2/K3 档位、简易 OLED
- PID 使用代码内固定默认参数

## 已删除

- DebugMode（硬件测试、传感器校准菜单）
- USB 串口调参、参数保存提示流程
- 状态回传遥测包（NRF 仅收遥控）

## 按键

| 键 | 功能 |
|----|------|
| 开机 K4 | 进入主程序 |
| K1 | 启动 / 停止平衡 |
| K2 / K3 | 速度档位 1～6 |

## 注意

校准偏移固定为 0。若车身机械不水平、站不稳，需改 `main.c` 里的 `GY_Offset` / `AngleAcc_Offset`，或回到原版工程做一次校准后把数值写死进来。
