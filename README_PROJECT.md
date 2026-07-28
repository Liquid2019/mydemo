# 智能避障项目说明

这个项目现在统一从 `democtl.sh` 进入。原来的脚本都保留，但日常建议只用这个入口，避免同时开多个旧窗口导致状态乱掉。

## 日常启动

在项目目录执行：

```bash
cd /home/jetson/Desktop/my_demo
./democtl.sh menu
```

也可以直接双击桌面的“智能避障控制台”。

## 推荐流程

1. 选择“状态检查”
2. 选择“启动/重启 Base”
3. 选择“重新画危险区”
4. 画好后退出画区窗口
5. 选择“启动距离检测”

## 常用命令

```bash
./democtl.sh status        # 检查相机、雷达、定位、危险区、检测窗口
./democtl.sh restart-base  # 重启 Base
./democtl.sh mark          # 重新画危险区
./democtl.sh detect        # 启动距离检测
./democtl.sh stop-app      # 只关画区/检测窗口
./democtl.sh stop-all      # 全部停止
./democtl.sh logs          # 看最近日志
```

## 当前项目结构

```text
my_demo/
  democtl.sh                 统一控制入口
  demo_config.env            日常参数配置
  2_fastlivo.py              危险区标注
  3_fastlivo.py              目标检测 + 距离显示
  fastlivo_calib.py          相机/雷达/位姿辅助计算
  run_mark.sh                旧的画区启动脚本
  run_3.sh                   旧的检测启动脚本
  scripts/base_worker.sh     后台 Base 启动器
  scripts/start_fastlivo_base.sh
  scripts/stop_fastlivo_base.sh
  desktop/智能避障控制台.desktop
  logs/                      日志
  backups/                   危险区和模型备份
```

## 状态怎么看

`./democtl.sh status` 里最关键的是这几项：

- 相机画面有 Hz：相机正常。
- 雷达点云有 Hz：雷达正常在线。
- 实时位姿有 Hz：FAST-LIVO 正在给出定位。
- 危险区显示“多少个点”：说明已经画过危险区。
- 距离检测窗口运行中：说明第二阶段正在跑。

如果雷达显示“暂时没有新数据”，通常是雷达没供电、网线/IP 没通，或者驱动起来了但设备没出数据。

如果实时位姿没有数据，距离和危险区会容易漂，要先解决 Base 的相机、雷达、定位。

## 重新画危险区

运行 `./democtl.sh mark` 时，会先把旧的 `danger_points_global.npz` 备份到 `backups/`，再打开画区窗口。

这样画错了也可以找回旧文件。

## 配置位置

日常只改：

```text
demo_config.env
```

现在默认是稳定优先的配置：不牺牲可靠性，优先保证距离不要错、不要漏显示。
