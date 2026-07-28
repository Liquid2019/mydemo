师兄版调用链打包（可独立跑，无大文件）
========================================

重要: 这一版 Base 用的是 FAST-LIVO2，不是 lite_slam。

调用链:
  ./democtl.sh menu
    -> scripts/base_worker.sh
         相机: drivers/launch/mvs_camera_demo.launch
         雷达: drivers/launch/livox_lidar_demo.launch
         定位: roslaunch fast_livo mapping_avia_demo.launch  (需 ~/fastlivo2_ws)
    -> run_mark.sh  /  run_3.sh

本包不含:
  lite_slam/、drivers/fastlivo/、tools/、backups/、logs/
  Start-Demo-Base / Lite-SLAM 等未走 democtl 的旧图标
  *.pt / *.engine / *.onnx / *.npz

前置:
  ROS Noetic
  ~/fastlivo2_ws 已编译: mvs_ros_driver、livox_ros_driver、fast_livo

安装:
  tar -xzf my_demo_standalone_YYYYMMDD.tar.gz -C ~/Desktop
  chmod +x ~/Desktop/my_demo/*.sh ~/Desktop/my_demo/scripts/*.sh
  cp ~/Desktop/my_demo/desktop/智能避障控制台.desktop ~/Desktop/
  gio set ~/Desktop/智能避障控制台.desktop metadata::trusted true 2>/dev/null || true

使用:
  cd ~/Desktop/my_demo && ./democtl.sh menu
  # 1 状态检查  2 启动/重启 Base  3 画危险区  4 距离检测

模型需自行准备（不在本包）:
  yolov8n-seg.pt  或  yolov8n-seg_i416.engine
