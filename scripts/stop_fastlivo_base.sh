#!/bin/bash
echo "stopping FAST-LIVO2 Demo Base..."

pkill -f "roslaunch.*mvs_camera_demo" 2>/dev/null || true
pkill -f "roslaunch.*livox_lidar_demo" 2>/dev/null || true
pkill -f "roslaunch.*mapping_avia_demo" 2>/dev/null || true
pkill -f "roslaunch.*mapping_avia" 2>/dev/null || true

sleep 2
echo "stopped"
