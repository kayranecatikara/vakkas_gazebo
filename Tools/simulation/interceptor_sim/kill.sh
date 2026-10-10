#!/usr/bin/env bash
#
# Kill all lingering simulation processes (Gazebo, PX4, Python background services)
# and clean up stale lock files and UNIX domain sockets.

echo "Tüm simülasyon süreçleri temizleniyor..."

# Kill PX4 instances
killall -9 px4 2>/dev/null || true
pkill -9 -f "bin/px4" 2>/dev/null || true

# Kill Gazebo Sim
killall -9 "gz sim" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true

# Kill Python background tools
pkill -9 -f "(target_sim|ground_relay|sim_detector|view_hud|web_gcs|gcs_gui)\.py" 2>/dev/null || true

# Clean up stale locks and domain sockets
rm -f /tmp/px4* /tmp/px4-sock-* 2>/dev/null || true

echo "Temizlik tamamlandı! Bütün süreçler sonlandırıldı."
