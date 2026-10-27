#!/bin/bash
# ============================================================
# 远程推送脚本: 将 nvr_player_release.tar.gz 推送到设备
# 目标: 192.168.9.59  root/root
# 用法: ./push_to_device.sh
# ============================================================

REMOTE_IP="192.168.9.199"
REMOTE_USER="root"
REMOTE_PASS="root"
LOCAL_FILE="/home/dell/rk3572_player_nvr/rk3572_player"
REMOTE_DIR="/userdata/rk3572_player_release/bin"

echo "=========================================="
echo "  推送 nvr_player 到设备 ${REMOTE_IP}"
echo "=========================================="

if [ ! -f "${LOCAL_FILE}" ]; then
    echo "错误: 本地文件不存在: ${LOCAL_FILE}"
    exit 1
fi

echo "本地文件: ${LOCAL_FILE} ($(du -h ${LOCAL_FILE} | cut -f1))"
echo "远程路径: ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_DIR}"
echo ""

expect -c "
    set timeout 120
    spawn scp -o StrictHostKeyChecking=no ${LOCAL_FILE} ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_DIR}/
    expect {
        \"password:\"  { send \"${REMOTE_PASS}\r\"; exp_continue }
        \"yes/no\"     { send \"yes\r\"; exp_continue }
        \"Text file busy\" { puts \"\\n错误: 文件正在使用中，请先在板端停止进程\"; exit 1 }
        eof
    }
    catch wait result
    exit [lindex \$result 3]
"

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "  推送成功!"
    echo "=========================================="
    echo ""
    echo "  cd ${REMOTE_DIR}"
    echo "  ./run.sh"
    echo "  cd nvr_player_release && ./run.sh"
else
    echo ""
    echo "错误: 推送失败!"
    exit 1
fi