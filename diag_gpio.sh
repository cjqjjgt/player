#!/bin/sh
# ============================================================
# NAU8822 GPIO 输出诊断脚本
# 在板端运行: chmod +x diag_gpio.sh && ./diag_gpio.sh
# ============================================================

echo "=== 1. NAU8822 全部控件列表 ==="
amixer -c 1 controls 2>/dev/null || amixer -c 0 controls 2>/dev/null

echo ""
echo "=== 2. 耳机/喇叭/GPIO 相关控件 ==="
amixer -c 1 controls 2>/dev/null | grep -iE "hp|spk|headphone|speaker|jack|gpio|detect" || \
amixer -c 0 controls 2>/dev/null | grep -iE "hp|spk|headphone|speaker|jack|gpio|detect"

echo ""
echo "=== 3. hp switch 详细状态 ==="
amixer -c 1 sget "hp switch" 2>/dev/null || amixer -c 0 sget "hp switch" 2>/dev/null

echo ""
echo "=== 4. spk switch 详细状态 ==="
amixer -c 1 sget "spk switch" 2>/dev/null || amixer -c 0 sget "spk switch" 2>/dev/null

echo ""
echo "=== 5. 尝试命令行写入后立即回读 ==="
echo "--- 写入 hp switch ---"
amixer -c 1 cset numid=80 1 2>/dev/null || amixer -c 0 cset numid=80 1 2>/dev/null
echo "--- 立即回读 hp switch ---"
amixer -c 1 cget numid=80 2>/dev/null || amixer -c 0 cget numid=80 2>/dev/null

echo ""
echo "--- 写入 spk switch ---"
amixer -c 1 cset numid=79 1 2>/dev/null || amixer -c 0 cset numid=79 1 2>/dev/null
echo "--- 立即回读 spk switch ---"
amixer -c 1 cget numid=79 2>/dev/null || amixer -c 0 cget numid=79 2>/dev/null

echo ""
echo "=== 6. Jack Detect / 耳机插入检测 ==="
amixer -c 1 controls 2>/dev/null | grep -i jack
amixer -c 1 sget "Headphone Jack" 2>/dev/null
amixer -c 1 sget "Jack Detect" 2>/dev/null
amixer -c 1 sget "Mic Jack" 2>/dev/null

echo ""
echo "=== 7. GPIO sysfs 信息 ==="
echo "--- /sys/class/gpio/ ---"
ls /sys/class/gpio/ 2>/dev/null
echo ""
echo "--- /sys/kernel/debug/gpio ---"
cat /sys/kernel/debug/gpio 2>/dev/null | head -40 || echo "debugfs not mounted"

echo ""
echo "=== 8. DAPM debugfs (如果可用) ==="
find /sys/kernel/debug/asoc -name "dapm" -type d 2>/dev/null
find /sys/kernel/debug/asoc -type f 2>/dev/null | head -20

echo ""
echo "=== 9. 设备树 NAU8822 信息 ==="
find /sys/firmware/devicetree -name "*nau*" 2>/dev/null
find /sys/firmware/devicetree -name "*audio*" 2>/dev/null

echo ""
echo "=== 10. NAU8822 寄存器 (如果可用) ==="
cat /sys/kernel/debug/regmap/1-001a/registers 2>/dev/null | head -20 || \
cat /sys/kernel/debug/regmap/*/registers 2>/dev/null | head -20 || \
echo "regmap debugfs not available"

echo ""
echo "=== 诊断完成 ==="