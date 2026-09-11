#!/bin/bash
set -euo pipefail

# ============================================================
# YoboGo 四足机器狗 — 探测机器人内部资源脚本
# 用途：SSH 连接到机器人并全面探测系统信息和资源
# 使用：bash scripts/explore_robot.sh
# 输出：结果保存到 scripts/robot_explore_result.txt
# ============================================================

# ---------- 颜色定义 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

# ---------- 机器人配置 ----------
ROBOT_IP="10.0.0.34"
ROBOT_USER="user"
ROBOT_PASS="123456"

# ---------- 输出文件 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_FILE="${SCRIPT_DIR}/robot_explore_result.txt"

# ---------- 工具函数 ----------
info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
ok()      { echo -e "${GREEN}[  OK]${RESET}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
fail()    { echo -e "${RED}[FAIL]${RESET}  $*"; }
section() { echo -e "\n${BOLD}===== $* =====${RESET}"; }

# ---------- 依赖检查 ----------
echo -e "${BOLD}YoboGo 四足机器狗 — 资源探测${RESET}"
echo ""

if ! command -v sshpass &>/dev/null; then
    fail "sshpass 未安装，无法自动 SSH 连接"
    echo -e "    安装命令: ${BOLD}sudo apt install sshpass${RESET}"
    exit 1
fi
ok "sshpass 已安装"

if ! command -v ping &>/dev/null; then
    fail "ping 未安装"
    echo -e "    安装命令: ${BOLD}sudo apt install iputils-ping${RESET}"
    exit 1
fi

# ---------- 连通性预检 ----------
section "连通性预检"
info "正在 ping ${ROBOT_IP}..."
if ! ping -c 3 -W 2 "$ROBOT_IP" &>/dev/null; then
    fail "机器人 ${ROBOT_IP} 不可达，请先运行 connect_check.sh 排查"
    exit 1
fi
ok "机器人 ${ROBOT_IP} 可达"

# ---------- SSH 远程执行函数 ----------
# 执行一条命令并捕获输出
remote_exec() {
    local title="$1"
    local cmd="$2"
    local separator="────────────────────────────────────────"
    
    echo "" >> "$OUTPUT_FILE"
    echo "$separator" >> "$OUTPUT_FILE"
    echo "【${title}】" >> "$OUTPUT_FILE"
    echo "命令: ${cmd}" >> "$OUTPUT_FILE"
    echo "$separator" >> "$OUTPUT_FILE"
    
    # 执行命令并写入文件
    sshpass -p "$ROBOT_PASS" ssh \
        -o StrictHostKeyChecking=no \
        -o ConnectTimeout=5 \
        -o BatchMode=no \
        "${ROBOT_USER}@${ROBOT_IP}" "$cmd" >> "$OUTPUT_FILE" 2>&1 || echo "[命令执行失败]" >> "$OUTPUT_FILE"
    
    echo "" >> "$OUTPUT_FILE"
}

# ============================================================
section "开始探测机器人"
# ============================================================
info "探测结果将保存到: ${OUTPUT_FILE}"
echo ""

# 初始化输出文件
cat > "$OUTPUT_FILE" << 'HEADER'
╔══════════════════════════════════════════════════════════════╗
║          YoboGo 四足机器狗 — 内部资源探测报告               ║
╚══════════════════════════════════════════════════════════════╝
HEADER

echo "生成时间: $(date '+%Y-%m-%d %H:%M:%S')" >> "$OUTPUT_FILE"
echo "探测目标: ${ROBOT_USER}@${ROBOT_IP}" >> "$OUTPUT_FILE"

# ---------- 1. 系统信息 ----------
info "1/16 获取系统信息..."
remote_exec "系统信息 (uname -a)" "uname -a"

info "2/16 获取发行版信息..."
remote_exec "发行版信息 (/etc/os-release)" "cat /etc/os-release"

# ---------- 2. 硬件资源 ----------
info "3/16 获取磁盘使用..."
remote_exec "磁盘使用 (df -h)" "df -h"

info "4/16 获取内存使用..."
remote_exec "内存使用 (free -h)" "free -h"

info "5/16 获取 CPU 和进程概览..."
remote_exec "CPU 和进程概览 (top -bn1)" "top -bn1 | head -30"

# ---------- 3. 用户目录结构 ----------
info "6/16 查看用户目录..."
remote_exec "用户目录 (/home/user/)" "ls -la /home/user/"

info "7/16 查看 dog 用户目录..."
remote_exec "dog 用户目录 (/home/dog/)" "ls -la /home/dog/"

# ---------- 4. 关键软件目录 ----------
info "8/16 查看运动控制软件..."
remote_exec "运动控制软件 (/home/user/robot-software/)" \
    "ls -la /home/user/robot-software/"

info "9/16 查看运动控制编译输出..."
remote_exec "运动控制编译输出 (/home/user/robot-software/build/)" \
    "ls -la /home/user/robot-software/build/"

info "10/16 查看循迹程序..."
remote_exec "循迹程序 (/home/user/track1.1/)" \
    "ls -la /home/user/track1.1/"

info "11/16 查看循迹编译输出..."
remote_exec "循迹编译输出 (/home/user/track1.1/build/)" \
    "ls -la /home/user/track1.1/build/"

info "12/16 查看视觉程序..."
remote_exec "视觉程序 (/home/dog/dogvision/)" \
    "ls -la /home/dog/dogvision/"

# ---------- 5. 硬件设备 ----------
info "13/16 扫描摄像头设备..."
remote_exec "摄像头设备 (/dev/video*)" "ls -la /dev/video* 2>/dev/null || echo '未找到摄像头设备'"

info "14/16 扫描 SPI 设备..."
remote_exec "SPI 设备 (/dev/spidev*)" "ls -la /dev/spidev* 2>/dev/null || echo '未找到 SPI 设备'"

# ---------- 6. 网络和进程 ----------
info "15/16 获取网络接口信息..."
remote_exec "网络接口 (ip addr)" "ip addr || ifconfig"
remote_exec "路由表" "ip route"

info "16/16 检查关键进程..."
remote_exec "关键进程 (mit_ctrl / track / ros)" \
    "ps aux | grep -E 'mit_ctrl|track|ros|dogvision' | grep -v grep || echo '未发现关键进程'"

# ---------- 7. 服务和网络配置 ----------
info "  补充: 获取运行中的服务..."
remote_exec "运行中的服务 (systemctl)" \
    "systemctl list-units --type=service --state=running 2>/dev/null || echo 'systemctl 不可用'"

info "  补充: 获取网络配置..."
remote_exec "网络配置 (/etc/network/interfaces)" \
    "cat /etc/network/interfaces 2>/dev/null || echo '文件不存在'"
remote_exec "网络配置 (/etc/netplan/)" \
    "ls -la /etc/netplan/ 2>/dev/null && cat /etc/netplan/*.yaml 2>/dev/null || echo 'netplan 目录不存在'"
remote_exec "hostname 和 hosts" "hostname && cat /etc/hosts"

# ---------- 8. 额外信息 ----------
info "  补充: USB 设备..."
remote_exec "USB 设备列表" "lsusb 2>/dev/null || echo 'lsusb 不可用'"
info "  补充: PCI 设备..."
remote_exec "PCI 设备列表" "lspci 2>/dev/null || echo 'lspci 不可用'"
info "  补充: 内核模块..."
remote_exec "已加载内核模块" "lsmod 2>/dev/null | head -30 || echo 'lsmod 不可用'"

# ---------- 完成 ----------
echo "" >> "$OUTPUT_FILE"
echo "═══════════════════════════════════════════════════════════════" >> "$OUTPUT_FILE"
echo "探测完成" >> "$OUTPUT_FILE"

echo ""
section "探测完成"
ok "所有探测结果已保存到: ${OUTPUT_FILE}"
info "文件大小: $(du -h "$OUTPUT_FILE" | awk '{print $1}')"
info "总行数: $(wc -l < "$OUTPUT_FILE")"
echo ""
echo -e "查看结果: ${BOLD}cat ${OUTPUT_FILE}${RESET}"
echo -e "或分段查看: ${BOLD}less ${OUTPUT_FILE}${RESET}"
echo ""
