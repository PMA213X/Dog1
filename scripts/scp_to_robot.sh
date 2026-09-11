#!/bin/bash
set -euo pipefail

# ============================================================
# YoboGo 四足机器狗 — 文件传输脚本
# 用途：将本地文件/目录传输到机器人
# 使用：bash scripts/scp_to_robot.sh <本地路径> [远程目录]
#       bash scripts/scp_to_robot.sh /path/to/file
#       bash scripts/scp_to_robot.sh /path/to/dir/ /home/user/custom/
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
DEFAULT_REMOTE_DIR="/home/user/"

# ---------- 工具函数 ----------
info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
ok()      { echo -e "${GREEN}[  OK]${RESET}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
fail()    { echo -e "${RED}[FAIL]${RESET}  $*"; }

# ---------- 使用帮助 ----------
usage() {
    echo -e "${BOLD}YoboGo 四足机器狗 — 文件传输脚本${RESET}"
    echo ""
    echo "用法: $0 <本地路径> [远程目录]"
    echo ""
    echo "参数:"
    echo "  <本地路径>     要传输的本地文件或目录路径（必填）"
    echo "  [远程目录]     机器人上的目标目录（可选，默认: ${DEFAULT_REMOTE_DIR}）"
    echo ""
    echo "示例:"
    echo "  $0 /path/to/file.txt"
    echo "  $0 /path/to/file.txt /home/user/"
    echo "  $0 /path/to/my_project/ /home/user/robot-software/"
    echo ""
    exit 1
}

# ---------- 参数检查 ----------
if [ $# -lt 1 ]; then
    usage
fi

LOCAL_PATH="$1"
REMOTE_DIR="${2:-$DEFAULT_REMOTE_DIR}"

# ---------- 依赖检查 ----------
if ! command -v sshpass &>/dev/null; then
    fail "sshpass 未安装"
    echo -e "    安装命令: ${BOLD}sudo apt install sshpass${RESET}"
    exit 1
fi

if ! command -v scp &>/dev/null; then
    fail "scp 未安装"
    echo -e "    安装命令: ${BOLD}sudo apt install openssh-client${RESET}"
    exit 1
fi

# ---------- 本地路径检查 ----------
if [ ! -e "$LOCAL_PATH" ]; then
    fail "本地路径不存在: ${LOCAL_PATH}"
    exit 1
fi

# 获取要传输的文件/目录名
TRANSFER_NAME=$(basename "$LOCAL_PATH")

echo -e "${BOLD}YoboGo 四足机器狗 — 文件传输${RESET}"
echo ""
info "本地路径:   ${LOCAL_PATH}"
info "远程目标:   ${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}"
echo ""

# 显示本地文件信息
if [ -d "$LOCAL_PATH" ]; then
    FILE_COUNT=$(find "$LOCAL_PATH" -type f | wc -l)
    DIR_SIZE=$(du -sh "$LOCAL_PATH" 2>/dev/null | awk '{print $1}')
    info "传输类型:   目录"
    info "文件数量:   ${FILE_COUNT} 个文件"
    info "目录大小:   ${DIR_SIZE}"
elif [ -f "$LOCAL_PATH" ]; then
    FILE_SIZE=$(du -h "$LOCAL_PATH" 2>/dev/null | awk '{print $1}')
    info "传输类型:   文件"
    info "文件大小:   ${FILE_SIZE}"
fi
echo ""

# ---------- 连通性预检 ----------
info "正在检查机器人连通性..."
if ! ping -c 1 -W 2 "$ROBOT_IP" &>/dev/null; then
    fail "机器人 ${ROBOT_IP} 不可达"
    echo -e "    请先运行 ${BOLD}connect_check.sh${RESET} 确认网络连接"
    exit 1
fi
ok "机器人在线"
echo ""

# ---------- 执行传输 ----------
info "开始传输..."

# 使用 -r 递归传输（目录），-p 保留权限，-o 关闭严格主机密钥检查
SCP_FLAGS="-o StrictHostKeyChecking=no -o ConnectTimeout=10"

if [ -d "$LOCAL_PATH" ]; then
    SCP_FLAGS="-r $SCP_FLAGS"
fi

# 执行 scp 传输
if sshpass -p "$ROBOT_PASS" scp $SCP_FLAGS "$LOCAL_PATH" \
    "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}"; then
    echo ""
    ok "传输成功！"
else
    echo ""
    fail "传输失败"
    echo -e "    可能原因:"
    echo -e "    1. 远程目录不存在: ${REMOTE_DIR}"
    echo -e "    2. 磁盘空间不足"
    echo -e "    3. 权限不足"
    echo -e "    4. 网络连接不稳定"
    exit 1
fi

# ---------- 确认传输结果 ----------
echo ""
info "验证传输结果..."
echo -e "${BOLD}--- 机器人上的文件列表 (${REMOTE_DIR}) ---${RESET}"

sshpass -p "$ROBOT_PASS" ssh \
    -o StrictHostKeyChecking=no \
    -o ConnectTimeout=5 \
    "${ROBOT_USER}@${ROBOT_IP}" \
    "echo '--- ${REMOTE_DIR} 内容 ---' && ls -lah '${REMOTE_DIR}' && echo '' && echo '--- 新传输的文件/目录 ---' && ls -lah '${REMOTE_DIR}${TRANSFER_NAME}' 2>/dev/null || ls -lah '${REMOTE_DIR}/${TRANSFER_NAME}' 2>/dev/null || echo '无法列出目标文件'"

echo ""
ok "文件传输完成"
echo ""
