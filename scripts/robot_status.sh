#!/bin/bash
set -euo pipefail

# ============================================================
# YoboGo 四足机器狗 — 实时状态监控脚本
# 用途：SSH 到机器人获取实时运行状态
# 使用：bash scripts/robot_status.sh
#       bash scripts/robot_status.sh --loop  # 持续监控（每 5 秒刷新）
# ============================================================

# ---------- 颜色定义 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
BOLD='\033[1m'
DIM='\033[2m'
RESET='\033[0m'

# ---------- 机器人配置 ----------
ROBOT_IP="10.0.0.34"
ROBOT_USER="user"
ROBOT_PASS="123456"

# ---------- 监控参数 ----------
LOOP_MODE=false
REFRESH_INTERVAL=5

if [ "${1:-}" = "--loop" ] || [ "${1:-}" = "-l" ]; then
    LOOP_MODE=true
    info "持续监控模式（每 ${REFRESH_INTERVAL} 秒刷新，按 Ctrl+C 退出）"
fi

# ---------- 工具函数 ----------
info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
ok()      { echo -e "${GREEN}[  OK]${RESET}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
fail()    { echo -e "${RED}[FAIL]${RESET}  $*"; }

# 在远程执行命令并返回结果
remote_cmd() {
    sshpass -p "$ROBOT_PASS" ssh \
        -o StrictHostKeyChecking=no \
        -o ConnectTimeout=3 \
        -o BatchMode=no \
        "${ROBOT_USER}@${ROBOT_IP}" "$1" 2>/dev/null || echo "N/A"
}

# 在本地执行命令并返回结果
local_cmd() {
    eval "$1" 2>/dev/null || echo "N/A"
}

# 清屏（仅循环模式）
clear_screen() {
    if [ "$LOOP_MODE" = true ]; then
        clear
    fi
}

# ---------- 依赖检查 ----------
if ! command -v sshpass &>/dev/null; then
    fail "sshpass 未安装"
    echo -e "    安装命令: ${BOLD}sudo apt install sshpass${RESET}"
    exit 1
fi

# ---------- 一次性连通性检查 ----------
check_connectivity() {
    if ! ping -c 1 -W 2 "$ROBOT_IP" &>/dev/null; then
        echo -e "  ${RED}●${RESET} 机器人 ${ROBOT_IP} 不可达"
        return 1
    fi
    return 0
}

# ============================================================
# 主监控函数
# ============================================================
monitor_once() {
    local TIMESTAMP
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

    clear_screen

    echo -e "${BOLD}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║          YoboGo 四足机器狗 — 实时状态监控                  ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
    echo -e "  ${DIM}时间: ${TIMESTAMP}  |  目标: ${ROBOT_IP}${RESET}"
    echo ""

    # ---------- 在线状态 ----------
    echo -e "${BOLD}┌─ 在线状态 ─────────────────────────────────────────────────┐${RESET}"
    if check_connectivity; then
        echo -e "  ${GREEN}●${RESET} 机器人在线 (${ROBOT_IP})"
    else
        echo -e "  ${RED}●${RESET} 机器人离线 — 后续数据可能不可用"
        echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
        return 1
    fi
    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""

    # ---------- 系统信息 ----------
    echo -e "${BOLD}┌─ 系统信息 ─────────────────────────────────────────────────┐${RESET}"
    
    # 主机名和内核
    local HOSTNAME KERNEL_INFO
    HOSTNAME=$(remote_cmd "hostname")
    KERNEL_INFO=$(remote_cmd "uname -r")
    echo -e "  主机名:    ${HOSTNAME}"
    echo -e "  内核版本:  ${KERNEL_INFO}"

    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""

    # ---------- CPU 状态 ----------
    echo -e "${BOLD}┌─ CPU 状态 ─────────────────────────────────────────────────┐${RESET}"
    
    # CPU 温度
    local CPU_TEMP_RAW CPU_TEMP
    CPU_TEMP_RAW=$(remote_cmd "cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0")
    # 去除空白
    CPU_TEMP_RAW=$(echo "$CPU_TEMP_RAW" | tr -d '[:space:]')
    
    if [ "$CPU_TEMP_RAW" != "0" ] && [ "$CPU_TEMP_RAW" != "N/A" ] && [ "$CPU_TEMP_RAW" -gt 0 ] 2>/dev/null; then
        CPU_TEMP=$((CPU_TEMP_RAW / 1000))
        local TEMP_COLOR="$GREEN"
        if [ "$CPU_TEMP" -ge 70 ]; then
            TEMP_COLOR="$RED"
        elif [ "$CPU_TEMP" -ge 55 ]; then
            TEMP_COLOR="$YELLOW"
        fi
        echo -e "  CPU 温度:  ${TEMP_COLOR}${CPU_TEMP}°C${RESET}"
    else
        echo -e "  CPU 温度:  ${DIM}无法读取${RESET}"
    fi

    # CPU 使用率（获取 1 秒快照）
    local CPU_USAGE
    CPU_USAGE=$(remote_cmd "top -bn1 | grep 'Cpu(s)' | awk '{print \$2}' | cut -d'%' -f1")
    CPU_USAGE=$(echo "$CPU_USAGE" | tr -d '[:space:]')
    
    if [ -n "$CPU_USAGE" ] && [ "$CPU_USAGE" != "N/A" ]; then
        local CPU_COLOR="$GREEN"
        # 比较整数部分
        local CPU_INT=${CPU_USAGE%%.*}
        if [ "${CPU_INT:-0}" -ge 90 ] 2>/dev/null; then
            CPU_COLOR="$RED"
        elif [ "${CPU_INT:-0}" -ge 70 ] 2>/dev/null; then
            CPU_COLOR="$YELLOW"
        fi
        echo -e "  CPU 使用率: ${CPU_COLOR}${CPU_USAGE}%${RESET}"
    else
        echo -e "  CPU 使用率: ${DIM}无法读取${RESET}"
    fi

    # 负载
    local LOAD_AVG
    LOAD_AVG=$(remote_cmd "cat /proc/loadavg | awk '{print \$1, \$2, \$3}'")
    echo -e "  负载均值:  ${LOAD_AVG}"
    
    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""

    # ---------- 内存使用 ----------
    echo -e "${BOLD}┌─ 内存使用 ─────────────────────────────────────────────────┐${RESET}"
    
    local MEM_INFO
    MEM_INFO=$(remote_cmd "free -h | awk '/^Mem:/{print \"总计: \"\$2\"  已用: \"\$3\"  可用: \"\$7}'")
    echo -e "  ${MEM_INFO}"
    
    # 内存使用百分比
    local MEM_PCT
    MEM_PCT=$(remote_cmd "free | awk '/^Mem:/{printf \"%.1f\", \$3/\$2*100}'")
    MEM_PCT=$(echo "$MEM_PCT" | tr -d '[:space:]')
    
    if [ -n "$MEM_PCT" ] && [ "$MEM_PCT" != "N/A" ]; then
        local MEM_INT=${MEM_PCT%%.*}
        local MEM_COLOR="$GREEN"
        if [ "${MEM_INT:-0}" -ge 90 ] 2>/dev/null; then
            MEM_COLOR="$RED"
        elif [ "${MEM_INT:-0}" -ge 75 ] 2>/dev/null; then
            MEM_COLOR="$YELLOW"
        fi
        echo -e "  使用率:    ${MEM_COLOR}${MEM_PCT}%${RESET}"
    fi
    
    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""

    # ---------- 磁盘空间 ----------
    echo -e "${BOLD}┌─ 磁盘空间 ─────────────────────────────────────────────────┐${RESET}"
    
    local DISK_INFO
    DISK_INFO=$(remote_cmd "df -h / | awk 'NR==2{print \"总计: \"\$2\"  已用: \"\$3\"  可用: \"\$4\"  使用率: \"\$5}'")
    echo -e "  根分区:  ${DISK_INFO}"
    
    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""

    # ---------- 网络接口 ----------
    echo -e "${BOLD}┌─ 网络接口 ─────────────────────────────────────────────────┐${RESET}"
    
    local ETH_INFO
    ETH_INFO=$(remote_cmd "ip -4 addr show eth0 2>/dev/null | grep 'inet ' | awk '{print \$2}' | head -1")
    ETH_INFO=$(echo "$ETH_INFO" | tr -d '[:space:]')
    
    if [ -n "$ETH_INFO" ] && [ "$ETH_INFO" != "N/A" ]; then
        echo -e "  eth0 IP:   ${GREEN}${ETH_INFO}${RESET}"
    else
        echo -e "  eth0 IP:   ${DIM}未配置${RESET}"
    fi
    
    # 网络流量统计
    local RX_BYTES TX_BYTES
    RX_BYTES=$(remote_cmd "cat /sys/class/net/eth0/statistics/rx_bytes 2>/dev/null || echo 0")
    TX_BYTES=$(remote_cmd "cat /sys/class/net/eth0/statistics/tx_bytes 2>/dev/null || echo 0")
    RX_BYTES=$(echo "$RX_BYTES" | tr -d '[:space:]')
    TX_BYTES=$(echo "$TX_BYTES" | tr -d '[:space:]')
    
    if [ "${RX_BYTES:-0}" != "0" ] && [ "$RX_BYTES" != "N/A" ] 2>/dev/null; then
        # 转换为可读格式
        RX_MB=$((RX_BYTES / 1048576))
        TX_MB=$((TX_BYTES / 1048576))
        echo -e "  流量统计:  RX: ${RX_MB} MB  TX: ${TX_MB} MB"
    fi
    
    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""

    # ---------- 关键进程 ----------
    echo -e "${BOLD}┌─ 关键进程 ─────────────────────────────────────────────────┐${RESET}"
    
    local PROCESSES
    PROCESSES=$(remote_cmd "ps aux | grep -E 'mit_ctrl|track|dogvision|ros' | grep -v grep")
    
    if [ -n "$PROCESSES" ] && [ "$PROCESSES" != "N/A" ]; then
        echo "$PROCESSES" | while IFS= read -r line; do
            local PROC_NAME PID CPU_PCT MEM_PCT
            PROC_NAME=$(echo "$line" | awk '{print $11}')
            PID=$(echo "$line" | awk '{print $2}')
            CPU_PCT=$(echo "$line" | awk '{print $3}')
            MEM_PCT=$(echo "$line" | awk '{print $4}')
            echo -e "  ${GREEN}●${RESET} ${PROC_NAME}  (PID: ${PID}, CPU: ${CPU_PCT}%, MEM: ${MEM_PCT}%)"
        done
    else
        echo -e "  ${YELLOW}●${RESET} 未检测到关键进程 (mit_ctrl / track / dogvision / ros)"
    fi
    
    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""

    # ---------- 电机状态（通过硬件监控接口） ----------
    echo -e "${BOLD}┌─ 硬件状态 ─────────────────────────────────────────────────┐${RESET}"
    
    # hwmon 设备
    local HWMON_INFO
    HWMON_INFO=$(remote_cmd "ls /sys/class/hwmon/ 2>/dev/null | head -10")
    if [ -n "$HWMON_INFO" ] && [ "$HWMON_INFO" != "N/A" ]; then
        echo -e "  Hwmon 设备: $(echo "$HWMON_INFO" | tr '\n' ' ')"
    else
        echo -e "  Hwmon: ${DIM}无设备${RESET}"
    fi
    
    # SPI 设备
    local SPI_INFO
    SPI_INFO=$(remote_cmd "ls /dev/spidev* 2>/dev/null | head -5")
    if [ -n "$SPI_INFO" ] && [ "$SPI_INFO" != "N/A" ]; then
        echo -e "  SPI 设备:   $(echo "$SPI_INFO" | tr '\n' ' ')"
    else
        echo -e "  SPI 设备:   ${DIM}未检测到${RESET}"
    fi
    
    # 电池状态（如果存在）
    local BATT_INFO
    BATT_INFO=$(remote_cmd "cat /sys/class/power_supply/BAT*/capacity 2>/dev/null || echo 'N/A'")
    BATT_INFO=$(echo "$BATT_INFO" | tr -d '[:space:]')
    if [ -n "$BATT_INFO" ] && [ "$BATT_INFO" != "N/A" ]; then
        echo -e "  电池电量:   ${BATT_INFO}%"
    fi
    
    echo -e "${BOLD}└────────────────────────────────────────────────────────────┘${RESET}"
    echo ""
    
    if [ "$LOOP_MODE" = true ]; then
        echo -e "  ${DIM}按 Ctrl+C 退出监控  |  ${REFRESH_INTERVAL}s 后刷新...${RESET}"
    fi
}

# ============================================================
# 主入口
# ============================================================

if [ "$LOOP_MODE" = true ]; then
    trap 'echo -e "\n${GREEN}监控已停止${RESET}"; exit 0' INT TERM
    while true; do
        monitor_once || true
        sleep "$REFRESH_INTERVAL"
    done
else
    monitor_once || {
        echo ""
        warn "部分数据可能不完整，请先运行 connect_check.sh 确认网络连接"
        exit 1
    }
fi
