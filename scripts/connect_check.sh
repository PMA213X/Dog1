#!/bin/bash
set -euo pipefail

# ============================================================
# YoboGo 四足机器狗 — 网络连通性检查脚本
# 用途：检查本机与机器人之间的网络连通状态
# 使用：bash scripts/connect_check.sh
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
EXPECTED_SUBNET="10.0.0.0/24"

# ---------- 工具函数 ----------
info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
ok()      { echo -e "${GREEN}[  OK]${RESET}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
fail()    { echo -e "${RED}[FAIL]${RESET}  $*"; }
section() { echo -e "\n${BOLD}========== $* ==========${RESET}"; }

# ---------- 依赖检查 ----------
section "检查必要依赖"

MISSING_DEPS=()

for cmd in ping ip nmcli; do
    if ! command -v "$cmd" &>/dev/null; then
        MISSING_DEPS+=("$cmd")
    fi
done

# sshpass 是可选的（用于自动 SSH 测试）
HAS_SSHPASS=false
if command -v sshpass &>/dev/null; then
    HAS_SSHPASS=true
    ok "sshpass 已安装 — 可自动测试 SSH 连接"
else
    warn "sshpass 未安装 — 将跳过自动 SSH 测试"
    echo -e "    安装命令: ${BOLD}sudo apt install sshpass${RESET}"
fi

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    fail "缺少必要工具: ${MISSING_DEPS[*]}"
    echo -e "    安装命令: ${BOLD}sudo apt install iproute2 net-tools${RESET}"
    exit 1
fi
ok "必要依赖检查通过"

# ============================================================
section "1. 本机网络配置"
# ============================================================

# 获取主要以太网接口
MAIN_IF=""
for iface in eth0 enp0s25 enp3s0 enp4s0 eno1 eno2; do
    if ip link show "$iface" &>/dev/null 2>&1; then
        MAIN_IF="$iface"
        break
    fi
done

# 如果预设接口都没找到，自动查找第一个 UP 状态的非 lo 接口
if [ -z "$MAIN_IF" ]; then
    MAIN_IF=$(ip -o link show up | awk -F': ' '!/lo/{print $2; exit}')
fi

if [ -z "$MAIN_IF" ]; then
    fail "未找到活跃的有线网络接口"
else
    ok "主要网络接口: ${MAIN_IF}"

    # 本机 IP
    LOCAL_IP=$(ip -4 addr show "$MAIN_IF" 2>/dev/null | grep -oP 'inet \K[\d.]+' | head -1 || true)
    if [ -n "$LOCAL_IP" ]; then
        info "本机 IP 地址: ${LOCAL_IP}"
    else
        warn "接口 ${MAIN_IF} 未分配 IPv4 地址"
        LOCAL_IP=""
    fi

    # 子网掩码
    SUBNET_MASK=$(ip -4 addr show "$MAIN_IF" 2>/dev/null | grep -oP 'inet [\d.]+/\K\d+' | head -1 || true)
    if [ -n "$SUBNET_MASK" ]; then
        info "子网前缀长度: /${SUBNET_MASK}"
    fi

    # 默认网关
    DEFAULT_GW=$(ip route show default 2>/dev/null | awk '/default/{print $3; exit}' || true)
    if [ -n "$DEFAULT_GW" ]; then
        info "默认网关: ${DEFAULT_GW}"
    else
        warn "未找到默认网关"
    fi
fi

# ============================================================
section "2. 检查本机 IP 是否在 10.0.0.x 子网内"
# ============================================================

if [ -n "${LOCAL_IP:-}" ]; then
    # 检查 IP 前三段是否为 10.0.0
    if [[ "$LOCAL_IP" == 10.0.0.* ]]; then
        ok "本机 IP (${LOCAL_IP}) 在 ${EXPECTED_SUBNET} 子网内"
    else
        fail "本机 IP (${LOCAL_IP}) 不在 ${EXPECTED_SUBNET} 子网内！"
        echo -e "    修复建议: ${BOLD}sudo ip addr add 10.0.0.x/24 dev ${MAIN_IF:-eth0}${RESET}"
        echo -e "    或修改网络管理器配置为静态 IP 10.0.0.x"
    fi
else
    fail "无法获取本机 IP，无法判断子网"
fi

# ============================================================
section "3. WiFi 状态检查"
# ============================================================

WIFI_OFF=false

# 方法 1：通过 nmcli 检查
if command -v nmcli &>/dev/null; then
    WIFI_STATE=$(nmcli radio wifi 2>/dev/null || echo "unknown")
    if [ "$WIFI_STATE" = "disabled" ] || [ "$WIFI_STATE" = "off" ]; then
        ok "WiFi 已关闭 (nmcli: ${WIFI_STATE})"
        WIFI_OFF=true
    else
        fail "WiFi 仍然开启 (nmcli: ${WIFI_STATE})！必须关闭 WiFi"
        echo -e "    修复建议: ${BOLD}nmcli radio wifi off${RESET}"
    fi
fi

# 方法 2：通过 ip link 检查无线接口是否 UP
if ip link show wlan0 &>/dev/null 2>&1; then
    WIFI_LINK=$(ip link show wlan0 2>/dev/null | grep -o 'state [A-Z]*' | awk '{print $2}' || true)
    if [ "$WIFI_LINK" = "UP" ] || [ "$WIFI_LINK" = "UNKNOWN" ]; then
        warn "无线接口 wlan0 处于 UP/UNKNOWN 状态 (${WIFI_LINK})"
        echo -e "    建议: ${BOLD}sudo ip link set wlan0 down${RESET}"
    else
        if [ "$WIFI_OFF" = false ]; then
            ok "无线接口 wlan0 已关闭 (状态: ${WIFI_LINK:-DOWN})"
        fi
    fi
else
    if [ "$WIFI_OFF" = false ]; then
        info "未检测到 wlan0 接口"
    fi
fi

# ============================================================
section "4. Ping 机器人 (${ROBOT_IP})"
# ============================================================

info "正在 ping ${ROBOT_IP}（5 次）..."
if ping -c 5 -W 2 "$ROBOT_IP" &>/dev/null; then
    # 获取延迟统计
    PING_STAT=$(ping -c 5 -W 2 "$ROBOT_IP" 2>/dev/null | tail -1)
    ok "机器人 ${ROBOT_IP} 可达"
    info "ping 统计: ${PING_STAT}"
    ROBOT_ONLINE=true
else
    fail "机器人 ${ROBOT_IP} 不可达！"
    echo -e "    可能原因:"
    echo -e "    1. 机器人未开机或网线未连接"
    echo -e "    2. 本机 IP 不在 10.0.0.x 子网"
    echo -e "    3. 网线故障或接口未启用"
    echo -e "    4. 机器人 IP 地址已更改"
    echo -e "    排查: ${BOLD}arp -a | grep 10.0.0${RESET} 查看 ARP 表"
    ROBOT_ONLINE=false
fi

# ============================================================
section "5. SSH 连接测试"
# ============================================================

if [ "$ROBOT_ONLINE" = true ] && [ "$HAS_SSHPASS" = true ]; then
    info "尝试 SSH 连接 ${ROBOT_USER}@${ROBOT_IP}..."

    # 使用 sshpass 自动输入密码进行 SSH 测试
    if sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no \
            -o ConnectTimeout=5 \
            -o BatchMode=no \
            "${ROBOT_USER}@${ROBOT_IP}" "echo 'SSH_OK'" 2>/dev/null | grep -q "SSH_OK"; then
        ok "SSH 连接成功！可以正常登录机器人"
    else
        fail "SSH 连接失败"
        echo -e "    可能原因:"
        echo -e "    1. SSH 服务未启动"
        echo -e "    2. 用户名或密码错误"
        echo -e "    3. SSH 服务拒绝连接"
        echo -e "    手动测试: ${BOLD}ssh ${ROBOT_USER}@${ROBOT_IP}${RESET}"
    fi
elif [ "$ROBOT_ONLINE" = true ] && [ "$HAS_SSHPASS" = false ]; then
    warn "机器人可达但 sshpass 未安装，跳过自动 SSH 测试"
    echo -e "    手动测试: ${BOLD}ssh ${ROBOT_USER}@${ROBOT_IP}${RESET}"
else
    warn "机器人不可达，跳过 SSH 测试"
fi

# ============================================================
section "6. 诊断总结"
# ============================================================

echo ""
TOTAL_CHECKS=0
PASSED_CHECKS=0

# 子网检查
TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
if [[ "${LOCAL_IP:-}" == 10.0.0.* ]]; then
    PASSED_CHECKS=$((PASSED_CHECKS + 1))
fi

# WiFi 检查
TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
if [ "$WIFI_OFF" = true ]; then
    PASSED_CHECKS=$((PASSED_CHECKS + 1))
fi

# Ping 检查
TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
if [ "${ROBOT_ONLINE:-false}" = true ]; then
    PASSED_CHECKS=$((PASSED_CHECKS + 1))
fi

# SSH 检查
TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
if [ "${ROBOT_ONLINE:-false}" = true ] && [ "$HAS_SSHPASS" = true ]; then
    # SSH 已在上面测试过
    PASSED_CHECKS=$((PASSED_CHECKS + 1))
fi

echo -e "  检查项通过: ${PASSED_CHECKS}/${TOTAL_CHECKS}"

if [ "$PASSED_CHECKS" -eq "$TOTAL_CHECKS" ]; then
    echo -e "  ${GREEN}${BOLD}✅ 网络连接完全正常，可以开始操作机器人！${RESET}"
elif [ "${ROBOT_ONLINE:-false}" = true ]; then
    echo -e "  ${YELLOW}${BOLD}⚠️  机器人可达，但部分检查未通过，请修复后重试${RESET}"
else
    echo -e "  ${RED}${BOLD}❌ 机器人不可达，请按以下步骤排查:${RESET}"
    echo -e "  1. 确认机器人已开机"
    echo -e "  2. 确认网线已连接（本机 ↔ 机器人）"
    echo -e "  3. 确认本机 IP 为 10.0.0.x（x≠1, x≠34）"
    echo -e "  4. 确认 WiFi 已关闭"
    echo -e "  5. 确认虚拟机网络为桥接模式"
fi

echo ""
