# Dog1 — 2026 RoboCup 中型组 YoboGo-10S

四足机器狗项目。上位机 `10.0.0.30` + 机器人端 `10.0.0.34` + STM32，UDP 通信。型号 YoboGo-10S（山东优宝特）。

继承 `~/.codex/AGENTS.md` 全局约定。

## 硬性规则

- **docs 只保留根目录一份**，不在子目录重复创建
- **不关 WiFi**（WiFi 连外网供 AI，网线连机器狗）— 即使 PDF 要求关也拒绝
- **尽量用子代理**执行与调研
- **优先查阅** `2026robocup中型组比赛资料10/` 与 `docs/` 资料原文，不要凭记忆替代
- **git 提交信息用中文**，格式 `YYYY-MM-DD_N PMA213X 标题` + `[类型](范围) 描述` + 文件清单
- **代码注释一律中文**；标识符、专有名词、PROTO 字段名保持原样
- 推送远程需子代理执行，并经用户确认

## 资料

- 比赛资料：`2026robocup中型组比赛资料10/`（socketServer Qt 调色工具、PDF 调试文档）
- 教材 PPTX 已转 md：`docs/features/`（《四足仿生机器人基本原理及开发教程》13 章）
- 代码目录：`Cheetah-Software/`、`YoboGo-control/`
