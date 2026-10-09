# 本地 SearXNG 搜索 MCP

## 当前状态

- 状态：文档骨架，待安装与联调；
- 调研日期：2026-10-07；
- 尚未安装 Docker、SearXNG 和 MCP Server；
- 尚未修改 Codex 用户配置，也未注册 `searxng-search`；
- 本文不记录任何密钥、口令或机密配置值。

## 目标

在 Ubuntu 本机部署不依赖第三方搜索 API Key 的搜索链路，使 Codex 可以通过
MCP 调用本地 `search_web` 工具，并从仅监听回环地址的 SearXNG 获取网页搜索
结果。

预期链路：

```text
Codex
  -> MCP / stdio
  -> 本地 MCP Server
  -> HTTP JSON
  -> 127.0.0.1:8080 上的 SearXNG
  -> 搜索引擎
```

## 范围

本次范围包括：

- 安装并启用 Docker 与 Docker Compose；
- 在用户主目录部署 SearXNG Compose 项目；
- 启用 SearXNG HTML/JSON 输出格式，并将服务限制为本机访问；
- 创建基于 `mcp[cli]` 与 `httpx` 的 stdio MCP Server；
- 注册名为 `searxng-search` 的 Codex MCP Server；
- 完成服务、接口、工具和 Codex 配置测试；
- 同步维护功能文档、API 文档和变更记录。

本次范围不包括：

- 对外网、局域网或反向代理开放 SearXNG；
- 删除或停用已有搜索服务；
- 修改机器人控制、训练或仿真代码；
- 任何 Git 写操作。

## 架构

### SearXNG 服务

- Compose 项目：`~/searxng`
- 配置目录：`~/searxng/core-config`
- 本地地址：`127.0.0.1:8080`
- JSON 地址：`/search`
- 必需配置：输出格式同时包含 `html` 与 `json`
- 安全边界：仅回环访问，不作为公开搜索实例

### MCP Server

- 项目目录：`~/mcp-searxng`
- 传输方式：stdio
- Python 管理：`uv`
- 计划工具：`search_web`
- 上游地址：`http://127.0.0.1:8080/search`
- 错误策略：上游返回非成功状态、响应非 JSON 或连接超时时，向调用方返回
  可诊断错误，不伪造搜索结果。

### Codex 集成

- MCP 名称：`searxng-search`
- 启动命令：使用绝对路径定位 `uv`
- 工作目录：`~/mcp-searxng`
- 生命周期：由 Codex 启动和管理 stdio 子进程
- 验收原则：必须能列出并实际调用 `search_web`，不能只以配置文件存在作为
  成功条件。

## 配置项

| 配置项 | 预期值或规则 | 状态 |
| --- | --- | --- |
| Docker | 已安装、服务已启用、当前用户可使用 | 待安装 |
| Compose | `docker compose version` 可执行 | 待安装 |
| SearXNG 目录 | `~/searxng` | 待创建 |
| 监听范围 | 仅 `127.0.0.1:8080` | 待配置 |
| JSON 输出 | `search.formats` 包含 `html` 和 `json` | 待配置 |
| 实例配置密钥 | 本机生成，不写入本文档 | 待配置 |
| MCP 项目目录 | `~/mcp-searxng` | 待创建 |
| Python 依赖 | `mcp[cli]`、`httpx` | 待安装 |
| MCP 传输 | stdio | 待实现 |
| Codex MCP 名称 | `searxng-search` | 待注册 |
| Codex 启动路径 | `uv` 绝对路径 | 待注册 |

## 测试计划

测试按依赖顺序执行；前一阶段失败时停止后续实施并记录退出码。

### 1. 环境检查

- 检查 Docker、Compose 与用户组状态；
- 检查 8080 端口无现有监听冲突；
- 检查网络可访问容器镜像、Python 包和搜索上游。

### 2. 服务检查

- Docker 容器冒烟测试成功；
- Compose 中 SearXNG 与依赖服务均为运行状态；
- 8080 仅监听 `127.0.0.1`，不监听通配地址；
- 从本机可访问首页和 JSON 搜索接口；
- JSON 响应结构合法，且测试查询能返回结果。

### 3. MCP 检查

- Python 依赖可复现安装；
- MCP Server 可启动；
- MCP 客户端完成初始化、列出工具并调用 `search_web`；
- 工具返回的查询、结果数组和结果字段符合 API 文档。

### 4. Codex 检查

- `codex mcp list` 显示 `searxng-search`；
- `codex mcp get searxng-search` 显示预期命令和工作目录；
- Codex 非交互测试实际调用 `searxng-search/search_web`；
- 测试记录工具调用和退出码，不以模型最终文案代替调用证据。

## 已知风险

- Docker 需要特权安装和用户组刷新，可能要求重新登录；
- 官方 Compose 模板可能默认发布到通配地址，必须验证并收紧到回环地址；
- 搜索引擎可能限流、返回验证码或暂时无结果；
- `latest` 镜像和主分支 Compose 模板会漂移，正式验收应记录实际版本；
- Python、MCP SDK 与 Codex 版本变化可能影响接口兼容性；
- 当前已有其他搜索或联网配置，不能假定 Codex 一定选择本地工具；
- stdio Server 启动成功不等于工具可调用，必须进行协议级测试；
- 文档、配置或日志中不得出现任何机密值。

## 实施检查表

- [ ] 创建文档骨架；
- [ ] 安装并验证 Docker/Compose；
- [ ] 创建 SearXNG Compose 与本地配置；
- [ ] 验证回环监听和 JSON API；
- [ ] 创建 MCP 项目与 `search_web`；
- [ ] 执行 MCP 协议级测试；
- [ ] 注册并验证 Codex MCP；
- [ ] 更新本文档、API 文档和变更记录；
- [ ] 汇总测试命令、结果与退出码。

