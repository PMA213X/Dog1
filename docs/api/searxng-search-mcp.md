# SearXNG 搜索 MCP API

## 状态

- 接口状态：规划中，尚未实现；
- 服务状态：SearXNG 与 MCP Server 均待安装；
- 文档状态：API 骨架，后续实现后补充实测结果；
- 安全说明：本文不记录任何实例配置密钥、环境变量值或认证凭据。

## 传输

| 项目 | 值 |
| --- | --- |
| MCP Server 名称 | `searxng-search` |
| 传输方式 | stdio |
| 本地 SearXNG 地址 | `http://127.0.0.1:8080` |
| SearXNG 搜索路径 | `/search` |
| 输出格式 | JSON |

## 工具 `search_web`

通过本地 SearXNG 查询网页结果。

### 请求参数

| 参数 | 类型 | 必填 | 默认值 | 说明 |
| --- | --- | --- | --- | --- |
| `query` | `string` | 是 | 无 | 搜索关键词 |
| `language` | `string` | 否 | `auto` | 搜索语言，如 `auto`、`en`、`zh` |
| `time_range` | `string \\| null` | 否 | `null` | 时间范围，实现按 SearXNG 支持值校验 |
| `page` | `integer` | 否 | `1` | 搜索结果页码，必须大于等于 1 |

### 成功响应

```json
{
  "query": "搜索词",
  "number_of_results": 1,
  "results": [
    {
      "title": "结果标题",
      "url": "https://example.com/",
      "content": "结果摘要",
      "engine": "engine-name",
      "score": 1.0
    }
  ]
}
```

字段约定：

- `query`：实际执行或规范化后的查询；
- `number_of_results`：返回结果数量或 SearXNG 提供的数量；
- `results`：结果数组，请求成功时必须为数组；
- `title`：网页标题；
- `url`：网页地址；
- `content`：搜索结果摘要；
- `engine`：产生结果的 SearXNG 引擎名称；
- `score`：引擎提供的结果分数。

### 错误场景

以下情况应返回明确错误，不得返回空的成功响应：

- SearXNG 未启动或无法连接；
- HTTP 状态码不是成功状态；
- 响应不是合法 JSON；
- 请求超时；
- 必填参数缺失或类型错误；
- `page` 小于 1。

## 约束

- 仅允许访问本机 SearXNG，不提供任意 URL 请求能力；
- 请求超时应受控，避免 MCP 子进程无限等待；
- 上游结果为空时应区分“查询成功但无结果”与“服务调用失败”；
- 不在响应、日志或文档中回显任何配置密钥。

## 测试契约

实现后至少覆盖：

1. 初始化 MCP 服务并列出工具；
2. 调用 `search_web(query="test")`；
3. 验证响应为 JSON 且 `results` 为数组；
4. 验证至少一个真实查询返回非空结果；
5. 验证无效 `page` 和 SearXNG 停止时的错误路径；
6. 由 Codex 实际调用 `searxng-search/search_web` 并保留调用证据。

## 关联文档

- 功能与部署：[`docs/features/local-searxng-search-mcp.md`](../features/local-searxng-search-mcp.md)
- 变更记录：[`docs/changes/2026-10-07.md`](../changes/2026-10-07.md)

