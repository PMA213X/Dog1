"""手动打开页面的离线入口与静态场景测试。"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
INDEX_PATH = PROJECT_ROOT / "index.html"
README_PATH = PROJECT_ROOT / "README.md"
SCREENSHOT_PATH = PROJECT_ROOT / "artifacts/manual_viewer_chrome.png"


class ManualViewerTests(unittest.TestCase):
    """检查无需服务器的入口和落地画面所需结构。"""

    @classmethod
    def setUpClass(cls) -> None:
        """读取页面与说明文件。"""

        cls.index = INDEX_PATH.read_text(encoding="utf-8")
        cls.readme = README_PATH.read_text(encoding="utf-8")

    def test_direct_open_entrypoint_is_obvious(self) -> None:
        """README 必须直接给出双击入口，且页面声明离线打开方式。"""

        self.assertTrue(INDEX_PATH.is_file())
        self.assertIn("双击 `index.html`", self.readme)
        self.assertIn("xdg-open index.html", self.readme)
        self.assertIn("双击本目录中的 <strong>index.html</strong>", self.index)
        self.assertIn("file://", self.index)

    def test_page_has_no_external_dependencies(self) -> None:
        """页面必须保持无脚本、无网络资源，避免 file:// 加载失败。"""

        self.assertIn("<!doctype html>", self.index.lower())
        self.assertNotIn("<script", self.index.lower())
        self.assertNotRegex(self.index, r"""(?i)\b(?:src|href)\s*=\s*["']https?://""")
        self.assertNotRegex(self.index, r"""(?i)url\(\s*["']?https?://""")
        self.assertNotRegex(self.index, r"""(?i)<(?:img|link|iframe|video|audio)\b""")

    def test_scene_contains_ground_and_complete_robot(self) -> None:
        """检查地面、四足、脚垫和完整取景结构均存在。"""

        self.assertIn('id="ground-plane"', self.index)
        self.assertIn('id="mini-cheetah"', self.index)
        self.assertIn('id="body"', self.index)
        self.assertEqual(
            len(re.findall(r'id="leg-(?:rear|front)-(?:far|near)"', self.index)),
            4,
        )
        self.assertEqual(
            len(re.findall(r'data-foot="(?:rear|front)-(?:far|near)"', self.index)),
            4,
        )
        self.assertEqual(
            len(
                re.findall(
                    r'data-foot-contact="(?:rear|front)-(?:far|near)"',
                    self.index,
                )
            ),
            4,
        )
        self.assertIn('viewBox="0 0 1200 700"', self.index)
        self.assertIn("preserveAspectRatio=\"xMidYMid meet\"", self.index)

    def test_scene_declares_static_landing_state(self) -> None:
        """检查页面对静止、接地、完整显示和能力边界的说明。"""

        for phrase in ("静止站立", "四足接地", "平地 · 无遮挡", "不替代 Webots"):
            with self.subTest(phrase=phrase):
                self.assertIn(phrase, self.index)
        self.assertIn("不替代 Webots 的动态被动落稳数据验收", self.index)

    def test_browser_screenshot_is_available(self) -> None:
        """真实浏览器截图必须存在且不是空文件。"""

        self.assertTrue(SCREENSHOT_PATH.is_file())
        self.assertGreater(SCREENSHOT_PATH.stat().st_size, 10_000)


if __name__ == "__main__":
    unittest.main()
