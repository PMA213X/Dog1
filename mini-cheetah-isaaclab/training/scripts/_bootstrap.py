"""训练脚本的项目路径引导工具。"""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TRAINING_ROOT = PROJECT_ROOT / "training"


def ensure_project_on_path() -> Path:
    """把项目根目录加入 sys.path，保证 training 包可被稳定导入。"""
    project_path = str(PROJECT_ROOT)
    if project_path not in sys.path:
        sys.path.insert(0, project_path)
    return PROJECT_ROOT

