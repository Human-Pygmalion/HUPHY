"""`sim/` 과 `src/` 를 경로에 넣음. 설치하지 않고 pytest 를 돌릴 수 있게 함."""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for path in (HERE, HERE.parent / "src"):
    if path.is_dir() and str(path) not in sys.path:
        sys.path.insert(0, str(path))
