#!/usr/bin/env python3
"""Compatibility wrapper for `twtool log download`.

New workflows should use `python tools/twtool.py log download ...`.  This file
remains so existing notes/commands continue to work during the toolbox migration.
"""

from __future__ import annotations

import sys
from pathlib import Path

TOOLS_ROOT = Path(__file__).resolve().parents[1]
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from triwhirl_tool.commands.log import download_main


if __name__ == "__main__":
    raise SystemExit(download_main(sys.argv[1:]))
