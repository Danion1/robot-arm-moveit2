# -*- coding: utf-8 -*-
"""Puts cad/lib/ on the import path. Import it before any lib module:
    import usar_lib  # noqa: F401
cad/lib/ is a frozen copy of the shared CAD library modules this project uses
(source version in README.md, "CAD library")."""
import os
import sys

LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib")
if LIB not in sys.path:
    sys.path.insert(0, LIB)
