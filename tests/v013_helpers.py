"""v0.1.3 test helper: load the FROZEN v0.1.2 source snapshot side by side.

The v0.1.3 regressions must be able to show what the code did BEFORE the repair
without keeping a second copy of the buggy implementation inside the test logic.
artifacts/v013/prefix_src/ is a byte-for-byte copy of
OpticalWaveguideRouterNative3D/src/native3d taken before the first v0.1.3 edit;
this module loads it as a SEPARATE package (``native3d_prefix``) in the same
interpreter, so both versions can be executed and compared in one test run.

Importing this module has no side effect on the package under test.
"""

from __future__ import annotations

import importlib.util
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(TESTS_DIR)
V013_DIR = os.path.join(PROJECT_ROOT, "artifacts", "v013")
PREFIX_SRC = os.path.join(V013_DIR, "prefix_src")
LOG_DIR = os.path.join(V013_DIR, "logs")
CASE_DIR = os.path.join(V013_DIR, "cases")

PREFIX_PACKAGE = "native3d_prefix"

# The commit-time identity of the frozen snapshot, so a stale snapshot cannot be
# used silently as "the prefix behaviour".
PREFIX_FILES = (
    "config.py", "cases.py", "identity.py",
    "constraints/checks.py", "geometry/primitives.py", "geometry/path3d.py",
    "geometry/vec.py", "io/artifacts.py", "io/caseio.py",
    "search/actions.py", "search/astar.py", "search/congestion.py",
    "search/connector.py", "search/finite_graph.py", "search/reference_graph.py",
    "search/router.py", "search/state.py", "validation/validate.py",
)


def prefix_available() -> bool:
    return all(os.path.exists(os.path.join(PREFIX_SRC, f)) for f in PREFIX_FILES)


def load_prefix_package():
    """Import (once) the frozen v0.1.2 snapshot as ``native3d_prefix``."""
    if PREFIX_PACKAGE in sys.modules:
        return sys.modules[PREFIX_PACKAGE]
    if not prefix_available():
        raise RuntimeError(
            "the frozen v0.1.2 source snapshot is missing under %s; the v0.1.3 "
            "pre-fix replay cannot be reproduced" % PREFIX_SRC)
    parent = importlib.util.module_from_spec(
        importlib.util.spec_from_loader(PREFIX_PACKAGE, loader=None))
    parent.__path__ = [PREFIX_SRC]
    sys.modules[PREFIX_PACKAGE] = parent
    return parent


def prefix_module(dotted: str):
    """``prefix_module('search.finite_graph')`` -> the frozen module object."""
    load_prefix_package()
    return importlib.import_module("%s.%s" % (PREFIX_PACKAGE, dotted))
