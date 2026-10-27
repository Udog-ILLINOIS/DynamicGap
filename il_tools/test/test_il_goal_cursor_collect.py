#!/usr/bin/env python3
"""il_goal_cursor_collect: it watches and stops only the launches it started, and
counts a stage as finished only when the benchmark logged its completion."""

import importlib.machinery
import importlib.util
import os
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "il_goal_cursor_collect")


def load():
    loader = importlib.machinery.SourceFileLoader("il_goal_cursor_collect", SCRIPT)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class TestCollect(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.collect = load()

    def test_only_own_processes(self):
        c = self.collect
        marker = "il_goal_cursor_collect_test_{}".format(os.getpid())
        cmd = [sys.executable, "-c", "import time; time.sleep(30)", marker]
        own = subprocess.Popen(cmd, env=dict(os.environ, **{c.TAG: str(os.getpid())}))
        other = subprocess.Popen(cmd, env=dict(os.environ, **{c.TAG: "1"}))
        try:
            time.sleep(0.3)
            self.assertEqual([str(own.pid)], c.pids(marker))
        finally:
            own.kill()
            other.kill()

    def test_completed(self):
        c = self.collect
        with tempfile.TemporaryDirectory() as bench:
            os.makedirs(os.path.join(bench, "logs"))
            since = time.time() - 1
            path = os.path.join(bench, "logs", "run.log")
            with open(path, "w") as f:
                f.write("run x\non suite il_goal_cursor_collect_a with 1 stages\n")
            self.assertFalse(c.completed(bench, "il_goal_cursor_collect_a.yaml", since))
            with open(path, "a") as f:
                f.write("1.0: BENCHMARK COMPLETED SUCCESSFULLY\n")
            self.assertTrue(c.completed(bench, "il_goal_cursor_collect_a.yaml", since))
            self.assertFalse(c.completed(bench, "il_goal_cursor_collect_a_r2.yaml", since))
            self.assertFalse(c.completed(bench, "il_goal_cursor_collect_a.yaml", time.time() + 10))


if __name__ == "__main__":
    unittest.main()
