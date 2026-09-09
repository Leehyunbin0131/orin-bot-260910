"""Regression for cleanup when a ROS launch parent exits before its children."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import unittest

spec = importlib.util.spec_from_file_location(
    'ros_base_runner', Path(__file__).with_name('ros_base_runner.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class ProcessCleanupTests(unittest.TestCase):
    def test_exited_parent_does_not_leave_controller_child_running(self):
        child_code = 'import time; time.sleep(60)'
        parent_code = (
            'import subprocess,sys; '
            f'p=subprocess.Popen([sys.executable,"-c",{child_code!r}],'
            'stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); '
            'print(p.pid,flush=True)')
        parent = subprocess.Popen([sys.executable, '-c', parent_code], stdout=subprocess.PIPE,
                                  text=True, start_new_session=True)
        try:
            output, _ = parent.communicate(timeout=3)
            child = int(output.strip())
            self.assertEqual(parent.returncode, 0)
            self.assertEqual(os.getpgid(child), parent.pid)
            runner.stop_process(parent)
            path = Path(f'/proc/{child}/stat')
            if path.exists():
                state = path.read_text().rsplit(')', 1)[1].split()[0]
                self.assertEqual(state, 'Z', 'Owned child still running after cleanup')
        finally:
            try:
                os.killpg(parent.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            parent.wait(timeout=3)


if __name__ == '__main__':
    unittest.main()
