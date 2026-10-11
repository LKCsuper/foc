import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "Scripts"))
import sensorless_observe_test as test


def check(reboot):
    commands = []
    ticks = [0]

    class Shell:
        def __init__(self, *args):
            self.closed = False

        def run(self, line, **kwargs):
            commands.append(line)
            if line == "olstatus":
                return "Enable : OFF\nTarget RPM : 0\n $ "
            if line == "olrpm":
                target = 72 if reboot else 300
                return f"rpm obs=0.1 meas=0 tgt={target} iq=0.5 OPEN_LOOP tk=0 fb=invalid fault=0\n $ "
            return "OK\n $ "

        def _got_prompt(self, raw):
            return True

        def drain(self, seconds):
            pass

        def close(self):
            self.closed = True

    def clock():
        ticks[0] += 1.5
        return ticks[0]

    with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as output:
        with patch.object(test, "Shell", Shell), patch.object(test, "REPO_ROOT", Path(output)), \
             patch.object(test.time, "monotonic", clock), \
             patch.object(sys, "argv", ["test", "--iq", "0.5", "--rpms", "300", "600", "800"]):
            try:
                test.main()
                raise AssertionError("expected abort")
            except RuntimeError as error:
                assert ("重启" if reboot else "未跟上") in str(error), str(error)
    assert "olspeed 600" not in commands and "olspeed 800" not in commands
    assert commands.count("olstop") == 2 and "olspeed 0" in commands
    assert "olfault" in commands and commands[-1] == "olstatus"


def check_cruise():
    commands = []
    target = [300]

    class Shell:
        def __init__(self, *args):
            pass

        def run(self, line, **kwargs):
            commands.append(line)
            if line == "olstatus":
                return "Enable : OFF\nTarget RPM : 0\n $ "
            if line.startswith("olspeed "):
                target[0] = int(line.split()[1])
            if line == "olrpm":
                return f"rpm obs={target[0]} meas=0 tgt={target[0]} iq=0.6 OPEN_LOOP tk=0 fb=invalid fault=0\n $ "
            return "OK\n $ "

        def _got_prompt(self, raw):
            return True

        def drain(self, seconds):
            pass

        def close(self):
            pass

    with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as output:
        with patch.object(test, "Shell", Shell), patch.object(test, "REPO_ROOT", Path(output)), \
             patch.object(sys, "argv", ["test", "--iq", "1", "--cruise-iq", "0.6",
                                      "--rpms", "300", "600", "--samples", "4"]):
            test.main()
    start = commands.index("olstart")
    decrease = commands.index("oliq 0.6")
    assert commands.index("oliq 1.0") < start < decrease < commands.index("olspeed 600")
    assert commands[start:decrease].count("olrpm") == 4
    assert commands[-5:] == ["olstop", "olspeed 0", "olfault", "slstats", "olstatus"]


check_cruise()
check(False)
check(True)
print("PASS: failed-start/reboot abort before speed escalation and execute stop cleanup")
