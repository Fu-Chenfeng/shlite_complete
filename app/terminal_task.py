"""Keep replay prompts and command errors visible in a desktop terminal."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


if __name__ == '__main__':
    try:
        result = subprocess.run([sys.executable, str(ROOT / 'robotctl'), *sys.argv[1:]], cwd=ROOT)
        print('\nFinished.' if result.returncode == 0 else '\nTask failed (exit %d).' % result.returncode)
        status = result.returncode
    except KeyboardInterrupt:
        print('\nTask interrupted.')
        status = 130
    try:
        input('Press Enter to close this terminal and return to the control panel. ')
    except (EOFError, KeyboardInterrupt):
        pass
    sys.exit(status)
