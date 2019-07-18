# Example code for TM700 robot arm

*Note*: In order to use the pyrobot codebase without installing it (due to constantly modifying). We must add some changes to the code.

Put the pyrobot files under home directory
```python
# Add system path before importing pyrobot
import sys
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/pyrobot/src')
from pyrobot.core import Robot # instead of from pyrobot import Robot
```
