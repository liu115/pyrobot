# Example code for TM700 robot arm

*Note*: In order to use the pyrobot codebase without installing it (due to constantly modifying). We must add some changes to the code.

```python
# Add system path before importing pyrobot
import sys
sys.path.append('[pyrobot path]/src')
from pyrobot.core import Robot # instead of from pyrobot import Robot
```
