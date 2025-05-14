---
file_format: mystnb
kernelspec:
  name: python3
  display_name: python3
---

# Installing

```{code-cell} ipython3
%%capture
%pip install dqrobotics --pre
%pip install dqrobotics --pre --break-system-packages
%pip install marinholab-papers-tro2022-adaptivecontrol
%pip install marinholab-papers-tro2022-adaptivecontrol --break-system-packages
```

# Importing

```{attention}
  You must import `DQ_SerialManipulator` before attempting to import the `adaptive_control` package.
```

```{code-cell}
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator
from marinholab.papers.tro2022.adaptive_control import *
```

# Usage

##