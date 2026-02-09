### Command to control the PTUD48 Gimba

In the folder ptuD48_control folder:

```
ptuD48_control
	-- ptu_api.py	# the ptu control api
	-- test_ptu.py	# demo how to use api
```

How to use api

```
# first initialize the api object
from ptu_api import D48PTU, D48Config
import time


ptu = D48PTU(D48Config(port="/dev/ttyUSB0"))	#initialize with the port configure
ptu.open()		# open the contrl loop

ptu.set_deg([160, 40])   # [azimuth, elevation] in deg
ptu.set_rad([0.0, 0.0])	 # [azimuth, elevation] in rad

ptu.close() # close the api

```

```
#!/usr/bin/env python3
# d48_ptu_api.py
#
# D48 PTU API: set/query by [azimuth, elevation] in deg or rad.
# - azimuth  -> Pan axis
# - elevation-> Tilt axis
#
# Requires: python3-serial (apt) OR pip pyserial
# Tested protocol assumptions from your minicom session:
#   pp  -> "* Current Pan position is X"
#   tp  -> "* Current Tilt position is X"
#   PP <pos> sets target pan position
#   TP <pos> sets target tilt position
#   PR  -> "* <arcsec_per_pos> seconds arc per position"
```

Following are the helper function for debugging

```
def show(ptu, tag):
    az_el = ptu.get_deg()
    pan_pos = ptu.cmd("pp")
    tilt_pos = ptu.cmd("tp")
    pan_tgt = ptu.cmd("PO")  # target pan
    tilt_tgt = ptu.cmd("TO") # target tilt
    print(f"\n--- {tag} ---")
    print("get_deg:", az_el)
    print("pp:", pan_pos)
    print("tp:", tilt_pos)
    print("PO:", pan_tgt)
    print("TO:", tilt_tgt)
```

