#!/usr/bin/env python3
from ptu_api import D48PTU, D48Config
import time

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



from ptu_api import D48PTU, D48Config
import time


ptu = D48PTU(D48Config(port="/dev/ttyUSB0"))
ptu.open()

show(ptu, "start")

# -160< azimuth < 160, -45 < elevation < 45
ptu.set_deg([160, 40])   # [azimuth, elevation] in deg
show(ptu, "after")
time.sleep(8)

# -160< azimuth < 160, -35 < elevation < 35
ptu.set_deg([-160, -40])   # [azimuth, elevation] in deg
time.sleep(8)
show(ptu, "after")


time.sleep(8)

ptu.set_rad([0.0, 0.0])
show(ptu, "stop")
ptu.close()
