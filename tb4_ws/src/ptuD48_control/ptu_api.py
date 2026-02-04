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

from __future__ import annotations
import re
import time
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List

import serial


@dataclass
class D48Config:
    port: str = "/dev/ttyUSB0"     # Windows: "COM3"
    baudrate: int = 9600
    timeout: float = 0.7           # seconds
    write_timeout: float = 0.7
    arcsec_per_pos: float = 92.571429  # from your device
    line_ending: str = "\r"

    # Optional soft limits (in degrees) for safety
    pan_min_deg: Optional[float] = -160.0
    pan_max_deg: Optional[float] = 160.0
    tilt_min_deg: Optional[float] = -35.1
    tilt_max_deg: Optional[float] = 35.1


class D48PTU:
    """
    Public API:
      - get_deg() / get_rad() -> [azimuth, elevation]
      - set_deg([az, el]) / set_rad([az, el])
      - goto_deg(az=..., el=...) / goto_rad(az=..., el=...)
      - stop()
      - refresh_scale_from_device()
    """

    def __init__(self, cfg: D48Config):
        self.cfg = cfg
        self.ser: Optional[serial.Serial] = None

    # ----------------- serial I/O -----------------
    def open(self) -> None:
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(
            port=self.cfg.port,
            baudrate=self.cfg.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.cfg.timeout,
            write_timeout=self.cfg.write_timeout,
        )
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass
        time.sleep(0.1)

    def close(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    def _write_line(self, line: str) -> None:
        assert self.ser is not None
        payload = (line.strip() + self.cfg.line_ending).encode("ascii", errors="ignore")
        self.ser.write(payload)
        self.ser.flush()

    def _read_all(self, max_wait: float = 0.4) -> str:
        """Read whatever comes back within a short window."""
        assert self.ser is not None
        t0 = time.time()
        chunks = []
        while time.time() - t0 < max_wait:
            n = getattr(self.ser, "in_waiting", 0)
            if n:
                chunks.append(self.ser.read(n).decode("ascii", errors="ignore"))
                t0 = time.time()  # extend to catch trailing lines
            else:
                time.sleep(0.01)
        return "".join(chunks).strip()

    def cmd(self, line: str, read: bool = True) -> str:
        if not self.ser or not self.ser.is_open:
            self.open()
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._write_line(line)
        return self._read_all() if read else ""

    # ----------------- conversion -----------------
    def deg_to_pos(self, deg: float) -> int:
        # 1 degree = 3600 arcsec
        pos = deg * 3600.0 / float(self.cfg.arcsec_per_pos)
        return int(round(pos))

    def pos_to_deg(self, pos: int) -> float:
        return float(pos) * float(self.cfg.arcsec_per_pos) / 3600.0

    def rad_to_pos(self, rad: float) -> int:
        return self.deg_to_pos(math.degrees(rad))

    def pos_to_rad(self, pos: int) -> float:
        return math.radians(self.pos_to_deg(pos))

    def _clamp_deg(self, axis: str, deg: float) -> float:
        if axis == "pan":
            mn, mx = self.cfg.pan_min_deg, self.cfg.pan_max_deg
        else:
            mn, mx = self.cfg.tilt_min_deg, self.cfg.tilt_max_deg
        if mn is not None and deg < mn:
            return mn
        if mx is not None and deg > mx:
            return mx
        return deg

    # ----------------- parsing -----------------
    _re_cur_pan = re.compile(r"Current\s+Pan\s+position\s+is\s+(-?\d+)", re.IGNORECASE)
    _re_cur_tilt = re.compile(r"Current\s+Tilt\s+position\s+is\s+(-?\d+)", re.IGNORECASE)
    _re_scale = re.compile(r"([0-9]+(?:\.[0-9]+)?)\s*seconds?\s*arc\s*per\s*position", re.IGNORECASE)

    def refresh_scale_from_device(self) -> float:
        out = self.cmd("PR")
        m = self._re_scale.search(out)
        if not m:
            raise RuntimeError(f"Could not parse scale from response:\n{out}")
        self.cfg.arcsec_per_pos = float(m.group(1))
        return self.cfg.arcsec_per_pos

    # ----------------- axis getters/setters (positions) -----------------
    def _get_pan_pos(self) -> int:
        out = self.cmd("pp")
        m = self._re_cur_pan.search(out)
        if not m:
            raise RuntimeError(f"Could not parse pan position from:\n{out}")
        return int(m.group(1))

    def _get_tilt_pos(self) -> int:
        out = self.cmd("tp")
        m = self._re_cur_tilt.search(out)
        if not m:
            raise RuntimeError(f"Could not parse tilt position from:\n{out}")
        return int(m.group(1))

    def _set_pan_pos(self, pos: int) -> str:
        return self.cmd(f"PP{pos}")

    def _set_tilt_pos(self, pos: int) -> str:
        return self.cmd(f"TP{pos}")


    # ----------------- unified angle API -----------------
    def get_deg(self) -> List[float]:
        """Return [azimuth_deg, elevation_deg]."""
        pan_pos = self._get_pan_pos()
        tilt_pos = self._get_tilt_pos()
        return [self.pos_to_deg(pan_pos), self.pos_to_deg(tilt_pos)]

    def get_rad(self) -> List[float]:
        """Return [azimuth_rad, elevation_rad]."""
        pan_pos = self._get_pan_pos()
        tilt_pos = self._get_tilt_pos()
        return [self.pos_to_rad(pan_pos), self.pos_to_rad(tilt_pos)]

    def set_deg(self, angles_deg: List[float]) -> None:
        """
        angles_deg = [azimuth_deg, elevation_deg]
        Sets target positions (non-blocking).
        """
        if len(angles_deg) != 2:
            raise ValueError("set_deg expects [azimuth_deg, elevation_deg]")
        az_deg, el_deg = float(angles_deg[0]), float(angles_deg[1])

        az_deg = self._clamp_deg("pan", az_deg)
        el_deg = self._clamp_deg("tilt", el_deg)

        self._set_pan_pos(self.deg_to_pos(az_deg))
        self._set_tilt_pos(self.deg_to_pos(el_deg))

    def set_rad(self, angles_rad: List[float]) -> None:
        """
        angles_rad = [azimuth_rad, elevation_rad]
        Sets target positions (non-blocking).
        """
        if len(angles_rad) != 2:
            raise ValueError("set_rad expects [azimuth_rad, elevation_rad]")
        az_rad, el_rad = float(angles_rad[0]), float(angles_rad[1])

        az_deg = math.degrees(az_rad)
        el_deg = math.degrees(el_rad)

        az_deg = self._clamp_deg("pan", az_deg)
        el_deg = self._clamp_deg("tilt", el_deg)

        self._set_pan_pos(self.deg_to_pos(az_deg))
        self._set_tilt_pos(self.deg_to_pos(el_deg))

    def goto_deg(self, azimuth_deg: Optional[float] = None, elevation_deg: Optional[float] = None) -> None:
        """Convenience: set only one axis if you want."""
        if azimuth_deg is not None:
            az = self._clamp_deg("pan", float(azimuth_deg))
            self._set_pan_pos(self.deg_to_pos(az))
        if elevation_deg is not None:
            el = self._clamp_deg("tilt", float(elevation_deg))
            self._set_tilt_pos(self.deg_to_pos(el))

    def goto_rad(self, azimuth_rad: Optional[float] = None, elevation_rad: Optional[float] = None) -> None:
        """Convenience: set only one axis if you want."""
        if azimuth_rad is not None:
            az_deg = self._clamp_deg("pan", math.degrees(float(azimuth_rad)))
            self._set_pan_pos(self.deg_to_pos(az_deg))
        if elevation_rad is not None:
            el_deg = self._clamp_deg("tilt", math.degrees(float(elevation_rad)))
            self._set_tilt_pos(self.deg_to_pos(el_deg))

    def stop(self) -> str:
        return self.cmd("ST")


# ----------------- quick demo -----------------
if __name__ == "__main__":
    cfg = D48Config(port="/dev/ttyUSB0")
    ptu = D48PTU(cfg)
    ptu.open()

    # Optional: update scale from device (uses PR)
    try:
        print("arcsec_per_pos =", ptu.refresh_scale_from_device())
    except Exception as e:
        print("[WARN] scale not refreshed:", e)

    print("Current [az, el] deg:", ptu.get_deg())

    # Move by setting absolute angles
    ptu.set_deg([10.0, 5.0])   # az=10°, el=5°
    time.sleep(2)

    ptu.set_rad([0.0, 0.0])    # back to 0 rad, 0 rad
    time.sleep(2)

    ptu.close()
