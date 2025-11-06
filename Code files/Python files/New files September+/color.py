# ========= 4x VL53L0X + TCS34725 (no time.sleep, fast custom TCS, RGB detection + Blue boost) =========
# - ToFs via XSHUT, readdress to 0x30..0x33 (no time.sleep)
# - Waits (busy-loop) for you to plug/power the TCS34725 (0x29)
# - Custom fast TCS driver (no Adafruit TCS lib), RGB-based detection
# - "Blue boost" makes blue stand out vs white much more
# ================================================================================================

import time
import board
import busio
import digitalio
import adafruit_vl53l0x   # keep Adafruit driver ONLY for ToF

# ---------------- I2C ----------------
i2c = busio.I2C(board.SCL, board.SDA)

def i2c_scan_contains(addr: int) -> bool:
    while not i2c.try_lock():
        pass
    try:
        return addr in i2c.scan()
    finally:
        i2c.unlock()

# ---------------- Fast custom TCS34725 driver ----------------
class TCS34725Fast:
    _ADDR = 0x29
    _CMD, _CMD_AUTOINC = 0x80, 0x20
    _ENABLE, _ATIME, _CONTROL, _ID, _STATUS, _CDATAL = 0x00, 0x01, 0x0F, 0x12, 0x13, 0x14
    _PON, _AEN = 0x01, 0x02
    GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X = 0x00, 0x01, 0x02, 0x03

    def __init__(self, i2c_bus, atime=0xFF, gain=GAIN_4X):
        self.i2c = i2c_bus
        self.addr = self._ADDR
        # optional ID read (fast)
        _ = self._read_u8(self._ID)
        self.set_integration_atime(atime)   # 0xFF=~2.4ms fastest
        self.set_gain(gain)
        # power on + enable RGBC (busy-wait ~3ms, no sleep)
        self._write_u8(self._ENABLE, self._PON)
        self._busy_wait_ms(3)
        self._write_u8(self._ENABLE, self._PON | self._AEN)

    # --- low-level ---
    def _busy_wait_ms(self, ms):
        t = time.monotonic() + ms/1000.0
        while time.monotonic() < t:
            pass

    def _write_u8(self, reg, val):
        buf = bytes([self._CMD | (reg & 0x1F), val & 0xFF])
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.addr, buf)
        finally:
            self.i2c.unlock()

    def _read_u8(self, reg) -> int:
        w = bytes([self._CMD | (reg & 0x1F)])
        r = bytearray(1)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto_then_readfrom(self.addr, w, r)
        finally:
            self.i2c.unlock()
        return r[0]

    def _read_block(self, start_reg, nbytes) -> bytes:
        w = bytes([self._CMD | self._CMD_AUTOINC | (start_reg & 0x1F)])
        r = bytearray(nbytes)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto_then_readfrom(self.addr, w, r)
        finally:
            self.i2c.unlock()
        return bytes(r)

    # --- public API ---
    def set_integration_atime(self, atime):
        # 0xFF≈2.4ms, 0xF6≈24ms ... 0x00≈614ms
        self._write_u8(self._ATIME, atime)

    def set_gain(self, gain_code):
        self._write_u8(self._CONTROL, gain_code & 0x03)

    def data_ready(self) -> bool:
        return (self._read_u8(self._STATUS) & 0x01) != 0  # AVALID

    def read_raw(self):
        b = self._read_block(self._CDATAL, 8)
        c = b[0] | (b[1] << 8)
        r = b[2] | (b[3] << 8)
        g = b[4] | (b[5] << 8)
        bl = b[6] | (b[7] << 8)
        return c, r, g, bl

    def read_rgb_bytes(self):
        c, r, g, bl = self.read_raw()
        if c == 0:
            return 0, 0, 0, 0
        rn = min(255, int((r * 255) / c))
        gn = min(255, int((g * 255) / c))
        bn = min(255, int((bl * 255) / c))
        return rn, gn, bn, c

# ------------- ToF XSHUT pins -------------
xshut_pins = {
    "left":  board.D16,
    "right": board.D25,
    "front": board.D26,
    "back":  board.D24,
}
addresses = {"left":0x30, "right":0x31, "front":0x32, "back":0x33}

xshuts = {}
for name, pin in xshut_pins.items():
    x = digitalio.DigitalInOut(pin)
    x.direction = digitalio.Direction.OUTPUT
    x.value = False   # hold all in reset
    xshuts[name] = x

sensors = {}

def wait_for_0x29(max_seconds=2.0) -> bool:
    t0 = time.monotonic()
    while (time.monotonic() - t0) < max_seconds:
        if i2c_scan_contains(0x29):
            return True
    return False

def init_one_tof(name: str):
    for n, x in xshuts.items():
        x.value = (n == name)
    if not wait_for_0x29(max_seconds=2.0):
        raise RuntimeError(f"[TOF:{name}] 0x29 did not appear (check XSHUT/wiring).")
    s = adafruit_vl53l0x.VL53L0X(i2c)   # default 0x29
    s.set_address(addresses[name])
    try:
        s.start_continuous()
    except AttributeError:
        pass
    sensors[name] = s
    print(f"[TOF] {name:6s} -> {hex(addresses[name])} ready")

# Bring up ToFs one-by-one
for name in ["left","right","front","back"]:
    init_one_tof(name)

print("\n✅ ToFs ready (0x30..0x33). Now plug/power the TCS34725 (0x29).")

def wait_for_tcs():
    while True:
        if i2c_scan_contains(0x29):
            return

wait_for_tcs()

# Fast TCS: keep shortest integration for speed, boost blue in software
tcs = TCS34725Fast(i2c, atime=0xFF, gain=TCS34725Fast.GAIN_4X)
print("[TCS34725] Fast driver initialized.\n")

# ---------------- Helpers ----------------
def tof_cm(sensor):
    try:
        cm = sensor.range / 10.0
        if cm <= 0 or cm > 1500:
            return 999.0
        return cm
    except Exception:
        return 999.0

# ===== RGB-only classifier with Blue boost =====
BLUE_GAIN = 4.0          # amplify blue difference (try 3–8)
BLUE_THRESH = 120         # detection threshold after boost
ORANGE_THRESH = 0       # how strong red dominance must be
MIN_CLEAR = 30           # ignore super dark frames

def classify_rgb_boost(r, g, b, clear):
    if clear < MIN_CLEAR:
        return "Other", 0, 0

    # Blue boost: how much B stands above the other channels
    blue_score = max(0, b - max(r, g))
    blue_boosted = int(min(255, blue_score * BLUE_GAIN))

    # Orange score: red dominance (red vs average of green+blue)
    orange_score = max(0, r - (g + b) / 2)

    if blue_boosted >= BLUE_THRESH and b > 127:
        return "Blue", blue_boosted, int(orange_score)
    if orange_score >= ORANGE_THRESH and r > 55:
        return "Orange", blue_boosted, int(orange_score)
    return "Other", blue_boosted, int(orange_score)

# Optional smoothing for display (no sleep)
alpha = 0.35
rf=gf=bf=0

print("=== Streaming (Ctrl+C to stop) ===")
try:
    last_label = "Other"
    while True:
        # ToF readings
        L = tof_cm(sensors["left"])
        R = tof_cm(sensors["right"])
        F = tof_cm(sensors["front"])
        Bk = tof_cm(sensors["back"])

        # Color when fresh
        color_text = " | RGB=?"
        if tcs.data_ready():
            r, g, b, clear = tcs.read_rgb_bytes()

            # simple EMA smoothing for nicer numbers (doesn't slow sampling)
            rf = int((1-alpha)*rf + alpha*r)
            gf = int((1-alpha)*gf + alpha*g)
            bf = int((1-alpha)*bf + alpha*b)

            label, blue_boosted, orange_score = classify_rgb_boost(rf, gf, bf, clear)
            last_label = label
            color_text = (
                f" | R={rf:3d} G={gf:3d} B={bf:3d} C={clear:5d}"
                f" BlueBoost={blue_boosted:3d} OrangeScore={orange_score:3d}"
                f" -> {label}"
            )
        else:
            color_text = f" | -> {last_label}"

        print(f"L={L:6.1f}  R={R:6.1f}  F={F:6.1f}  B={Bk:6.1f}{color_text}")

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    for s in sensors.values():
        try: s.stop_continuous()
        except Exception: pass
    for x in xshuts.values():
        x.value = False
    print("Cleaned up.")
