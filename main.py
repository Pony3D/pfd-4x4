#!/usr/bin/env python3
"""Lightweight 320x240 GPS/IMU PFD for Raspberry Pi framebuffer displays.

Reads GPS NMEA data over USB serial and pitch/roll from an MPU-6050 over I2C.
Use --demo to render simulated telemetry without sensor hardware.
"""

import argparse
import glob
import math
import sys
import time
from datetime import datetime, timezone

from PIL import Image, ImageDraw, ImageFont

try:
    import numpy as np
except ImportError:
    np = None

WIDTH, HEIGHT = 320, 240
BG = (0, 0, 0)
GREEN = (0, 255, 40)
MAGENTA = (238, 35, 230)
WHITE = (245, 245, 245)
GREY = (66, 66, 66)
SKY = (56, 99, 220)
GROUND = (132, 66, 23)


def font(size):
    for name in ("DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


F_SMALL = font(10)
F_MED = font(13)
F_BIG = font(20)


def label(draw, xy, value, fill=WHITE, face=F_SMALL, anchor=None):
    draw.text(xy, str(value), fill=fill, font=face, anchor=anchor)


def make_demo_data(t):
    return {
        "lat": 40.4168 + math.sin(t / 30) * 0.0003,
        "lon": -3.7038 + math.cos(t / 35) * 0.0003,
        "speed": 42 + 8 * math.sin(t / 8),
        "alt": 667 + 12 * math.sin(t / 16),
        "heading": (84 + 28 * math.sin(t / 13)) % 360,
        "pitch": 9 * math.sin(t / 7),
        "roll": 24 * math.sin(t / 9),
        "gps_fix": True,
        "imu_ok": True,
        "utc": datetime.now(timezone.utc),
    }


def _signed_word(high, low):
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value


class MPU6050:
    """MPU-6050 I2C reader with a complementary pitch/roll filter."""

    def __init__(self, address=None):
        from smbus import SMBus

        self.bus = SMBus(1)
        candidates = [address] if address is not None else [0x68, 0x69]
        self.address = None
        for candidate in candidates:
            try:
                if self.bus.read_byte_data(candidate, 0x75) in (0x68, 0x69):
                    self.address = candidate
                    break
            except OSError:
                continue
        if self.address is None:
            self.bus.close()
            raise OSError("No encuentro el MPU-6050 en 0x68 ni 0x69")

        # Wake the MPU-6050, use the default +/-250 dps and +/-2 g ranges,
        # and enable a modest low-pass filter for vehicle vibration.
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        self.bus.write_byte_data(self.address, 0x1A, 0x03)
        time.sleep(0.1)

        # Keep the vehicle still during this short startup gyro-bias sample.
        samples = [self._raw() for _ in range(40)]
        self.bias_x = sum(row[4] for row in samples) / len(samples)
        self.bias_y = sum(row[5] for row in samples) / len(samples)
        ax, ay, az = samples[-1][:3]
        self.roll = math.degrees(math.atan2(ay, az))
        self.pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        self.last_time = time.monotonic()

    def _raw(self):
        values = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
        words = [_signed_word(values[i], values[i + 1]) for i in range(0, 14, 2)]
        # accel X/Y/Z, temperature, gyro X/Y/Z
        return words[0], words[1], words[2], words[3], words[4], words[5], words[6]

    def read(self):
        ax, ay, az, _temperature, gx, gy, _gz = self._raw()
        now = time.monotonic()
        dt = max(0.001, min(now - self.last_time, 0.25))
        self.last_time = now

        ax_g, ay_g, az_g = ax / 16384.0, ay / 16384.0, az / 16384.0
        accel_roll = math.degrees(math.atan2(ay_g, az_g))
        accel_pitch = math.degrees(math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)))

        alpha = 0.96
        self.roll = alpha * (self.roll + (gx - self.bias_x) / 131.0 * dt) + (1 - alpha) * accel_roll
        self.pitch = alpha * (self.pitch + (gy - self.bias_y) / 131.0 * dt) + (1 - alpha) * accel_pitch
        return self.pitch, self.roll

    def close(self):
        self.bus.close()


class ResetButton:
    """Debounced normally-open button between BCM GPIO21 and GND."""

    def __init__(self, gpio=21, debounce=0.05):
        import RPi.GPIO as GPIO

        self.GPIO = GPIO
        self.gpio = gpio
        self.debounce = debounce
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.raw_state = GPIO.input(gpio)
        self.stable_state = self.raw_state
        self.changed_at = time.monotonic()

    def pressed(self):
        raw = self.GPIO.input(self.gpio)
        now = time.monotonic()
        if raw != self.raw_state:
            self.raw_state = raw
            self.changed_at = now
        if raw != self.stable_state and now - self.changed_at >= self.debounce:
            self.stable_state = raw
            return raw == self.GPIO.LOW
        return False

    def close(self):
        self.GPIO.cleanup(self.gpio)


def _nmea_coord(value, hemisphere, degree_digits):
    if not value:
        return None
    degrees = float(value[:degree_digits])
    minutes = float(value[degree_digits:])
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


class GPSReader:
    """Read standard NMEA RMC/GGA sentences from a USB serial GPS."""

    def __init__(self, port=None, baud=9600):
        self.port = port
        self.baud = baud
        self.serial = None
        self.buffer = b""
        self.next_scan = 0.0
        self.last_fix = 0.0
        self.lat = self.lon = self.alt = self.speed = None
        self.heading = None
        self.utc = None

    def _open(self):
        try:
            import serial
        except ImportError:
            self.next_scan = time.monotonic() + 10
            print("GPS: falta python3-serial", file=sys.stderr)
            return

        paths = [self.port] if self.port else sorted(
            set(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        )
        for path in paths:
            try:
                self.serial = serial.Serial(path, self.baud, timeout=0)
                self.buffer = b""
                print(f"GPS: puerto {path}, {self.baud} baudios")
                return
            except (OSError, serial.SerialException):
                continue
        self.next_scan = time.monotonic() + 5

    def _parse_line(self, line):
        try:
            sentence = line.decode("ascii", errors="ignore").strip()
            if not sentence.startswith("$"):
                return
            fields = sentence[1:].split("*")[0].split(",")
            kind = fields[0][-3:]

            if kind == "RMC" and len(fields) >= 10:
                # RMC provides date/time, coordinates, speed and course.
                time_field, status = fields[1], fields[2]
                if len(time_field) >= 6 and len(fields[9]) == 6:
                    hh, mm, ss = int(time_field[0:2]), int(time_field[2:4]), int(time_field[4:6])
                    fraction = time_field.split(".", 1)[1] if "." in time_field else ""
                    microsecond = int((fraction + "000000")[:6])
                    dd, month, yy = int(fields[9][0:2]), int(fields[9][2:4]), int(fields[9][4:6])
                    year = 2000 + yy if yy < 80 else 1900 + yy
                    self.utc = datetime(year, month, dd, hh, mm, ss, microsecond, tzinfo=timezone.utc)

                if status == "A":
                    lat = _nmea_coord(fields[3], fields[4], 2)
                    lon = _nmea_coord(fields[5], fields[6], 3)
                    if lat is not None and lon is not None:
                        self.lat, self.lon = lat, lon
                        knots = float(fields[7]) if fields[7] else 0.0
                        self.speed = knots * 1.852
                        if fields[8] and self.speed >= 2.0:
                            self.heading = float(fields[8]) % 360
                        self.last_fix = time.monotonic()

            elif kind == "GGA" and len(fields) >= 10:
                # GGA provides fix quality and GPS altitude in metres.
                if int(fields[6] or "0") > 0:
                    lat = _nmea_coord(fields[2], fields[3], 2)
                    lon = _nmea_coord(fields[4], fields[5], 3)
                    if lat is not None and lon is not None:
                        self.lat, self.lon = lat, lon
                    if fields[9]:
                        self.alt = float(fields[9])
                    self.last_fix = time.monotonic()
        except (ValueError, IndexError):
            return

    def poll(self):
        if self.serial is None:
            if time.monotonic() >= self.next_scan:
                self._open()
            return
        try:
            chunk = self.serial.read(256)
            if chunk:
                self.buffer += chunk
                while b"\n" in self.buffer:
                    line, self.buffer = self.buffer.split(b"\n", 1)
                    self._parse_line(line)
        except OSError as exc:
            print(f"GPS: lectura USB fallida: {exc}", file=sys.stderr)
            self.serial.close()
            self.serial = None
            self.next_scan = time.monotonic() + 3

    def snapshot(self):
        fix = time.monotonic() - self.last_fix < 5
        return {
            "gps_fix": fix,
            "lat": self.lat if fix else None,
            "lon": self.lon if fix else None,
            "speed": self.speed if fix and self.speed is not None else 0.0,
            "alt": self.alt if fix and self.alt is not None else 0.0,
            # Keep the last course while stopped; GPS course is undefined at rest.
            "heading": self.heading,
            "utc": self.utc,
        }


def draw_tape(draw, x0, x1, y0, y1, value, step, scale, unit, color=WHITE):
    draw.rectangle((x0, y0, x1, y1), fill=GREY)
    center = (y0 + y1) // 2
    base = int(value // step) * step
    for n in range(-4, 5):
        tick_value = base + n * step
        y = center - int((tick_value - value) * scale)
        if y0 + 2 <= y <= y1 - 2:
            draw.line((x1 - 15, y, x1 - 1, y), fill=WHITE, width=1)
            if n % 2 == 0:
                label(draw, (x0 + 2, y), f"{tick_value:g}", WHITE, F_SMALL, "lm")
    # Fixed-value window; moving ticks provide the tape animation.
    draw.rectangle((x0 - 1, center - 13, x1 + 1, center + 13), fill=BG, outline=WHITE, width=1)
    label(draw, ((x0 + x1) // 2, center), f"{value:.0f}", WHITE, F_MED, "mm")
    label(draw, (x1 - 1, y1 + 3), unit, GREEN, F_SMALL, "ra")


def render(data, now=None):
    now = data.get("utc") or now or datetime.now(timezone.utc)
    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    d = ImageDraw.Draw(image)

    # Top line: location at left, live UTC clock at right.
    if data.get("gps_fix") and data.get("lat") is not None and data.get("lon") is not None:
        location = f"{data['lat']:.5f} {data['lon']:.5f}"
    else:
        location = "GPS WAIT"
    label(d, (5, 4), location, GREEN if data.get("gps_fix") else MAGENTA, F_SMALL)
    label(d, (315, 4), now.strftime("%H:%M:%S UTC"), GREEN, F_SMALL, "ra")

    draw_tape(d, 3, 48, 31, 177, data["speed"], 10, 1.15, "km/h")
    draw_tape(d, 272, 317, 31, 177, data["alt"], 20, 0.72, "m")

    # Attitude display, clipped to a rounded cockpit-style window.
    box = (56, 34, 264, 187)
    mask = Image.new("L", (WIDTH, HEIGHT), 0)
    ImageDraw.Draw(mask).rounded_rectangle(box, radius=20, fill=255)
    horizon = Image.new("RGB", (WIDTH, HEIGHT), SKY)
    hd = ImageDraw.Draw(horizon)
    cx, cy = 160, 110
    angle = math.radians(data["roll"])
    dx, dy = math.cos(angle), math.sin(angle)
    nx, ny = -dy, dx
    offset = data["pitch"] * 2.15
    # The horizon line is centered at the pitch offset. Fill the half-plane
    # below it with terrain, extending the polygon well beyond the display.
    p1 = (cx - dx * 500 + nx * offset, cy - dy * 500 + ny * offset)
    p2 = (cx + dx * 500 + nx * offset, cy + dy * 500 + ny * offset)
    p3 = (p2[0] + nx * 700, p2[1] + ny * 700)
    p4 = (p1[0] + nx * 700, p1[1] + ny * 700)
    hd.polygon((p1, p2, p3, p4), fill=GROUND)
    hd.line((p1, p2), fill=WHITE, width=2)
    for degrees in (-30, -20, -10, 10, 20, 30):
        dist = -degrees * 2.15
        half = 23 if abs(degrees) == 10 else 15
        a = (cx - dx * half + nx * (offset + dist), cy - dy * half + ny * (offset + dist))
        b = (cx + dx * half + nx * (offset + dist), cy + dy * half + ny * (offset + dist))
        hd.line((a, b), fill=WHITE, width=1)
    image.paste(horizon, (0, 0), mask)
    d = ImageDraw.Draw(image)
    d.rounded_rectangle(box, radius=20, outline=WHITE, width=1)

    # Fixed aircraft reference marks and small bank scale.
    d.line((112, 111, 147, 111), fill=WHITE, width=3)
    d.line((173, 111, 208, 111), fill=WHITE, width=3)
    d.line((147, 111, 147, 121, 160, 121, 160, 116), fill=WHITE, width=2)
    d.line((173, 111, 173, 121, 160, 121), fill=WHITE, width=2)
    for x, h in ((119, 7), (133, 9), (147, 11), (160, 12), (173, 11), (187, 9), (201, 7)):
        d.line((x, 48, x, 48 + h), fill=WHITE, width=1)
    d.rounded_rectangle((62, 167, 111, 183), radius=3, fill=BG)
    d.rounded_rectangle((209, 167, 258, 183), radius=3, fill=BG)
    if data.get("imu_ok"):
        pitch_text = f"P {data['pitch']:+.0f}°"
        roll_text = f"R {data['roll']:+.0f}°"
    else:
        pitch_text, roll_text = "P --°", "R --°"
    label(d, (67, 175), pitch_text, GREEN if data.get("imu_ok") else MAGENTA, F_SMALL, "lm")
    label(d, (253, 175), roll_text, GREEN if data.get("imu_ok") else MAGENTA, F_SMALL, "rm")
    label(d, (160, 21), "PFD - 4x4", GREEN, F_MED, "mm")

    # Animated heading strip. The tick positions move under the fixed index.
    d.rectangle((61, 202, 259, 231), fill=GREY)
    heading_value = data.get("heading")
    heading = (heading_value or 0) % 360
    d.line((160, 202, 160, 210), fill=MAGENTA, width=2)
    for offset_deg in range(-60, 61, 10):
        val = (int(round(heading / 10) * 10) + offset_deg) % 360
        x = 160 + int((offset_deg - (heading % 10)) * 1.45)
        if 63 <= x <= 257:
            d.line((x, 222, x, 222 - (8 if val % 30 == 0 else 4)), fill=WHITE, width=1)
            if val % 30 == 0:
                label(d, (x, 210), f"{val:03d}", WHITE, F_SMALL, "mm")
    d.polygon(((155, 201), (165, 201), (160, 207)), fill=MAGENTA)
    heading_text = f"HDG {heading:03.0f}°" if heading_value is not None else "HDG ---"
    label(d, (160, 234), heading_text, GREEN if heading_value is not None else MAGENTA, F_SMALL, "ms")
    return image


def write_rgb565(frame, path):
    """Write a 320x240 little-endian RGB565 frame to a Linux framebuffer."""
    pixels = frame.convert("RGB").tobytes()
    if np is not None:
        rgb = np.frombuffer(pixels, dtype=np.uint8).reshape((-1, 3)).astype(np.uint16)
        packed = (((rgb[:, 0] >> 3) << 11) | ((rgb[:, 1] >> 2) << 5) | (rgb[:, 2] >> 3))
        packed = packed.astype("<u2", copy=False).tobytes()
    else:
        # Slow fallback for systems where NumPy has not been installed.
        output = bytearray(WIDTH * HEIGHT * 2)
        out = 0
        for i in range(0, len(pixels), 3):
            r, g, b = pixels[i], pixels[i + 1], pixels[i + 2]
            value = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            output[out] = value & 0xFF
            output[out + 1] = value >> 8
            out += 2
        packed = output
    with open(path, "wb", buffering=0) as fb:
        fb.write(packed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fb", default="/dev/fb1", help="framebuffer device (default: /dev/fb1)")
    parser.add_argument("--fps", type=int, default=24, help="refresh rate (default: 24)")
    parser.add_argument("--preview", help="save one PNG frame and exit")
    parser.add_argument("--no-fb", action="store_true", help="run animation without writing to framebuffer")
    parser.add_argument("--demo", action="store_true", help="use simulated telemetry instead of GPS and MPU hardware")
    parser.add_argument("--gps-port", help="GPS serial device, e.g. /dev/ttyACM0 (auto-detected if omitted)")
    parser.add_argument("--gps-baud", type=int, default=9600, help="GPS serial speed (default: 9600)")
    parser.add_argument("--i2c-address", type=lambda value: int(value, 0), help="MPU address, normally 0x68; otherwise scan 0x68/0x69")
    args = parser.parse_args()

    if args.preview:
        render(make_demo_data(time.monotonic())).save(args.preview)
        print(f"Preview saved to {args.preview}")
        return

    imu = None
    gps = None
    button = None
    pitch_zero = 0.0
    roll_zero = 0.0
    if not args.demo:
        try:
            imu = MPU6050(args.i2c_address)
            print(f"MPU-6050: conectado en I2C 0x{imu.address:02x}; mantén el vehículo quieto al arrancar para calibrar gyro")
        except (ImportError, OSError) as exc:
            print(f"MPU-6050: no disponible ({exc}); mostraré pitch/roll como --")
        gps = GPSReader(args.gps_port, args.gps_baud)
        try:
            button = ResetButton(21)
            print("Cero de montaje: botón BCM GPIO21 activo")
        except (ImportError, RuntimeError, OSError) as exc:
            print(f"Botón de cero no disponible ({exc}); instalar python3-rpi.gpio")

    start = time.monotonic()
    delay = 1.0 / max(1, min(args.fps, 30))
    try:
        while True:
            if args.demo:
                data = make_demo_data(time.monotonic() - start)
            else:
                data = {
                    "lat": None, "lon": None, "speed": 0.0, "alt": 0.0,
                    "heading": None, "pitch": 0.0, "roll": 0.0,
                    "gps_fix": False, "imu_ok": False, "utc": None,
                }
                if imu is not None:
                    try:
                        measured_pitch, measured_roll = imu.read()
                        if button is not None and button.pressed():
                            pitch_zero, roll_zero = measured_pitch, measured_roll
                            print(f"Cero fijado: pitch={pitch_zero:+.1f}°, roll={roll_zero:+.1f}°")
                        data["pitch"] = measured_pitch - pitch_zero
                        data["roll"] = (measured_roll - roll_zero + 180.0) % 360.0 - 180.0
                        data["imu_ok"] = True
                    except OSError as exc:
                        print(f"MPU-6050: lectura I2C fallida ({exc})")
                        imu.close()
                        imu = None
                gps.poll()
                data.update(gps.snapshot())
            frame = render(data)
            if not args.no_fb:
                write_rgb565(frame, args.fb)
            time.sleep(delay)
    except KeyboardInterrupt:
        if not args.no_fb:
            write_rgb565(Image.new("RGB", (WIDTH, HEIGHT), BG), args.fb)
    except PermissionError:
        print(f"No access to {args.fb}; run with framebuffer permissions or use --preview.", file=sys.stderr)
        return 2
    finally:
        if imu is not None:
            imu.close()
        if button is not None:
            button.close()
        if gps is not None and gps.serial is not None:
            gps.serial.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
