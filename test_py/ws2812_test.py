#!/usr/bin/env python3
# File name   : ws2812_test.py
# Description : Testskript fuer die adressierbaren WS2812 / SK6812 RGB-LEDs
#               am Adeept Robot HAT V3.2 (SPI0 MOSI, GPIO 10)
#               Gemaess Adeept Lesson 11: Light up the WS2812
import math
import sys
import time
import gpiod
from gpiod.line import Direction, Value
import numpy as np
import spidev

# Standardkonfiguration
DEFAULT_BUS = 0
DEFAULT_DEVICE = 0
DEFAULT_LED_COUNT = 14      # 2 Onboard-LEDs (Index 0, 1) + bis zu 12 externe LEDs
DEFAULT_BRIGHTNESS = 60     # 0..255 (moderate Helligkeit schont Augen und 5V-Schiene)
SPI_SPEED_HZ = 6400000      # 6.4 MHz (Standard WS2812 SPI Timing fuer 1.25us/Bit)


class WS2812Controller:
    """Treiberklasse fuer WS2812/SK6812 RGB-LEDs ueber SPI0 MOSI."""

    def __init__(self, count=DEFAULT_LED_COUNT, brightness=DEFAULT_BRIGHTNESS,
                 sequence='GRB', bus=DEFAULT_BUS, device=DEFAULT_DEVICE):
        self.count = count
        self.brightness = max(0, min(255, brightness))
        self.bus = bus
        self.device = device

        # Pins 9 (LED1/MISO) und 11 (LED3/SCLK) per gpiod auf LOW pinnen.
        # Grund: Auf dem Adeept Robot HAT liegt LED3 auf GPIO 11 (dem Hardware-SPI-Clock).
        # Indem wir GPIO 11 als reinen GPIO-Ausgang auf LOW festsetzen, trennt der Pinmux
        # den Pin vom internen SPI-Taktgenerator und LED3 flackert nicht mehr bei jedem Farbwechsel!
        self.gpiod_chip = None
        self.gpiod_req = None
        try:
            self.gpiod_chip = gpiod.Chip("/dev/gpiochip0")
            self.gpiod_req = self.gpiod_chip.request_lines(
                consumer="ws2812_spi_clock_suppress",
                config={
                    (9, 11): gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.INACTIVE
                    )
                }
            )
        except Exception:
            # Falls die Lines bereits von einem anderen Prozess reserviert sind
            self.gpiod_req = None
            if self.gpiod_chip:
                try:
                    self.gpiod_chip.close()
                except Exception:
                    pass
                self.gpiod_chip = None

        # Farbreihenfolge auflösen ('GRB' Standard fuer WS2812 / SK6812)
        led_types = ['RGB', 'RBG', 'GRB', 'GBR', 'BRG', 'BGR']
        offsets = [0x06, 0x09, 0x12, 0x21, 0x18, 0x24]
        try:
            idx = led_types.index(sequence.upper())
            val = offsets[idx]
        except ValueError:
            val = 0x12  # Fallback GRB

        self.r_offset = (val >> 4) & 0x03
        self.g_offset = (val >> 2) & 0x03
        self.b_offset = (val >> 0) & 0x03

        # LED-Farbdaten initialisieren (Rohwerte 0..255)
        self.led_data = np.zeros(self.count * 3, dtype=np.uint8)

        # SPI öffnen
        self.spi = spidev.SpiDev()
        try:
            self.spi.open(self.bus, self.device)
            self.spi.mode = 0
            self.spi.max_speed_hz = SPI_SPEED_HZ
        except Exception as e:
            print(f"Fehler beim Oeffnen von SPI-Bus {self.bus}, Device {self.device}: {e}")
            raise

    def set_brightness(self, brightness):
        """Setzt die Helligkeitsskalierung (0 bis 255)."""
        self.brightness = max(0, min(255, brightness))

    def set_pixel(self, index, r, g, b):
        """Setzt die Farbe einer einzelnen LED (0 bis count-1)."""
        if index < 0 or index >= self.count:
            return
        scale = self.brightness / 255.0
        base = index * 3
        self.led_data[base + self.r_offset] = int(round(r * scale))
        self.led_data[base + self.g_offset] = int(round(g * scale))
        self.led_data[base + self.b_offset] = int(round(b * scale))

    def set_all(self, r, g, b):
        """Setzt alle LEDs auf dieselbe Farbe."""
        for i in range(self.count):
            self.set_pixel(i, r, g, b)

    def clear(self):
        """Schaltet alle LEDs aus."""
        self.led_data.fill(0)
        self.show()

    def show(self):
        """Uebertraegt den Puffer via SPI MOSI an die WS2812 LEDs."""
        # 1 Byte RGB-Wert wird in 8 SPI-Bits kodiert (1.25 µs pro Bit bei 6.4 MHz)
        # T0 = 0b10000000 (0x80), T1 = 0b11111000 (0xF8)
        d = self.led_data
        # 64 Null-Bytes am Ende garantieren den WS2812-Reset-Latch (>50 µs)
        tx = np.zeros(len(d) * 8 + 64, dtype=np.uint8)
        for ibit in range(8):
            tx[7 - ibit:len(d) * 8:8] = ((d >> ibit) & 1) * 0x78 + 0x80

        # Senden via SPI
        self.spi.writebytes2(tx)

    def close(self):
        """Schaltet LEDs aus und schliesst den SPI-Bus."""
        try:
            self.clear()
            self.spi.close()
        except Exception:
            pass
        finally:
            if self.gpiod_req:
                try:
                    self.gpiod_req.release()
                except Exception:
                    pass
                self.gpiod_req = None
            if self.gpiod_chip:
                try:
                    self.gpiod_chip.close()
                except Exception:
                    pass
                self.gpiod_chip = None


# ==============================================================================
# Demo-Funktionen
# ==============================================================================

def demo_single_step(strip: WS2812Controller, delay=0.15):
    """Laesst jede LED einzeln aufleuchten (ideal zur Zaehlung der LEDs)."""
    print("\n[Demo: Einzel-LED Durchlauf]")
    print("  Tipp: Zaehlen Sie mit, um die genaue Anzahl der LEDs zu bestimmen.")
    print("  (LED 0 & 1 sind die Onboard-LEDs, ab LED 2 die externen Module)")

    for i in range(strip.count):
        strip.set_all(0, 0, 0)
        # Onboard gruen, externe cyan
        if i < 2:
            strip.set_pixel(i, 0, 255, 0)
            tag = "Onboard"
        else:
            strip.set_pixel(i, 0, 200, 255)
            tag = "Extern"
        strip.show()
        print(f"  -> LED {i:2d} ({tag}) aktiv")
        time.sleep(delay)

    strip.clear()


def demo_colors(strip: WS2812Controller, hold_time=0.8):
    """Grundfarbentest auf allen LEDs."""
    print("\n[Demo: Grundfarben-Test]")
    colors = [
        ("Rot",      255,   0,   0),
        ("Gruen",      0, 255,   0),
        ("Blau",       0,   0, 255),
        ("Gelb",     255, 200,   0),
        ("Cyan",       0, 255, 255),
        ("Magenta",  255,   0, 255),
        ("Weiss",    255, 255, 255),
    ]

    for name, r, g, b in colors:
        print(f"  Farbe: {name}")
        strip.set_all(r, g, b)
        strip.show()
        time.sleep(hold_time)

    strip.clear()


def demo_breathing(strip: WS2812Controller, cycles=2, color=(0, 160, 255)):
    """Weicher Atmungs-Effekt (Lesson 11, S. 5-10)."""
    print(f"\n[Demo: Atmungs-Effekt (Breathing Light, {cycles} Zyklen)]")
    r_target, g_target, b_target = color
    steps = 40

    for _ in range(cycles):
        # Aufblenden
        for s in range(steps + 1):
            ratio = (1.0 - math.cos(math.pi * s / steps)) / 2.0
            strip.set_all(int(r_target * ratio), int(g_target * ratio), int(b_target * ratio))
            strip.show()
            time.sleep(0.02)
        # Abblenden
        for s in range(steps, -1, -1):
            ratio = (1.0 - math.cos(math.pi * s / steps)) / 2.0
            strip.set_all(int(r_target * ratio), int(g_target * ratio), int(b_target * ratio))
            strip.show()
            time.sleep(0.02)

    strip.clear()


def demo_police(strip: WS2812Controller, cycles=10):
    """Polizei-Blaulicht und Rotlicht-Blitzer (Lesson 11, S. 9-10)."""
    print(f"\n[Demo: Polizei-Blaulicht / Rotlicht ({cycles} Zyklen)]")
    for _ in range(cycles):
        # Blau blitzen
        for _ in range(3):
            strip.set_all(0, 0, 255)
            strip.show()
            time.sleep(0.04)
            strip.set_all(0, 0, 0)
            strip.show()
            time.sleep(0.04)
        time.sleep(0.08)

        # Rot blitzen
        for _ in range(3):
            strip.set_all(255, 0, 0)
            strip.show()
            time.sleep(0.04)
            strip.set_all(0, 0, 0)
            strip.show()
            time.sleep(0.04)
        time.sleep(0.08)

    strip.clear()


def demo_flowing_rainbow(strip: WS2812Controller, duration_sec=5.0):
    """Regenbogen-Lauflicht (Lesson 11, FlowingLights)."""
    print(f"\n[Demo: Regenbogen-Lauflicht ({duration_sec}s)]")

    def wheel(pos):
        pos = pos % 256
        if pos < 85:
            return (255 - pos * 3, pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return (0, 255 - pos * 3, pos * 3)
        else:
            pos -= 170
            return (pos * 3, 0, 255 - pos * 3)

    t_end = time.time() + duration_sec
    offset = 0
    while time.time() < t_end:
        for i in range(strip.count):
            color_index = int((i * 256 / strip.count) + offset) & 255
            r, g, b = wheel(color_index)
            strip.set_pixel(i, r, g, b)
        strip.show()
        offset = (offset + 4) % 256
        time.sleep(0.02)

    strip.clear()


# ==============================================================================
# Hauptprogramm
# ==============================================================================

def main():
    mode = "all"
    count = DEFAULT_LED_COUNT
    brightness = DEFAULT_BRIGHTNESS

    # Parameter: python3 ws2812_test.py [mode] [count] [brightness]
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
    if len(sys.argv) > 2:
        try:
            count = int(sys.argv[2])
        except ValueError:
            pass
    if len(sys.argv) > 3:
        try:
            brightness = int(sys.argv[3])
        except ValueError:
            pass

    print("=" * 65)
    print("Adeept Robot HAT V3.2 - WS2812 RGB LED Test (SPI0 MOSI / GPIO 10)")
    print(f"Modus: '{mode}' | LED-Anzahl: {count} | Helligkeit: {brightness}/255")
    print("Beenden jederzeit mit Strg+C.")
    print("=" * 65)

    try:
        strip = WS2812Controller(count=count, brightness=brightness, sequence='GRB')
    except Exception as e:
        print(f"Initialisierungsfehler: {e}")
        sys.exit(1)

    try:
        if mode in ("single", "step", "count"):
            demo_single_step(strip)

        elif mode in ("color", "colors", "rgb"):
            demo_colors(strip)

        elif mode in ("breath", "breathing"):
            demo_breathing(strip, cycles=3)

        elif mode in ("police", "cop"):
            demo_police(strip, cycles=15)

        elif mode in ("flow", "rainbow", "flowing"):
            demo_flowing_rainbow(strip, duration_sec=10.0)

        elif mode in ("on", "white"):
            print("\nAlle LEDs auf Weiss geschaltet.")
            strip.set_all(255, 255, 255)
            strip.show()
            print("Druecken Sie Strg+C zum Ausschalten.")
            while True:
                time.sleep(1)

        elif mode in ("off", "clear"):
            print("\nSchalte alle LEDs aus...")
            strip.clear()
            print("Alle LEDs ausgeschaltet.")

        else:
            # Standard: Alle Demos durchlaufen
            print("\nStarte vollstaendigen Funktionstest aller Lichteffekte...")
            demo_single_step(strip)
            time.sleep(0.5)

            demo_colors(strip)
            time.sleep(0.5)

            demo_breathing(strip, cycles=2)
            time.sleep(0.5)

            demo_flowing_rainbow(strip, duration_sec=6.0)
            time.sleep(0.5)

            demo_police(strip, cycles=6)

            print("\nTest erfolgreich beendet.")

    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer (Strg+C).")
    finally:
        strip.close()
        print("LEDs deaktiviert und SPI-Bus freigegeben.")


if __name__ == "__main__":
    main()
