#!/usr/bin/env python3
# File name   : battery_test.py
# Description : Batteriestatus- und Spannungsueberwachung fuer Adeept Robot HAT V3.2
#               Verwendet den 8-Bit I2C ADC ADS7830 (Adresse 0x48, Kanal A0)
#               Gemaess Adeept Lesson 17: Battery Level Detection and Alarm
import json
import sys
import time

try:
    import smbus
except ImportError:
    import smbus2 as smbus

# Hardware-Konfiguration (ADS7830 ADC am I2C-Bus 1)
I2C_BUS = 1
ADS7830_ADDR = 0x48
CMD_BASE = 0x84
CHANNEL_BATTERY = 0  # A0: Batteriespannung

# Spannungsteiler & Akkuparameter (2x 18650 Li-Ion in Reihe)
R15 = 3000.0                       # 3.0 kOhm
R17 = 1000.0                       # 1.0 kOhm
DIVISION_RATIO = R17 / (R15 + R17)  # 0.25 (Spannungsteiler 1:4)
V_REF_MAX = 8.4                    # Volle 2S Li-Ion Akkuspannung (2 x 4.2V)
V_WARN_MIN = 6.75                  # Entladeschluss- / Warnschwelle (ca. 2 x 3.375V)


class BatteryMonitor:
    """Liest die Akkuspannung und den Ladezustand ueber den ADS7830 ADC aus."""

    def __init__(self, bus_id=I2C_BUS, addr=ADS7830_ADDR):
        self.addr = addr
        self.bus = smbus.SMBus(bus_id)
        # Steuerbyte fuer Kanal 0 (Single-Ended, interne Referenz aus, ADC ein)
        self.control_byte = CMD_BASE | (((CHANNEL_BATTERY << 2 | CHANNEL_BATTERY >> 1) & 0x07) << 4)

    def read_raw_adc(self) -> int:
        """Liest den rohen 8-Bit ADC-Wert (0..255) von Kanal A0."""
        return self.bus.read_byte_data(self.addr, self.control_byte)

    def get_status(self) -> dict:
        """
        Liest den Akku aus und gibt ein Dictionary mit allen relevanten Werten zurueck.
        """
        adc_val = self.read_raw_adc()

        # Spannung am ADC-Pin A0 (Referenzspannung 5.0V)
        a0_voltage = (adc_val / 255.0) * 5.0

        # Tatsaechliche Batteriespannung vor dem 1:4 Spannungsteiler
        actual_voltage = a0_voltage / DIVISION_RATIO

        # Prozentberechnung gemaess Lesson 17
        percentage = ((actual_voltage - V_WARN_MIN) / (V_REF_MAX - V_WARN_MIN)) * 100.0
        percentage = max(0.0, min(100.0, percentage))

        is_warning = percentage < 20.0 or actual_voltage <= V_WARN_MIN
        is_critical = actual_voltage < 6.4

        if is_critical:
            health = "KRITISCH LEER!"
        elif is_warning:
            health = "SCHWACH (Bitte laden)"
        elif percentage > 75.0:
            health = "Sehr gut / Voll"
        elif percentage > 40.0:
            health = "Gut"
        else:
            health = "Normal"

        return {
            "raw_adc": adc_val,
            "a0_voltage": round(a0_voltage, 3),
            "voltage": round(actual_voltage, 2),
            "percentage": round(percentage, 1),
            "warning": is_warning,
            "critical": is_critical,
            "status": health
        }

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


def render_battery_bar(pct: float, length: int = 15) -> str:
    """Erzeugt eine visuelle Batterie-Balkenanzeige."""
    filled = int(round((pct / 100.0) * length))
    bar = "#" * filled + "-" * (length - filled)
    return f"[{bar}]"


def run_oled_display(status: dict):
    """Optionale Anzeige des Akkustatus auf dem SSD1306 OLED Display."""
    try:
        import board
        import busio
        import adafruit_ssd1306
        from PIL import Image, ImageDraw, ImageFont

        i2c = busio.I2C(board.SCL, board.SDA)
        oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)
        font = ImageFont.load_default()

        img = Image.new("1", (128, 64))
        draw = ImageDraw.Draw(img)

        # Rahmen & Titel
        draw.rectangle((0, 0, 127, 14), fill=255)
        draw.text((4, 1), "AKKU STATUS (2S)", font=font, fill=0)

        # Großer Batterie-Umriss
        # Korpus: (10, 22) bis (100, 42), Kontakt: (100, 27) bis (105, 37)
        draw.rectangle((10, 22, 100, 42), outline=255, fill=0)
        draw.rectangle((101, 27, 105, 37), fill=255)

        # Füllstand
        fill_w = int(round((status["percentage"] / 100.0) * 86))
        if fill_w > 0:
            draw.rectangle((12, 24, 12 + fill_w, 40), fill=255)

        # Text darunter
        draw.text((10, 48), f"{status['percentage']:.1f}% ({status['voltage']:.2f}V)", font=font, fill=255)
        draw.text((80, 48), status["status"][:8], font=font, fill=255)

        oled.image(img)
        oled.show()
    except Exception as e:
        print(f"OLED-Anzeige nicht moeglich: {e}")


def main():
    mode = "loop"
    interval = 1.0

    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg in ("once", "single", "check", "1"):
            mode = "once"
        elif arg in ("json",):
            mode = "json"
        elif arg in ("oled",):
            mode = "oled"
        else:
            try:
                interval = float(arg)
            except ValueError:
                pass

    try:
        mon = BatteryMonitor()
    except Exception as e:
        print(f"Fehler beim Zugriff auf ADS7830 (Adresse {hex(ADS7830_ADDR)}): {e}")
        print("Hinweis: Pruefen Sie die I2C-Verbindung und stellen Sie sicher, dass das Robot HAT V3.2 eingeschaltet ist.")
        sys.exit(1)

    try:
        if mode == "once":
            s = mon.get_status()
            bar = render_battery_bar(s["percentage"])
            print("=" * 60)
            print("Adeept Robot HAT V3.2 - Akkustand (2S 18650 Li-Ion)")
            print("=" * 60)
            print(f"Spannung:    {s['voltage']:.2f} V (ADC A0: {s['a0_voltage']:.2f} V, Rohwert: {s['raw_adc']})")
            print(f"Ladezustand: {bar} {s['percentage']:.1f} %")
            print(f"Status:      {s['status']}")
            if s["warning"]:
                print(">>> WARNUNG: Akkustand unter 20%! Bitte zeitnah aufladen! <<<")
            print("=" * 60)

        elif mode == "json":
            s = mon.get_status()
            print(json.dumps(s))

        elif mode == "oled":
            s = mon.get_status()
            print(f"Akku: {s['percentage']:.1f}% ({s['voltage']:.2f}V) - {s['status']}")
            run_oled_display(s)

        else:
            print("=" * 65)
            print("Adeept Robot HAT V3.2 - Live-Akkumonitor (ADS7830 ADC)")
            print(f"Intervall: {interval:.1f}s | Beenden mit Strg+C.")
            print("=" * 65)

            count = 0
            while True:
                count += 1
                s = mon.get_status()
                bar = render_battery_bar(s["percentage"])
                warn_tag = " [!] BITTE LADEN" if s["warning"] else ""
                print(f"#{count:04d} | {bar} {s['percentage']:5.1f} % | {s['voltage']:.2f} V | {s['status']}{warn_tag}", flush=True)
                time.sleep(interval)

    except KeyboardInterrupt:
        print("\nUeberwachung durch Benutzer beendet.")
    finally:
        mon.close()


if __name__ == "__main__":
    main()
