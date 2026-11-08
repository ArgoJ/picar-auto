#!/usr/bin/env python3
# File name   : oled_test.py
# Description : Testskript fuer das 0.96 Zoll SSD1306 I2C OLED-Display (128x64)
#               am Adeept Robot HAT V3.2 (IIC Port X2, Adresse 0x3C)
#               Gemaess Adeept Lesson 14: OLED Display
import datetime
import random
import socket
import sys
import time
import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont

# Display-Spezifikation
OLED_WIDTH = 128
OLED_HEIGHT = 64
I2C_ADDRESS = 0x3C


def init_display(retries=3):
    """Initialisiert die I2C-Verbindung und das SSD1306 OLED mit automatischer Wiederholung."""
    for attempt in range(1, retries + 1):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            oled = adafruit_ssd1306.SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c, addr=I2C_ADDRESS)

            # Robuster Wrapper fuer oled.show: faengt sporadische I2C Remote I/O Fehler ab
            orig_show = oled.show

            def safe_show():
                for retry in range(3):
                    try:
                        orig_show()
                        return
                    except OSError:
                        time.sleep(0.01)
                try:
                    orig_show()
                except OSError:
                    pass

            oled.show = safe_show

            oled.fill(0)
            oled.show()
            return oled
        except Exception as e:
            if attempt == retries:
                print(f"Fehler bei der Initialisierung des OLED-Displays (Adresse {hex(I2C_ADDRESS)}): {e}", flush=True)
                print("Hinweis: Pruefen Sie das 4-Pin-Kabel am Port 'IIC' (X2) des Adeept Robot HAT V3.2.", flush=True)
                sys.exit(1)
            time.sleep(0.3)


def get_ip_address():
    """Ermittelt die aktuelle IP-Adresse des Roboters im Netzwerk."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def get_cpu_temp():
    """Liest die aktuelle CPU-Temperatur des Raspberry Pi aus."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return float(f.read().strip()) / 1000.0
    except Exception:
        return 0.0


def get_battery_info():
    """Liest Batteriespannung und Prozentstand ueber ADS7830 (A0, 0x48) aus."""
    try:
        import smbus
        bus = smbus.SMBus(1)
        raw = bus.read_byte_data(0x48, 0x84)
        bus.close()
        voltage = ((raw / 255.0) * 5.0) / 0.25
        pct = max(0.0, min(100.0, ((voltage - 6.75) / (8.4 - 6.75)) * 100.0))
        return voltage, pct
    except Exception:
        return None, None


# ==============================================================================
# Demo 1: System Info & Uhrzeit (Lesson 14 TimeOled.py)
# ==============================================================================

def demo_system_info(oled, duration_sec=None):
    """Zeigt Uhrzeit, Hostname, IP-Adresse, CPU und Akku in Echtzeit an."""
    print("\n[Demo: System-Info & Uhrzeit]")
    print("Aktualisiert jede Sekunde. Beenden mit Strg+C.")

    font = ImageFont.load_default()
    hostname = socket.gethostname()
    t_end = time.time() + duration_sec if duration_sec else None

    try:
        while True:
            if t_end and time.time() > t_end:
                break

            now = datetime.datetime.now()
            time_str = now.strftime("%H:%M:%S")
            date_str = now.strftime("%Y-%m-%d")
            ip_str = get_ip_address()
            temp_c = get_cpu_temp()
            v_bat, pct_bat = get_battery_info()

            img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
            draw = ImageDraw.Draw(img)

            # Kopfzeile mit Rahmen
            draw.rectangle((0, 0, OLED_WIDTH - 1, 14), outline=255, fill=255)
            draw.text((4, 1), f"PiCar-Pro | {hostname}", font=font, fill=0)

            # Wertezeilen
            draw.text((2, 18), f"Zeit: {date_str} {time_str}", font=font, fill=255)
            draw.text((2, 32), f"IP:   {ip_str}", font=font, fill=255)

            if v_bat is not None:
                draw.text((2, 46), f"CPU: {temp_c:.0f}C | Bat: {pct_bat:.0f}%", font=font, fill=255)
            else:
                draw.text((2, 46), f"CPU: {temp_c:.1f} C", font=font, fill=255)

            # Kleiner Fortschrittsbalken am unteren Rand
            sec_progress = int((now.second / 59.0) * (OLED_WIDTH - 4))
            draw.line((2, 60, 2 + sec_progress, 60), fill=255)

            oled.image(img)
            oled.show()
            time.sleep(1.0)

    except KeyboardInterrupt:
        pass


# ==============================================================================
# Demo 2: Schneefall-Animation (Lesson 14 SnowOled.py)
# ==============================================================================

class Snowflake:
    def __init__(self, x, y, speed):
        self.x = x
        self.y = y
        self.speed = speed

    def fall(self):
        self.y += self.speed
        if self.y > OLED_HEIGHT - 1:
            self.y = 0
            self.x = random.randint(0, OLED_WIDTH - 1)

    def draw(self, draw):
        draw.point((int(self.x), int(self.y)), fill=255)


class Star:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.brightness = random.choice([0, 255])

    def twinkle(self):
        if random.random() < 0.15:
            self.brightness = 255 if self.brightness == 0 else 0

    def draw(self, draw):
        if self.brightness:
            draw.point((self.x, self.y), fill=255)


def demo_snow_animation(oled, duration_sec=10.0):
    """Schneefall und funkelnde Sterne gemaess Lesson 14."""
    print(f"\n[Demo: Schneefall-Animation ({duration_sec}s)]")
    font = ImageFont.load_default()

    snowflakes = [
        Snowflake(random.randint(0, OLED_WIDTH - 1), random.randint(0, OLED_HEIGHT - 1), random.uniform(1.0, 2.5))
        for _ in range(25)
    ]
    stars = [
        Star(random.randint(0, OLED_WIDTH - 1), random.randint(0, OLED_HEIGHT - 1))
        for _ in range(15)
    ]

    t_end = time.time() + duration_sec if duration_sec else None

    try:
        while True:
            if t_end and time.time() > t_end:
                break

            img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
            draw = ImageDraw.Draw(img)

            # Sterne zeichnen
            for star in stars:
                star.twinkle()
                star.draw(draw)

            # Schneeflocken bewegen & zeichnen
            for flake in snowflakes:
                flake.fall()
                flake.draw(draw)

            # Kleines Banner in der Mitte
            draw.rectangle((24, 24, 104, 38), outline=255, fill=0)
            draw.text((28, 26), "* Adeept PiCar *", font=font, fill=255)

            oled.image(img)
            oled.show()
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass


# ==============================================================================
# Demo 3: Robot Face / Animierte Roboteraugen
# ==============================================================================

def demo_robot_eyes(oled, cycles=5):
    """Animierte Augen fuer das Roboter-Gesicht auf dem PiCar."""
    print(f"\n[Demo: Roboter-Augen ({cycles} Zyklen)]", flush=True)

    eye_w, eye_h = 24, 32
    left_x, right_x = 24, 80
    center_y = 16

    try:
        for _ in range(cycles):
            # Geradeaus schauen
            for _ in range(15):
                img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
                draw = ImageDraw.Draw(img)
                draw.rounded_rectangle((left_x, center_y, left_x + eye_w, center_y + eye_h), radius=6, fill=255)
                draw.rounded_rectangle((right_x, center_y, right_x + eye_w, center_y + eye_h), radius=6, fill=255)
                oled.image(img)
                oled.show()
                time.sleep(0.05)

            # Blinzeln (Augen schliessen)
            for h in [20, 8, 2, 8, 20]:
                img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
                draw = ImageDraw.Draw(img)
                y_off = center_y + (eye_h - h) // 2
                draw.rounded_rectangle((left_x, y_off, left_x + eye_w, y_off + h), radius=2, fill=255)
                draw.rounded_rectangle((right_x, y_off, right_x + eye_w, y_off + h), radius=2, fill=255)
                oled.image(img)
                oled.show()
                time.sleep(0.04)

            # Nach links schauen
            img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
            draw = ImageDraw.Draw(img)
            draw.rounded_rectangle((left_x - 10, center_y, left_x + eye_w - 10, center_y + eye_h), radius=6, fill=255)
            draw.rounded_rectangle((right_x - 10, center_y, right_x + eye_w - 10, center_y + eye_h), radius=6, fill=255)
            oled.image(img)
            oled.show()
            time.sleep(0.8)

            # Nach rechts schauen
            img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
            draw = ImageDraw.Draw(img)
            draw.rounded_rectangle((left_x + 10, center_y, left_x + eye_w + 10, center_y + eye_h), radius=6, fill=255)
            draw.rounded_rectangle((right_x + 10, center_y, right_x + eye_w + 10, center_y + eye_h), radius=6, fill=255)
            oled.image(img)
            oled.show()
            time.sleep(0.8)

    except KeyboardInterrupt:
        pass


# ==============================================================================
# Demo 4: Geometrische Formen & Grafiktest
# ==============================================================================

def demo_shapes(oled, hold_sec=4.0):
    """Testet geometrische Grundformen, Kreise und Linien."""
    print(f"\n[Demo: Grafik & Geometrie ({hold_sec}s)]")
    img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    # Äußerer Rahmen
    draw.rectangle((0, 0, OLED_WIDTH - 1, OLED_HEIGHT - 1), outline=255)

    # Diagonale Linien
    draw.line((0, 0, OLED_WIDTH - 1, OLED_HEIGHT - 1), fill=255)
    draw.line((0, OLED_HEIGHT - 1, OLED_WIDTH - 1, 0), fill=255)

    # Konzentrische Kreise in der Mitte
    draw.ellipse((44, 12, 84, 52), outline=255, fill=0)
    draw.ellipse((52, 20, 76, 44), outline=255, fill=255)
    draw.ellipse((60, 28, 68, 36), fill=0)

    # Eck-Beschriftung
    draw.text((4, 4), "0,0", font=font, fill=255)
    draw.text((80, 50), "128x64", font=font, fill=255)

    oled.image(img)
    oled.show()
    time.sleep(hold_sec)


# ==============================================================================
# Benutzerdefinierter Text
# ==============================================================================

def display_custom_text(oled, text):
    """Gibt einen beliebigen benutzerdefinierten Text mit automatischem Zeilenumbruch aus."""
    img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    draw.rectangle((0, 0, OLED_WIDTH - 1, OLED_HEIGHT - 1), outline=255)

    lines = []
    current_line = ""
    for word in text.split():
        test_line = (current_line + " " + word).strip()
        bbox = draw.textbbox((0, 0), test_line, font=font)
        if (bbox[2] - bbox[0]) <= (OLED_WIDTH - 12):
            current_line = test_line
        else:
            if current_line:
                lines.append(current_line)
            current_line = word
    if current_line:
        lines.append(current_line)

    y = 6
    for line in lines:
        draw.text((6, y), line, font=font, fill=255)
        bbox = draw.textbbox((0, 0), line, font=font)
        y += (bbox[3] - bbox[1]) + 4

    oled.image(img)
    oled.show()
    print(f"Text auf OLED angezeigt: '{text}'")


# ==============================================================================
# Hauptprogramm
# ==============================================================================

def main():
    mode = "all"
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()

    print("=" * 65, flush=True)
    print("Adeept Robot HAT V3.2 - SSD1306 OLED Display Test (IIC Port X2)", flush=True)
    print(f"Aufloesung: {OLED_WIDTH}x{OLED_HEIGHT} | I2C-Adresse: {hex(I2C_ADDRESS)}", flush=True)
    print(f"Modus: '{mode}' | Beenden mit Strg+C.", flush=True)
    print("=" * 65, flush=True)

    oled = init_display()

    try:
        if mode in ("time", "info", "sysinfo"):
            demo_system_info(oled)

        elif mode in ("snow", "stars"):
            demo_snow_animation(oled, duration_sec=None)

        elif mode in ("eyes", "face"):
            demo_robot_eyes(oled, cycles=20)

        elif mode in ("shapes", "graphic", "graphics"):
            demo_shapes(oled, hold_sec=10.0)

        elif mode in ("text", "msg") and len(sys.argv) > 2:
            custom_msg = " ".join(sys.argv[2:])
            display_custom_text(oled, custom_msg)
            print("Druecken Sie Strg+C zum Loeschen.")
            while True:
                time.sleep(1)

        elif mode in ("off", "clear"):
            print("Schalte Display aus...")
            oled.fill(0)
            oled.show()
            print("Display ausgeschaltet.")

        else:
            # Standard: Alle Demos nacheinander praesentieren
            print("\nStarte Show-Modus mit allen Effekten...")
            demo_shapes(oled, hold_sec=3.0)
            demo_robot_eyes(oled, cycles=3)
            demo_snow_animation(oled, duration_sec=6.0)
            print("\nWechsle zu Live-Systemanzeige (5 Sekunden)...")
            demo_system_info(oled, duration_sec=5.0)

            print("\nAlle Demos abgeschlossen. Schalte Display aus.")

    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer (Strg+C).", flush=True)
    finally:
        oled.fill(0)
        oled.show()
        print("Display gereinigt und zurueckgesetzt.", flush=True)


if __name__ == "__main__":
    main()
