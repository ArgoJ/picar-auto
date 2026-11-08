#!/usr/bin/env python3
# File name   : ultrasonic_test.py
# Description : Hochpraeziser Hardware-Test fuer den HC-SR04 Ultraschallsensor
#               am Adeept Robot HAT V3.2 (Port X9: Trig=GPIO 23, Echo=GPIO 24)
#               Direkte libgpiod Zeitmessung ohne kuenstliche Glaettungsverzoegerung.
import sys
import time
import gpiod
from gpiod.line import Direction, Value

# Hardware-Konfiguration (Port X9)
CHIP_PATH = "/dev/gpiochip0"
PIN_TRIG = 23               # GPIO 23 (GEN4)
PIN_ECHO = 24               # GPIO 24 (GEN5)
SPEED_OF_SOUND_CM_S = 34300 # Schallgeschwindigkeit in cm/s bei 20°C
MAX_DISTANCE_CM = 400.0     # Maximaler physikalischer Messbereich des HC-SR04 (4 m)
TIMEOUT_SEC = 0.030         # 30 ms entspricht ca. 5.14 Meter Reichweite


def render_distance_bar(dist_cm: float, max_display_cm: float = 150.0, bar_length: int = 25) -> str:
    """Erzeugt einen visuellen ASCII-Balken fuer das Terminal."""
    if dist_cm is None or dist_cm > max_display_cm:
        return "[" + " " * bar_length + "]"
    filled = int(round((max(0.0, dist_cm) / max_display_cm) * bar_length))
    bar = "=" * filled + " " * (bar_length - filled)
    return f"[{bar}]"


class UltrasonicSensor:
    """Direkte libgpiod-Ansteuerung des HC-SR04 Sensors."""

    def __init__(self, chip_path=CHIP_PATH, trig=PIN_TRIG, echo=PIN_ECHO):
        self.trig = trig
        self.echo = echo
        self.chip = gpiod.Chip(chip_path)
        self.req = self.chip.request_lines(
            consumer="ultrasonic_test",
            config={
                (self.trig,): gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE
                ),
                (self.echo,): gpiod.LineSettings(
                    direction=Direction.INPUT
                )
            }
        )
        # Kurze Beruhigungspause
        time.sleep(0.05)

    def measure_raw(self):
        """
        Sendet einen 15µs Trigger-Impuls und misst die Echo-Pulsdauer.
        Rueckgabe: (dist_cm, pulse_us) oder (None, status_text)
        """
        # 1. 15µs HIGH-Puls an TRIG senden
        self.req.set_value(self.trig, Value.ACTIVE)
        time.sleep(0.000015)
        self.req.set_value(self.trig, Value.INACTIVE)

        t_trigger = time.perf_counter()
        t_rise = None
        t_fall = None

        # 2. Warten bis ECHO auf HIGH geht (max. 10 ms)
        while time.perf_counter() - t_trigger < 0.010:
            if self.req.get_value(self.echo) == Value.ACTIVE:
                t_rise = time.perf_counter()
                break

        if t_rise is None:
            return None, "Kein Echo-Start"

        # 3. Warten bis ECHO wieder auf LOW faellt (max. 30 ms = ~5.1 m)
        while time.perf_counter() - t_rise < TIMEOUT_SEC:
            if self.req.get_value(self.echo) == Value.INACTIVE:
                t_fall = time.perf_counter()
                break

        if t_fall is None:
            return None, "Echo-Timeout (> 400 cm)"

        # 4. Flugzeit berechnen
        dt = t_fall - t_rise
        dist_cm = (dt * SPEED_OF_SOUND_CM_S) / 2.0
        pulse_us = dt * 1e6

        if dist_cm > MAX_DISTANCE_CM:
            return None, "Ausserhalb Messbereich (> 400 cm)"

        return dist_cm, pulse_us

    def close(self):
        try:
            self.req.release()
            self.chip.close()
        except Exception:
            pass


def main():
    interval = 0.2
    if len(sys.argv) > 1:
        try:
            interval = float(sys.argv[1])
        except ValueError:
            pass

    print("=" * 70, flush=True)
    print("Adeept Robot HAT V3.2 - HC-SR04 Ultraschallsensor (Port X9)", flush=True)
    print(f"Hardware: GPIO {PIN_TRIG} (Trig), GPIO {PIN_ECHO} (Echo)", flush=True)
    print(f"Messbereich: 2 cm bis {MAX_DISTANCE_CM:.0f} cm | Messintervall: {interval:.2f} s", flush=True)
    print("Beenden mit Strg+C.", flush=True)
    print("=" * 70, flush=True)
    print("Hinweis: Der Schallkegel betraegt ca. 15°-30°.", flush=True)
    print("Testen Sie den Sensor mit einer flachen Flaeche (z.B. Buch/Karton/Wand).", flush=True)
    print("-" * 70, flush=True)

    try:
        sensor = UltrasonicSensor()
    except Exception as e:
        print(f"Fehler beim Oeffnen von {CHIP_PATH}: {e}")
        sys.exit(1)

    count = 0
    near_count = 0

    try:
        while True:
            t0 = time.time()
            count += 1
            dist_cm, pulse_info = sensor.measure_raw()

            if dist_cm is not None:
                bar = render_distance_bar(dist_cm, max_display_cm=150.0)
                pulse_us = pulse_info

                if dist_cm < 15.0:
                    status = "HINDERNIS NAH!"
                elif dist_cm < 40.0:
                    status = "Objekt erkannt"
                    near_count += 1
                else:
                    status = "Freie Bahn"
                    near_count = 0

                print(f"#{count:04d} | {bar} {dist_cm:6.2f} cm ({pulse_us:6.0f} µs) | {status}", flush=True)

                if near_count == 8:
                    print("  --> HINWEIS: Wiederholt ~30-40 cm gemessen!", flush=True)
                    print("      Prüfen Sie, ob Roboterarm, Greifer, Scheinwerfer oder", flush=True)
                    print("      die Tischkante in den Schallkegel ragen!", flush=True)
            else:
                bar = render_distance_bar(None)
                print(f"#{count:04d} | {bar}   ---   cm (             ) | {pulse_info}", flush=True)

            elapsed = time.time() - t0
            sleep_time = max(0.04, interval - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nMessung durch Benutzer abgebrochen.")
    finally:
        sensor.close()
        print("Sensor erfolgreich geschlossen.")


if __name__ == "__main__":
    main()
