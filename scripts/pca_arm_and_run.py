#!/usr/bin/env python3
"""Arm the ESC and throttle it — signal never drops. PROP OFF, hand on kill.

Run this FIRST, then power the ESC, wait for the arming tone, then commands.

Calibration: this PCA9685's oscillator was MEASURED at ~28.73 MHz (nominal
25) via Arduino pulseIn, 19 Aug 2026 — commanded 1500us was leaving the pin
as 1307us, which is why no ESC ever armed. See ADR-020. Prescale derives
from the measured value, which fixes frame rate AND every pulse width.

Platform note: BUS is the I2C bus number — 1 on Raspberry Pi (header pins
3/5), was 7 on the Jetson Orin. The oscillator calibration belongs to the
PCA9685 BOARD and survives computer swaps (ADR-020/022).
"""
import time, threading
from smbus2 import SMBus

BUS, ADDR, CH = 1, 0x40, 2

# --- Board calibration (ADR-020: per-board, re-measure if board changes) ----
OSC_HZ = 28_730_000     # MEASURED. Nominal is 25_000_000.
FREQ   = 50             # target frame rate, Hz

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, ALLCALL, OUTDRV = 0x80, 0x10, 0x20, 0x01, 0x04
US_MIN, US_MAX = 1250, 1900
STOP_US = 1500          # nominal neutral — refine after the creep walk

def init(bus, f=FREQ):
    bus.write_byte_data(ADDR, MODE2, OUTDRV)
    bus.write_byte_data(ADDR, MODE1, ALLCALL); time.sleep(0.005)
    bus.write_byte_data(ADDR, MODE1, bus.read_byte_data(ADDR, MODE1) & ~SLEEP); time.sleep(0.005)
    # Prescale from the MEASURED oscillator, not the nominal 25 MHz.
    # 28.73 MHz / 50 Hz -> 139 (was 121 under the false assumption).
    pre = int(round(OSC_HZ / (4096.0 * f)) - 1)
    old = bus.read_byte_data(ADDR, MODE1)
    bus.write_byte_data(ADDR, MODE1, (old & 0x7F) | SLEEP)
    bus.write_byte_data(ADDR, PRESCALE, pre)
    bus.write_byte_data(ADDR, MODE1, old); time.sleep(0.005)
    bus.write_byte_data(ADDR, MODE1, old | RESTART | AI)
    time.sleep(0.005)
    got = bus.read_byte_data(ADDR, PRESCALE)
    assert got == pre, f"PRESCALE read-back {got} != {pre} — do not trust this board"
    print(f"PCA9685 verified: PRESCALE={got} ({f} Hz from measured {OSC_HZ/1e6:.2f} MHz)")

current_us = STOP_US
lock = threading.Lock()
bus_dead = False

def set_us(bus, us):
    us = max(US_MIN, min(US_MAX, us))
    ticks = int(4096 * us / (1_000_000.0 / FREQ)); ticks = max(0, min(4095, ticks))
    b = LED0_ON_L + 4 * CH
    bus.write_byte_data(ADDR, b, 0); bus.write_byte_data(ADDR, b + 1, 0)
    bus.write_byte_data(ADDR, b + 2, ticks & 0xFF); bus.write_byte_data(ADDR, b + 3, (ticks >> 8) & 0x0F)

with SMBus(BUS) as bus:
    init(bus)
    set_us(bus, STOP_US)   # stream neutral immediately

    running = True
    def refresh():
        # The ONLY writer after startup. If the bus dies (Errno 121), say so
        # LOUDLY and exit — a prompt that accepts commands into a dead bus
        # manufactures false data (learned the hard way, 18 Aug).
        global bus_dead
        while running:
            try:
                with lock:
                    set_us(bus, current_us)
            except OSError as e:
                bus_dead = True
                print(f"\n*** I2C BUS DEAD ({e}) — outputs frozen, commands now "
                      f"MEANINGLESS. Ctrl-C, then i2cdetect -y {BUS}. ***")
                return
            time.sleep(0.05)
    t = threading.Thread(target=refresh, daemon=True); t.start()

    print(f"Streaming STOP ({STOP_US}us) on channel {CH}, i2c bus {BUS}.")
    print(">>> NOW power the ESC. Listen for the arming tone.")

    try:
        input("Press Enter once you hear it... ")
        print("Commands: f=1600  F=1750  r=1400  s=stop  u/d=±5us  p=print  q=quit")
        while True:
            c = input("> ").strip()
            if bus_dead:
                print("Bus is dead — fix hardware, restart script."); break
            with lock:
                if c == 'q': current_us = STOP_US; break
                elif c == 's': current_us = STOP_US; print("  stop")
                elif c == 'f': current_us = 1600; print("  1600us")
                elif c == 'F': current_us = 1750; print("  1750us")
                elif c == 'r': current_us = 1400; print("  1400us")
                elif c == 'u': current_us += 5;  print(f"  {current_us}us")
                elif c == 'd': current_us -= 5;  print(f"  {current_us}us")
                elif c == 'p': print(f"  currently {current_us}us")
                else: print("  f/F/r/s/u/d/p/q")
    except KeyboardInterrupt:
        print("\n^C — stopping.")
    finally:
        running = False
        time.sleep(0.1)
        try:
            set_us(bus, STOP_US)          # direct write — don't trust the dead thread
            print("stopped (STOP streamed).")
        except OSError:
            print("stopped (bus dead — could not stream STOP; power the ESC off).")