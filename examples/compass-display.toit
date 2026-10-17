// Copyright (C) 2026 Toitware ApS.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

// A compass dial and an X-Y scatter plot on a 128x64 SSD1306 display.
//
// The sensor and the display share one I2C bus: SDA on GPIO 21, SCL on GPIO 20.
// Hold the board flat and turn it. The needle should keep pointing the same
// way in the room while the board turns under it, and the dots on the right
// should trace a ring.
//
// Run calibrate.toit first to get numbers for CALIBRATION. Without them, the
// needle will be off by an amount that depends on the board.

import math
import bmm350 show Bmm350
import bmm350.calibration show MagCalibration
import pixel-display show *
import pixel-display.two-color show BLACK

import .display-views

// Paste the line that calibrate.toit prints here.
CALIBRATION ::= MagCalibration.NONE

// Turns the dial clockwise by this many degrees, to match how the chip sits on
// your breakout. Try 90, 180 or 270 if the needle moves the wrong way.
ROTATE-DEGREES ::= 0

// The edge of the scatter plot, in microtesla from zero. A choice made for this
// example, wide enough to hold the Earth's field plus a modest offset.
SCATTER-RANGE ::= 80

main:
  bus := open-bus
  if not (bus.test Bmm350.I2C-ADDRESS):
    print "No BMM350 found on the bus. Check the wiring and the I2C address."
    return
  display := open-display bus

  sensor := Bmm350 (bus.device Bmm350.I2C-ADDRESS)
      --odr=Bmm350.ODR-50HZ
      --avg=Bmm350.AVG-4-SAMPLES

  compass := CompassView --x=0 --y=0 --size=64
  scatter := ScatterView --x=64 --y=0 --width=64 --height=48 --range=SCATTER-RANGE
  readout := Label --x=66 --y=61 --text="" --style=text-style
  display.add
      Div --x=0 --y=0 --w=DISPLAY-WIDTH --h=DISPLAY-HEIGHT --background=BLACK [
          compass,
          scatter,
          readout,
      ]

  last-draw := Time.monotonic-us
  last-print := last-draw
  while true:
    field := CALIBRATION.apply sensor.read-mag-compensated
    compass.update field.x field.y --rotate-degrees=ROTATE-DEGREES
    scatter.add field.x field.y

    heading := (math.atan2 field.y field.x) * 180.0 / math.PI
    if heading < 0.0: heading += 360.0
    strength := math.sqrt (field.x * field.x + field.y * field.y + field.z * field.z)

    now := Time.monotonic-us
    if now - last-draw >= 100_000:
      readout.text = "$(%3d heading.round) $(%3d strength.round)uT"
      display.draw
      last-draw = now
    if now - last-print >= 1_000_000:
      print "Heading $(%d heading.round) degrees, strength $(%.1f strength) uT"
      last-print = now
