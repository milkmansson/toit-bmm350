// Copyright (C) 2026 Toitware ApS.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

// Works out a calibration for the board and prints it as a line of Toit.
//
// The sensor and the display share one I2C bus: SDA on GPIO 21, SCL on GPIO 20.
//
// Mount the board where it will live, with its wires and battery in place, and
// run this. For the collection time, turn the board slowly through every
// orientation, like tracing a figure of eight in the air. The scatter plot on
// the right shows the raw X and Y readings. Then the example shows the
// corrected readings for ten seconds so you can see whether the ring sits on
// the cross.
//
// Copy the line it prints into compass-display.toit.

import bmm350 show Bmm350
import bmm350.calibration show MagCalibration MagCalibrator
import pixel-display show *
import pixel-display.two-color show BLACK

import .display-views

// How long to collect for, in seconds. A choice made for this example.
COLLECT-SECONDS ::= 40

// How long to show the corrected readings afterwards, in seconds.
CHECK-SECONDS ::= 10

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

  scatter := ScatterView --x=0 --y=0 --width=128 --height=48 --range=SCATTER-RANGE --keep=100
  readout := Label --x=2 --y=61 --text="Turn the board" --style=text-style
  display.add
      Div --x=0 --y=0 --w=DISPLAY-WIDTH --h=DISPLAY-HEIGHT --background=BLACK [
          scatter,
          readout,
      ]

  calibrator := MagCalibrator
  print "Collecting for $COLLECT-SECONDS seconds. Turn the board through every orientation."
  start := Time.monotonic-us
  last-draw := start
  while (Time.monotonic-us - start) < COLLECT-SECONDS * 1_000_000:
    field := sensor.read-mag-compensated
    calibrator.add field
    scatter.add field.x field.y
    now := Time.monotonic-us
    if now - last-draw >= 100_000:
      left := COLLECT-SECONDS - (now - start) / 1_000_000
      readout.text = "$(%3d left)s  $(%3d (calibrator.progress * 100).round)%"
      display.draw
      last-draw = now

  print "Spans: X $(%.1f calibrator.span-x) uT, Y $(%.1f calibrator.span-y) uT, Z $(%.1f calibrator.span-z) uT"
  if not calibrator.is-ready:
    print "Not every axis covered enough range. Turn the board further and run it again."
    readout.text = "Too little turn"
    display.draw
    return

  calibration := calibrator.calibration
  print "Calibration: $calibration"
  print "Paste this into compass-display.toit:"
  print "CALIBRATION ::= $calibration.to-source"

  scatter.clear
  readout.text = "Corrected"
  end := Time.monotonic-us + CHECK-SECONDS * 1_000_000
  while Time.monotonic-us < end:
    field := calibration.apply sensor.read-mag-compensated
    scatter.add field.x field.y
    now := Time.monotonic-us
    if now - last-draw >= 100_000:
      display.draw
      last-draw = now
  readout.text = "Done"
  display.draw
