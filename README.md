# Toit driver for the Bosch Sensortec BMM350 magnetometer

A Toit driver for the BMM350, a three-axis magnetometer. It returns the magnetic
field in microtesla (µT) and the die temperature in °C. The chip supports I2C
and I3C. This driver implements I2C only.

## Use cases

The datasheet lists these typical applications:

- "Magnetic heading information"
- "Tilt-compensated electronic compass for map rotation, navigation and augmented reality"
- "Gyroscope calibration in 9-DoF applications for mobile devices"
- "In-door navigation, e.g. step counting in combination with accelerometer"
- "Gaming (AR/VR)"

Some ideas that go beyond the datasheet's list. They are suggestions, and none
has been tried with this driver:

- A compass or heading indicator. You will need a calibration for steel and
  magnets near the sensor. The driver does not apply one, and `src/calibration.toit`
  is a helper for making one.
- Sensing a magnet, for example on a lid, a door or a knob. The datasheet gives
  the field range as "±2000µT".
- A magnetic joystick or position sensor, with a magnet on the moving part.
- Logging the field strength over time to see changes in the surroundings.
- Battery projects that only need a reading now and then. The datasheet names
  "triggered operation (forced mode)", and the chip rests in suspend mode
  between readings.

## Wiring

Four wires: power, ground, SDA and SCL. The I2C address is 0x14 when the ADSEL
pin is low and 0x15 when it is high. Change the pin numbers to match your board
when using the examples.

The chip itself takes "VDD supply voltage range: 1.72V to 1.98V" and "VDDIO
interface voltage range: 1.72V to 3.6V" (datasheet). A breakout board will
probably add a regulator, which is a guess. Check what yours expects on its
power pin.

## Quick start

```
import i2c
import bmm350 show Bmm350

main:
  bus := i2c.Bus --sda=19 --scl=20
  sensor := Bmm350 (bus.device Bmm350.I2C-ADDRESS)
  print sensor.read-mag-compensated  // The field in µT.
```

The constructor resets the chip, loads the factory trim values from its OTP
memory, runs a magnetic reset, and starts normal mode at 400 Hz without
averaging. Pass `--odr` and `--avg` to start with other settings.

## Reading data

`read-raw-data` returns the bytes of one measurement: three axes, temperature
and sensortime. `read-mag-compensated`, `read-mag`, `read-temp` and
`read-sensortime` turn those bytes into numbers. Pass the same bytes to several
of them and every value comes from the same instant. Call them without an
argument and each call waits for a fresh measurement.

`read-mag-compensated` applies the offset, gain, temperature and cross-axis
corrections that Bosch stores in the chip. `read-mag` skips them and is mostly
useful for comparison. `read-temp` applies the temperature trim too.

Sensortime is a 24-bit counter with 39.0625 µs per tick, a figure taken from
Bosch's C code. The datasheet rounds it to "40us resolution". The counter wraps
around and only advances in normal mode. Use `Bmm350.sensortime-elapsed` to
subtract two readings across a wrap.

## Rate, averaging and power modes

`configure-mag --odr=... --avg=...` sets the output data rate and the number of
samples averaged per measurement. Averaging lowers noise and costs current. The
chip does not allow every pair (datasheet table 5):

- 400 Hz: no averaging
- 200 Hz: up to 2 samples
- 100 Hz: up to 4 samples
- 50 Hz and slower: up to 8 samples

The datasheet describes Bosch's own function as one that "checks for the
combination of ODR and averaging setting and chooses the highest possible
averaging setting if the desired averaging setting is too high". This driver
does the same. `get-odr` and `get-avg` return what the chip actually uses.

`set-mode` switches between `MODE-SUSPEND`, `MODE-NORMAL`, `MODE-FORCED` and
`MODE-FORCED-FAST`. The datasheet says that in normal mode "it is possible to
require only the SUS and UPD_OAE commands", so the driver passes through
suspend whenever it has to. A forced measurement ends back in suspend mode.

## Examples

All examples use the pins from the wiring section.

- `read-simple.toit` prints 100 compensated readings.
- `read-verbose.toit` prints raw and compensated readings with their strength,
  the sensortime and the temperature.
- `configure-rate.toit` asks for a rate and an averaging setting, and prints
  what the chip uses.
- `forced-mode.toit` takes one reading a second in forced mode.
- `heading.toit` turns the X and Y field into a compass angle. It assumes the
  board is level and has no calibration.
- `compass-display.toit` shows a compass dial and an X-Y scatter plot on a
  128x64 SSD1306 display. See the next section.
- `calibrate.toit` collects readings while you turn the board, then prints a
  calibration to paste into your code.

## Calibration and the display demo

The chip's own trim values do not remove what the rest of the board adds to
the field. A magnet, a steel screw or a wire with current in it shifts the
reading by an amount that turns with the board. `src/calibration.toit` has two
small classes for that. It is a separate import, and the driver does not use it
yet:

```
import bmm350.calibration show MagCalibration

CALIBRATION ::= MagCalibration
    --offset-x=12.0
    --offset-y=-30.0
    --offset-z=7.5

main:
  ...
  field := CALIBRATION.apply sensor.read-mag-compensated
```

`MagCalibrator` collects readings and works out the offsets and the axis
scales with the min/max method. It is a common technique, and it comes from
neither the datasheet nor Bosch. It handles a steady offset (hard-iron) well,
and a difference in axis sensitivity in a simple way. It does not fit a tilted
ellipsoid.

The two display examples put SDA on GPIO 21 and SCL on GPIO 20, with the
display on the same I2C bus as the sensor. They need two more packages,
`pixel_display` and `ssd1306`. Run `toit pkg install` in `examples` once. To
calibrate:

1. Mount the board where it will stay, with its wires and battery in place.
2. Run `calibrate.toit` and turn the board through every orientation for 40
   seconds. The scatter plot shows the raw X and Y readings.
3. Hold the board level with the top of the display towards north when it
   says "Point N", then a quarter turn clockwise, towards east, when it says
   "Point E". Use a compass you trust to find north, away from steel and magnets.
   This works out `ROTATE-DEGREES` and `MIRROR`.
4. Copy the lines it prints into `compass-display.toit`. The display also steps
   through the numbers after "Done", for when the board runs from a battery.
5. Run `compass-display.toit`. Turn the board flat on a table. The needle
   should keep pointing the same way in the room, and the dots should trace
   a ring.

The dial assumes that the board is level, that the chip's +X axis points to
the top of the display, and that Z points up. Those are assumptions, and the
real directions depend on your breakout. `calibrate.toit` measures
`ROTATE-DEGREES` and `MIRROR` for you from the two poses. The mirror test
rests on my assumption that a clockwise turn raises the field angle by 90
degrees when the chip's Z axis points up. If the needle still misbehaves, change
`ROTATE-DEGREES` or `MIRROR` by hand. If the picture on the screen
is turned, set `DISPLAY-ROTATION` to 90, 180 or 270 in `calibrate.toit` and
`compass-display.toit`. It turns the picture clockwise from how it looked before
the setting existed, and a tall picture (90 or 270) puts the parts of the screen
underneath each other. If the picture is a mirror image, set `DISPLAY-FLIP` to
`true`. These two settings are about the screen, and `ROTATE-DEGREES` and
`MIRROR` are about the chip.

A calibration belongs to the whole assembly, not only to the chip. Another
board built the same way will probably need its own numbers. `to-map` and
`from-map` turn a calibration into a map of numbers, which makes it easy to
keep one per device in a file or in flash.

## Tests

The tests are for working on the driver, and you do not need them to use it.
They are described in [tests/README.md](tests/README.md).

## Limits

- The datasheet gives a zero-field offset drift of "±25" µT before or after
  soldering. Compass work needs a calibration, which this driver does not apply. See
  the calibration section for a separate helper.
- Not implemented: I3C, the chip's self-test, the in-band interrupt, pad drive
  strength, the I2C watchdog and the always-on sensortime option.
- The INT pin setters are only checked for the right register bits. No test has
  watched the pin itself.

## Links

- [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf)
- [Bosch's sensor API](https://github.com/boschsensortec/BMM350_SensorAPI/blob/main/README.md) with functions and examples.

## Issues

If you find a problem or have a change to suggest, please
[raise an issue](./issues). Feedback is welcome.

## Disclaimer

- This driver was developed using the DFRobot breakout version of the chip, v1.0.
- All trademarks belong to their respective owners.
- No warranties for this work, express or implied.

## Credits

- [Florian](https://github.com/floitsch) for help and encouragement.
- The wider Toit developer team, past and present, for a truly excellent product.
- AI helped with code and text reviews, with analysing and compiling data and
  results, and with checking accuracy. Several parts of the Bosch source code
  were too hard for me to decipher and convert, and it has been a long time
  since high-school calculus.

## About Toit

You are probably here because you know what Toit is. If not:
> Toit is a high-level, memory-safe language, with container/VM technology built
> specifically for microcontrollers (not a desktop language port). It gives fast
> iteration (live reloads over Wi-Fi in seconds), robust serviceability, and
> performance that’s far closer to C than typical scripting options on the
> ESP32. [[link](https://toitlang.org/)]
- [Review on Soracom](https://soracom.io/blog/internet-of-microcontrollers-made-easy-with-toit-x-soracom/)
- [Review on eeJournal](https://www.eejournal.com/article/its-time-to-get-toit)
