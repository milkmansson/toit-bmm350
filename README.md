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

- A compass or heading indicator. You will need your own calibration for steel
  and magnets near the sensor, because the driver does not provide one.
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
import gpio
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

## Tests

### Without hardware

```
cd tests
toit pkg install
toit bmm350-mock-test.toit
```

The mock test runs against a simulated chip in `tests/mock-bmm350.toit`. The
simulation returns the two dummy bytes on every read, refuses power commands
that the datasheet says are illegal, applies rate changes only after the update
command, and wraps sensortime at 24 bits.

The compensation checks compare the driver with numbers from Bosch's own
sensor API. `tests/golden/bmm350-golden.c` feeds made-up trim words and raw
bytes into Bosch's code, and `tests/golden-vectors.toit` holds what came back.
To regenerate the file, put Bosch's API files in `.reference/` and run:

```
cd tests/golden
gcc -std=gnu99 -I../../.reference -o bmm350-golden bmm350-golden.c -lm
./bmm350-golden > ../golden-vectors.toit
```

The test exits with a non-zero code when a check fails, so it can run in CI.

### On the board

```
jag run tests/bmm350-hardware-test.toit
```

Keep the board still and away from motors, magnets and steel while it runs. The
test checks the chip ID, the temperature and field ranges, every rate and
averaging pair, the sensortime step at six rates, the chip clock against the
ESP32 clock, noise at 100 Hz, forced mode, axis switching, magnetic reset and
soft reset.

Each line ends in PASS, FAIL, WARN or SKIP. A FAIL points at the driver, the
wiring or the chip. A WARN means a value looks odd without being wrong, usually
because of the surroundings. Limits from the datasheet are named in the source.
The others are choices made for this test and are marked as such at the top of
the file.

Set `ROTATE-TEST` to `true` to add a hands-on check. It asks you to turn the
board through every orientation for 20 seconds, then compares how constant the
field strength stays with and without compensation. It also prints the middle of
each axis's range, which is a starting point for your own hard-iron calibration.

## Limits

- The datasheet gives a zero-field offset drift of "±25" µT before or after
  soldering. Compass work needs a calibration that this driver does not do.
- Not implemented: I3C, the chip's self-test, the in-band interrupt, pad drive
  strength, the I2C watchdog and the always-on sensortime option.
- The INT pin setters write the right register bits in the mock test. No test
  has watched the pin itself.

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
