# Tests for the BMM350 driver

These tests are for people working on the driver. You do not need them to use
it. There are three kinds: tests that run without any hardware, a test that runs
on a board with the chip attached, and tests for the calibration maths.

The files in this folder:

- `bmm350-mock-test.toit` runs the driver against a simulated chip.
- `mock-bmm350.toit` is the simulated chip.
- `golden-vectors.toit` holds numbers from Bosch's own sensor API, and
  `golden/bmm350-golden.c` is the program that makes them.
- `calibration-test.toit` checks the calibration maths.
- `bmm350-hardware-test.toit` runs on a real board.
- `test-helpers.toit` is a small test runner shared by the mock and hardware
  tests.

Run everything from this folder. The first time, install the packages:

```
toit pkg install
```

## Without hardware

```
toit bmm350-mock-test.toit
```

The mock test runs against a simulated chip in `mock-bmm350.toit`. The
simulation returns the two dummy bytes on every read, refuses power commands
that the datasheet says are illegal, applies rate changes only after the update
command, and wraps sensortime at 24 bits.

The compensation checks compare the driver with numbers from Bosch's own
sensor API. `golden/bmm350-golden.c` feeds made-up trim words and raw
bytes into Bosch's code, and `golden-vectors.toit` holds what came back.
To regenerate the file, put Bosch's API files in `.reference/` at the root of
the package and run:

```
cd golden
gcc -std=gnu99 -I../../.reference -o bmm350-golden bmm350-golden.c -lm
./bmm350-golden > ../golden-vectors.toit
```

The test exits with a non-zero code when a check fails, so it can run in CI.

`calibration-test.toit` checks the calibration maths without any
hardware. It builds readings from a sphere with a known offset and known axis
sensitivities, and checks that the calibrator finds them again.

## On the board

```
jag run bmm350-hardware-test.toit
```

Keep the board still and away from motors, magnets and steel while it runs. The
test checks the chip ID, the temperature and field ranges, every rate and
averaging pair, the sensortime step at six rates, the chip clock against the
ESP32 clock, noise at 100 Hz, forced mode, axis switching, magnetic reset and
soft reset.

If the chip's INT pin is wired to the ESP32, the test also watches the pin. Set
`INT-PIN` at the top of the file to the ESP32 pin it is wired to (7 by default),
or to `null` if INT is not wired. The checks cover:

- the pin staying low while the INT output is off,
- latched mode, where the pin rises and stays high until the status is read,
- pulsed mode, where the pin must not stay high,
- active-low polarity,
- open-drain output with the ESP32's internal pull-up,
- the interrupt count over three seconds against the data rate.

Each line ends in PASS, FAIL, WARN or SKIP. A FAIL points at the driver, the
wiring or the chip. A WARN means a value looks odd without being wrong, usually
because of the surroundings. Limits from the datasheet are named in the source.
The others are choices made for this test and are marked as such at the top of
the file.

Set `ROTATE-TEST` to `true` to add a hands-on check. It asks you to turn the
board through every orientation for 20 seconds, then compares how constant the
field strength stays with and without compensation. It also prints the middle of
each axis's range, which is a starting point for your own hard-iron calibration.
