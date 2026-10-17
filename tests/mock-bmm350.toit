// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import serial

/**
A simulated BMM350 register file, for testing the driver without hardware.

The simulation follows the datasheet and the Bosch Sensortec reference code:
- Every register read returns two dummy bytes first. They are deliberately not
  zero, so a driver that forgets to skip them fails loudly.
- In normal mode the chip only accepts the suspend and update commands. Any
  other command sets the "illegal command" flag and is ignored.
- Updates to the ODR and averaging settings only take effect after the update
  command, and the effective values appear in the status register.
- Reading the interrupt status register clears the data ready flag.
- Sensortime counts in 39.0625 microsecond ticks and wraps at 24 bits.

It does not model the analog side. Tests choose the raw field and temperature
  counts themselves.

The simulation produces samples against the real clock, so rate tests need
  real time to pass.
*/
class MockBmm350Registers extends serial.Registers:
  static DUMMY-BYTE-0 ::= 0xA5
  static DUMMY-BYTE-1 ::= 0x5A
  static TICK-US ::= 39.0625

  regs_/ByteArray := ByteArray 0x80
  otp-powered_/bool := true
  normal_/bool := false
  last-cmd_/int := 0
  illegal_/bool := false
  drdy_/bool := false
  sensortime_/int := 0
  next-sample-us_/int := 0

  /** Every register write as `[register, value]`, oldest first. */
  writes/List := []

  /** The 32 OTP words the chip serves during boot. */
  otp-words/List := List 32: 0

  /** Raw magnetic counts for the next sample (21-bit signed range). */
  raw-x/int := 0
  raw-y/int := 0
  raw-z/int := 0

  /** Raw temperature counts for the next sample (24-bit signed range). */
  raw-temp/int := 51452

  /**
  Extra bits returned in the interrupt status register, on top of data ready.
  */
  int-status-noise/int := 0

  /** Number of commands the chip rejected as illegal. */
  illegal-count/int := 0

  /** Whether data bytes sent for 21-bit values are sign-extended to 24 bits. */
  sign-extend-24/bool := true

  /** The chip ID the chip answers with after a reset. */
  chip-id/int := 0x33

  /** Whether OTP word reads should end in a "page read" error. */
  otp-fail/bool := false

  constructor:
    super
    reset_

  reset_ -> none:
    regs_.fill 0
    regs_[0x00] = chip-id
    regs_[0x04] = 0x14  // AGGR_SET reset value, datasheet 8.4.
    regs_[0x05] = 0x07  // All axes enabled.
    regs_[0x08] = 0x14
    otp-powered_ = true
    normal_ = false
    last-cmd_ = 0
    illegal_ = false
    drdy_ = false
    sensortime_ = 0

  /** Reads a register without any side effects, for assertions. */
  peek register/int -> int:
    return regs_[register]

  /** Overwrites a register directly, for setting up a test. */
  poke register/int value/int -> none:
    regs_[register] = value

  /** Whether the chip is in normal mode. */
  is-normal -> bool: return normal_

  /** The sensortime counter, in ticks. */
  sensortime -> int: return sensortime_

  set-sensortime ticks/int -> none:
    sensortime_ = ticks & 0xFFFFFF

  /** The effective output data rate in Hz, from the status register. */
  effective-odr-hz -> float:
    odr := regs_[0x08] & 0x0F
    if odr < 2 or odr > 10: return 0.0
    return 1600.0 / (1 << odr)

  /**
  Stores a sample now, as if the conversion just finished.

  Uses $raw-x, $raw-y, $raw-z and $raw-temp, and advances sensortime by
    $ticks.
  */
  inject-sample --ticks/int=0 -> none:
    sensortime_ = (sensortime_ + ticks) & 0xFFFFFF
    store-sample_

  /**
  Stores the given 15 data bytes as the current sample, then raises data
    ready if it is enabled.
  */
  inject-bytes bytes/ByteArray -> none:
    assert: bytes.size == 15
    bytes.size.repeat: regs_[0x31 + it] = bytes[it]
    if (regs_[0x2E] & 0x80) != 0: drdy_ = true

  store-sample_ -> none:
    put24_ 0x31 (fit21_ raw-x)
    put24_ 0x34 (fit21_ raw-y)
    put24_ 0x37 (fit21_ raw-z)
    put24_ 0x3A (raw-temp & 0xFFFFFF)
    put24_ 0x3D sensortime_
    if (regs_[0x2E] & 0x80) != 0: drdy_ = true

  fit21_ value/int -> int:
    if sign-extend-24: return value & 0xFFFFFF
    return value & 0x1FFFFF

  put24_ register/int value/int -> none:
    regs_[register] = value & 0xFF
    regs_[register + 1] = (value >> 8) & 0xFF
    regs_[register + 2] = (value >> 16) & 0xFF

  period-us_ -> int:
    hz := effective-odr-hz
    if hz == 0.0: return 1_000_000
    return (1_000_000.0 / hz).to-int

  /** Produces samples that are due, based on the real clock. */
  advance_ -> none:
    if not normal_: return
    now := Time.monotonic-us
    if now < next-sample-us_: return
    ticks := ((period-us_ / TICK-US) + 0.5).to-int
    sensortime_ = (sensortime_ + ticks) & 0xFFFFFF
    store-sample_
    next-sample-us_ += period-us_
    if next-sample-us_ < now: next-sample-us_ = now + period-us_

  read-bytes register/int count/int -> ByteArray:
    advance_
    result := ByteArray count
    if count > 0: result[0] = DUMMY-BYTE-0
    if count > 1: result[1] = DUMMY-BYTE-1
    (count - 2).repeat: result[2 + it] = read-register_ (register + it)
    return result

  read-register_ register/int -> int:
    if register == 0x30:
      value := int-status-noise
      if drdy_: value |= 0x04
      drdy_ = false
      return value
    if register == 0x07:
      value := (last-cmd_ & 0x07) << 5
      if illegal_: value |= 0x10
      if normal_: value |= 0x08
      return value
    return regs_[register]

  write-bytes register/int data/ByteArray -> none:
    data.size.repeat:
      writes.add [register + it, data[it]]
      write-register_ (register + it) data[it]

  write-register_ register/int value/int -> none:
    if register == 0x7E:
      if value == 0xB6: reset_
      return
    if register == 0x50:
      command := value >> 5
      if command == 0b100:
        otp-powered_ = false
      else if command == 0b001 and otp-powered_ and otp-fail:
        regs_[0x55] = 0b010_00000  // Page read error.
      else if command == 0b001 and otp-powered_:
        word := otp-words[value & 0x1F]
        regs_[0x52] = word >> 8
        regs_[0x53] = word & 0xFF
        regs_[0x55] = 0x01
      else:
        regs_[0x55] = 0b101_00000  // Invalid command error.
      return
    if register == 0x06:
      handle-pmu-command_ (value & 0x0F)
      return
    if register == 0x05 and normal_:
      return  // Axis changes are ignored in normal mode (datasheet 5.2.1).
    regs_[register] = value

  handle-pmu-command_ command/int -> none:
    illegal_ = false
    if normal_ and command != 0 and command != 2:
      illegal_ = true
      illegal-count++
      return
    last-cmd_ = command
    if command == 0:
      normal_ = false
    else if command == 1:
      normal_ = true
      next-sample-us_ = Time.monotonic-us + period-us_
    else if command == 2:
      regs_[0x08] = regs_[0x04] & 0x3F
      if normal_: next-sample-us_ = Time.monotonic-us + period-us_
    else if command == 3 or command == 4:
      store-sample_

/**
A $serial.Device wrapper around $MockBmm350Registers.
*/
class MockBmm350Device implements serial.Device:
  registers/MockBmm350Registers

  constructor:
    registers = MockBmm350Registers

  read amount/int -> ByteArray:
    throw "UNIMPLEMENTED"

  write bytes/ByteArray -> none:
    throw "UNIMPLEMENTED"
