# `crossbeam-power-backpack`

This is part of the Tiliqua crossbeam display.

Unfortunately, the DVI/MIPI bridge IC that comes with the round panel will often
fail to lock on a new video signal if it disappears momentarily (this happens whenever
the Tiliqua switches bitstreams!). The only reliable way to recover the video bridge
IC and lock back on to the video signal is to power cycle it.

Fortunately, the display draws much less current when it is not locked, so it is easy
to tell from the current consumption whether the display needs a power cycle.

This is a simple PCBA designed to:
- Take +12V from a eurorack 10-pin power cable
- Regulate it down to +5V for a USB-powered display controller
- Monitor the +5V current consumption, and if it is too low for too long, power cycle the +5V line to reset the display.

At its core this board is a buck converter and a current shunt with a switch, both monitored by an RP2040.

The KiCAD design files and (rust) firmware for the RP2040 can be found in this repository.
