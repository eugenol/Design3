#!/bin/bash
avrdude -c usbasp -p m328p -u -U flash:w:levelsensor.hex
