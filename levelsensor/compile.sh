#!/bin/bash
avr-gcc -Os -mmcu=atmega328p -c -o levelsensor.o levelsensor.c
avr-gcc -mmcu=atmega328p levelsensor.o -o levelsensor
avr-objcopy -O ihex -R .eeprom levelsensor levelsensor.hex
