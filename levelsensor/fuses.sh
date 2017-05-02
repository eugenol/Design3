#!/bin/bash
avrdude -c usbasp -p m328p -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
#avrdude -c usbasp -p m328p  -U lfuse:w:0xc2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m


