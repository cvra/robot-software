.PHONY: flash
flash: all
	openocd -f openocd.cfg -c "program build/motor-control-firmware.elf verify reset"

.PHONY: r
r: reset
.PHONY: reset
reset:
	openocd -f openocd.cfg -c "init" -c "reset" -c "shutdown"
