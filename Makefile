
SDK_PATH_WINDOWS := G:/_business/_nrf_SDK/17.0.2_d674dde
SDK_PATH_POSIX := /home/xianii/SDK/nRF5_SDK_17.0.2_d674dde

ARM_GCC_PATH := C:/ARM-GCC

SDK_ROOT := $(if $(filter Windows%,$(OS)),$(SDK_PATH_WINDOWS),$(SDK_PATH_POSIX))
OUTPUT_PATH := build
OUTPUT_BIN := ./build/project_name

.PHONY: default format flash flash_softdevice erase

default:
	@echo CMAKE build
	cmake \
	-S "./" \
	-B "$(OUTPUT_PATH)" \
	-G "Ninja" \
	-DTOOLCHAIN_PREFIX="$(ARM_GCC_PATH)" \
	-DCMAKE_TOOLCHAIN_FILE="./cmake/arm-none-eabi.cmake" \
	-DNRF5_SDK_PATH="$(SDK_PATH_WINDOWS)" \
	-DNRF5_APPCONFIG_PATH="./config"
	cmake --build $(OUTPUT_PATH)

# Format
format:
	@echo Astyle source code format
	astyle --project=".astylerc" -r **.c,**.h --exclude=build -v -Q

# Flash the program
flash:
	@echo OpenOCD Flashing: $(OUTPUT_BIN)
	openocd -c 'set OUTPUT_BIN $(OUTPUT_BIN)' -f ./script/openocd.cfg

flash_erase:
	@echo OpenOCD Eraseall
	openocd -f ./script/openocd_erase.cfg

# Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_7.2.0_softdevice.hex
ifneq (,$(filter Windows%,$(OS)))
	openocd -c 'set SDK_ROOT $(SDK_PATH_WINDOWS)' -f ./script/openocd_softdevice.cfg
else
	openocd -f ./script/sd_path_posix.cfg -f ./script/openocd_softdevice.cfg
endif

# Flash the program
debug:
	@echo OpenOCD DEBUG
	openocd -f ./script/openocd_gdb.cfg

jlink_flash: default
	@echo jlink Flashing: $(OUTPUT_BIN)
	nrfjprog -f nrf52 --program $(OUTPUT_BIN) --sectorerase
	nrfjprog -f nrf52 --reset

jlink_flash_softdevice:
	@echo Flashing: s132_nrf52_7.2.0_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_7.2.0_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

jlink_erase:
	nrfjprog -f nrf52 --eraseall
