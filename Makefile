# Host tests use the system C compiler; firmware needs arm-none-eabi-gcc.
CC      ?= cc
ARM     ?= arm-none-eabi-
CORE     = core/foc.c core/foc_math.c core/observer.c
FW       = firmware/startup.c firmware/main.c firmware/telemetry.c

HOSTFLAGS = -O2 -std=gnu11 -Wall -Wextra -Wno-unused-parameter
ARMFLAGS  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
            -O2 -std=gnu11 -Wall -Wextra -Wno-unused-parameter \
            -ffunction-sections -fdata-sections -fno-unwind-tables -fsingle-precision-constant -fno-math-errno
LDFLAGS   = -T firmware/stm32g431cb.ld -nostartfiles --specs=nano.specs --specs=nosys.specs \
            -Wl,--gc-sections -Wl,-Map=build/foc.map

.PHONY: all test firmware plots clean

all: test firmware

build:
	mkdir -p build

build/test: tests/test_main.c tests/motor_sim.c $(CORE) | build
	$(CC) $(HOSTFLAGS) -o $@ tests/test_main.c tests/motor_sim.c $(CORE) -lm

test: build/test
	./build/test

docs/run.csv: build/test
	./build/test --csv docs/run.csv

plots: docs/run.csv
	python3 tools/plot.py docs/run.csv docs

firmware: build/foc.bin

build/foc.elf: $(FW) $(CORE) firmware/*.h core/*.h | build
	$(ARM)gcc $(ARMFLAGS) $(LDFLAGS) -o $@ $(FW) $(CORE) -lm
	$(ARM)size $@

build/foc.bin: build/foc.elf
	$(ARM)objcopy -O binary $< $@

flash: build/foc.bin
	st-flash --reset write $< 0x08000000

clean:
	rm -rf build
