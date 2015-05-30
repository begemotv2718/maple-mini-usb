.PHONY: all firmware libopencm3

all: libopencm3 firmware

libopencm3:
	$(MAKE) -C libopencm3

firmware:
	$(MAKE) -C src/ usbhid.bin	
