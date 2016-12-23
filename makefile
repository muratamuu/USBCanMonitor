TARGET = main.hex
SRC = main.c usb_cdc.c
CC = /Applications/microchip/xc8/v1.40/bin/xc8

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(SRC) --chip=18F14K50 --CODEOFFSET=1000h --ROM=default,-0-fff

flash:
	mphidflash -write $(TARGET) -n -reset

clean:
	rm -f *.hex funclist *.cof *.hxl *.p1 *.sdb startup.* *.lst *.pre *.sym *.d *.as *.cmf *.obj

install:
	mphidflash -w $(TARGET) -n -r
