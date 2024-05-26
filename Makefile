PJSIP_DIR=pjproject-2.11.1
PKG_CONFIG_PATH=pjsip.install/lib/pkgconfig

all: modem slmodemd

d-modem: d-modem.c $(PKG_CONFIG_PATH)/libpjproject.pc
	$(CC) -o $@ $< `PKG_CONFIG_PATH="$(PKG_CONFIG_PATH)" pkg-config --static --cflags --libs libpjproject`

slmodemd:
	$(MAKE) -C slmodemd

clean:
	rm -f modem.o modem
	$(MAKE) -C slmodemd clean

.PHONY: all clean realclean slmodemd
