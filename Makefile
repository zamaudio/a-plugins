#!/usr/bin/make -f

all:
#	$(MAKE) -C ./a-comp
	$(MAKE) -C ./a-filter
	$(MAKE) -C ./a-delay
	$(MAKE) -C ./a-eq

clean:
#	$(MAKE) -C ./a-comp clean
	$(MAKE) -C ./a-filter clean
	$(MAKE) -C ./a-delay clean
	$(MAKE) -C ./a-eq clean

install:
#	$(MAKE) -C ./a-comp install
	$(MAKE) -C ./a-filter install
	$(MAKE) -C ./a-delay install
	$(MAKE) -C ./a-eq install

uninstall:
	$(MAKE) -C ./a-comp uninstall
	$(MAKE) -C ./a-filter uninstall
	$(MAKE) -C ./a-delay uninstall
	$(MAKE) -C ./a-eq uninstall
