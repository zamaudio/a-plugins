#!/usr/bin/make -f

all:
	$(MAKE) -C ./a-comp
	$(MAKE) -C ./a-filter

clean:
	$(MAKE) -C ./a-comp clean
	$(MAKE) -C ./a-filter clean

install:
	$(MAKE) -C ./a-comp install
	$(MAKE) -C ./a-filter install

uninstall:
	$(MAKE) -C ./a-comp uninstall
	$(MAKE) -C ./a-filter uninstall
