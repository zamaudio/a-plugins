#!/usr/bin/make -f

PREFIX ?= /usr/local
LIBDIR ?= lib
LV2DIR ?= $(PREFIX)/$(LIBDIR)/lv2

#OPTIMIZATIONS ?= -msse -msse2 -mfpmath=sse -ffast-math -fomit-frame-pointer -O3 -fno-finite-math-only
OPTIMIZATIONS ?= -ffast-math -fomit-frame-pointer -O3 -fno-finite-math-only

LDFLAGS ?= -Wl,--as-needed
CFLAGS ?= $(OPTIMIZATIONS) -Wall

###############################################################################
BUNDLE = a-comp.lv2

CFLAGS += -fPIC -DPIC

UNAME=$(shell uname)
ifeq ($(UNAME),Darwin)
  LIB_EXT=.dylib
  LDFLAGS += -dynamiclib
else
  LDFLAGS += -shared -Wl,-Bstatic -Wl,-Bdynamic
  LIB_EXT=.so
endif


ifeq ($(shell pkg-config --exists lv2 || echo no), no)
  $(error "LV2 SDK was not found")
else
  LV2FLAGS=`pkg-config --cflags --libs lv2`
endif

$(BUNDLE): manifest.ttl a-comp.ttl a-comp$(LIB_EXT)
	mkdir -p ../bin/$(BUNDLE)
	cp presets.ttl manifest.ttl a-comp.ttl a-comp$(LIB_EXT) ../bin/$(BUNDLE)

a-comp$(LIB_EXT): a-comp.c
	$(CC) -o a-comp$(LIB_EXT) \
		$(CFLAGS) \
		a-comp.c \
		$(LV2FLAGS) $(LDFLAGS)

install: $(BUNDLE)
	install -d $(DESTDIR)$(LV2DIR)/$(BUNDLE)
	install -t $(DESTDIR)$(LV2DIR)/$(BUNDLE) ../bin/$(BUNDLE)/*

uninstall:
	rm -rf $(DESTDIR)$(LV2DIR)/$(BUNDLE)

clean:
	rm -rf ../bin/$(BUNDLE) a-comp$(LIB_EXT)

.PHONY: clean install uninstall
