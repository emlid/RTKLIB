#
# make all cui applications by gcc
#

BINDIR   = /usr/local/bin
IERS_DIR = ../lib/iers
DIRS     = pos2kml str2str rnx2rtkp convbin rtkrcv

.DEFAULT_GOAL := release
.PHONY: all release prerelease debug build install clean 

all:     release

release: TARGET = release
release: build

prerelease: TARGET = prerelease
prerelease: build

debug: TARGET = debug
debug: build

build:
	@$(MAKE) -C $(IERS_DIR)/gcc
	@for i in $(DIRS); do $(MAKE) $(TARGET) -C $$i/gcc; done

install:
	@echo installing...
	@for i in $(DIRS); do cp $$i/gcc/$$i $(BINDIR); done

clean:
	@echo cleaning up...
	@for i in $(DIRS) $(IERS_DIR); do $(MAKE) -C $$i/gcc clean; done

