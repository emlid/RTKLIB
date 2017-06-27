#
# make all cui applications by gcc
#

BINDIR = /usr/bin/RTKLIB/app
DIRS   = ../lib/iers pos2kml str2str rnx2rtkp convbin rtkrcv

all:
	@for i in $(DIRS); do $(MAKE) -C $$i/gcc; done

install:
	@echo installing...
	@for i in $(DIRS); do cp $$i/gcc/$$i $(BINDIR)/$$i/gcc/$$i; done

clean:
	@echo cleaning up...
	@for i in $(DIRS); do $(MAKE) -C $$i/gcc clean; done

