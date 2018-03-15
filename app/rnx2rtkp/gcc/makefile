#
# makefile for rnx2rtkp
#

BINDIR      = /usr/local/bin

# source files
SRC_DIR_APP = ..
SRC_DIR_1   = ../../../src
SRC_DIR_2   = ../../../src/rcv
SRC_DIR_3   = ../../../src/MULTIHYPOTHESIS
SRC_DIRS   := $(SRC_DIR_APP) $(SRC_DIR_1) $(SRC_DIR_2) $(SRC_DIR_3)

vpath %.c $(SRC_DIRS)

SRC_NAMES_APP = rnx2rtkp.c
SRC_NAMES_1   = stream.c rtkcmn.c rinex.c sbas.c preceph.c rcvraw.c convrnx.c \
    rtcm.c rtcm2.c rtcm3.c rtcm3e.c pntpos.c ephemeris.c ionex.c qzslex.c \
    lambda.c geoid.c options.c ppp.c ppp_ar.c ppp_corr.c tides.c \
    postpos.c rtkpos.c erb.c solution.c
SRC_NAMES_2   = novatel.c ss2.c ublox.c crescent.c skytraq.c gw10.c javad.c nvs.c \
    binex.c rt17.c tersus.c septentrio.c cmr.c
SRC_NAMES_3   = multihypothesis.c fix_and_hold_refinement_strategy.c
SRC_NAMES    := $(SRC_NAMES_APP) $(SRC_NAMES_1) $(SRC_NAMES_2) $(SRC_NAMES_3)

# object files
OBJ_NAMES    := $(SRC_NAMES:%.c=%.o)

# common compile options
INCLUDE := -I$(SRC_DIR_1) -I$(SRC_DIR_3)
OPTIONS := -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=3 -DIERS_MODEL
CFLAGS_CMN := -ansi -pedantic -Wall -Wno-unused-but-set-variable -fno-strict-overflow \
    -Werror -Wno-error=unused-function -Wno-error=unused-variable $(INCLUDE) $(OPTIONS) -g
LDLIBS  := ../../../lib/iers/gcc/iers.a -lgfortran -lm -lrt -lpthread
EXEC    := rnx2rtkp

# target-specific options
REL_OPTS    = -O3 -DNDEBUG
PREREL_OPTS = -O3
DBG_OPTS    = -O0

####################################################################
##### release / prerelease / debug targets 

.DEFAULT_GOAL := release
.PHONY: all release prerelease debug mkdir install clean

all: release

REL_DIR    = ./release
PREREL_DIR = ./prerelease
DBG_DIR    = ./debug

REL_EXEC    := $(EXEC)
PREREL_EXEC := $(addsuffix _prerel,$(EXEC))
DBG_EXEC    := $(addsuffix _dbg,$(EXEC))

release: CFLAGS  := $(CFLAGS_CMN) $(REL_OPTS)
release: OBJ_DIR := $(REL_DIR)
release: mkdir
release: $(REL_EXEC)

prerelease: CFLAGS  := $(CFLAGS_CMN) $(PREREL_OPTS)
prerelease: OBJ_DIR := $(PREREL_DIR)
prerelease: mkdir
prerelease: $(PREREL_EXEC)

debug: CFLAGS  := $(CFLAGS_CMN) $(DBG_OPTS)
debug: OBJ_DIR := $(DBG_DIR)
debug: mkdir
debug: $(DBG_EXEC)

####################################################################

#release
$(REL_EXEC): $(addprefix $(REL_DIR)/,$(OBJ_NAMES))
	$(CC) $^ -o $@ $(LDLIBS)

$(REL_DIR)/%.o: %.c $(SRC_DIR_1)/rtklib.h $(SRC_DIR_3)/multihypothesis.h $(SRC_DIR_3)/fix_and_hold_refinement_strategy.h
	$(CC) -c $(CFLAGS) $< -o $@

#prerelease
$(PREREL_EXEC): $(addprefix $(PREREL_DIR)/,$(OBJ_NAMES))
	$(CC) $^ -o $@ $(LDLIBS)

$(PREREL_DIR)/%.o: %.c $(SRC_DIR_1)/rtklib.h $(SRC_DIR_3)/multihypothesis.h $(SRC_DIR_3)/fix_and_hold_refinement_strategy.h
	$(CC) -c $(CFLAGS) $< -o $@

# debug
$(DBG_EXEC): $(addprefix $(DBG_DIR)/,$(OBJ_NAMES))
	$(CC) $^ -o $@ $(LDLIBS) 

$(DBG_DIR)/%.o: %.c $(SRC_DIR_1)/rtklib.h $(SRC_DIR_3)/multihypothesis.h $(SRC_DIR_3)/fix_and_hold_refinement_strategy.h
	$(CC) -c $(CFLAGS) $< -o $@

####################################################################

mkdir: 
	@mkdir -p $(OBJ_DIR)

install:
	@cp $(REL_EXEC) $(BINDIR)

clean:
	@rm -f $(REL_EXEC) $(PREREL_EXEC) $(DBG_EXEC) *.o
	@rm -rf $(REL_DIR) $(PREREL_DIR) $(DBG_DIR)