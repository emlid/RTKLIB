DEPDIR_REL := build/release/.d
DEPDIR_PREREL := build/prerelease/.d
DEPDIR_DBG := build/debug/.d
$(shell mkdir -p $(DEPDIR_REL) $(DEPDIR_PREREL) $(DEPDIR_DBG) >/dev/null)
DEPFLAGS_REL = -MT $@ -MMD -MP -MF $(DEPDIR_REL)/$*.Td
DEPFLAGS_PREREL = -MT $@ -MMD -MP -MF $(DEPDIR_PREREL)/$*.Td
DEPFLAGS_DBG = -MT $@ -MMD -MP -MF $(DEPDIR_DBG)/$*.Td

BINDIR      = /usr/lib
CURDIR		= $(shell pwd)
APPS = rtkrcv rnx2rtkp str2str convbin pos2kml
# source files

SRC_DIR_1	 = src
SRC_DIR_2	 = src/rcv
SRC_DIR_APPS = $(addprefix app/,$(APPS))
SRC_DIRS	 = $(SRC_DIR_1) $(SRC_DIR_2) $(SRC_DIR_APPS)

vpath %.c $(SRC_DIRS)

SRC_NAMES_1  = $(notdir $(wildcard $(SRC_DIR_1)/*.c))
SRC_NAMES_2  = $(notdir $(wildcard $(SRC_DIR_2)/*.c))
SRC_NAMES_RTKRCV = rtkrcv.c vt.c
SRC_NAMES_RNX2RTKP = rnx2rtkp.c
SRC_NAMES_POS2KML = pos2kml.c
SRC_NAMES_CONVBIN = convbin.c
SRC_NAMES_STR2STR = str2str.c
SRC_NAMES_APPS = $(SRC_NAMES_RTKRCV) $(SRC_NAMES_RNX2RTKP) $(STC_NAMES_POS2KML) $(CONVBIN) $(STR2STR)
SRC_NAMES    = $(SRC_NAMES_1) $(SRC_NAMES_2) 

# object files
OBJ_NAMES =$(patsubst %.c,%.o,$(SRC_NAMES))

# common compile options
INCLUDE := -I$(SRC_DIR_1)
OPTIONS	   = -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=3 -DSVR_REUSEADDR
CFLAGS_CMN = -std=c99 -pedantic -Wall -fpic -fno-strict-overflow \
			   $(INCLUDE) $(OPTIONS) -g
LDLIBS	   = lib/iers/gcc/iers.a -lgfortran -lm -lrt -lpthread
LDFLAGS    = -shared
TARGET_LIB = librtk.so
POSTCOMPILE_REL = @mv -f $(DEPDIR_REL)/$*.Td $(DEPDIR_REL)/$*.d && touch $@
POSTCOMPILE_PREREL = @mv -f $(DEPDIR_PREREL)/$*.Td $(DEPDIR_PREREL)/$*.d && touch $@
POSTCOMPILE_DBG = @mv -f $(DEPDIR_DBG)/$*.Td $(DEPDIR_DBG)/$*.d && touch $@

# target-specific options
REL_OPTS    = -O3 -DNDEBUG 
PREREL_OPTS = -O3
DBG_OPTS    = -O0

####################################################################
##### release / prerelease / debug targets 

.DEFAULT_GOAL = release
.PHONY: all release prerelease debug mkdir install clean

all: IERS release 

REL_DIR    = build/release
PREREL_DIR = build/prerelease
DBG_DIR    = build/debug

REL_LIB    := $(addprefix $(REL_DIR)/,$(TARGET_LIB))
PREREL_LIB := $(addprefix $(PREREL_DIR)/,$(TARGET_LIB))
DBG_LIB    := $(addprefix $(DBG_DIR)/,$(TARGET_LIB))

REL_EXEC = $(addprefix $(REL_DIR)/,$(APPS))
PREREL_EXEC = $(addprefix $(PREREL_DIR)/,$(APPS))
DBG_EXEC = $(addprefix $(DBG_DIR)/,$(APPS))
release: CFLAGS  = $(CFLAGS_CMN) $(REL_OPTS)
release: mkdir
release: $(REL_LIB) 
release: $(REL_EXEC)


prerelease: CFLAGS  = $(CFLAGS_CMN) $(PREREL_OPTS)
prerelease: mkdir
prerelease: $(PREREL_LIB)
prerelease: $(PREREL_EXEC)

debug: CFLAGS  = $(CFLAGS_CMN) $(DBG_OPTS)
debug: mkdir
debug: $(DBG_LIB)
debug: $(DBG_EXEC)

####################################################################
IERS:
	@$(MAKE) -C $(IERS_DIR)/gcc
# release lib
$(REL_LIB): $(addprefix $(REL_DIR)/src/,$(OBJ_NAMES)) 
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(REL_DIR)/src/%.o: %.c  $(DEPDIR_REL)/%.d 
	$(CC) $(DEPFLAGS_REL) -c $(CFLAGS) -fpic $< -o $@
	$(POSTCOMPILE_REL)
 
# prerelease lib
$(PREREL_LIB): $(addprefix $(PREREL_DIR)/src/,$(OBJ_NAMES))
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(PREREL_DIR)/src/%.o: %.c $(DEPDIR_PREREL)/%.d $(SRC_DIR_1)/rtklib.h
	$(CC) $(DEPFLAGS_PREREL) -c $(CFLAGS) $< -o $@
	$(POSTCOMPILE_PREREL)

# debug
$(DBG_LIB): $(addprefix $(DBG_DIR)/src/,$(OBJ_NAMES))
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(DBG_DIR)/src/%.o: %.c $(DEPDIR_DBG)/%.d
	$(CC) $(DEPFLAGS_DBG) -c $(CFLAGS) $< -o $@
	$(POSTCOMPILE_DBG)

$(DEPDIR_REL)/%.d: ;
.PRECIOUS: $(DEPDIR_REL)/%.d
$(DEPDIR_PREREL)/%.d: ;
.PRECIOUS: $(DEPDIR_PREREL)/%.d
$(DEPDIR_DBG)/%.d: ;
.PRECIOUS: $(DEPDIR_DBG)/%.d
####################################################################

# apps release
$(REL_DIR)/rtkrcv: $(REL_DIR)/$(TARGET_LIB) $(addprefix $(REL_DIR)/app/, $(SRC_NAMES_RTKRCV:%.c=%.o))	
	$(CC) $^ -o $@ $(LDLIBS) $(CURDIR)/$(REL_DIR)/$(TARGET_LIB)

$(REL_DIR)/rnx2rtkp: $(REL_DIR)/$(TARGET_LIB) $(addprefix $(REL_DIR)/app/, $(SRC_NAMES_RNX2RTKP:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(REL_DIR)/$(TARGET_LIB)

$(REL_DIR)/pos2kml: $(REL_DIR)/$(TARGET_LIB) $(addprefix $(REL_DIR)/app/, $(SRC_NAMES_POS2KML:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(REL_DIR)/$(TARGET_LIB)

$(REL_DIR)/convbin: $(REL_DIR)/$(TARGET_LIB) $(addprefix $(REL_DIR)/app/, $(SRC_NAMES_CONVBIN:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(REL_DIR)/$(TARGET_LIB)

$(REL_DIR)/str2str: $(REL_DIR)/$(TARGET_LIB) $(addprefix $(REL_DIR)/app/, $(SRC_NAMES_STR2STR:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(REL_DIR)/$(TARGET_LIB)

$(REL_DIR)/app/%.o: %.c $(DEPDIR_REL)/%.d
	$(CC) $(DEPFLAGS_REL) -c $(CFLAGS) $< -o $@
	$(POSTCOMPILE_REL)

# apps prerelease
$(PREREL_DIR)/rtkrcv: $(PREREL_DIR)/$(TARGET_LIB) $(addprefix $(PREREL_DIR)/app/, $(SRC_NAMES_RTKRCV:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(PREREL_DIR)/$(TARGET_LIB)

$(PREREL_DIR)/rnx2rtkp: $(PREREL_DIR)/$(TARGET_LIB) $(addprefix $(PREREL_DIR)/app/, $(SRC_NAMES_RNX2RTKP:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(PREREL_DIR)/$(TARGET_LIB)

$(PREREL_DIR)/pos2kml: $(PREREL_DIR)/$(TARGET_LIB) $(addprefix $(PREREL_DIR)/app/, $(SRC_NAMES_POS2KML:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(PREREL_DIR)/$(TARGET_LIB)

$(PREREL_DIR)/convbin: $(PREREL_DIR)/$(TARGET_LIB) $(addprefix $(PREREL_DIR)/app/, $(SRC_NAMES_CONVBIN:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(PREREL_DIR)/$(TARGET_LIB)

$(PREREL_DIR)/str2str: $(PREREL_DIR)/$(TARGET_LIB) $(addprefix $(PREREL_DIR)/app/, $(SRC_NAMES_STR2STR:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(PREREL_DIR)/$(TARGET_LIB)

$(PREREL_DIR)/app/%.o: %.c $(DEPDIR_PREREL)/%.d
	$(CC) $(DEPFLAGS_PREREL) -c $(CFLAGS) $< -o $@
	$(POSTCOMPILE_PREREL)

# apps debug
$(DBG_DIR)/rtkrcv: $(DBG_DIR)/$(TARGET_LIB) $(addprefix $(DBG_DIR)/app/, $(SRC_NAMES_RTKRCV:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(DBG_DIR)/$(TARGET_LIB)

$(DBG_DIR)/rnx2rtkp: $(DBG_DIR)/$(TARGET_LIB) $(addprefix $(DBG_DIR)/app/, $(SRC_NAMES_RNX2RTKP:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(DBG_DIR)/$(TARGET_LIB)

$(DBG_DIR)/pos2kml: $(DBG_DIR)/$(TARGET_LIB) $(addprefix $(DBG_DIR)/app/, $(SRC_NAMES_POS2KML:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(DBG_DIR)/$(TARGET_LIB)

$(DBG_DIR)/convbin: $(DBG_DIR)/$(TARGET_LIB) $(addprefix $(DBG_DIR)/app/, $(SRC_NAMES_CONVBIN:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(DBG_DIR)/$(TARGET_LIB)

$(DBG_DIR)/str2str: $(DBG_DIR)/$(TARGET_LIB) $(addprefix $(DBG_DIR)/app/, $(SRC_NAMES_STR2STR:%.c=%.o))	
	$(CC)  $^ -o $@ $(LDLIBS) $(CURDIR)/$(DBG_DIR)/$(TARGET_LIB)

$(DBG_DIR)/app/%.o: %.c $(DEPDIR_DBG)/%.d
	$(CC) $(DEPFLAGS_DBG) -c $(CFLAGS) $< -o $@
	$(POSTCOMPILE_DBG)
####################################################################
mkdir: 
	mkdir -p $(addsuffix /src, $(REL_DIR) $(PREREL_DIR) $(DBG_DIR)) \
			  $(addsuffix /app, $(REL_DIR) $(PREREL_DIR) $(DBG_DIR)) 
install:
	cp $(REL_LIB) $(BINDIR)
	cp $(addprefix $(REL_DIR)/, $(APPS)) /usr/bin
	

clean:
	rm -f $(REL_EXEC) $(PREREL_EXEC) $(DBG_EXEC) *.o
	rm -rf $(REL_DIR) $(PREREL_DIR) $(DBG_DIR) $(DEPDIR_REL) $(DEPDIR_PREREL) $(DEPDIR_DBG)

include $(wildcard $(patsubst %,$(DEPDIR_REL)/%.d,$(basename $(SRC_NAMES))))
include $(wildcard $(patsubst %,$(DEPDIR_PREREL)/%.d,$(basename $(SRC_NAMES))))
include $(wildcard $(patsubst %,$(DEPDIR_DBG)/%.d,$(basename $(SRC_NAMES))))