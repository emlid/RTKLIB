DESTDIR	= /usr/local

APPS = rtkrcv rnx2rtkp str2str convbin pos2kml
# source files

SRC_DIR_1	 = src
SRC_DIR_2	 = src/rcv
SRC_DIR_APPS = $(addprefix app/,$(APPS))
SRC_DIRS	 = $(SRC_DIR_1) $(SRC_DIR_2) $(SRC_DIR_APPS)
IERS_DIR	 = lib/iers
vpath %.c $(SRC_DIRS)

SRC_NAMES_1  = $(notdir $(wildcard $(SRC_DIR_1)/*.c))
SRC_NAMES_2  = $(notdir $(wildcard $(SRC_DIR_2)/*.c))
SRC_NAMES_RTKRCV = rtkrcv.c vt.c
SRC_NAMES_RNX2RTKP = rnx2rtkp.c
SRC_NAMES_POS2KML = pos2kml.c
SRC_NAMES_CONVBIN = convbin.c
SRC_NAMES_STR2STR = str2str.c
SRC_NAMES    = $(SRC_NAMES_1) $(SRC_NAMES_2)

# object files
OBJ_NAMES =$(patsubst %.c,%.o,$(SRC_NAMES))

# common compile options
INCLUDEDIR := -I$(SRC_DIR_1)
OPTIONS	   = -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=3 -DSVR_REUSEADDR
CFLAGS_CMN = -std=c99 -pedantic -Wall -Werror -fno-strict-overflow -Wno-error=unused-but-set-variable \
					-Wno-error=unused-function -Wno-error=unused-result $(INCLUDEDIR) $(OPTIONS)
LDLIBS	   = lib/iers/gcc/iers.a -lm -lrt -lpthread
TARGET_LIB = librtk.a

# target-specific options
REL_OPTS    = -O3 -DNDEBUG
PREREL_OPTS = -O3
DBG_OPTS    = -O0 -g

####################################################################
##### release / prerelease / debug targets

.DEFAULT_GOAL = all
.PHONY: all release prerelease debug mkdir install clean IERS deps \
		qt_apps qt_appimages qmake qt_debs make_qt clean_qt

all: release

REL_DIR    = build/release
PREREL_DIR = build/prerelease
DBG_DIR    = build/debug

# default dirs
BUILD_DIR = $(REL_DIR)
DEPDIR = build/release/.d

ifneq "$(findstring release, $(MAKECMDGOALS))" ""
 DEPDIR = build/release/.d
 BUILD_DIR=$(REL_DIR)
endif

ifneq "$(findstring prerelease, $(MAKECMDGOALS))" ""
 DEPDIR = build/prerelease/.d
 BUILD_DIR=$(PREREL_DIR)
endif

ifneq "$(findstring debug, $(MAKECMDGOALS))" ""
 DEPDIR = build/debug/.d
 BUILD_DIR=$(DBG_DIR)
endif

deps:
	 mkdir -p $(DEPDIR)
deps: DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td
deps: POSTCOMPILE = @mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

LIB = $(addprefix $(BUILD_DIR)/,$(TARGET_LIB))
OBJS = $(addprefix $(BUILD_DIR)/src/,$(OBJ_NAMES))

release: deps
release: IERS
release: CFLAGS  = $(CFLAGS_CMN) $(REL_OPTS)
release: mkdir
release: $(LIB) | $(APPS)

prerelease: deps
prerelease: IERS
prerelease: CFLAGS  = $(CFLAGS_CMN) $(PREREL_OPTS)
prerelease: mkdir
prerelease: $(LIB) | $(APPS)

debug: deps
debug: IERS
debug: CFLAGS  = $(CFLAGS_CMN) $(DBG_OPTS)
debug: mkdir
debug: $(LIB) | $(APPS)

####################################################################
IERS:
	@$(MAKE) -C $(IERS_DIR)/gcc
# release lib
$(LIB):  $(OBJS)
	ar rcs $@ $^

$(BUILD_DIR)/src/%.o: %.c  $(DEPDIR)/%.d
	$(CC) $(DEPFLAGS) -c $(CFLAGS) $< -o $@
	$(POSTCOMPILE)

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d
####################################################################

# apps
rtkrcv: $(addprefix $(BUILD_DIR)/app/, $(SRC_NAMES_RTKRCV:%.c=%.o)) | $(LIB)
	$(CC) $^ -o $(BUILD_DIR)/$@ -L$(BUILD_DIR) -lrtk $(LDLIBS)

rnx2rtkp: $(addprefix $(BUILD_DIR)/app/, $(SRC_NAMES_RNX2RTKP:%.c=%.o)) | $(LIB)
	$(CC) $^ -o $(BUILD_DIR)/$@  -L$(BUILD_DIR) -lrtk $(LDLIBS)

pos2kml: $(addprefix $(BUILD_DIR)/app/, $(SRC_NAMES_POS2KML:%.c=%.o))  | $(LIB)
	$(CC) $^ -o $(BUILD_DIR)/$@  -L$(BUILD_DIR)  -lrtk $(LDLIBS)

convbin: $(addprefix $(BUILD_DIR)/app/, $(SRC_NAMES_CONVBIN:%.c=%.o)) | $(LIB)
	$(CC) $^ -o $(BUILD_DIR)/$@  -L$(BUILD_DIR)  -lrtk $(LDLIBS)

str2str: $(addprefix $(BUILD_DIR)/app/, $(SRC_NAMES_STR2STR:%.c=%.o)) | $(LIB)
	$(CC) $^ -o $(BUILD_DIR)/$@  -L$(BUILD_DIR)  -lrtk $(LDLIBS)

$(BUILD_DIR)/app/%.o: %.c $(DEPDIR)/%.d
	$(CC) $(DEPFLAGS) -c $(CFLAGS) $< -o $@
	$(POSTCOMPILE)

####################################################################

mkdir:
	mkdir -p $(addsuffix /src, $(BUILD_DIR)) \
			  $(addsuffix /app, $(BUILD_DIR))
install:
	cp $(LIB) $(addprefix $(DESTDIR), /lib)
	cp $(addprefix $(BUILD_DIR)/, $(APPS)) $(addprefix $(DESTDIR), /bin)


clean:
	rm -rf build/
	@$(MAKE) -C $(IERS_DIR)/gcc clean

include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRC_NAMES))))

####################################################################
LINUX_DEPLOY_QT = linuxdeployqt-continuous-x86_64.AppImage
qt_apps: qmake make_qt

qmake:
	qmake RTKLib.pro -spec linux-g++ -o QtMakefile

make_qt: qmake
	make -f QtMakefile -j `nproc`

clean_qt:
	make -f QtMakefile clean -j `nproc`
	rm QtMakefile
	rm -rf build/Qt

qt_appimages: qt_apps
	./util/build_scripts/AppImageDeploy.sh $(LINUX_DEPLOY_QT)

qt_debs: qt_apps
	./util/build_scripts/debdeploy.sh