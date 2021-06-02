#-------------------------------------------------
#
# Project created by QtCreator 2016-02-09T08:58:44
#
#-------------------------------------------------

QT       -= core gui

TARGET = RTKLib
TEMPLATE = lib
CONFIG += staticlib

include(../RTKLib.pri)
INCLUDEPATH += math/ math/lsq/ extensions/glo_ifb/ extensions/tdiff_phases/

DESTDIR = $${OUT_PWD}

QMAKE_CFLAGS += -std=c99 -pedantic -Wall -Werror -fpic -fno-strict-overflow \
    -Wno-error=unused-but-set-variable -Wno-error=unused-function \
    -Wno-error=unused-result -Wno-error=pointer-to-int-cast \
    -Wno-error=unused-variable -Wno-error=int-conversion \
    -Wno-error=stringop-truncation -Wno-error=array-bounds \
    -Wno-error=format-overflow -Wno-error=stringop-overread
QMAKE_CFLAGS_DEBUG = -O0 -g
QMAKE_CFLAGS_RELEASE = -O3
QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO = -O3 -g
CONFIG -= warn_on

CONFIG(release, debug|release) {
    DEFINES += NDEBUG
}

DEFINES -= UNICODE

SOURCES += rtkcmn.c \
    convkml.c \
    convrnx.c \
    convgpx.c \
    datum.c \
    download.c \
    ephemeris.c \
    geoid.c \
    gis.c \
    ionex.c \
    lambda.c \
    options.c \
    pntpos.c \
    postpos.c \
    ppp.c \
    ppp_ar.c \
    ppp_corr.c \
    preceph.c \
    qzslex.c \
    rcvraw.c \
    rinex.c \
    rtcm.c \
    rtcm2.c \
    rtcm3.c \
    rtcm3e.c \
    rtkpos.c \
    rtksvr.c \
    sbas.c \
    solution.c \
    stream.c \
    streamsvr.c \
    tides.c \
    tle.c \
    rcv/binex.c \
    rcv/crescent.c \
    rcv/gw10.c \
    rcv/javad.c \
    rcv/novatel.c \
    rcv/nvs.c \
    rcv/rcvlex.c \
    rcv/rt17.c \
    rcv/septentrio.c \
    rcv/skytraq.c \
    rcv/ss2.c \
    rcv/ublox.c \
    rcv/cmr.c \
    rcv/tersus.c \
    erb.c \
    math/rtklib_math.c \
    math/lsq/irls.c \
    math/lsq/lsq.c \
    math/lsq/ols.c \
    math/lsq/ransac.c \
    math/lsq/robust_lsq.c \
    extensions/glo_ifb/glo_ifb.c \
    extensions/tdiff_phases/tdpd.c \

HEADERS += rtklib.h \
    math/rtklib_math.h \
    math/lsq/robust_lsq.h \
    math/lsq/ransac.h \
    math/lsq/ols.h \
    math/lsq/lsq.h \
    math/lsq/irls.h \
    extensions/glo_ifb/glo_ifb.h \
    extensions/tdiff_phases/tdpd.h \

unix {
    target.path = /usr/lib
    INSTALLS += target
}
