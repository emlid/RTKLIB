/*------------------------------------------------------------------------------
* erb.c : output solution in the form of ERB protocol
*-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "rtklib.h"

/* math ----------------------------------------------------------------------*/
#define SQRT(x)    ((x) < 0.0 ? 0.0 : sqrt(x))

/* calculate checksum for ERB protocol ---------------------------------------*/
static void calculatesum(const char *buff, int len, unsigned char *cka,
                         unsigned char *ckb)
{
    int i;
    *cka=0;
    *ckb=0;
    for (i=2;i<len;i++) {
        *cka += buff[i];
        *ckb += *cka;
    }
}
/* build ERB-VER -------------------------------------------------------------*/
static void buildver(char *payload, struct erb_ver version, const uint32_t time,
                     const uint8_t verH, const uint8_t verM, const uint8_t verL)
{
    version.timeGPS = time;
    version.verH = verH;
    version.verM = verM;
    version.verL = verL;
    memcpy(payload, &version, LENGTH_VER);
}
/* build ERB-POS -------------------------------------------------------------*/
static void buildpos(char *payload, struct erb_pos position, const uint32_t time,
                     const sol_t *sol)
{
    float stdX2,stdY2,stdZ2;
    double pos[3];

    ecef2pos(sol->rr, pos);
    /* if position available, then get std^2 X/Y/Z */
    stdX2 = (sol->stat != SOLQ_NONE && sol->stat != SOLQ_DR && sol->qr[0] > 0) ? sol->qr[0] : 0;
    stdY2 = (sol->stat != SOLQ_NONE && sol->stat != SOLQ_DR && sol->qr[1] > 0) ? sol->qr[1] : 0;
    stdZ2 = (sol->stat != SOLQ_NONE && sol->stat != SOLQ_DR && sol->qr[2] > 0) ? sol->qr[2] : 0;

    position.timeGPS = time;
    position.lng = pos[1] * R2D;
    position.lat = pos[0] * R2D;
    position.altEl = pos[2];
    position.altMsl = pos[2] - geoidh(pos);
    position.accHor = 1000 * SQRT(stdX2 + stdY2);  /* convert float [m] to int32 [mm] */
    position.accVer = 1000 * SQRT(stdZ2);          /* convert float [m] to int32 [mm] */

    memcpy(payload, &position, LENGTH_POS);
}
/* build ERB-STAT message ----------------------------------------------------*/
static void buildstat(char *payload, struct erb_stat status, const uint32_t time,
                      const uint16_t week, const sol_t *sol)
{
    uint8_t fixStatus,fixType;

    if (sol->stat == SOLQ_SINGLE) {
        fixStatus = 0x01;
        fixType = 0x01;
    } else if (sol->stat == SOLQ_FLOAT) {
        fixStatus = 0x01;
        fixType = 0x02;
    } else if (sol->stat == SOLQ_FIX) {
        fixStatus = 0x01;
        fixType = 0x03;
    } else {
        fixStatus = 0x00;
        fixType = 0x00;
    }

    status.timeGPS = time;
    status.weekGPS = week;
    status.fixType = fixType;
    status.fixStatus = fixStatus;
    status.numSV = sol->ns;
    memcpy(payload, &status, LENGTH_STAT);
}
/* build ERB-DOPS message ----------------------------------------------------*/
static void builddops(char *payload, struct erb_dops dops, const uint32_t time,
                      const sol_t *sol)
{
    dops.timeGPS = time;
    dops.dopGeo = 100 * sol->dop[0];
    dops.dopPos = 100 * sol->dop[1];
    dops.dopHor = 100 * sol->dop[2];
    dops.dopVer = 100 * sol->dop[3];
    memcpy(payload, &dops, LENGTH_DOPS);
}
/* build ERB-VEL message -----------------------------------------------------*/
static void buildvel(char *payload, struct erb_vel velocity, const uint32_t time,
                     const sol_t *sol)
{
    double pos[3],vel[3];

    ecef2pos(sol->rr, pos);
    ecef2enu(pos, sol->rr+3, vel);

    velocity.timeGPS = time;
    velocity.velN = 100 * vel[1];
    velocity.velE = 100 * vel[0];
    velocity.velD = 100 * -vel[2];
    velocity.speed = 100 * SQRT(vel[0] * vel[0] + vel[1] * vel[1]);
    velocity.heading = atan2(vel[0],vel[1]) * R2D * 1e5; /* North = 0deg */
    velocity.accS = 0x00;
    memcpy(payload, &velocity, LENGTH_VEL);
}
/* build ERB-SVI message -----------------------------------------------------*/
static int buildsvi(char *payload, struct erb_svi_head sviHead, struct erb_svi_sat *sviSat,
                    const uint32_t time, const sol_t *sol)
{
    int i=0,nSV=sol->nSV;

    sviHead.timeGPS = time;
    sviHead.nSV = nSV;
    memcpy(payload, &sviHead, LENGTH_SVI_HEAD);

    for (i=0;i<nSV;i++) {
        sviSat[i].idSV = sol->idSV[i];
        sviSat[i].typeSV = sol->typeSV[i];
        sviSat[i].carPh = sol->carPh[i];
        sviSat[i].psRan = sol->psRan[i];
        sviSat[i].freqD = sol->freqD[i] * 1e3;
        sviSat[i].snr = sol->snr[i];
        sviSat[i].azim = sol->azim[i] * 1e1;
        sviSat[i].elev = sol->elev[i] * 1e1;
    }
    memcpy(payload+LENGTH_SVI_HEAD, sviSat, nSV*LENGTH_SVI_SV);

    return (LENGTH_SVI_HEAD + nSV * LENGTH_SVI_SV);
}
/* append message ------------------------------------------------------------*/
static void appendmessage(char **p, const char mesID, const char *payload, const int length)
{
    unsigned char cka=0,ckb=0;
    *p += sprintf(*p, "%c%c", ERB_SYNC_CHAR1, ERB_SYNC_CHAR2);
    *p += sprintf(*p, "%c", mesID);
    *p += sprintf(*p, "%c%c", (unsigned char)length, (unsigned char)(length >> 8));
    memcpy(*p, payload, length);
    *p += length;
    calculatesum(*p-(length+5), length+5, &cka, &ckb);
    *p += sprintf(*p, "%c%c", cka, ckb);
}
/* output solution in the form of ERB protocol -------------------------------*/
extern int outerb(unsigned char *buff, const sol_t *sol)
{
    gtime_t time;
    char *p=(char *)buff;
    char payload[1024];
    struct erb_ver version;
    struct erb_pos position;
    struct erb_stat status;
    struct erb_dops dops;
    struct erb_vel velocity;
    struct erb_svi_head sviHead;
    struct erb_svi_sat sviSat[32];
    uint32_t gpst;
    int week, lengthSvi;

    trace(3,"outerb:\n");

    /* convert GPS time of week (double) into milliseconds */
    time = sol->time;
    gpst = time2gpst(time, &week) * 1000;


    /*-------------- ERB-VER -----------------------*/
    buildver(payload, version, gpst, VERSION_HIGH, VERSION_MEDIUM, VERSION_LOW);
    appendmessage(&p, ID_VER, payload, LENGTH_VER);
    /*-------------- ERB-POS -----------------------*/
    buildpos(payload, position, gpst, sol);
    appendmessage(&p, ID_POS, payload, LENGTH_POS);
    /*------------ ERB-STAT ----------------------*/
    buildstat(payload, status, gpst, week, sol);
    appendmessage(&p, ID_STAT, payload, LENGTH_STAT);
    /*------------ ERB-DOPS ------------------------*/
    builddops(payload, dops, gpst, sol);
    appendmessage(&p, ID_DOPS, payload, LENGTH_DOPS);
    /*------------ ERB-VEL --------------------*/
    buildvel(payload, velocity, gpst, sol);
    appendmessage(&p, ID_VEL, payload, LENGTH_VEL);
    /*------------ ERB-SVI --------------------*/
    lengthSvi = buildsvi(payload, sviHead, sviSat, gpst, sol);
    appendmessage(&p, ID_SVI, payload, lengthSvi);

    return p - (char *)buff;
}
