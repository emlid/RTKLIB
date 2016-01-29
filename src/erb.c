/*------------------------------------------------------------------------------
* erb.c : output solution in the form of ERB protocol
*-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "rtklib.h"

/* structures for ERB protocol -----------------------------------------------*/
struct erb_ver {
    uint32_t timeGPS;
    uint8_t verH;
    uint8_t verM;
    uint8_t verL;
} __attribute__((packed));
struct erb_pos {
    uint32_t timeGPS;
    double lng;
    double lat;
    double altEl;
    double altMsl;
    uint32_t accHor;
    uint32_t accVer;
} __attribute__((packed));
struct erb_stat {
    uint32_t timeGPS;
    uint16_t weekGPS;
    uint8_t fixType;
    uint8_t fixStatus;
    uint8_t numSV;
} __attribute__((packed));
struct erb_dops {
    uint32_t timeGPS;
    uint16_t dopGeo;
    uint16_t dopPos;
    uint16_t dopVer;
    uint16_t dopHor;
} __attribute__((packed));
struct erb_vel {
    uint32_t timeGPS;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t speed;
    int32_t heading;
    uint32_t accS;
} __attribute__((packed));
struct erb_svi_head {
    uint32_t timeGPS;
    uint8_t nSV;
} __attribute__((packed));
struct erb_svi_sat {
    uint8_t idSV;
    uint8_t typeSV;
    int32_t carPh;
    int32_t psRan;
    int32_t freqD;
    uint16_t snr;
    uint16_t azim;
    uint16_t elev;
} __attribute__((packed));
/* math ----------------------------------------------------------------------*/
#define SQRT(x)    ((x) < 0.0 ? 0.0 : sqrt(x))
/* definitions for ERB protocol ----------------------------------------------*/
#define ERB_SYNC_CHAR1    0x45 /* erb message sync code 1 */
#define ERB_SYNC_CHAR2    0x52 /* erb message sync code 2 */
#define ID_VER            0x01 /* message id ERB-VER */
#define ID_POS            0x02 /* message id ERB-POS */
#define ID_STAT           0x03 /* message id ERB-STAT */
#define ID_DOPS           0x04 /* message id ERB-DOPS */
#define ID_VEL            0x05 /* message id ERB-VEL */
#define ID_SVI            0x06 /* message id ERB-SVI */
#define LENGTH_VER           7 /* length of payload ERB-VER message */
#define LENGTH_POS          44 /* length of payload ERB-POS message */
#define LENGTH_STAT          9 /* length of payload ERB-STAT message */
#define LENGTH_DOPS         12 /* length of payload ERB-DOPS message */
#define LENGTH_VEL          28 /* length of payload ERB-VEL message */
#define LENGTH_SVI_HEAD      5 /* length of head of payload ERB-SVI message */
#define LENGTH_SVI_SV       20 /* length of 1 SV information in ERB-SVI message */
#define VERSION_HIGH         0 /* High level of version */
#define VERSION_MEDIUM       1 /* Medium level of version */
#define VERSION_LOW          0 /* Low level of version */

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
    float stdX,stdY,stdZ;
    double pos[3];

    ecef2pos(sol->rr, pos);
    /* if fix or float, then get std X/Y/Z */
    stdX = (sol->stat == SOLQ_FIX || sol->stat == SOLQ_FLOAT) ? SQRT(sol->qr[0]) : 0;
    stdY = (sol->stat == SOLQ_FIX || sol->stat == SOLQ_FLOAT) ? SQRT(sol->qr[1]) : 0;
    stdZ = (sol->stat == SOLQ_FIX || sol->stat == SOLQ_FLOAT) ? SQRT(sol->qr[2]) : 0;

    position.timeGPS = time;
    position.lng = pos[1] * R2D;
    position.lat = pos[0] * R2D;
    position.altEl = pos[2];
    position.altMsl = position.altEl;
    position.accHor = 1000 * SQRT(stdX * stdX + stdY * stdY);
    position.accVer = 1000 * stdZ;
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
    dops.dopVer = 100 * sol->dop[2];
    dops.dopHor = 100 * sol->dop[3];
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
    velocity.heading = atan2(vel[1],vel[0]) * R2D * 1e5;
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
        sviSat[i].carPh = sol->carPh[i] * 1e2;
        sviSat[i].psRan = sol->psRan[i];
        sviSat[i].freqD = sol->freqD[i] * 1e3;
        sviSat[i].snr = sol->snr[i];
        sviSat[i].azim = sol->azim[i] * 1e1;
        sviSat[i].elev = sol->elev[i] * 1e1;
    }
    memcpy(payload+LENGTH_SVI_HEAD, &sviSat, nSV*LENGTH_SVI_SV);

    return (LENGTH_SVI_HEAD + nSV * LENGTH_SVI_SV);
}
/* append message ------------------------------------------------------------*/
static void appendmessage(char **p, const char mesID, const char *payload, const int length)
{
    unsigned char cka=0,ckb=0;
    *p += sprintf(*p, "%c%c", ERB_SYNC_CHAR1, ERB_SYNC_CHAR2);
    *p += sprintf(*p, "%c", mesID);
    *p += sprintf(*p, "%c%c", length, 0x00);
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
    char payload[256];
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

    time = sol->time;
    gpst = time2gpst(time, &week);

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
