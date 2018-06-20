/*------------------------------------------------------------------------------
* geoid.c : geoid models
*
*          Copyright (C) 2007-2009 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] EGM96 The NASA GSFC and NIMA Joint Geopotential Model
*     [2] Earth Gravitational Model 2008 (EGM2008)
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/07 1.0  new
*           2009/09/04 1.1  replace geoid data by global model
*           2009/12/05 1.2  added api:
*                               opengeoid(),closegeoid()
*-----------------------------------------------------------------------------*/
#include "geoid.h"
static FILE *fp_geoid=NULL;         /* geoid file pointer */
static int model_geoid=GEOID_EMBEDDED; /* geoid model */

/* bilinear interpolation ----------------------------------------------------*/
static double interpb(const double *y, double a, double b)
{
    return y[0]*(1.0-a)*(1.0-b)+y[1]*a*(1.0-b)+y[2]*(1.0-a)*b+y[3]*a*b;
}
/* embedded geoid model ------------------------------------------------------*/
static double geoidh_emb(const double *pos)
{
    const double dlon=1.0,dlat=1.0;
    double a,b,y[4];
    int i1,i2,j1,j2;
    
    if (pos[1]<range[0]||range[1]<pos[1]||pos[0]<range[2]||range[3]<pos[0]) {
        trace(2,"out of geoid model range: lat=%.3f lon=%.3f\n",pos[0],pos[1]);
        return 0.0;
    }
    a=(pos[1]-range[0])/dlon;
    b=(pos[0]-range[2])/dlat;
    i1=(int)a; a-=i1; i2=i1<360?i1+1:i1;
    j1=(int)b; b-=j1; j2=j1<180?j1+1:j1;
    y[0]=geoid[i1][j1];
    y[1]=geoid[i2][j1];
    y[2]=geoid[i1][j2];
    y[3]=geoid[i2][j2];
    return interpb(y,a,b);
}
/* get 2 byte signed integer from file ---------------------------------------*/
static short fget2b(FILE *fp, long off)
{
    unsigned char v[2];
    if (fseek(fp,off,SEEK_SET)==EOF||fread(v,2,1,fp)<1) {
        trace(2,"geoid data file range error: off=%ld\n",off);
    }
    return ((short)v[0]<<8)+v[1]; /* big-endian */
}
/* egm96 15x15" model --------------------------------------------------------*/
static double geoidh_egm96(const double *pos)
{
    const double lon0=0.0,lat0=90.0,dlon=15.0/60.0,dlat=-15.0/60.0;
    const int nlon=1440,nlat=721;
    double a,b,y[4];
    long i1,i2,j1,j2;
    
    if (!fp_geoid) return 0.0;
    
    a=(pos[1]-lon0)/dlon;
    b=(pos[0]-lat0)/dlat;
    i1=(long)a; a-=i1; i2=i1<nlon-1?i1+1:0;
    j1=(long)b; b-=j1; j2=j1<nlat-1?j1+1:j1;
    y[0]=fget2b(fp_geoid,2L*(i1+j1*nlon))*0.01;
    y[1]=fget2b(fp_geoid,2L*(i2+j1*nlon))*0.01;
    y[2]=fget2b(fp_geoid,2L*(i1+j2*nlon))*0.01;
    y[3]=fget2b(fp_geoid,2L*(i2+j2*nlon))*0.01;
    return interpb(y,a,b);
}
/* get 4byte float from file -------------------------------------------------*/
static float fget4f(FILE *fp, long off)
{
    float v=0.0;
    if (fseek(fp,off,SEEK_SET)==EOF||fread(&v,4,1,fp)<1) {
        trace(2,"geoid data file range error: off=%ld\n",off);
    }
    return v; /* small-endian */
}
/* egm2008 model -------------------------------------------------------------*/
static double geoidh_egm08(const double *pos, int model)
{
    const double lon0=0.0,lat0=90.0;
    double dlon,dlat;
    double a,b,y[4];
    long i1,i2,j1,j2;
    int nlon,nlat;
    
    if (!fp_geoid) return 0.0;
    
    if (model==GEOID_EGM2008_M25) { /* 2.5 x 2.5" grid */
        dlon= 2.5/60.0;
        dlat=-2.5/60.0;
        nlon=8640;
        nlat=4321;
    }
    else { /* 1 x 1" grid */
        dlon= 1.0/60.0;
        dlat=-1.0/60.0;
        nlon=21600;
        nlat=10801;
    }
    a=(pos[1]-lon0)/dlon;
    b=(pos[0]-lat0)/dlat;
    i1=(long)a; a-=i1; i2=i1<nlon-1?i1+1:0;
    j1=(long)b; b-=j1; j2=j1<nlat-1?j1+1:j1;
    
    /* notes: 4byte-zeros are inserted at first and last field of a record */
    /*        for current geid data files */
    /* http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm2008/egm08_wgs84.html */
    /* (1) Und_min1x1_egm2008_isw=82_WGS84_TideFree_SE.gz */
    /* (2) Und_min2.5x2.5_egm2008_isw=82_WGS84_TideFree_SE.gz */
#if 0
    /* not zero-inserted */
    y[0]=fget4f(fp_geoid,4L*(i1+j1*(nlon)));
    y[1]=fget4f(fp_geoid,4L*(i2+j1*(nlon)));
    y[2]=fget4f(fp_geoid,4L*(i1+j2*(nlon)));
    y[3]=fget4f(fp_geoid,4L*(i2+j2*(nlon)));
#else
    /* zero-inserted version (2009/12/10) */
    y[0]=fget4f(fp_geoid,4L*(i1+j1*(nlon+2)+1));
    y[1]=fget4f(fp_geoid,4L*(i2+j1*(nlon+2)+1));
    y[2]=fget4f(fp_geoid,4L*(i1+j2*(nlon+2)+1));
    y[3]=fget4f(fp_geoid,4L*(i2+j2*(nlon+2)+1));
#endif
    return interpb(y,a,b);
}
/* get gsi geoid data --------------------------------------------------------*/
static double fgetgsi(FILE *fp, int nlon, int nlat, int i, int j)
{
    const int nf=28,wf=9,nl=nf*wf+2,nr=(nlon-1)/nf+1;
    double v;
    long off=nl+j*nr*nl+i/nf*nl+i%nf*wf;
    char buff[16]="";
    
    if (fseek(fp,off,SEEK_SET)==EOF||fread(buff,wf,1,fp)<1) {
        trace(2,"out of range for gsi geoid: i=%d j=%d\n",i,j);
        return 0.0;
    }
    if (sscanf(buff,"%lf",&v)<1) {
        trace(2,"gsi geoid data format error: i=%d j=%d buff=%s\n",i,j,buff);
        return 0.0;
    }
    return v;
}
/* gsi geoid 2000 1.0x1.5" model ---------------------------------------------*/
static double geoidh_gsi(const double *pos)
{
    const double lon0=120.0,lon1=150.0,lat0=20.0,lat1=50.0;
    const double dlon=1.5/60.0,dlat=1.0/60.0;
    const int nlon=1201,nlat=1801;
    double a,b,y[4];
    int i1,i2,j1,j2;
    
    if (!fp_geoid||pos[1]<lon0||lon1<pos[1]||pos[0]<lat0||lat1<pos[0]) {
        trace(2,"out of range for gsi geoid: lat=%.3f lon=%.3f\n",pos[0],pos[1]);
        return 0.0;
    }
    a=(pos[1]-lon0)/dlon;
    b=(pos[0]-lat0)/dlat;
    i1=(int)a; a-=i1; i2=i1<nlon-1?i1+1:i1;
    j1=(int)b; b-=j1; j2=j1<nlat-1?j1+1:j1;
    y[0]=fgetgsi(fp_geoid,nlon,nlat,i1,j1);
    y[1]=fgetgsi(fp_geoid,nlon,nlat,i2,j1);
    y[2]=fgetgsi(fp_geoid,nlon,nlat,i1,j2);
    y[3]=fgetgsi(fp_geoid,nlon,nlat,i2,j2);
    if (y[0]==999.0||y[1]==999.0||y[2]==999.0||y[3]==999.0) {
        trace(2,"geoidh_gsi: data outage (lat=%.3f lon=%.3f)\n",pos[0],pos[1]);
        return 0.0;
    }
    return interpb(y,a,b);
}
/* open geoid model file -------------------------------------------------------
* open geoid model file
* args   : int    model     I   geoid model type
*                               GEOID_EMBEDDED   : embedded model(1x1deg)
*                               GEOID_EGM96_M150 : EGM96 15x15"
*                               GEOID_EGM2008_M25: EGM2008 2.5x2.5"
*                               GEOID_EGM2008_M10: EGM2008 1.0x1.0"
*                               GEOID_GSI2000_M15: GSI geoid 2000 1.0x1.5"
*          char   *file     I   geoid model file path
* return : status (1:ok,0:error)
* notes  : the following geoid models can be used
*          WW15MGH.DAC   : EGM96 15x15" binary grid height
*          Und_min2.5x2.5_egm2008_isw=82_WGS84_TideFree_SE: EGM2008 2.5x2.5"
*          Und_min1x1_egm2008_isw=82_WGS84_TideFree_SE    : EGM2008 1.0x1.0"
*          gsigeome_ver4 : GSI geoid 2000 1.0x1.5" (japanese area)
*          (byte-order of binary files must be compatible to cpu)
*-----------------------------------------------------------------------------*/
extern int opengeoid(int model, const char *file)
{
    trace(3,"opengeoid: model=%d file=%s\n",model,file);
    
    closegeoid();
    if (model==GEOID_EMBEDDED) {
        return 1;
    }
    if (model!=GEOID_EGM96_M150 &&model!=GEOID_EGM2008_M25&&
        model!=GEOID_EGM2008_M10&&model!=GEOID_GSI2000_M15) {
        trace(2,"invalid geoid model: model=%d file=%s\n",model,file);
        return 0;
    }
    if (!(fp_geoid=fopen(file,"rb"))) {
        trace(2,"geoid model file open error: model=%d file=%s\n",model,file);
        return 0;
    }
    model_geoid=model;
    return 1;
}
/* close geoid model file ------------------------------------------------------
* close geoid model file
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern void closegeoid(void)
{
    trace(3,"closegoid:\n");
    
    if (fp_geoid) fclose(fp_geoid);
    fp_geoid=NULL;
    model_geoid=GEOID_EMBEDDED;
}
/* geoid height ----------------------------------------------------------------
* get geoid height from geoid model
* args   : double *pos      I   geodetic position {lat,lon} (rad)
* return : geoid height (m) (0.0:error)
* notes  : to use external geoid model, call function opengeoid() to open
*          geoid model before calling the function. If the external geoid model
*          is not open, the function uses embedded geoid model.
*-----------------------------------------------------------------------------*/
extern double geoidh(const double *pos)
{
    double posd[2],h;
    
    posd[1]=pos[1]*R2D; posd[0]=pos[0]*R2D; if (posd[1]<0.0) posd[1]+=360.0;
    
    if (posd[1]<0.0||360.0-1E-12<posd[1]||posd[0]<-90.0||90.0<posd[0]) {
        trace(2,"out of range for geoid model: lat=%.3f lon=%.3f\n",posd[0],posd[1]);
        return 0.0;
    }
    switch (model_geoid) {
        case GEOID_EMBEDDED   : h=geoidh_emb  (posd); break;
        case GEOID_EGM96_M150 : h=geoidh_egm96(posd); break;
        case GEOID_EGM2008_M25: h=geoidh_egm08(posd,model_geoid); break;
        case GEOID_EGM2008_M10: h=geoidh_egm08(posd,model_geoid); break;
        case GEOID_GSI2000_M15: h=geoidh_gsi  (posd); break;
        default: return 0.0;
    }
    if (fabs(h)>200.0) {
        trace(2,"invalid geoid model: lat=%.3f lon=%.3f h=%.3f\n",posd[0],posd[1],h);
        return 0.0;
    }
    return h;
}