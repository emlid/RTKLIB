#ifndef __SPI_H__
#define __SPI_H__

struct spi_s
{
   int fd;   
};

typedef struct spi_s spi_t;

spi_t *openspi(const char *path, int mode, char *msg);

#endif
