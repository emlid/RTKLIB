#include <stddef.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <linux/spi/spidev.h>
#include <linux/limits.h>

#include "spi.h"
#include "rtklib.h"

#define SPI_IO_DEBUG 0
#ifdef  SPI_IO_DEBUG
#define debug(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

#define BUFFER_LENGTH 300
#define SPI_DEFAULT_SPEED 245000

static int spi_parse_path(const char *path, char *device_path, uint32_t *speed, uint16_t *mode)
{
    char *p;
    size_t path_length;

    /* Check if any additional parameters have been provided */
    if ((p = strchr(path,':')) == NULL) {

        /* If there's no additional parameters, use default */
        strcpy(device_path, path);
        *speed = SPI_DEFAULT_SPEED;

        return 1;
    }

    sscanf(p + 1, "%" SCNu32 ":%" SCNu16, speed, mode); /* p points to the delimiter. */
    debug("%" SCNu32 ":%" SCNu16, *speed, *mode); 

    switch (*mode) {
        case 0:
            *mode = SPI_MODE_0;
            break;
        case 1:
            *mode = SPI_MODE_1;
            break;
        case 2:
            *mode = SPI_MODE_2;
            break;
        case 3:
            *mode = SPI_MODE_3;
            break;
        default:
            fprintf(stderr, "Wrong mode for SPI\n");
            return 0;
    }

    path_length = p - path;
    strncpy(device_path, path, path_length);
    device_path[path_length] = '\0'; /* strncpy doesn't null-terminate the string */

    return 1;
}

spi_t *openspi(const char *path, int mode, char *msg)
{
    spi_t *device;
    int flags = O_RDWR;
    uint32_t speed = 0;
    uint16_t spi_mode = SPI_MODE_0;
    char device_path[PATH_MAX];

    if (!spi_parse_path(path, device_path, &speed, &spi_mode)) {
        return NULL;
    }

    device = malloc(sizeof(spi_t));

    if (device == NULL) {
        perror("malloc: ");
        return NULL;
    }

    if ((mode & STR_MODE_R) &&
         (mode & STR_MODE_W)) {
        flags = O_RDWR;
    } else if (mode & STR_MODE_R) {
        flags = O_RDONLY;
    } else if (mode & STR_MODE_W) {
        flags = O_WRONLY;
    }

    device->fd = open(device_path, flags);

    if (device->fd < 0) {
        perror("open: ");
        goto errout_with_free;
    }

    /* hardcoded spi mode for Ublox */
    device->mode = mode;
    device->speed = speed;

    return device;

errout_with_free:
    free(device);
    return NULL;
}


static const uint8_t io_buffer[BUFFER_LENGTH];

int  writespi (spi_t *device, unsigned char *buff, int n, char *msg)
{
    int rc;
    struct spi_ioc_transfer transaction = {0};

    ioctl(device->fd, SPI_IOC_WR_MODE, device->mode);

    if (n > BUFFER_LENGTH) {
        n = BUFFER_LENGTH;
    }

    transaction.tx_buf = (unsigned long) buff;
    transaction.rx_buf = (unsigned long) NULL;
    transaction.len = n;
    transaction.speed_hz = device->speed;
    transaction.bits_per_word = 8;

    rc = ioctl(device->fd, SPI_IOC_MESSAGE(1), &transaction);
    if (rc < 0) {
        debug("n: %d\n", n);
        perror("ioctl: ");
        return 0;
    }

    return n;
}

int  readspi  (spi_t *device, unsigned char *buff, int n, char *msg)
{
    int rc;
    struct spi_ioc_transfer transaction = {0};

    ioctl(device->fd, SPI_IOC_WR_MODE, device->mode);

    if (n > BUFFER_LENGTH) {
        n = BUFFER_LENGTH;
    }

    transaction.tx_buf = (unsigned long) NULL;
    transaction.rx_buf = (unsigned long) buff;
    transaction.len = n;
    transaction.speed_hz = device->speed;
    transaction.bits_per_word = 8;

    rc = ioctl(device->fd, SPI_IOC_MESSAGE(1), &transaction);
    if (rc < 0) {
        debug("n: %d\n", n);
        perror("ioctl: ");
        return 0;
    }

    return n;
}

int  statespi (spi_t *device)
{
    return 3;
}

void closespi (spi_t *device)
{
    int rc;

    rc = close(device->fd);

    if (rc < 0) {
        perror("close: ");
    }
}
