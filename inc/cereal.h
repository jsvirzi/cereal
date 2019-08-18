#ifndef CEREAL_H
#define CEREAL_H

#include <stdint.h>

#include <pool_queue.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct TxLooperArgs {
    PoolQueue queue;
    unsigned int *thread_run; /* overall thread keeps running */
    unsigned int *arm; /* pause before running (setup daq) */
    unsigned int *run; /* main loop run (inner loop of thread) */
    int fd;
    int (*write)(void *hw, void *buff, unsigned int buff_size);
    unsigned int verbose, debug, format;
    unsigned char *msg_buff;
    unsigned int msg_size;
    int (*log)(void *device, void *buff, unsigned int buff_size);
    unsigned int nap_time; /* length of micronaps to avoid cpu saturation */
} TxLooperArgs;

typedef struct RxLooperArgs {
    PoolQueue queue;
    unsigned int *thread_run; /* overall thread keeps running */
    unsigned int *arm; /* pause before running (setup daq) */
    unsigned int *run; /* main loop run (inner loop of thread) */
    int fd;
    int (*read)(void *hw, void *buff, unsigned int buff_size);
    unsigned int verbose, debug, format;
    unsigned char *log_buff;
    unsigned int log_buff_size;
    void *log_device;
    int (*log_fxn)(void *log_device, void *log_buff, unsigned int log_buff_size);
    unsigned int nap_time;
} RxLooperArgs;

int initialize_serial_port(const char *dev, int canonical, int parity, int min_chars);
int set_blocking_mode(int fd, int blocking);
void *rx_looper(void *args);
void *tx_looper(void *args);

enum {
    FORMAT_ASCII = 0,
    FORMAT_HEX,
    FORMATS
};

uint16_t crc16(const uint8_t *p, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif
