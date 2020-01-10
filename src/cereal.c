#include <strings.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cereal.h"

/* parity = 0 (no parity), = 1 odd parity, = 2 even parity */
int initialize_serial_port(const char *dev, int canonical, int parity, int min_chars) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    // int fd = open(dev, O_RDWR | O_NOCTTY);
    // int fd = open(dev, O_WRONLY | O_NOCTTY);
    if(fd < 0) { return fd; }
    fcntl(fd, F_SETFL, 0);
    struct termios *settings, current_settings;

    memset(&current_settings, 0, sizeof(current_settings));
    tcgetattr(fd, &current_settings);

    /* effect new settings */
    settings = &current_settings;
    cfmakeraw(settings);
    if (parity == 0) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARENB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= CS8; /* eight bits */
    } else if (parity == 1) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB | PARODD); /* eight bits, odd parity */
    } else if (parity == 2) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARODD); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB); /* eight bits, odd parity is clear for even parity */
    }
    settings->c_cflag |= (CLOCAL | CREAD); /* ignore carrier detect. enable receiver */
    settings->c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    settings->c_iflag |= ( IGNPAR | IGNBRK);
    settings->c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    if (canonical) { settings->c_lflag |= ICANON; } /* set canonical */
    else { settings->c_lflag &= ~ICANON; } /* or clear it */
    settings->c_oflag &= ~(OPOST | ONLCR);
    settings->c_cc[VMIN] = min_chars;
    settings->c_cc[VTIME] = 1; /* 200ms timeout */

    cfsetispeed(settings, B115200);
    cfsetospeed(settings, B115200);

    tcsetattr(fd, TCSANOW, settings); /* apply settings */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void *tx_looper(void *ext) {
    TxLooperArgs *args = (TxLooperArgs *) ext;
    PoolQueue *queue = &args->queue;
    unsigned int arm = 1;
    if (args->nap_time == 0) { args->nap_time = 1000; } /* defaults to 1ms */
    if (args->arm == 0) { args->arm = &arm; }

    do {
        while ((args->arm == 0) || (*args->arm)) { usleep(args->nap_time); }
        while (*args->run) {
            while (queue->head >= queue->tail) {
                if (queue->length[queue->tail] && args->fd) {
                    ssize_t remaining = queue->length[queue->tail];
                    while (remaining > 0) {
                        ssize_t n_bytes = write(args->fd, queue->buff[queue->tail], (size_t) remaining);
                        remaining = remaining - n_bytes;
                    }
                    queue->length[queue->tail] = 0; /* clear once done */
                    queue->tail = (queue->tail + 1) & queue->mask;
                } else {
                    usleep(args->nap_time);
                }
            }
        }
    } while ((args->thread_run != 0) && (*args->thread_run));
    printf("good-bye from tx_looper\n");
    return NULL;
}

void *rx_looper(void *ext) {
    RxLooperArgs *args = (RxLooperArgs *) ext;
    PoolQueue *q = &args->queue;
    unsigned int arm = 1;
    if (args->nap_time == 0) { args->nap_time = 1000; } /* defaults to 1ms */
    if (args->arm == 0) { args->arm = &arm; }
    do {
        while ((args->arm == 0) || (*args->arm == 0)) { usleep(args->nap_time); }
        while ((args->run == 0) || (*args->run)) {
            ssize_t n_read = read(args->fd, q->buff[q->head], q->buff_mask);
            if (n_read <= 0) { continue; }
            unsigned int new_head = (q->head + 1) & q->mask;
            if (new_head != q->tail) {
                if (n_read > 0) {
                    q->length[q->head] = n_read & q->buff_mask;
                    q->head = new_head;
                }
            }
            usleep(args->nap_time);
        }
    } while ((args->thread_run != 0) && (*args->thread_run));
    printf("good-bye from rx_looper\n");
    return NULL;
}

/* TODO provide interface to change polynomial */

static uint16_t crc16_polynomial = 0x8d95;

const uint16_t crc16_lut[256] = {
    0x0000, 0x3596, 0x6b2c, 0x5eba, 0xd658, 0xe3ce, 0xbd74, 0x88e2,
    0xb79b, 0x820d, 0xdcb7, 0xe921, 0x61c3, 0x5455, 0x0aef, 0x3f79,
    0x741d, 0x418b, 0x1f31, 0x2aa7, 0xa245, 0x97d3, 0xc969, 0xfcff,
    0xc386, 0xf610, 0xa8aa, 0x9d3c, 0x15de, 0x2048, 0x7ef2, 0x4b64,
    0xe83a, 0xddac, 0x8316, 0xb680, 0x3e62, 0x0bf4, 0x554e, 0x60d8,
    0x5fa1, 0x6a37, 0x348d, 0x011b, 0x89f9, 0xbc6f, 0xe2d5, 0xd743,
    0x9c27, 0xa9b1, 0xf70b, 0xc29d, 0x4a7f, 0x7fe9, 0x2153, 0x14c5,
    0x2bbc, 0x1e2a, 0x4090, 0x7506, 0xfde4, 0xc872, 0x96c8, 0xa35e,
    0xcb5f, 0xfec9, 0xa073, 0x95e5, 0x1d07, 0x2891, 0x762b, 0x43bd,
    0x7cc4, 0x4952, 0x17e8, 0x227e, 0xaa9c, 0x9f0a, 0xc1b0, 0xf426,
    0xbf42, 0x8ad4, 0xd46e, 0xe1f8, 0x691a, 0x5c8c, 0x0236, 0x37a0,
    0x08d9, 0x3d4f, 0x63f5, 0x5663, 0xde81, 0xeb17, 0xb5ad, 0x803b,
    0x2365, 0x16f3, 0x4849, 0x7ddf, 0xf53d, 0xc0ab, 0x9e11, 0xab87,
    0x94fe, 0xa168, 0xffd2, 0xca44, 0x42a6, 0x7730, 0x298a, 0x1c1c,
    0x5778, 0x62ee, 0x3c54, 0x09c2, 0x8120, 0xb4b6, 0xea0c, 0xdf9a,
    0xe0e3, 0xd575, 0x8bcf, 0xbe59, 0x36bb, 0x032d, 0x5d97, 0x6801,
    0x8d95, 0xb803, 0xe6b9, 0xd32f, 0x5bcd, 0x6e5b, 0x30e1, 0x0577,
    0x3a0e, 0x0f98, 0x5122, 0x64b4, 0xec56, 0xd9c0, 0x877a, 0xb2ec,
    0xf988, 0xcc1e, 0x92a4, 0xa732, 0x2fd0, 0x1a46, 0x44fc, 0x716a,
    0x4e13, 0x7b85, 0x253f, 0x10a9, 0x984b, 0xaddd, 0xf367, 0xc6f1,
    0x65af, 0x5039, 0x0e83, 0x3b15, 0xb3f7, 0x8661, 0xd8db, 0xed4d,
    0xd234, 0xe7a2, 0xb918, 0x8c8e, 0x046c, 0x31fa, 0x6f40, 0x5ad6,
    0x11b2, 0x2424, 0x7a9e, 0x4f08, 0xc7ea, 0xf27c, 0xacc6, 0x9950,
    0xa629, 0x93bf, 0xcd05, 0xf893, 0x7071, 0x45e7, 0x1b5d, 0x2ecb,
    0x46ca, 0x735c, 0x2de6, 0x1870, 0x9092, 0xa504, 0xfbbe, 0xce28,
    0xf151, 0xc4c7, 0x9a7d, 0xafeb, 0x2709, 0x129f, 0x4c25, 0x79b3,
    0x32d7, 0x0741, 0x59fb, 0x6c6d, 0xe48f, 0xd119, 0x8fa3, 0xba35,
    0x854c, 0xb0da, 0xee60, 0xdbf6, 0x5314, 0x6682, 0x3838, 0x0dae,
    0xaef0, 0x9b66, 0xc5dc, 0xf04a, 0x78a8, 0x4d3e, 0x1384, 0x2612,
    0x196b, 0x2cfd, 0x7247, 0x47d1, 0xcf33, 0xfaa5, 0xa41f, 0x9189,
    0xdaed, 0xef7b, 0xb1c1, 0x8457, 0x0cb5, 0x3923, 0x6799, 0x520f,
    0x6d76, 0x58e0, 0x065a, 0x33cc, 0xbb2e, 0x8eb8, 0xd002, 0xe594
};

uint16_t crc16(const uint8_t *p, unsigned int len) {
    uint16_t crc = ~0;

    while (len--) {
        uint8_t ch1 = *p++;
        uint8_t ch2 = crc >> 8;
        uint16_t tCrc = crc << 8;
        crc = tCrc ^ crc16_lut[ch2 ^ ch1];
    }
    return ~crc;
}
