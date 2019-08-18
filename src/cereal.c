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

int set_blocking_mode(int fd, int blocking) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (blocking) { flags &= ~O_NONBLOCK; }
    else { flags |= O_NONBLOCK; }
    fcntl(fd, F_SETFL, flags);
    return 0;
}

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
            unsigned int new_head = (q->head + 1) & q->mask;
            if (new_head != q->tail) {
                uint8_t *ptr = q->buff[q->head];
                ssize_t n_read = read(args->fd, q->buff[q->head], q->buff_mask + 1);
                if (n_read > 0) {
                    q->length[q->head] = n_read & q->buff_mask;
                    q->head = new_head;
                    if (args->verbose) {
                        char *msg = (char *) args->log_buff;
                        int remaining = args->log_buff_size;
                        int n_bytes = snprintf(msg, remaining, "\n");
                        msg += n_bytes;
                        remaining -= n_bytes;
                        for (unsigned int i = 0; i < n_read; ++i) {
                            uint8_t byte = ptr[i];
                            if (args->format == FORMAT_ASCII) {
                                snprintf(msg, remaining, "%c", (char) byte);
                                msg += n_bytes;
                                remaining -= n_bytes;
                            } else if (args->format == FORMAT_HEX) {
                                snprintf(msg, remaining, "%2.2x ", byte);
                                msg += n_bytes;
                                remaining -= n_bytes;
                            }
                        }
                        n_bytes = snprintf(msg, remaining, "\n");
                        msg += n_bytes;
                        remaining -= n_bytes;
                        n_bytes = args->log_buff_size - remaining; /* how many bytes written into msg */
                        if (args->log_fxn) { args->log_fxn(args->log_device, args->log_buff, n_bytes); }
                    }
                }
            }
            usleep(args->nap_time);
        }
    } while ((args->thread_run != 0) && (*args->thread_run));
    return NULL;
}

/* TODO provide interface to change polynomial */

static uint16_t crc16_polynomial = 0x8005;

static const uint16_t crc16_lut[256] = {
    0x0000, 0x9705, 0x2E01, 0xB904, 0x5C02, 0xCB07, 0x7203, 0xE506,
    0xB804, 0x2F01, 0x9605, 0x0100, 0xE406, 0x7303, 0xCA07, 0x5D02,
    0x7003, 0xE706, 0x5E02, 0xC907, 0x2C01, 0xBB04, 0x0200, 0x9505,
    0xC807, 0x5F02, 0xE606, 0x7103, 0x9405, 0x0300, 0xBA04, 0x2D01,
    0xE006, 0x7703, 0xCE07, 0x5902, 0xBC04, 0x2B01, 0x9205, 0x0500,
    0x5802, 0xCF07, 0x7603, 0xE106, 0x0400, 0x9305, 0x2A01, 0xBD04,
    0x9005, 0x0700, 0xBE04, 0x2901, 0xCC07, 0x5B02, 0xE206, 0x7503,
    0x2801, 0xBF04, 0x0600, 0x9105, 0x7403, 0xE306, 0x5A02, 0xCD07,
    0xC007, 0x5702, 0xEE06, 0x7903, 0x9C05, 0x0B00, 0xB204, 0x2501,
    0x7803, 0xEF06, 0x5602, 0xC107, 0x2401, 0xB304, 0x0A00, 0x9D05,
    0xB004, 0x2701, 0x9E05, 0x0900, 0xEC06, 0x7B03, 0xC207, 0x5502,
    0x0800, 0x9F05, 0x2601, 0xB104, 0x5402, 0xC307, 0x7A03, 0xED06,
    0x2001, 0xB704, 0x0E00, 0x9905, 0x7C03, 0xEB06, 0x5202, 0xC507,
    0x9805, 0x0F00, 0xB604, 0x2101, 0xC407, 0x5302, 0xEA06, 0x7D03,
    0x5002, 0xC707, 0x7E03, 0xE906, 0x0C00, 0x9B05, 0x2201, 0xB504,
    0xE806, 0x7F03, 0xC607, 0x5102, 0xB404, 0x2301, 0x9A05, 0x0D00,
    0x8005, 0x1700, 0xAE04, 0x3901, 0xDC07, 0x4B02, 0xF206, 0x6503,
    0x3801, 0xAF04, 0x1600, 0x8105, 0x6403, 0xF306, 0x4A02, 0xDD07,
    0xF006, 0x6703, 0xDE07, 0x4902, 0xAC04, 0x3B01, 0x8205, 0x1500,
    0x4802, 0xDF07, 0x6603, 0xF106, 0x1400, 0x8305, 0x3A01, 0xAD04,
    0x6003, 0xF706, 0x4E02, 0xD907, 0x3C01, 0xAB04, 0x1200, 0x8505,
    0xD807, 0x4F02, 0xF606, 0x6103, 0x8405, 0x1300, 0xAA04, 0x3D01,
    0x1000, 0x8705, 0x3E01, 0xA904, 0x4C02, 0xDB07, 0x6203, 0xF506,
    0xA804, 0x3F01, 0x8605, 0x1100, 0xF406, 0x6303, 0xDA07, 0x4D02,
    0x4002, 0xD707, 0x6E03, 0xF906, 0x1C00, 0x8B05, 0x3201, 0xA504,
    0xF806, 0x6F03, 0xD607, 0x4102, 0xA404, 0x3301, 0x8A05, 0x1D00,
    0x3001, 0xA704, 0x1E00, 0x8905, 0x6C03, 0xFB06, 0x4202, 0xD507,
    0x8805, 0x1F00, 0xA604, 0x3101, 0xD407, 0x4302, 0xFA06, 0x6D03,
    0xA004, 0x3701, 0x8E05, 0x1900, 0xFC06, 0x6B03, 0xD207, 0x4502,
    0x1800, 0x8F05, 0x3601, 0xA104, 0x4402, 0xD307, 0x6A03, 0xFD06,
    0xD007, 0x4702, 0xFE06, 0x6903, 0x8C05, 0x1B00, 0xA204, 0x3501,
    0x6803, 0xFF06, 0x4602, 0xD107, 0x3401, 0xA304, 0x1A00, 0x8D05
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
