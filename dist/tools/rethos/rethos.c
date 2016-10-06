/*
 * Copyright (C) 2016 Sam Kumar <samkumar@berkeley.edu>
 *
 * This is a re-implementation of ethos (originally by Kaspar Schleiser)
 * that creates a reliable multi-channel duplex link over serial.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file LICENSE for more details.
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <netinet/in.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/un.h>
#include <stdlib.h>

#include <netdb.h>

#include <termios.h>

#define MTU 9000

#define TRACE(x)
#define TTY_TIMEOUT_MS (500)

#define case_baudrate(val)    \
    case val:                 \
        *baudrate = B ## val; \
        break

#define BAUDRATE_DEFAULT B115200

#define STDIN_CHANNEL 0
#define TUNTAP_CHANNEL 1
#define NUM_CHANNELS 256

#define TCP_DEV "tcp:"
#define IOTLAB_TCP_PORT "20000"

static void usage(void)
{
    fprintf(stderr, "Usage: rethos <tap> <serial> [baudrate]\n");
    fprintf(stderr, "       rethos <tap> tcp:<host> [port]\n");
}

static void checked_write(int handle, const void *buffer, int nbyte)
{
    if (write(handle, buffer, nbyte) != nbyte) {
        fprintf(stderr, "write to fd %i failed: %s\n", handle, strerror(errno));
    }
}

int set_serial_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        perror ("error in tcgetattr");
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8-bit chars*/
                                        /* disable IGNBRK for mismatched speed
                                         * tests; otherwise receive break*/
                                        /* as \000 chars*/
    tty.c_iflag &= ~IGNBRK;             /* disable break processing*/
    tty.c_lflag = 0;                    /* no signaling chars, no echo,*/
                                        /* no canonical processing*/
    tty.c_oflag = 0;                    /* no remapping, no delays*/
    tty.c_cc[VMIN]  = 0;                /* read doesn't block*/
    tty.c_cc[VTIME] = TTY_TIMEOUT_MS / 100; /* 0.5 seconds read timeout*/
                                            /* in tenths of a second*/

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* shut off xon/xoff ctrl*/

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls,*/
                                        /* enable reading*/
    tty.c_cflag &= ~(PARENB | PARODD);  /* shut off parity*/
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    cfmakeraw(&tty);

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        perror("error from tcsetattr");
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        perror("error from tggetattr");
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = TTY_TIMEOUT_MS / 100; /* 0.5 seconds read timeout*/
                                            /* in tenths of a second*/

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        perror("error setting term attributes");
}

/**
 * @name Escape char definitions
 * @{
 */
#define RETHOS_ESC_CHAR                  (0xBE)
/* This means that a stream of ESC_CHAR still keeps us inside the escape state */
#define RETHOS_LITERAL_ESC               (0x55)
#define RETHOS_FRAME_START               (0xEF)
#define RETHOS_FRAME_END                 (0xE5)

#define RETHOS_FRAME_TYPE_DATA           (0x1)

#define RETHOS_FRAME_TYPE_HB             (0x2)
#define RETHOS_FRAME_TYPE_HB_REPLY       (0x3)

/** @} */

static const uint8_t _esc_esc[] = {RETHOS_ESC_CHAR, RETHOS_LITERAL_ESC};
static const uint8_t _start_frame[] = {RETHOS_ESC_CHAR, RETHOS_FRAME_START};
static const uint8_t _end_frame[] = {RETHOS_ESC_CHAR, RETHOS_FRAME_END};

typedef enum {
    WAIT_FRAMESTART,
    WAIT_FRAMETYPE,
    WAIT_SEQNO_1,
    WAIT_SEQNO_2,
    WAIT_CHANNEL,
    IN_FRAME,
    WAIT_CHECKSUM_1,
    WAIT_CHECKSUM_2
} line_state_t;

typedef struct {
    int fd;

    /* State for reading data. */
    line_state_t state;
    uint8_t frametype;
    uint16_t in_seqno;
    uint8_t channel;
    size_t numbytes;
    char frame[MTU];
    uint16_t checksum;

    bool in_escape;

    /* State for writing data. */
    uint16_t out_seqno;
} serial_t;

static bool _serial_handle_byte(serial_t *serial, char c)
{
    bool ready = false;

    if (c == RETHOS_ESC_CHAR) {
        serial->in_escape = true;
        return ready;
    }

    if (serial->in_escape) {
        if (c == RETHOS_LITERAL_ESC) {
            c = RETHOS_ESC_CHAR;
            serial->in_escape = false;
        } else if (c == RETHOS_FRAME_START) {
            /* If we receive the start sequence, then drop the current frame and start receiving. */
            if (serial->state != WAIT_FRAMESTART) {
                fprintf(stderr, "Got unexpected start-of-frame sequence: dropping current frame\n");
            }
            serial->state = WAIT_FRAMETYPE;
            goto done_char;
        } else if (c == RETHOS_FRAME_END) {
            if (serial->state != IN_FRAME) {
                fprintf(stderr, "Got unexpected end-of-frame sequence: dropping current frame\n");
                goto handle_corrupt_frame;
            }

            serial->state = WAIT_CHECKSUM_1;
            goto done_char;
        } else {
            fprintf(stderr, "Got unexpected escape sequence 0xBE%X: dropping current frame\n", (uint8_t) c);
            goto handle_corrupt_frame;
        }
    }

    switch (serial->state) {
        case WAIT_FRAMESTART:
            fprintf(stderr, "Got stray byte %c\n", c);
            break;
        case WAIT_FRAMETYPE:
            serial->frametype = (uint8_t) c;
            serial->state = WAIT_SEQNO_1;
            break;
        case WAIT_SEQNO_1:
            serial->in_seqno = ((uint8_t) c);
            serial->state = WAIT_SEQNO_2;
            break;
        case WAIT_SEQNO_2:
            serial->in_seqno |= (((uint16_t) c) << 8);
            serial->state = WAIT_CHANNEL;
            break;
        case WAIT_CHANNEL:
            serial->channel = (uint8_t) c;
            serial->state = IN_FRAME;
            serial->numbytes = 0;
            break;
        case IN_FRAME:
            if (serial->numbytes >= MTU) {
                fprintf(stderr, "Dropping runaway frame\n");
                goto handle_corrupt_frame;
            }
            serial->frame[serial->numbytes] = c;
            serial->numbytes++;
            break;
        case WAIT_CHECKSUM_1:
            serial->checksum = (uint8_t) c;
            serial->state = WAIT_CHECKSUM_2;
            break;
        case WAIT_CHECKSUM_2:
            serial->checksum |= (((uint16_t) c) << 8);

            /* Set "ready" so the frame is delivered to the handler. */
            ready = true;

            goto drop_frame_state;
    }

    goto done_char;

handle_corrupt_frame:
    /* TODO: Send a NACK here. */

drop_frame_state:
    /* Start listening for a frame at the beginning. */
    serial->state = WAIT_FRAMESTART;

done_char:
    /* Finished handling this character. */
    serial->in_escape = false;
    return ready;
}

static void fletcher16_add(const uint8_t *data, size_t bytes, uint16_t *sum1i, uint16_t *sum2i)
{
    uint16_t sum1 = *sum1i, sum2 = *sum2i;

    while (bytes) {
        size_t tlen = bytes > 20 ? 20 : bytes;
        bytes -= tlen;
        do {
            sum2 += sum1 += *data++;
        } while (--tlen);
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
    }
    *sum1i = sum1;
    *sum2i = sum2;
}

static uint16_t fletcher16_fin(uint16_t sum1, uint16_t sum2)
{
  sum1 = (sum1 & 0xff) + (sum1 >> 8);
  sum2 = (sum2 & 0xff) + (sum2 >> 8);
  return (sum2 << 8) | sum1;
}

/**************************************************************************
 * tun_alloc: allocates or reconnects to a tun/tap device. The caller     *
 *            needs to reserve enough space in *dev.                      *
 **************************************************************************/
int tun_alloc(char *dev, int flags) {

  struct ifreq ifr;
  int fd, err;

  if( (fd = open("/dev/net/tun", O_RDWR)) < 0 ) {
    perror("Opening /dev/net/tun");
    return fd;
  }

  memset(&ifr, 0, sizeof(ifr));

  ifr.ifr_flags = flags;

  if (*dev) {
    strncpy(ifr.ifr_name, dev, IFNAMSIZ);
  }

  if( (err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0 ) {
    perror("ioctl(TUNSETIFF)");
    close(fd);
    return err;
  }

  strcpy(dev, ifr.ifr_name);

  return fd;
}

static void _write_escaped(int fd, const char* buf, ssize_t n)
{
    for (ssize_t i = 0; i < n; i++) {
        char c = buf[i];
        if (c == RETHOS_ESC_CHAR) {
            checked_write(fd, _esc_esc, sizeof(_esc_esc));
        }
        checked_write(fd, &buf[i], 1);
    }
}

void rethos_send_frame(serial_t* serial, const uint8_t *data, size_t thislen, uint8_t channel, uint8_t frame_type)
{
  uint8_t preamble_buffer[4];
  uint8_t postamble_buffer[2];

  uint16_t flsum1 = 0xFF;
  uint16_t flsum2 = 0xFF;

  uint16_t seqno = ++serial->out_seqno;

  //This is where the checksum starts
  preamble_buffer[0] = frame_type;
  preamble_buffer[1] = seqno & 0xFF; //Little endian cos im a rebel
  preamble_buffer[2] = seqno >> 8;
  preamble_buffer[3] = channel;

  checked_write(serial->fd, _start_frame, sizeof(_start_frame));

  fletcher16_add(preamble_buffer, sizeof(preamble_buffer), &flsum1, &flsum2);
  _write_escaped(serial->fd, preamble_buffer, sizeof(preamble_buffer));

  fletcher16_add(data, thislen, &flsum1, &flsum2);
  _write_escaped(serial->fd, data, thislen);

  checked_write(serial->fd, _end_frame, sizeof(_end_frame));

  uint16_t cksum = fletcher16_fin(flsum1, flsum2);
  postamble_buffer[0] = (uint8_t) cksum;
  postamble_buffer[1] = (uint8_t) (cksum >> 8);

  _write_escaped(serial->fd, postamble_buffer, sizeof(postamble_buffer));

}

static void _clear_neighbor_cache(const char *ifname)
{
    char tmp[20 + IFNAMSIZ];
    snprintf(tmp, sizeof(tmp), "ip neigh flush dev %s", ifname);
    if (system(tmp) < 0) {
        fprintf(stderr, "error while flushing device neighbor cache\n");
    }
}

static int _parse_baudrate(const char *arg, unsigned *baudrate)
{
    if (arg == NULL) {
        *baudrate = BAUDRATE_DEFAULT;
        return 0;
    }

    switch(strtol(arg, (char**)NULL, 10)) {
    case 9600:
        *baudrate = B9600;
        break;
    case 19200:
        *baudrate = B19200;
        break;
    case 38400:
        *baudrate = B38400;
        break;
    case 57600:
        *baudrate = B57600;
        break;
    case 115200:
        *baudrate = B115200;
        break;
    /* the following baudrates might not be available on all platforms */
    #ifdef B234000
        case_baudrate(230400);
    #endif
    #ifdef B460800
        case_baudrate(460800);
    #endif
    #ifdef B500000
        case_baudrate(500000);
    #endif
    #ifdef B576000
        case_baudrate(576000);
    #endif
    #ifdef B921600
        case_baudrate(921600);
    #endif
    #ifdef B1000000
        case_baudrate(1000000);
    #endif
    #ifdef B1152000
        case_baudrate(1152000);
    #endif
    #ifdef B1500000
        case_baudrate(1500000);
    #endif
    #ifdef B2000000
        case_baudrate(2000000);
    #endif
    #ifdef B2500000
        case_baudrate(2500000);
    #endif
    #ifdef B3000000
        case_baudrate(3000000);
    #endif
    #ifdef B3500000
        case_baudrate(3500000);
    #endif
    #ifdef B4000000
        case_baudrate(4000000);
    #endif
    default:
        return -1;
    }

    return 0;
}

int _parse_tcp_arg(char *name, char *port_arg, char **host, char **port)
{
    /* Remove 'tcp:' */
    name = &name[sizeof(TCP_DEV) - 1];

    /* Set default if NULL */
    if (!port_arg) {
        port_arg = IOTLAB_TCP_PORT;
    }

    *host = name;
    *port = port_arg;

    return 0;
}

/* Adapted from 'getaddrinfo' manpage example */
int _tcp_connect(char *host, char *port)
{
    int sfd = -1;
    struct addrinfo hints, *result, *rp;

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int s = getaddrinfo(host, port, &hints, &result);
    if (s) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
        return -1;
    }

    /* getaddrinfo() returns a list of address structures.
       Try each address until we successfully connect(2).
       If socket(2) (or connect(2)) fails, we (close the socket
       and) try the next address. */
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sfd == -1)
            continue;

        if (connect(sfd, rp->ai_addr, rp->ai_addrlen) != -1)
            break;

        close(sfd);
    }

    freeaddrinfo(result);

    if (rp == NULL) {
        fprintf(stderr, "Could not connect to '%s:%s'\n", host, port);
        return -1;
    }

    return sfd;
}

int _set_socket_timeout(int sfd)
{
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = TTY_TIMEOUT_MS * 1000,
    };

    if (setsockopt(sfd, SOL_SOCKET, SO_RCVTIMEO,
                   (char *)&timeout, sizeof(timeout)) == -1) {
        perror("setsockopt failed\n");
        return 1;
    }

    if (setsockopt(sfd, SOL_SOCKET, SO_SNDTIMEO,
                   (char *)&timeout, sizeof(timeout)) == -1) {
        perror("setsockopt failed\n");
        return 1;
    }
    return 0;
}

int _open_tcp_connection(char *name, char *port_arg)
{
    char *host;
    char *port;

    int ret = _parse_tcp_arg(name, port_arg, &host, &port);
    if (ret) {
        fprintf(stderr, "Error while parsing tcp arguments\n");
        return -1;
    }

    int sfd = _tcp_connect(host, port);
    if (_set_socket_timeout(sfd)) {
        fprintf(stderr, "Error while setting socket options\n");
        return -1;
    }
    return sfd;
}

int _open_serial_connection(char *name, char *baudrate_arg)
{
    unsigned baudrate = 0;
    if (_parse_baudrate(baudrate_arg, &baudrate) == -1) {
        fprintf(stderr, "Invalid baudrate specified: %s\n", baudrate_arg);
        return 1;
    }

    int serial_fd = open(name, O_RDWR | O_NOCTTY | O_SYNC);

    if (serial_fd < 0) {
        fprintf(stderr, "Error opening serial device %s\n", name);
        return -1;
    }

    set_serial_attribs(serial_fd, baudrate, 0);
    set_blocking(serial_fd, 1);

    return serial_fd;
}

int _open_connection(char *name, char* option)
{
    if (strncmp(name, TCP_DEV, strlen(TCP_DEV)) == 0) {
        return _open_tcp_connection(name, option);
    } else {
        return _open_serial_connection(name, option);
    }
}

void check_fatal_error(const char* msg)
{
    if (errno) {
        perror(msg);
        exit(1);
    }
}

typedef struct {
    int server_socket;
    int client_socket;
} channel_t;

void channel_listen(channel_t* chan, int channel_number) {
    int dsock;
    int flags;
    struct sockaddr_un bound_name;
    size_t total_size;
    memset(&bound_name, 0, sizeof(bound_name));
    bound_name.sun_family = AF_UNIX;
    snprintf(&bound_name.sun_path[1], sizeof(bound_name.sun_path) - 1, "rethos/%d", channel_number);
    total_size = sizeof(bound_name.sun_family) + 1 + strlen(&bound_name.sun_path[1]);
    dsock = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    check_fatal_error("Could not create domain socket");
    flags = fcntl(dsock, F_GETFL);
    check_fatal_error("Could not get socket flags");
    assert(flags != -1);
    fcntl(dsock, F_SETFL, flags | O_NONBLOCK);
    check_fatal_error("Could not set socket flags");
    bind(dsock, (struct sockaddr*) &bound_name, total_size);
    check_fatal_error("Could not bind domain socket");
    listen(dsock, 0);
    check_fatal_error("Could not listen on domain socket");

    chan->server_socket = dsock;
    chan->client_socket = -1;
}

int main(int argc, char *argv[])
{
    char inbuf[MTU];
    char *serial_option = NULL;

    serial_t serial = {0};

    if (argc < 3) {
        usage();
        return 1;
    }

    if (argc >= 4) {
        serial_option = argv[3];
    }

    char ifname[IFNAMSIZ];
    strncpy(ifname, argv[1], IFNAMSIZ);
    int tap_fd = tun_alloc(ifname, IFF_TAP | IFF_NO_PI);

    if (tap_fd < 0) {
        return 1;
    }


    int serial_fd = _open_connection(argv[2], serial_option);
    if (serial_fd < 0) {
        fprintf(stderr, "Error opening serial device %s\n", argv[2]);
        return 1;
    }

    serial.fd = serial_fd;

    fd_set readfds;

    channel_t domain_sockets[NUM_CHANNELS];
    int i;
    for (i = 0; i < NUM_CHANNELS; i++) {
        channel_listen(&domain_sockets[i], i);
    }

    while (true) {
        int activity;
        int max_fd = 0;
        #define UPDATE_MAX_FD(fd) max_fd = ((fd) > max_fd ? (fd) : max_fd);
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        UPDATE_MAX_FD(STDIN_FILENO);
        FD_SET(tap_fd, &readfds);
        UPDATE_MAX_FD(tap_fd);
        FD_SET(serial_fd, &readfds);
        UPDATE_MAX_FD(serial_fd);
        for (i = 0; i < NUM_CHANNELS; i++) {
            if (domain_sockets[i].client_socket == -1) {
                FD_SET(domain_sockets[i].server_socket, &readfds);
                UPDATE_MAX_FD(domain_sockets[i].server_socket);
            } else {
                FD_SET(domain_sockets[i].client_socket, &readfds);
                UPDATE_MAX_FD(domain_sockets[i].client_socket);
            }
        }
        activity = select(max_fd + 1, &readfds, NULL, NULL, NULL);

        if ((activity < 0) && (errno != EINTR))
        {
            perror("select error");
        }

        if (FD_ISSET(serial_fd, &readfds)) {
            ssize_t n = read(serial_fd, inbuf, sizeof(inbuf));
            if (n > 0) {
                char *ptr = inbuf;
                for (i = 0; i != n; i++) {
                    bool ready = _serial_handle_byte(&serial, *ptr++);
                    if (ready) {
                        /* TODO check the checksum, and use the sequence number, and
                         * message type for reliable delivery. */

                        int out_fd;
                        if (serial.channel == STDIN_CHANNEL) {
                            out_fd = STDOUT_FILENO;
                        } else if (serial.channel == TUNTAP_CHANNEL) {
                            out_fd = tap_fd;
                        } else if (serial.channel < NUM_CHANNELS && domain_sockets[serial.channel].client_socket != -1) {
                            out_fd = domain_sockets[serial.channel].client_socket;
                        } else {
                            fprintf(stderr, "Got message on channel %d, which is not connected: dropping message\n", serial.channel);
                            continue;
                        }

                        printf("Got a frame on channel %d\n", serial.channel);

                        checked_write(out_fd, serial.frame, serial.numbytes);
                    }
                }
            }
            else {
                fprintf(stderr, "lost serial connection.\n");
                exit(1);
            }
        }

        if (FD_ISSET(tap_fd, &readfds)) {
            ssize_t res = read(tap_fd, inbuf, sizeof(inbuf));
            if (res <= 0) {
                fprintf(stderr, "error reading from tap device. res=%zi\n", res);
                continue;
            }
            rethos_send_frame(&serial, inbuf, res, TUNTAP_CHANNEL, RETHOS_FRAME_TYPE_DATA);
        }

        if (FD_ISSET(STDIN_FILENO, &readfds)) {
            ssize_t res = read(STDIN_FILENO, inbuf, sizeof(inbuf));
            if (res <= 0) {
                fprintf(stderr, "error reading from stdio. res=%zi\n", res);
                continue;
            }
            rethos_send_frame(&serial, inbuf, res, STDIN_CHANNEL, RETHOS_FRAME_TYPE_DATA);
        }

        for (i = 0; i < NUM_CHANNELS; i++) {
            int dsock;
            if (domain_sockets[i].client_socket == -1) {
                dsock = domain_sockets[i].server_socket;
                if (FD_ISSET(dsock, &readfds)) {
                    struct sockaddr_un client_addr;
                    socklen_t client_addr_len;
                    int client_socket = accept(dsock, (struct sockaddr*) &client_addr, &client_addr_len);
                    check_fatal_error("accept connection on domain socket");
                    assert(client_socket != -1);
                    printf("Accepted client process on channel %d\n", i);
                    domain_sockets[i].client_socket = client_socket;

                    /* Stop listening on this channel. It only makes sense to have one entity listening and writing. */
                    close(domain_sockets[i].server_socket);
                    domain_sockets[i].server_socket = -1;
                }
            } else {
                dsock = domain_sockets[i].client_socket;
                if (FD_ISSET(dsock, &readfds)) {
                    ssize_t res = read(dsock, inbuf, sizeof(inbuf));
                    char msgbuf[100];
                    if (res <= 0) {
                        assert(errno != EWOULDBLOCK);
                        close(dsock);
                        domain_sockets[i].client_socket = -1;
                        channel_listen(&domain_sockets[i], i);
                        printf("Client process on channel %d disconnected\n", i);
                        continue;
                    }
                    rethos_send_frame(&serial, inbuf, res, i, RETHOS_FRAME_TYPE_DATA);
                }
            }
        }
    }

    return 0;
}
