
/*
 *
 *    Copyright (c) 2002, Smart Link Ltd.
 *    Copyright (c) 2021, Aon plc
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions
 *    are met:
 *
 *        1. Redistributions of source code must retain the above copyright
 *           notice, this list of conditions and the following disclaimer.
 *        2. Redistributions in binary form must reproduce the above
 *           copyright notice, this list of conditions and the following
 *           disclaimer in the documentation and/or other materials provided
 *           with the distribution.
 *        3. Neither the name of the Smart Link Ltd. nor the names of its
 *           contributors may be used to endorse or promote products derived
 *           from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *
 *    modem_main.c  --  modem main func.
 *
 *    Author: Sasha K (sashak@smlink.com)
 *
 *
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <limits.h>
#include <netinet/in.h>
#include <pwd.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#define ENOIOCTLCMD 515
#define BUFFER_PERIODS 12
#define SHORT_BUFFER_PERIODS 4
#define INTERNAL_DELAY 40

#include <modem.h>
#include <modem_debug.h>

#define INFO(fmt, args...) fprintf(stderr, fmt, ##args);
#define ERR(fmt, args...) fprintf(stderr, "error: " fmt, ##args);

#define DBG(fmt, args...) dprintf("main: " fmt, ##args)

// #define SLMODEMD_USER "nobody"
#define LOCKED_MEM_MIN_KB (8UL * 1024)
#define LOCKED_MEM_MIN (LOCKED_MEM_MIN_KB * 1024)

#define CLOSE_COUNT_MAX 100

/* modem init externals : FIXME remove it */
extern int dp_dummy_init(void);
extern void dp_dummy_exit(void);
extern int dp_sinus_init(void);
extern void dp_sinus_exit(void);
extern int prop_dp_init(void);
extern void prop_dp_exit(void);
extern int datafile_load_info(char *name, struct dsp_info *info);
extern int datafile_save_info(char *name, struct dsp_info *info);
extern int modem_ring_detector_start(struct modem *m);

/* global config data */
extern unsigned int ring_detector;
extern unsigned int need_realtime;
extern const char *modem_group;
extern mode_t modem_perm;
extern unsigned int use_short_buffer;
extern const int shared_fd;

struct device_struct {
    int num;
    int fd;
    int delay;
};

static char inbuf[4096];
static char outbuf[4096];

/*
 *    'driver' stuff
 *
 */
static int socket_start(struct modem *m) {
    struct device_struct *dev = m->dev_data;
    int ret;
    DBG("Device Start: Data-rate: %d\n", m->srate);
    DBG("Period: %d\n", m->frag);
    DBG("Buf Periods: %d\n", (use_short_buffer ? SHORT_BUFFER_PERIODS : BUFFER_PERIODS));

    if (fcntl(shared_fd, F_GETFL) == -1) {
        if (errno == EBADF) {
            fprintf(stderr, "Invalid file descriptor: %d\n", shared_fd);
            exit(EXIT_FAILURE);
        } else {
            perror("fcntl");
            exit(EXIT_FAILURE);
        }
    }

    dev->fd = shared_fd;
    dev->delay = 0;
    ret = 384;
    memset(outbuf, 0, ret);
    ret = write(dev->fd, outbuf, ret);
    if (ret < 0) {
        close(dev->fd);
        dev->fd = -1;
        return ret;
    }
    dev->delay = 0;
    return 0;
}

static int socket_stop(struct modem *m) {
    struct device_struct *dev = m->dev_data;
    DBG("socket_stop...\n");
    close(dev->fd);
    dev->fd = -1;
    return 0;
}

static int socket_ioctl(struct modem *m, unsigned int cmd, unsigned long arg) {
    // struct device_struct *dev = m->dev_data;
    int ret = 0;
    struct device_struct *dev = m->dev_data;
    DBG("socket_ioctl: cmd %x, arg %lx...\n", cmd, arg);
    if (cmd == MDMCTL_SETFRAG) arg <<= MFMT_SHIFT(m->format);

    switch (cmd) {
        case MDMCTL_CAPABILITIES:
            ret = -EINVAL;
            break;
        case MDMCTL_CODECTYPE:
            ret = 4;  // CODEC_STLC7550; XXX this worked fine as 0
                      // (CODEC_UNKNOWN)...
            break;
        case MDMCTL_IODELAY:  // kernel module returns s->delay +
                              // ST7554_HW_IODELAY (48)
            return dev->delay;
            break;
        case MDMCTL_HOOKSTATE:  // 0 = on, 1 = off
        case MDMCTL_SPEED:      // sample rate (9600)
        case MDMCTL_GETFMTS:
        case MDMCTL_SETFMT:
        case MDMCTL_SETFRAGMENT:  // (30)
        case MDMCTL_START:
        case MDMCTL_STOP:
        case MDMCTL_GETSTAT:
            ret = 0;
            break;
        default:
            return -ENOIOCTLCMD;
    }

    DBG("socket_ioctl: returning %x\n", ret);
    return ret;
}

struct modem_driver socket_modem_driver = {
    .name = "socket driver",
    .start = socket_start,
    .stop = socket_stop,
    .ioctl = socket_ioctl,
};

static int mdm_device_read(struct device_struct *dev, char *buf, int size) {
    int ret = read(dev->fd, buf, size * 2);
    if (ret > 0) {
        ret /= 2;
    }
    return ret;
}

static int mdm_device_write(struct device_struct *dev, const char *buf, int size) {
    int ret = write(dev->fd, buf, size * 2);
    if (ret > 0) {
        ret /= 2;
    }
    return ret;
}

static int mdm_device_release(struct device_struct *dev) {
    close(dev->fd);
    return 0;
}

static int socket_device_setup(struct device_struct *dev, const char *dev_name) {
    memset(dev, 0, sizeof(*dev));
    return 0;
}

/*
 *    PTY creation (or re-creation)
 *
 */

static char link_name[PATH_MAX];

int create_pty(struct modem *m) {
    struct termios termios;
    const char *pty_name;
    int pty, ret;

    if (m->pty) close(m->pty);

    pty = getpt();
    if (pty < 0 || grantpt(pty) < 0 || unlockpt(pty) < 0) {
        ERR("getpt: %s\n", strerror(errno));
        return -1;
    }

    if (m->pty) {
        termios = m->termios;
    } else {
        ret = tcgetattr(pty, &termios);
        /* non canonical raw tty */
        cfmakeraw(&termios);
        cfsetispeed(&termios, B115200);
        cfsetospeed(&termios, B115200);
    }

    ret = tcsetattr(pty, TCSANOW, &termios);
    if (ret) {
        ERR("tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    fcntl(pty, F_SETFL, O_NONBLOCK);

    pty_name = ptsname(pty);

    m->pty = pty;
    m->pty_name = pty_name;

    modem_update_termios(m, &termios);

    if (modem_group && *modem_group) {
        struct group *grp = getgrnam(modem_group);
        if (!grp) {
            ERR("cannot find group '%s': %s\n", modem_group, strerror(errno));
        } else {
            ret = chown(pty_name, -1, grp->gr_gid);
            if (ret < 0) {
                ERR("cannot chown '%s' to ':%s': %s\n", pty_name, modem_group, strerror(errno));
            }
        }
    }

    ret = chmod(pty_name, modem_perm);
    if (ret < 0) {
        ERR("cannot chmod '%s' to %o: %s\n", pty_name, modem_perm, strerror(errno));
    }

    if (*link_name) {
        unlink(link_name);
        if (symlink(pty_name, link_name)) {
            ERR("cannot create symbolink link `%s' -> `%s': %s\n", link_name, pty_name, strerror(errno));
            *link_name = '\0';
        } else {
            INFO("symbolic link `%s' -> `%s' created.\n", link_name, pty_name);
        }
    }

    return 0;
}

/*
 *    main run cycle
 *
 */

static int (*device_setup)(struct device_struct *dev, const char *dev_name);
static int (*device_release)(struct device_struct *dev);
static int (*device_read)(struct device_struct *dev, char *buf, int size);
static int (*device_write)(struct device_struct *dev, const char *buf, int size);
static struct modem_driver *modem_driver;

static volatile sig_atomic_t keep_running = 1;

void mark_termination(int signum) {
    DBG("signal %d: mark termination.\n", signum);
    keep_running = 0;
}

static int modem_run(struct modem *m, struct device_struct *dev) {
    struct timeval tmo;
    fd_set rset, eset;
    struct termios termios;
    unsigned pty_closed = 0, close_count = 0;
    int max_fd;
    int ret, count;
    void *in;

    while (keep_running) {
        if (m->event) modem_event(m);

#ifdef MODEM_CONFIG_RING_DETECTOR
        if (ring_detector && !m->started) modem_ring_detector_start(m);
#endif

        tmo.tv_sec = 1;
        tmo.tv_usec = 0;
        FD_ZERO(&rset);
        FD_ZERO(&eset);

        if (m->started) FD_SET(dev->fd, &rset);

        FD_SET(dev->fd, &eset);
        max_fd = dev->fd;

        if (pty_closed && close_count > 0) {
            if (!m->started || ++close_count > CLOSE_COUNT_MAX) {
                close_count = 0;
            }
        } else if (m->xmit.size - m->xmit.count > 0) {
            FD_SET(m->pty, &rset);
            if (m->pty > max_fd) {
                max_fd = m->pty;
            }
        }

        ret = select(max_fd + 1, &rset, NULL, &eset, &tmo);

        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            ERR("select: %s\n", strerror(errno));
            return ret;
        }

        if (ret == 0) {
            continue;
        }

        // error fd
        if (FD_ISSET(dev->fd, &eset)) {
            unsigned stat;
            DBG("dev exception...\n");
            ret = ioctl(dev->fd, 100000 + MDMCTL_GETSTAT, &stat);
            if (ret < 0) {
                ERR("dev ioctl: %s\n", strerror(errno));
                return -1;
            }

            if (stat & MDMSTAT_ERROR) {
                modem_error(m);
            }

            if (stat & MDMSTAT_RING) {
                modem_ring(m);
            }
            continue;
        }

        // read fd
        if (FD_ISSET(dev->fd, &rset)) {
            count = device_read(dev, inbuf, sizeof(inbuf) / 2);
            if (count <= 0) {
                if (errno == ECONNRESET) {
                    DBG("lost connection to child socket process\n");
                } else {
                    ERR("dev read: %s\n", strerror(errno));
                }
                // hack to force hangup
                modem_hangup(m);  // sets sample_timer_func to run_modem_stop()
                m->sample_timer_func(m);
                m->sample_timer = 0;
                m->sample_timer_func = NULL;
                continue;
            }

            in = inbuf;
            if (m->update_delay < 0) {
                if (-m->update_delay >= count) {
                    DBG("change delay -%d...\n", count);
                    dev->delay -= count;
                    m->update_delay += count;
                    continue;
                }
                DBG("change delay %d...\n", m->update_delay);
                in -= m->update_delay;
                count += m->update_delay;
                dev->delay += m->update_delay;
                m->update_delay = 0;
            }

            modem_process(m, in, outbuf, count);

            if (dev->fd == -1) {
                DBG("closed connection to child socket process\n");
                continue;
            }

            count = device_write(dev, outbuf, count);
            if (count < 0) {
                ERR("dev write: %s\n", strerror(errno));
                return -1;
            } else if (count == 0) {
                DBG("dev write = 0\n");
            }

            if (m->update_delay > 0) {
                DBG("change delay +%d...\n", m->update_delay);
                memset(outbuf, 0, m->update_delay * 2);
                count = device_write(dev, outbuf, m->update_delay);
                if (count < 0) {
                    ERR("dev write: %s\n", strerror(errno));
                    return -1;
                }
                if (count != m->update_delay) {
                    ERR("cannot update delay: %d instead of %d.\n", count, m->update_delay);
                    return -1;
                }
                dev->delay += m->update_delay;
                m->update_delay = 0;
            }
        }

        // read pty
        if (FD_ISSET(m->pty, &rset)) {
            /* check termios */
            tcgetattr(m->pty, &termios);
            if (memcmp(&termios, &m->termios, sizeof(termios))) {
                DBG("termios changed.\n");
                modem_update_termios(m, &termios);
            }

            /* read data */
            count = m->xmit.size - m->xmit.count;
            if (count == 0) {
                continue;
            }

            if (count > sizeof(inbuf)) {
                count = sizeof(inbuf);
            }

            count = read(m->pty, inbuf, count);

            if (count < 0) {
                if (errno == EAGAIN) {
                    DBG("pty read, errno = EAGAIN\n");
                    continue;
                }
                if (errno == EIO) { /* closed */
                    if (!pty_closed) {
                        DBG("pty closed.\n");
                        if (termios.c_cflag & HUPCL) {
                            modem_hangup(m);
                            /* re-create PTM - simulate hangup */
                            ret = create_pty(m);
                            if (ret < 0) {
                                ERR("cannot re-create PTY.\n");
                                return -1;
                            }
                        } else
                            pty_closed = 1;
                    }
                    // DBG("pty read, errno = EIO\n");
                    close_count = 1;
                    continue;
                } else
                    ERR("pty read: %s\n", strerror(errno));
                return -1;
            } else if (count == 0) {
                DBG("pty read = 0\n");
            }
            pty_closed = 0;
            count = modem_write(m, inbuf, count);
            if (count < 0) {
                ERR("modem_write failed.\n");
                return -1;
            }
        }
    }

    return 0;
}

int modem_main(const char *dev_name) {
    char path_name[PATH_MAX];
    struct device_struct device;
    struct modem *m;
    int pty;
    int ret = 0;

    modem_debug_init(basename(dev_name));
    ret = device_setup(&device, dev_name);
    if (ret) {
        ERR("cannot setup device `%s'\n", dev_name);
        exit(-1);
    }

    dp_dummy_init();
    dp_sinus_init();
    prop_dp_init();
    modem_timer_init();

    sprintf(link_name, "/dev/ttySL%d", device.num);

    m = modem_create(modem_driver, basename(dev_name));
    m->name = basename(dev_name);
    m->dev_data = &device;
    m->dev_name = dev_name;
    modem_set_sreg(m, SREG_DTMF_DURATION, 6000);

    ret = create_pty(m);
    if (ret < 0) {
        ERR("cannot create PTY.\n");
        exit(-1);
    }

    INFO("modem `%s' created. TTY is `%s'\n", m->name, m->pty_name);

    sprintf(path_name, "/var/lib/slmodem/data.%s", basename(dev_name));
    datafile_load_info(path_name, &m->dsp_info);

    if (need_realtime) {
        struct sched_param prm;
        if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
            ERR("mlockall: %s\n", strerror(errno));
        }
        prm.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (sched_setscheduler(0, SCHED_FIFO, &prm)) {
            ERR("sched_setscheduler: %s\n", strerror(errno));
        }
        DBG("rt applyed: SCHED_FIFO, pri %d\n", prm.sched_priority);
    }

    signal(SIGINT, mark_termination);
    signal(SIGTERM, mark_termination);

#ifdef SLMODEMD_USER
    struct passwd *pwd;
    if (need_realtime) {
        struct rlimit limit;
        if (getrlimit(RLIMIT_MEMLOCK, &limit)) {
            ERR("getrlimit failed to read RLIMIT_MEMLOCK\n");
            exit(-1);
        }
        if (limit.rlim_cur != RLIM_INFINITY && limit.rlim_cur < LOCKED_MEM_MIN) {
            ERR("locked memory limit too low:\n");
            ERR("need %lu bytes, have %lu bytes\n", LOCKED_MEM_MIN, (unsigned long)limit.rlim_cur);
            ERR("try 'ulimit -l %lu'\n", LOCKED_MEM_MIN_KB);
            exit(-1);
        }
    }

    pwd = getpwnam(SLMODEMD_USER);
    if (!pwd) {
        ERR("getpwnam " SLMODEMD_USER ": %s\n", strerror(errno));
        exit(-1);
    }

    ret = (setgroups(1, &pwd->pw_gid) || setgid(pwd->pw_gid) || setuid(pwd->pw_uid));
    if (ret) {
        ERR("setgroups or setgid %ld or setuid %ld failed: %s\n", (long)pwd->pw_gid, (long)pwd->pw_uid, strerror(errno));
        exit(-1);
    }

    if (setuid(0) != -1) {
        ERR("setuid 0 succeeded after dropping privileges!\n");
        exit(-1);
    }
    DBG("dropped privileges to %ld.%ld\n", (long)pwd->pw_gid, (long)pwd->pw_uid);
#endif

    INFO("Use `%s' as modem device, Ctrl+C for termination.\n", *link_name ? link_name : m->pty_name);

    /* main loop here */
    ret = modem_run(m, &device);

    datafile_save_info(path_name, &m->dsp_info);

    pty = m->pty;
    modem_delete(m);

    usleep(100000);
    close(pty);
    if (*link_name) unlink(link_name);

    dp_dummy_exit();
    dp_sinus_exit();
    prop_dp_exit();

    device_release(&device);

    modem_debug_exit();

    exit(ret);
    return 0;
}

int main(int argc, char *argv[]) {
    extern void modem_cmdline(int argc, char *argv[]);
    int ret;
    modem_cmdline(argc, argv);

    device_setup = socket_device_setup;
    device_release = mdm_device_release;
    device_read = mdm_device_read;
    device_write = mdm_device_write;
    modem_driver = &socket_modem_driver;

    ret = modem_main("modem");
    return ret;
}
