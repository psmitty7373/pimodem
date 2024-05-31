#include <alsa/asoundlib.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>

#define BUFFER_SIZE     4096
#define SAMPLE_RATE     9600
#define PERIOD          48
#define PERIOD_BUFFER   16

void alsa_error(const char *msg, int err) {
    fprintf(stderr, "%s (%s)\n", msg, snd_strerror(err));
}

int setup_mixer(snd_mixer_t **mixer, char *dev) {
    snd_mixer_selem_id_t *sid, *sidb;
    snd_mixer_elem_t *elem;
    long min, max;
    int err = 0;

    if ((err = snd_mixer_open(mixer, 0)) < 0) {
        alsa_error("cannot open mixer", err);
        goto exit;
    }

    if ((err = snd_mixer_attach(*mixer, dev)) < 0) {
        alsa_error("cannot attach to mixer", err);
        goto exit;
    }

    if ((err = snd_mixer_selem_register(*mixer, NULL, NULL)) < 0) {
        alsa_error("cannot register mixer element", err);
        goto exit;
    }

    if ((err = snd_mixer_load(*mixer)) < 0) {
        alsa_error("cannot load mixer elements", err);
        goto exit;
    }

    for (elem = snd_mixer_first_elem(*mixer); elem; elem = snd_mixer_elem_next(elem)) {
        printf("Element: %s\n", snd_mixer_selem_get_name(elem));
    }

    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_name(sid, "Speaker");

    elem = snd_mixer_find_selem(*mixer, sid);
    if (!elem) {
        printf("could not find mixer element: Speaker");
        goto exit;
    }

    snd_mixer_selem_get_playback_volume_range(elem, &min, &max);

    if ((err = snd_mixer_selem_set_playback_volume_all(elem, 45 * max / 100)) < 0) {
        alsa_error("could not set volume", err);
        goto exit;
    }

    snd_mixer_selem_id_alloca(&sidb);
    snd_mixer_selem_id_set_index(sidb, 0);
    snd_mixer_selem_id_set_name(sidb, "Mic");

    elem = snd_mixer_find_selem(*mixer, sidb);

    snd_mixer_selem_get_playback_volume_range(elem, &min, &max);

    if (!elem) {
        printf("could not find mixer element: Mic");
        goto exit;
    }

    if (((err = snd_mixer_selem_set_playback_volume_all(elem, 1 * max / 100) < 0) < 0) || (err = snd_mixer_selem_set_capture_volume_all(elem, 1 * max / 100)) < 0) {
        alsa_error("could not set volume", err);
        goto exit;
    }

    exit:
    return err;
}

int setup_stream(snd_pcm_t **handle, char *dev, int stream_type) {
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;
    snd_pcm_format_t format;
    char buf[4096];
    int err = 0;

    if ((err = snd_pcm_open(handle, dev, stream_type, SND_PCM_NONBLOCK)) < 0) {
        alsa_error("cannot open audio device", err);
        goto exit;
    }

    // HW
    if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
        alsa_error("cannot allocate hardware parameter structure", err);
        goto exit;
    }

    if ((err = snd_pcm_hw_params_any(*handle, hw_params)) < 0) {
        alsa_error("cannot initialize hardware parameter structure", err);
        goto exit;
    }

    if ((err = snd_pcm_hw_params_set_access(*handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        alsa_error("cannot set access type", err);
        goto exit;
    }

    if ((err = snd_pcm_hw_params_set_format(*handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        alsa_error("cannot set sample format", err);
        goto exit;
    }

    if ((err = snd_pcm_hw_params_set_channels(*handle, hw_params, 1)) < 0) {
        alsa_error("cannot set channel count", err);
        goto exit;
    }

    unsigned int rate = SAMPLE_RATE;
    unsigned int exact_rate = rate;
    if ((err = snd_pcm_hw_params_set_rate_near(*handle, hw_params, &exact_rate, 0)) < 0) {
        alsa_error("cannot set sample rate", err);
    }

    if (rate != exact_rate) {
        printf("Error! asked for rate %d, got %d\n", rate, exact_rate);
        err = 1;
        goto exit;
    }

    snd_pcm_uframes_t min_period_size;
    int dir;
    if ((err = snd_pcm_hw_params_get_period_size_min(hw_params, &min_period_size, &dir)) < 0) {
        alsa_error("error getting minimum period size", err);
    }

    // Print the minimum period size
    printf("Minimum period size: %lu frames\n", min_period_size);

    snd_pcm_uframes_t period = PERIOD;
    snd_pcm_uframes_t exact_period = period;
    if ((err = snd_pcm_hw_params_set_period_size_near(*handle, hw_params, &exact_period, NULL)) < 0) {
        alsa_error("cannot set sample rate", err);
        goto exit;
    }
    printf("Asked for period %d, got %d\n", period, exact_period);

    snd_pcm_uframes_t buffer_size = period * PERIOD_BUFFER;
    snd_pcm_uframes_t exact_buffer_size = buffer_size;
    if ((err = snd_pcm_hw_params_set_buffer_size_near(*handle, hw_params, &exact_buffer_size)) < 0) {
        alsa_error("cannot set sample rate", err);
        goto exit;
    }
    printf("Asked for buffer size %d, got %d\n", buffer_size, exact_buffer_size);

    if ((err = snd_pcm_hw_params(*handle, hw_params)) < 0) {
        alsa_error("cannot set parameters", err);
        goto exit;
    }

    if ((err = snd_pcm_prepare(*handle)) < 0) {
        alsa_error("cannot prepare audio interface", err);
        goto exit;
    }

    // SW
    if ((err = snd_pcm_sw_params_malloc(&sw_params)) < 0) {
        alsa_error("cannot allocate hardware parameter structure", err);
        goto exit;
    }

    if ((err = snd_pcm_sw_params_current(*handle, sw_params)) < 0) {
        alsa_error("cannot initialize software parameter structure", err);
        goto exit;
    }

    if ((err = snd_pcm_sw_params_set_start_threshold(*handle, sw_params, INT_MAX)) < 0) {
        alsa_error("cannot set threshold", err);
        goto exit;
    }

    if ((err = snd_pcm_sw_params_set_avail_min(*handle, sw_params, 4)) < 0) {
        alsa_error("cannot set avail min", err);
        goto exit;
    }

    if ((err = snd_pcm_sw_params(*handle, sw_params)) < 0) {
        alsa_error("cannot set software params", err);
    }

    exit:
    snd_pcm_hw_params_free(hw_params);
    snd_pcm_sw_params_free(sw_params);
    return err;
}

int main(int argc, char *argv[]) {
    int ret;

    if (argc < 4) {
        printf("Usage: ./sound <mixer eg. hw:2> <alsa playback device (eg. plughw:2,0)> <alsa capture device>\n");
        return(EXIT_FAILURE);
    }

    int sv[2];  // Socket pair file descriptors

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == -1) {
        perror("socketpair");
        return(EXIT_FAILURE);
    }

    if (fork() == 0) {
        char arg[16];
        // child
        close(sv[0]);

        snprintf(arg, sizeof(arg), "%d", sv[1]);

        if (prctl(PR_SET_PDEATHSIG, SIGHUP) == -1) {
            perror("prctl");
            exit(EXIT_FAILURE);
        }

        // Verify that the parent is still alive
        if (getppid() == 1) {
            // Parent has already died, exit
            exit(EXIT_FAILURE);
        }
        /*
        ret = execl("/usr/bin/taskset", "taskset", "-c", "0",
            "/usr/bin/chrt", "1",
            "/usr/bin/qemu-i386-static", "qemu-i386-static",
            "./slmodemd/slmodemd", "-n", "-d0", "-f", arg,
            (char *)NULL);
        */
        ret = execl("/usr/bin/qemu-i386-static", "qemu-i386-static", "./slmodemd/slmodemd", "-n", "-d0", "-f", arg, NULL);
        if (ret == -1) {
            perror("execl");
            return(EXIT_FAILURE);
        }
    } else {
        // parent
    /*
        struct sched_param param;
        int max_priority, policy, ret;

        policy = SCHED_FIFO; // or SCHED_RR for round-robin scheduling

        // Get the maximum priority value for the selected policy
        max_priority = sched_get_priority_max(policy);
        if (max_priority == -1) {
            perror("sched_get_priority_max");
            exit(EXIT_FAILURE);
        }

        // Set the desired priority value (max_priority for highest priority)
        param.sched_priority = max_priority;

        // Set the scheduling policy and priority for the current process
        ret = sched_setscheduler(0, policy, &param);
        if (ret == -1) {
            perror("sched_setscheduler");
            exit(EXIT_FAILURE);
        }

    */
        int cap_count, play_count, err;
        snd_mixer_t *mixer_handle = NULL;
        snd_pcm_t *playback_handle = NULL;
        snd_pcm_t *capture_handle = NULL;

        if ((ret = setup_mixer(&mixer_handle, argv[1])) != 0) {
            goto exit;
        }
        if ((ret = setup_stream(&playback_handle, argv[2], SND_PCM_STREAM_PLAYBACK)) != 0) {
            goto exit;
        }
        if ((ret = setup_stream(&capture_handle, argv[3], SND_PCM_STREAM_CAPTURE)) != 0) {
            goto exit;
        }

        if ((cap_count = snd_pcm_poll_descriptors_count(capture_handle)) < 0) {
            alsa_error("cannot get descriptor count", cap_count);
            ret = cap_count;
            goto exit;
        }

        if ((play_count = snd_pcm_poll_descriptors_count(playback_handle)) < 0) {
            alsa_error("cannot get descriptor count", play_count);
            ret = play_count;
            goto exit;
        }

        struct pollfd *fds;
        struct pollfd out;

        fds = malloc(sizeof(struct pollfd) * (cap_count + play_count + 1));
        if (fds == NULL) {
            perror("malloc");
            goto exit;
        }

        if ((err = snd_pcm_poll_descriptors(capture_handle, fds, cap_count)) < 0) {
            alsa_error("cannot get poll descriptors", err);
        }

        if ((err = snd_pcm_poll_descriptors(playback_handle, fds + cap_count, play_count)) < 0) {
            alsa_error("cannot get poll descriptors", err);
        }

        fds[0].events = POLLIN;
        fds[1].events = 0;

        fds[cap_count + play_count].fd = sv[0];
        fds[cap_count + play_count].events = POLLIN;
        out.fd = sv[0];
        out.events = POLLOUT;
        int sum;

        snd_pcm_start(playback_handle);
        snd_pcm_start(capture_handle);

        char inbuf[BUFFER_SIZE];
        char outbuf[BUFFER_SIZE];
        size_t outbuf_count = 0;
        size_t outbuf_written = 0;
        size_t bytes_pending = 0;
        unsigned short revents;

        while (1) {
            if (poll(fds, cap_count + play_count + 1, -1) < 0) {
                perror("poll");
                goto exit;
            }

            // alsa side
            if (fds[0].revents & POLLIN) {
                int inbuf_count = snd_pcm_readi(capture_handle, inbuf, BUFFER_SIZE / 2);
                if (inbuf_count > 0) {
                    poll(&out, 1, 0);
                    if (out.revents & POLLOUT) {
                        if (ioctl(sv[1], FIONREAD, &bytes_pending) == -1) {
                            perror("ioctl");
                            goto exit;
                        }
                        if (bytes_pending < 96*12) {
                            int ret = write(sv[0], inbuf, inbuf_count * 2);
                            if (ret < inbuf_count * 2) {
                                printf("write failure\n");
                            }
                        } else {
                        }
                    } else {
                        printf("write would block\n");
                    }

                } else if (inbuf_count == -EPIPE) {
                    printf("EPIPE!#!#\n");
                } else {
                    printf("WEIRD!#!#\n");
                }
            }
            if (fds[0].revents & POLLERR) {
                // Buffer underrun or other error
                printf("buffer underrun or error\n");
            }

            if (outbuf_count > 0 && fds[1].revents & POLLOUT) {
                while (1) {
                    ret = snd_pcm_writei(playback_handle, outbuf + outbuf_written, outbuf_count);
                    if (ret == -EAGAIN) {
                        printf("EAGAIN\n");
                        break;
                    }
                    else if (ret == -EPIPE) {
                        //ret = alsa_xrun_recovery(dev);
                        printf("EPIPE WRITE\n");
                    }
                    else if (ret < 0) {
                        printf("uh-oh!\n");
                        exit(EXIT_FAILURE);
                    }
                    else if (ret > 0) {
                        outbuf_count -= ret;
                        outbuf_written += ret * 2;
                    }
                    if (outbuf_count <= 0) {
                        outbuf_written = 0;
                        outbuf_count = 0;
                        fds[1].events = 0;
                        fds[2].events = POLLIN;
                        break;
                    }
                }
            } 
            // socketpair side
            if (outbuf_count <= 0 && fds[2].revents & POLLIN) {
                outbuf_count = read(sv[0], outbuf, BUFFER_SIZE) / 2;
                outbuf_written = 0;
                fds[1].events = POLLOUT;
                fds[2].events = 0;
            }
        }

    exit:
        if (capture_handle != NULL) {
            snd_pcm_close(capture_handle);
        }
        if (playback_handle != NULL) {
            (playback_handle);
        }
        if (mixer_handle != NULL) {
            snd_mixer_close(mixer_handle);
        }
    }

    return ret;
}
