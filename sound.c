#include <alsa/asoundlib.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define BUFFER_SIZE 4096

void error_exit(const char *msg, int err) {
    fprintf(stderr, "%s (%s)\n", msg, snd_strerror(err));
    exit(EXIT_FAILURE);
}

int setup_mixer(snd_mixer_t **mixer, char *dev) {
    snd_mixer_selem_id_t *sid, *sidb;
    snd_mixer_elem_t *elem;
    long min, max;
    int err;

    if ((err = snd_mixer_open(mixer, 0)) < 0) {
        error_exit("cannot open mixer", err);
    }

    if ((err = snd_mixer_attach(*mixer, dev)) < 0) {
        snd_mixer_close(*mixer);
        error_exit("cannot attach to mixer", err);
    }

    if ((err = snd_mixer_selem_register(*mixer, NULL, NULL)) < 0) {
        snd_mixer_close(*mixer);
        error_exit("cannot register mixer element", err);
    }

    if ((err = snd_mixer_load(*mixer)) < 0) {
        snd_mixer_close(*mixer);
        error_exit("cannot load mixer elements", err);
    }

    for (elem = snd_mixer_first_elem(*mixer); elem; elem = snd_mixer_elem_next(elem)) {
        printf("Element: %s\n", snd_mixer_selem_get_name(elem));
    }

    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_name(sid, "Speaker");

    elem = snd_mixer_find_selem(*mixer, sid);
    if (!elem) {
        snd_mixer_close(*mixer);
        error_exit("could not find mixer element", 0);
    }

    snd_mixer_selem_get_playback_volume_range(elem, &min, &max);

    if ((err = snd_mixer_selem_set_playback_volume_all(elem, 45 * max / 100)) < 0) {
        snd_mixer_close(*mixer);
        error_exit("could not set volume", err);
    }

    snd_mixer_selem_id_alloca(&sidb);
    snd_mixer_selem_id_set_index(sidb, 0);
    snd_mixer_selem_id_set_name(sidb, "Mic");

    elem = snd_mixer_find_selem(*mixer, sidb);

    snd_mixer_selem_get_playback_volume_range(elem, &min, &max);

    if (!elem) {
        snd_mixer_close(*mixer);
        error_exit("could not find mixer element", 0);
    }

    if (((err = snd_mixer_selem_set_playback_volume_all(elem, 20 * max / 100) < 0) < 0) || (err = snd_mixer_selem_set_capture_volume_all(elem, 20 * max / 100)) < 0) {
        snd_mixer_close(*mixer);
        error_exit("could not set volume", err);
    }

    return 0;
}

int setup_stream(snd_pcm_t **handle, char *dev, int stream_type) {
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;
    snd_pcm_format_t format;
    char buf[4096];
    int err;

    if ((err = snd_pcm_open(handle, dev, stream_type, SND_PCM_NONBLOCK)) < 0) {
        error_exit("cannot open audio device", err);
    }

    // HW
    if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
        error_exit("cannot allocate hardware parameter structure", err);
    }

    if ((err = snd_pcm_hw_params_any(*handle, hw_params)) < 0) {
        error_exit("cannot initialize hardware parameter structure", err);
    }

    if ((err = snd_pcm_hw_params_set_access(*handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        error_exit("cannot set access type", err);
    }

    if ((err = snd_pcm_hw_params_set_format(*handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        error_exit("cannot set sample format", err);
    }

    if ((err = snd_pcm_hw_params_set_channels(*handle, hw_params, 1)) < 0) {
        error_exit("cannot set channel count", err);
    }

    unsigned int rate = 9600;
    unsigned int exact_rate = rate;
    if ((err = snd_pcm_hw_params_set_rate_near(*handle, hw_params, &exact_rate, 0)) < 0) {
        error_exit("cannot set sample rate", err);
    }
    printf("Asked for rate %d, got %d\n", rate, exact_rate);

    snd_pcm_uframes_t min_period_size;
    int dir;
    if ((err = snd_pcm_hw_params_get_period_size_min(hw_params, &min_period_size, &dir)) < 0) {
        error_exit("error getting minimum period size", err);
        return 1;
    }

    // Print the minimum period size
    printf("Minimum period size: %lu frames\n", min_period_size);

    snd_pcm_uframes_t period = 48;
    snd_pcm_uframes_t exact_period = period;
    if ((err = snd_pcm_hw_params_set_period_size_near(*handle, hw_params, &exact_period, NULL)) < 0) {
        error_exit("cannot set sample rate", err);
    }
    printf("Asked for period %d, got %d\n", period, exact_period);

    snd_pcm_uframes_t buffer_size = period * 12;
    snd_pcm_uframes_t exact_buffer_size = buffer_size;
    if ((err = snd_pcm_hw_params_set_buffer_size_near(*handle, hw_params, &exact_buffer_size)) < 0) {
        error_exit("cannot set sample rate", err);
    }
    printf("Asked for buffer size %d, got %d\n", buffer_size, exact_buffer_size);

    if ((err = snd_pcm_hw_params(*handle, hw_params)) < 0) {
        error_exit("cannot set parameters", err);
    }

    if ((err = snd_pcm_prepare(*handle)) < 0) {
        error_exit("cannot prepare audio interface", err);
    }

    snd_pcm_hw_params_free(hw_params);

    // SW
    if ((err = snd_pcm_sw_params_malloc(&sw_params)) < 0) {
        error_exit("cannot allocate hardware parameter structure", err);
    }

    if ((err = snd_pcm_sw_params_current(*handle, sw_params)) < 0) {
        error_exit("cannot initialize software parameter structure", err);
    }

    if ((err = snd_pcm_sw_params_set_start_threshold(*handle, sw_params, INT_MAX)) < 0) {
        error_exit("cannot set threshold", err);
    }

    if ((err = snd_pcm_sw_params_set_avail_min(*handle, sw_params, 4)) < 0) {
        error_exit("cannot set avail min", err);
    }

    if ((err = snd_pcm_sw_params(*handle, sw_params)) < 0) {
        error_exit("cannot set software params", err);
    }

    snd_pcm_sw_params_free(sw_params);

    if (stream_type == SND_PCM_STREAM_PLAYBACK) {
        if ((err = snd_pcm_format_set_silence(SND_PCM_FORMAT_S16_LE, buf, 384)) < 0) {
            error_exit("silence error", err);
        }

        err = snd_pcm_writei(*handle, buf, 384);
        if (err < 0) {
            error_exit("pcm write error", err);
        }
        printf("Delay: %d\n", err);
    }

    return 0;
}

typedef struct {
    snd_pcm_t *capture_handle;
    snd_pcm_t *playback_handle;
    int socket_fd;
} thread_data_t;

void *capture_thread(void *arg) {
    FILE *fin = fopen("/tmp/in.bin", "wb");
    if (fin == NULL) {
        perror("failed to open in.bin");
        exit(EXIT_FAILURE);
    }

    thread_data_t *data = (thread_data_t *)arg;
    char buffer[BUFFER_SIZE];
    struct pollfd ufds;
    int err, count;

    struct pollfd out;
    out.fd = data->socket_fd;
    out.events = POLLOUT;

    if ((count = snd_pcm_poll_descriptors_count(data->capture_handle)) < 0) {
        error_exit("cannot get descriptor count", count);
    }

    if ((err = snd_pcm_poll_descriptors(data->capture_handle, &ufds, count)) < 0) {
        error_exit("cannot get poll descriptors", err);
    }

    while (1) {
        if (poll(&ufds, count, -1) < 0) {
            perror("poll");
            exit(EXIT_FAILURE);
        }

        if (ufds.revents & POLLIN) {
            int len = snd_pcm_readi(data->capture_handle, buffer, BUFFER_SIZE / 2) * 2;

            if (len > 0) {
                poll(&out, 1, 0);
                if (out.revents & POLLOUT) {
                    int ret = write(data->socket_fd, buffer, len);
                    if (ret < len) {
                        printf("check this!\n");
                    }
                    //fwrite(buffer, sizeof(char), len, fin);
                }

            } else if (len == -EPIPE) {
                printf("EPIPE!#!#\n");
            } else {
                printf("WEIRD\n");
            }
        }
        if (ufds.revents & POLLERR) {
            // Buffer underrun or other error
            printf("Buffer underrun or error\n");
        }
    }
    fclose(fin);
    return NULL;
}

void *playback_thread(void *arg) {
    FILE *fout = fopen("/tmp/out.bin", "wb");
    if (fout == NULL) {
        perror("failed to open out.bin");
        exit(EXIT_FAILURE);
    }

    thread_data_t *data = (thread_data_t *)arg;
    char buffer[BUFFER_SIZE];
    struct pollfd out;
    int err;
    int ret, count, written = 0;

    out.fd = data->socket_fd;
    out.events = POLLIN;

    while (1) {
        if (poll(&out, 1, -1) < 0) {
            perror("poll");
            exit(EXIT_FAILURE);
        }

        if (out.revents & POLLIN) {
            count = read(data->socket_fd, buffer, BUFFER_SIZE) / 2;
            written = 0;

            while (count > 0) {
                ret = snd_pcm_writei(data->playback_handle, buffer + written, count);
                if (ret == -EAGAIN) {
                    usleep(1);
                    continue;
                }
                else if (ret == -EPIPE) {
                    //ret = alsa_xrun_recovery(dev);
                    printf("EPIPE WRTIE\n");
                }
                else {
                    count -= ret;
                    written += ret * 2;
                }
            }
        }
    }

    fclose(fout);
    return NULL;
}

int main(int argc, char *argv[]) {
    int ret;

    if (argc < 3) {
        printf("Usage: ./sound <mixer eg. hw:2> <alsa device (eg. plughw:2,0)>\n");
        exit(EXIT_FAILURE);
    }

    int sv[2];  // Socket pair file descriptors

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == -1) {
        perror("socketpair");
        exit(1);
    }

    if (fork() == 0) {
        char arg[16];
        // child
        close(sv[0]);

        snprintf(arg, sizeof(arg), "%d", sv[1]);
        /*
        ret = execl("/usr/bin/taskset", "taskset", "-c", "0",
            "/usr/bin/chrt", "1",
            "/usr/bin/qemu-i386-static", "qemu-i386-static",
            "./slmodemd/slmodemd", "-n", "-d0", "-f", arg,
            (char *)NULL);
        */
        ret = execl("/usr/bin/qemu-i386-static", "qemu-i386-static", "./slmodemd/slmodemd", "-n", "-d3", "-f", arg, NULL);
        if (ret == -1) {
            perror("execl");
            exit(EXIT_FAILURE);
        }
    } else {
        // parent
        close(sv[1]);

        int err;
        pthread_t capture_tid, playback_tid;
        thread_data_t data;
        snd_mixer_t *mixer_handle;
        struct sched_param capture_sched_param;
        struct sched_param playback_sched_param;

        printf("Setting up Mixer\n");
        setup_mixer(&mixer_handle, argv[1]);
        setup_stream(&data.capture_handle, argv[2], SND_PCM_STREAM_CAPTURE);
        setup_stream(&data.playback_handle, argv[2], SND_PCM_STREAM_PLAYBACK);

        data.socket_fd = sv[0];

        if ((err = pthread_create(&capture_tid, NULL, capture_thread, &data)) != 0) {
            perror("pthread_create for capture");
            goto exit;
        }
//        capture_sched_param.sched_priority = 85;  // sched_get_priority_max(SCHED_FIFO);
//        if (pthread_setschedparam(capture_tid, SCHED_FIFO, &capture_sched_param) != 0) {
 //           perror("Failed to set real-time scheduling");
  //          goto exit;
   //     }

        if ((err = pthread_create(&playback_tid, NULL, playback_thread, &data)) != 0) {
            perror("pthread_create for playback");
            goto exit;
        }
     //   playback_sched_param.sched_priority = 85;  // sched_get_priority_max(SCHED_FIFO);
      //  if (pthread_setschedparam(playback_tid, SCHED_FIFO, &playback_sched_param) != 0) {
     //       perror("Failed to set real-time scheduling");
    //        goto exit;
      //  }        

        snd_pcm_start(data.playback_handle);
        snd_pcm_start(data.capture_handle);
        pthread_join(capture_tid, NULL);
        pthread_join(playback_tid, NULL);

    exit:
        snd_pcm_close(data.capture_handle);
        snd_pcm_close(data.playback_handle);
        snd_mixer_close(mixer_handle);
    }

    return 0;
}
