#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <alsa/asoundlib.h>
#include <sys/socket.h>

#define CAPTURE_DEVICE "plughw:2,0"
#define PLAYBACK_DEVICE "plughw:2,0"
#define BUFFER_SIZE 8192

void error_exit(const char *msg, int err) {
    fprintf(stderr, "%s (%s)\n", msg, snd_strerror(err));
    exit(EXIT_FAILURE);
}

int main() {
    int sv[2]; // Socket pair file descriptors

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == -1) {
        perror("socketpair");
        exit(1);
    }

    if (fork() == 0) {
        char arg[16];
        // child
        close(sv[0]);

        snprintf(arg, sizeof(arg), "%d", sv[1]);
        if (execl("/usr/bin/qemu-i386-static", "qemu-i386-static", "./slmodemd/slmodemd", "-n", "-d9", "-f", arg, NULL) == -1) {
            perror("execl");
            exit(EXIT_FAILURE);
        }
    } else {
        // parent
        sleep(1);
        close(sv[1]);

        snd_pcm_t *capture_handle;
        snd_pcm_t *playback_handle;
        snd_pcm_hw_params_t *cap_hw_params;
        snd_pcm_hw_params_t *play_hw_params;
        int err;

        if ((err = snd_pcm_open(&capture_handle, CAPTURE_DEVICE, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
            error_exit("cannot open capture audio device", err);
        }

        if ((err = snd_pcm_open(&playback_handle, PLAYBACK_DEVICE, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
            error_exit("cannot open playback audio device", err);
        }

        if ((err = snd_pcm_hw_params_malloc(&cap_hw_params)) < 0) {
            error_exit("cannot allocate hardware parameter structure", err);
        }

        if ((err = snd_pcm_hw_params_malloc(&play_hw_params)) < 0) {
            error_exit("cannot allocate hardware parameter structure", err);
        }

        // Capture device setup
        if ((err = snd_pcm_hw_params_any(capture_handle, cap_hw_params)) < 0) {
            error_exit("cannot initialize hardware parameter structure for capture", err);
        }

        if ((err = snd_pcm_hw_params_set_access(capture_handle, cap_hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
            error_exit("cannot set access type for capture", err);
        }

        if ((err = snd_pcm_hw_params_set_format(capture_handle, cap_hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
            error_exit("cannot set sample format for capture", err);
        }

        snd_pcm_nonblock(capture_handle, 1);

        unsigned int rate = 9600;
        unsigned int exact_rate = rate;
        if ((err = snd_pcm_hw_params_set_rate_near(capture_handle, cap_hw_params, &exact_rate, 0)) < 0) {
            error_exit("cannot set sample rate for capture", err);
        }

        printf("Asked for %d, got %d\n", rate, exact_rate);

        if ((err = snd_pcm_hw_params_set_channels(capture_handle, cap_hw_params, 1)) < 0) {
            error_exit("cannot set channel count for capture", err);
        }

        if ((err = snd_pcm_hw_params(capture_handle, cap_hw_params)) < 0) {
            error_exit("cannot set parameters for capture", err);
        }

        if ((err = snd_pcm_prepare(capture_handle)) < 0) {
            error_exit("cannot prepare audio interface for capture", err);
        }

        // Playback device setup
        if ((err = snd_pcm_hw_params_any(playback_handle, play_hw_params)) < 0) {
            error_exit("cannot initialize hardware parameter structure for playback", err);
        }

        if ((err = snd_pcm_hw_params_set_access(playback_handle, play_hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
            error_exit("cannot set access type for playback", err);
        }

        if ((err = snd_pcm_hw_params_set_format(playback_handle, play_hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
            error_exit("cannot set sample format for playback", err);
        }

        if ((err = snd_pcm_hw_params_set_rate_near(playback_handle, play_hw_params, &rate, 0)) < 0) {
            error_exit("cannot set sample rate for playback", err);
        }

        if ((err = snd_pcm_hw_params_set_channels(playback_handle, play_hw_params, 1)) < 0) {
            error_exit("cannot set channel count for playback", err);
        }

        if ((err = snd_pcm_hw_params(playback_handle, play_hw_params)) < 0) {
            error_exit("cannot set parameters for playback", err);
        }

        snd_pcm_hw_params_free(cap_hw_params);
        snd_pcm_hw_params_free(play_hw_params);

        if ((err = snd_pcm_prepare(playback_handle)) < 0) {
            error_exit("cannot prepare audio interface for playback", err);
        }

        snd_pcm_start(capture_handle);

        char buffer[BUFFER_SIZE];
        struct pollfd *ufds;
        struct pollfd out;
        int count;
        unsigned short revents;

        if((count = snd_pcm_poll_descriptors_count(capture_handle)) < 0) {
            error_exit("cannot get descriptor count", count);
        }
        ufds = malloc(sizeof(struct pollfd) * (count + 1));
        if (ufds == NULL) {
            perror("malloc");
        exit(EXIT_FAILURE);
        }

        if ((err = snd_pcm_poll_descriptors(capture_handle, ufds, count)) < 0) {
            error_exit("cannot get poll descriptors", err);
        }

        ufds[count].fd = sv[0];
        ufds[count].events = POLLIN;
        out.fd = sv[0];
        out.events = POLLOUT;

        while (1) {
            if (poll(ufds, count + 1, -1) < 0) {
            free(ufds);
                perror("select");
                exit(EXIT_FAILURE);
            }

            if (ufds[0].revents & POLLIN) {
                //snd_pcm_poll_descriptors_revents(capture_handle, ufds, count, &revents);
                int len = snd_pcm_readi(capture_handle, buffer, BUFFER_SIZE / 2); // Assuming 16-bit stereo (2 bytes * 2 channels)
                if (len > 0) {
                    //snd_pcm_writei(playback_handle, buffer, len); // Assuming 16-bit stereo (2 bytes * 2 channels)
                    poll(&out, 1, 0);
                    if (out.revents & POLLOUT) {
                        write(sv[0], buffer, len * 2);
                    } else {
                    }
                }
            }

            if (ufds[1].revents & POLLIN) {
                int len = read(sv[0], buffer, BUFFER_SIZE);
                if (len > 0) {
                    snd_pcm_writei(playback_handle, buffer, len / 2); // Assuming 16-bit stereo (2 bytes * 2 channels)
                }
            }
        }


        snd_pcm_close(capture_handle);
        snd_pcm_close(playback_handle);
        free(ufds);
    }

    return 0;
}
