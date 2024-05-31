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
#define SAMPLE_FACTOR 10
#define SAMPLE_RATE 9600

void error_exit(const char *msg, int err) {
    fprintf(stderr, "%s (%s)\n", msg, snd_strerror(err));
    exit(EXIT_FAILURE);
}

#define SIN_AMPLITUDE 16384
#define SIN_PERIOD    (sizeof(sintab)/sizeof(sintab[0]))

const int16_t sintab[] = {
 0, 3196, 6269, 9102, 11585, 13622, 15136, 16069,
 16384, 16069, 15136, 13622, 11585, 9102, 6269, 3196,
 0, -3196, -6269, -9102, -11585, -13622, -15136, -16069,
 -16384, -16069, -15136, -13622, -11585, -9102, -6269, -3196};

struct sin_state {
	unsigned phase;
};

static void sin_run(struct sin_state *s, void *out, int cnt)
{
	//struct sin_state *s = dp->dp_data;
	int16_t *smpl = out;
	int i;
	//SIN_DBG("run...\n");
	for (i=0;i<cnt;i++) {
		smpl[i]  = sintab[s->phase%SIN_PERIOD];
		s->phase = s->phase+1;
	}
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

    if (((err = snd_mixer_selem_set_playback_volume_all(elem, 0 * max / 100) < 0) < 0) || (err = snd_mixer_selem_set_capture_volume_all(elem, 30 * max / 100)) < 0) {
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

    unsigned int rate = SAMPLE_RATE;
    unsigned int exact_rate = rate;
    if ((err = snd_pcm_hw_params_set_rate_near(*handle, hw_params, &exact_rate, 0)) < 0) {
        error_exit("cannot set sample rate", err);
    }

    if (rate != exact_rate) {
        printf("Error! asked for rate %d, got %d\n", rate, exact_rate);
        exit(EXIT_FAILURE);
    }

    snd_pcm_uframes_t min_period_size;
    int dir;
    if ((err = snd_pcm_hw_params_get_period_size_min(hw_params, &min_period_size, &dir)) < 0) {
        error_exit("error getting minimum period size", err);
    }

    // Print the minimum period size
    printf("Minimum period size: %lu frames\n", min_period_size);

    snd_pcm_uframes_t period = 48;
    snd_pcm_uframes_t exact_period = period;
    if ((err = snd_pcm_hw_params_set_period_size_near(*handle, hw_params, &exact_period, NULL)) < 0) {
        error_exit("cannot set sample rate", err);
    }
    printf("Asked for period %d, got %d\n", period, exact_period);

    snd_pcm_uframes_t buffer_size = period * 32;
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

    if ((err = snd_pcm_sw_params_set_avail_min(*handle, sw_params, 4)) < 0) {
        error_exit("cannot set avail min", err);
    }

    if ((err = snd_pcm_sw_params(*handle, sw_params)) < 0) {
        error_exit("cannot set software params", err);
    }

    return 0;
}


void alsa_error_handler(const char *file,
                        int line,
                        const char *function,
                        int err,
                        const char *fmt, ...)
{
    va_list arg;
    fprintf(stderr, "ALSA error: %s:%d (%s) %d: ", file, line, function, err);
    va_start(arg, fmt);
    vfprintf(stderr, fmt, arg);
    va_end(arg);
    fprintf(stderr, "\n");
}


int main(int argc, char *argv[]) {
    if (argc < 5) {
        printf("Usage: ./sound <mixer eg. hw:2> <alsa playback device (eg. plughw:2,0)> <alsa capture device>\n");
        exit(EXIT_FAILURE);
    }

    int ret;
    int cap_count, play_count, err;
    snd_mixer_t *mixer_handle;
    snd_pcm_t *playback_handle;
    snd_pcm_t *capture_handle;

    snd_lib_error_set_handler(alsa_error_handler);
    setup_mixer(&mixer_handle, argv[1]);
    setup_stream(&playback_handle, argv[2], SND_PCM_STREAM_PLAYBACK);
    setup_stream(&capture_handle, argv[3], SND_PCM_STREAM_CAPTURE);

    if ((cap_count = snd_pcm_poll_descriptors_count(capture_handle)) < 0) {
        error_exit("cannot get descriptor count", cap_count);
    }

    if ((play_count = snd_pcm_poll_descriptors_count(playback_handle)) < 0) {
        error_exit("cannot get descriptor count", play_count);
    }


    struct pollfd *fds;
    struct pollfd out;

    fds = malloc(sizeof(struct pollfd) * (cap_count + play_count + 1));
    if (fds == NULL) {
        perror("malloc");
        goto exit;
    }

    if ((err = snd_pcm_poll_descriptors(capture_handle, fds, cap_count)) < 0) {
        error_exit("cannot get poll descriptors", err);
    }

    if ((err = snd_pcm_poll_descriptors(playback_handle, fds + cap_count, play_count)) < 0) {
        error_exit("cannot get poll descriptors", err);
    }

    fds[0].events = POLLIN;
    fds[1].events = 0;

    snd_pcm_start(playback_handle);
    snd_pcm_start(capture_handle);

    char inbuf[BUFFER_SIZE];
    char outbuf[BUFFER_SIZE];
    size_t outbuf_count = 0;
    size_t outbuf_written = 0;
    unsigned short revents;
    int loopback = atoi(argv[4]);

    FILE *outf = fopen("/tmp/out.raw", "wb");
    FILE *inf = fopen("/tmp/in.raw", "wb");
    FILE *test = fopen("./test.raw", "rb");
    char *testbuf;
    int testsize = 0;
    int testoffset = 0;

    setenv("ALSA_VERBOSE", "1", 1);
    setenv("ALSA_DEBUG", "1", 1);
    // Seek to the end of the file to determine the file size
    fseek(test, 0, SEEK_END);
    testsize = ftell(test);
    fseek(test, 0, SEEK_SET);
    // Allocate memory for the buffer
    testbuf = (char *)malloc(testsize);
    if (!testbuf) {
        perror("Failed to allocate memory");
        fclose(test);
        return 1;
    }
    // Read the file into the buffer
    fread(testbuf, 1, testsize, test);
    // Close the file
    fclose(test);
    struct sin_state s;
    s.phase = 0;
    int skip = 25;

    while (1) {
        if (poll(fds, cap_count + play_count + 1, -1) < 0) {
            exit(EXIT_FAILURE);
        }

        // alsa side
        if (fds[0].revents & POLLIN) {
            int inbuf_count = snd_pcm_readi(capture_handle, inbuf, BUFFER_SIZE / 2);
            if (inbuf_count > 0) {
                fwrite(inbuf, sizeof(char), inbuf_count * 2, inf);
                fflush(inf);
                if (loopback == 1) {
                    snd_pcm_writei(playback_handle, inbuf, inbuf_count);
                    fwrite(inbuf, sizeof(char), inbuf_count * 2, outf);
                    fflush(outf);
                } else {
                    if (skip <= 0 && testoffset < testsize) {
                        //sin_run(&s, outbuf, inbuf_count);
                        if (testoffset < testsize) {
                            int sent = snd_pcm_writei(playback_handle, testbuf + testoffset, 48);
                            printf("sent: %d\n", sent);
                            fwrite(testbuf + testoffset, sizeof(char), sent * 2, outf);
                            fflush(outf);
                            testoffset += (sent * 2);
                        }
                    } else {
                        skip -= 1;
                        int sent = snd_pcm_writei(playback_handle, outbuf, inbuf_count);
                        printf("sent: %d\n", sent);
                        fwrite(outbuf, sizeof(char), inbuf_count * 2, outf);
                        fflush(outf);
                    }
                }

            } else if (inbuf_count == -EPIPE) {
                printf("EPIPE!#!#\n");
            } else {
                printf("WEIRD\n");
            }
        }
        if (fds[0].revents & POLLERR) {
            // Buffer underrun or other error
            printf("Buffer underrun or error\n");
        }

    }

exit:
    fclose(outf);
    fclose(inf);
    snd_pcm_close(capture_handle);
    snd_pcm_close(playback_handle);
    snd_mixer_close(mixer_handle);

    return 0;
}
