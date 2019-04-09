#include <sensors/sensors.h>
#include <sensors/error.h>
#include <unistd.h>
#include <wait.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define NUM_TEMPERATURE_FEATURES 6
#define NUM_FAN_FEATURES 2
const struct timespec hot_delay = { 1, 0 };
const struct timespec cool_delay = { 0, 100 * 1000000 };

volatile int killed = 0;     /* ctrl-C was pressed and not yet handled */
volatile int hot_killed = 0; /* ctrl-C pressed while suspended */

typedef struct feature_s {
    const char* chip_name;
    const char* feature_name;
    const sensors_chip_name* chip;
    const sensors_feature* feature;
    int subfeature_i;
} feature_t;

feature_t temperature_features[NUM_TEMPERATURE_FEATURES] = {
    { "coretemp-isa-0000", "temp2" }, /* Core 0 */
    { "coretemp-isa-0000", "temp3" }, /* Core 1 */
    { "coretemp-isa-0000", "temp4" }, /* Core 2 */
    { "coretemp-isa-0000", "temp5" }, /* Core 3 */
    { "coretemp-isa-0000", "temp6" }, /* Core 4 */
    { "coretemp-isa-0000", "temp7" }  /* Core 5 */
};
feature_t fan_features[NUM_FAN_FEATURES] = {
    { "nct6776-isa-0290", "fan1" },
    { "nct6776-isa-0290", "fan2" }
};

void init_feature(feature_t* f, int feature_type) {
    int rc, sci, sfi;
    sensors_chip_name sc;
    const sensors_subfeature* sf;

    rc = sensors_parse_chip_name(f->chip_name, &sc);
    if (rc != 0) {
        fprintf(stderr, "Unable to parse chip name '%s' (%d): %s\n",
                f->chip_name, rc, sensors_strerror(rc));
        exit(-1);
    }

    sci = 0;
    while (1) {
        f->chip = sensors_get_detected_chips(&sc, &sci);
        if (f->chip == (sensors_chip_name*)NULL) {
            fprintf(stderr, "Failed to find feature '%s' on chip '%s'\n",
                    f->feature_name, f->chip_name);
            exit(-1);
        }
        sfi = 0;
        while (1) {
            f->feature = sensors_get_features(f->chip, &sfi);
            if (f->feature == (sensors_feature*)NULL) {
                break;
            }
            if (f->feature->name != (char*)NULL
                    && strcmp(f->feature_name, f->feature->name) == 0) {
                goto found_feature;
            }
        }
    }

  found_feature:
    sf = sensors_get_subfeature(f->chip, f->feature, feature_type);
    if (sf == (sensors_subfeature *)NULL) {
        fprintf(stderr, "Can't find input subfeature for %s:%s\n",
                f->chip_name, f->feature_name);
        exit(-1);
    }
    f->subfeature_i = sf->number;
    sensors_free_chip_name(&sc);
}

void handle_INT(int signum) {
    killed = 1;
    hot_killed = 1;
}

void init(void) {
    int rc, i;
    struct sigaction action;

    /* /usr/bin/sensors source passes NULL for default, I assume that's ok */
    rc = sensors_init((FILE*)NULL);
    if (rc != 0) {
        fprintf(stderr, "sensors_init() error (%d): %s\n",
                rc, sensors_strerror(rc));
        exit(-1);
    }
    for (i = 0; i < NUM_TEMPERATURE_FEATURES; ++i) {
        feature_t* f = &temperature_features[i];
        init_feature(f, SENSORS_SUBFEATURE_TEMP_INPUT);
    }
    for (i = 0; i < NUM_FAN_FEATURES; ++i) {
        feature_t* f = &fan_features[i];
        init_feature(f, SENSORS_SUBFEATURE_FAN_INPUT);
    }

    /* We must catch SIGINT so as to propagate it to the child */
    action.sa_handler = handle_INT;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, NULL);
}

void cleanup(void) {
    sensors_cleanup();
}

double detect_temp(void) {
    int i, rc;
    double value;
    double max = -1.0;

    for (i = 0; i < NUM_TEMPERATURE_FEATURES; ++i) {
        feature_t* f = &temperature_features[i];
        rc = sensors_get_value(f->chip, f->subfeature_i, &value);
        if (rc != 0) {
            fprintf(stderr, "Unable to read value for %s:%s (%d): %s\n",
                    f->chip_name, f->feature_name, rc, sensors_strerror(rc));
            exit(-1);
        }
        if (value > max) {
            max = value;
        }
    }
    return max;
}

void detect_fan(void) {
    int i, rc;
    double value;

    for (i = 0; i < NUM_FAN_FEATURES; ++i) {
        feature_t* f = &fan_features[i];
        rc = sensors_get_value(f->chip, f->subfeature_i, &value);
        if (rc != 0) {
            fprintf(stderr, "Unable to read value for %s:%s (%d): %s\n",
                    f->chip_name, f->feature_name, rc, sensors_strerror(rc));
            exit(-1);
        }
        printf("Got %s:%s = %.3f\n", f->chip_name, f->feature_name, value); 
    }
}

pid_t start_child(int argc, char** argv) {
    pid_t p = fork();
    /* set the child to its own process group in both branches: we want to
     * be sure that it is set before further action in either child or parent
     */
    if (p) {
        if (setpgid(p, 0) != 0) {
            int err = errno;
            /* complain only if the child hasn't already set it */
            if (getpgid(p) != p) {
                fprintf(stderr,
                    "Could not set pgrp for child %ld, errno %d (%s)",
                    (long)p, err, strerror(err)
                );
                exit(-1);
            }
        }
        return p;
    }
    /* I'm the child */
    setpgid(0, 0);
    execvp(argv[0], argv);
    fprintf(stderr, "Error running subprocess, errno %d (%s)\n",
            errno, strerror(errno));
    exit(-1);
}

void resume(pid_t child) {
    int rc = kill(-child, SIGCONT);
    if (rc != 0) {
        fprintf(stderr, "Tried to CONT pgrp %ld, errno %d\n", (long)child, rc);
    }
}

void suspend(pid_t child) {
    int rc = kill(-child, SIGSTOP);
    if (rc != 0) {
        fprintf(stderr, "Tried to STOP pgrp %ld, errno %d\n", (long)child, rc);
    }
}

/* try to kill the child, return TRUE if it has exited */
void kill_child(pid_t child) {
    int rc = kill(child, SIGKILL);
    if (rc != 0) {
        fprintf(stderr, "Tried to INT pid %ld, errno %d\n", (long)child, rc);
    }
}
    
int main(int argc, char** argv) {
    double t, cool_threshold, hot_threshold;
    pid_t child;
    int hot = 0, waited;
    siginfo_t si;

    if (argc < 4) {
        fprintf(stderr,
            "Usage: %s <hot_threshold> <cool_threshold> <prog> <args ...>\n",
            argv[0]
        );
        exit(-1);
    }
    hot_threshold = strtod(argv[1], (char**)NULL);
    if (hot_threshold > 90.0) {
        fprintf(stderr, "Hot threshold %f must not exceed 90\n", hot_threshold);
        exit(-1);
    }
    cool_threshold = strtod(argv[2], (char**)NULL);
    if (cool_threshold < 30.0) {
        fprintf(stderr,
                "Cool threshold %f must be at least 30\n", cool_threshold);
        exit(-1);
    }
    if (hot_threshold < cool_threshold) {
        fprintf(stderr, "Hot threshold must be more than cool threshold\n");
        exit(-1);
    }

    init();
    si.si_status = 0;
    child = start_child(argc - 3, &argv[3]);
    while (1) {
        t = detect_temp();
        if (hot) {
            if (hot_killed) {
                printf("173 Ctrl-C detected while suspended"
                        ", will kill child on resume\n");
                hot_killed = 0;
            }
            if (t < cool_threshold) {
                hot = 0;
                printf("172 Temperature down to %.0f, resuming pid %ld\n",
                        t, (long)child);
                resume(child);
            }
        } else {
            if (killed) {
                printf("174 Ctrl-C detected, killing child\n");
                killed = 0;
                kill_child(child);
            }
            waited = waitid(P_PID, child, &si, WEXITED | WNOHANG);
            if (waited != 0)
                break;
            if (t > hot_threshold) {
                hot = 1;
                printf("171 Temperature up to %.0f, suspending pid %ld\n",
                        t, (long)child);
                suspend(child);
            }
        }
        (void)nanosleep(
            hot ? &hot_delay : &cool_delay, (struct timespec *)NULL
        );
    }
    cleanup();
    return si.si_status;
}
