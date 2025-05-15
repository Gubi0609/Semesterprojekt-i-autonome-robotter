#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico_lfs.h"
#include "lfs.h"

// ——— CONFIG ———
#define BTN1            9       // close grip
#define BTN2           12       // open grip
#define ADC_PIN        27
#define SAMPLE_DT       5       // ms between samples
#define MAX_SAMPLES   200       // ~1 s @ 5 ms
#define ADC_THRESHOLD 920       // stop condition

// ——— SAMPLE BUFFER ———
typedef struct {
    uint32_t t_ms;
    uint16_t raw;
} sample_t;
static sample_t samples[MAX_SAMPLES];
static size_t    sample_count;

// ——— LittleFS objects ———
static struct lfs_config *lfs_cfg;
static lfs_t lfs;
static lfs_file_t logfile;

// initialize flash FS
void fs_init() {
    lfs_cfg = pico_lfs_init(0x10100000, 0x00100000);
    if (lfs_mount(&lfs, lfs_cfg) != LFS_ERR_OK) {
        lfs_format(&lfs, lfs_cfg);
        lfs_mount(&lfs, lfs_cfg);
    }
    lfs_file_open(&lfs, &logfile, "log.csv",
                  LFS_O_CREAT | LFS_O_APPEND | LFS_O_WRONLY);
}

// dump the in‐RAM samples to flash
void dump_samples() {
    char buf[32];
    for (size_t i = 0; i < sample_count; ++i) {
        int n = snprintf(buf, sizeof(buf), "%u,%u\n",
                         samples[i].t_ms, samples[i].raw);
        lfs_file_write(&logfile, buf, n);
    }
    lfs_file_sync(&logfile);
}

// record one cycle up to max_duration_ms, but stop early if ADC ≥ threshold
void record_cycle(uint32_t max_duration_ms, int pwm_chan, uint16_t pwm_level) {
    // start motion
    pwm_set_chan_level(pwm_gpio_to_slice_num(0), pwm_chan, pwm_level);
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    sample_count = 0;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        // read ADC once per iteration
        uint16_t raw = adc_read();

        // break if either time is up, buffer is full, or threshold reached
        if (now - t0 >= max_duration_ms ||
            sample_count >= MAX_SAMPLES ||
            raw >= ADC_THRESHOLD) {
            // record this last sample if it crossed threshold
            samples[sample_count++] = (sample_t){ .t_ms = now - t0, .raw = raw };
            break;
        }

        // otherwise, store sample
        samples[sample_count++] = (sample_t){ .t_ms = now - t0, .raw = raw };
        sleep_ms(SAMPLE_DT);
    }

    // stop motion and dump
    pwm_set_chan_level(pwm_gpio_to_slice_num(0), pwm_chan, 0);
    dump_samples();
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_PIN - 26);

    // buttons
    for (int b : (int[]){BTN1, BTN2, -1}) {
        if (b < 0) break;
        gpio_init(b);
        gpio_set_dir(b, GPIO_IN);
        gpio_pull_down(b);
    }

    // PWM setup
    const uint PWM_A = 0, PWM_B = 1;
    gpio_set_function(PWM_A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_B, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);
    pwm_config_set_wrap(&cfg, 999);
    pwm_init(pwm_gpio_to_slice_num(PWM_A), &cfg, true);

    // mount FS + open file
    fs_init();
    // write header if file was just created
    lfs_file_write(&logfile, "t_ms,adc_raw\n", 14);
    lfs_file_sync(&logfile);

    while (true) {
        if (gpio_get(BTN1)) {
            // close-grip: record up to 700 ms, but stop early on threshold
            record_cycle(700, PWM_CHAN_B, 200);
        }
        else if (gpio_get(BTN2)) {
            // open-grip: record up to 350 ms, but stop early on threshold
            record_cycle(350, PWM_CHAN_A, 200);
        }
        else {
            sleep_ms(10);
        }
    }

    // (never reached) cleanup
    lfs_file_close(&logfile);
    lfs_unmount(&lfs);
    return 0;
}
