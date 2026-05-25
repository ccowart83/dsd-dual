/*
 * rtl_sdr_fm2.cpp — second RTL-SDR dongle pipeline for dual-dongle EDACS operation.
 *
 * Dongle 1 (primary, rtl_sdr_fm.cpp): stays on the EDACS control channel.
 * Dongle 2 (this file): tuned to voice channels on grant; DSD-FME switches its
 * sample source to this pipeline when opts->rtl_vc_active == 1.
 *
 * All internal DSP helpers are static to avoid linker conflicts with rtl_sdr_fm.cpp.
 * Public API: open_rtlsdr_stream2, cleanup_rtlsdr_stream2, get_rtlsdr_sample2,
 *             rtl_dev_tune2, rtl_return_rms2, rtl_clean_queue2.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <queue>
#include <rtl-sdr.h>
#include "dsd.h"

#define DEFAULT_SAMPLE_RATE2    48000
#define DEFAULT_BUF_LENGTH2     (1 * 16384)
#define MAXIMUM_OVERSAMPLE2     16
#define MAXIMUM_BUF_LENGTH2     (MAXIMUM_OVERSAMPLE2 * DEFAULT_BUF_LENGTH2)
#define AUTO_GAIN2              -100
#define BUFFER_DUMP2            4096

static int lcm_post2[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};
static int ACTUAL_BUF_LENGTH2;

static int *atan_lut2 = NULL;
static int atan_lut_size2 = 131072;
static int atan_lut_coef2 = 8;

static int rtl_bandwidth2_hz;
static int bandwidth_multiplier2;
static int bandwidth_divisor2 = 48000;

#define safe_cond_signal2(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait2(n, m)   pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

struct dongle_state2
{
    int      exit_flag;
    pthread_t thread;
    rtlsdr_dev_t *dev;
    int      dev_index;
    uint32_t freq;
    uint32_t rate;
    int      gain;
    uint16_t buf16[MAXIMUM_BUF_LENGTH2];
    uint32_t buf_len;
    int      ppm_error;
    int      offset_tuning;
    int      direct_sampling;
    int      mute;
    struct demod_state2 *demod_target;
};

struct demod_state2
{
    int      exit_flag;
    pthread_t thread;
    int16_t  lowpassed[MAXIMUM_BUF_LENGTH2];
    int      lp_len;
    int16_t  lp_i_hist[10][6];
    int16_t  lp_q_hist[10][6];
    int16_t  result[MAXIMUM_BUF_LENGTH2];
    int16_t  droop_i_hist[9];
    int16_t  droop_q_hist[9];
    int      result_len;
    int      rate_in;
    int      rate_out;
    int      rate_out2;
    int      now_r, now_j;
    int      pre_r, pre_j;
    int      prev_index;
    int      downsample;
    int      post_downsample;
    int      output_scale;
    int      squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
    int      downsample_passes;
    int      comp_fir_size;
    int      custom_atan;
    int      deemph, deemph_a;
    int      now_lpr;
    int      prev_lpr_index;
    int      dc_block, dc_avg;
    void     (*mode_demod)(struct demod_state2*);
    pthread_rwlock_t rw;
    pthread_cond_t ready;
    pthread_mutex_t ready_m;
    struct output_state2 *output_target;
};

struct output_state2
{
    int      rate;
    std::queue<int16_t> queue;
    pthread_rwlock_t rw;
    pthread_cond_t ready;
    pthread_mutex_t ready_m;
};

static struct dongle_state2 dongle2;
static struct demod_state2  demod2;
static struct output_state2 output2;

/* ------------------------------------------------------------------ */
/* Pure DSP helpers — static to avoid duplicate-symbol linker errors   */
/* ------------------------------------------------------------------ */

static void rotate_90_2(unsigned char *buf, uint32_t len)
{
    uint32_t i;
    unsigned char tmp;
    for (i = 0; i < len; i += 8) {
        tmp = 255 - buf[i+3];
        buf[i+3] = buf[i+2];
        buf[i+2] = tmp;
        buf[i+4] = 255 - buf[i+4];
        buf[i+5] = 255 - buf[i+5];
        tmp = 255 - buf[i+6];
        buf[i+6] = buf[i+7];
        buf[i+7] = tmp;
    }
}

static void low_pass2(struct demod_state2 *d)
{
    int i = 0, i2 = 0;
    while (i < d->lp_len) {
        d->now_r += d->lowpassed[i];
        d->now_j += d->lowpassed[i+1];
        i += 2;
        d->prev_index++;
        if (d->prev_index < d->downsample) continue;
        d->lowpassed[i2]   = d->now_r;
        d->lowpassed[i2+1] = d->now_j;
        d->prev_index = 0;
        d->now_r = 0;
        d->now_j = 0;
        i2 += 2;
    }
    d->lp_len = i2;
}

static int low_pass_simple2(int16_t *signal2, int len, int step)
{
    int i, i2, sum;
    for (i = 0; i < len; i += step) {
        sum = 0;
        for (i2 = 0; i2 < step; i2++) sum += (int)signal2[i + i2];
        signal2[i/step] = (int16_t)(sum);
    }
    signal2[i/step + 1] = signal2[i/step];
    return len / step;
}

static void low_pass_real2(struct demod_state2 *s)
{
    int i = 0, i2 = 0;
    int fast = (int)s->rate_out;
    int slow = s->rate_out2;
    while (i < s->result_len) {
        s->now_lpr += s->result[i];
        i++;
        s->prev_lpr_index += slow;
        if (s->prev_lpr_index < fast) continue;
        s->result[i2] = (int16_t)(s->now_lpr / (fast/slow));
        s->prev_lpr_index -= fast;
        s->now_lpr = 0;
        i2++;
    }
    s->result_len = i2;
}

static void fifth_order2(int16_t *data, int length, int16_t *hist)
{
    int i;
    int16_t a, b, c, d, e, f;
    a = hist[1]; b = hist[2]; c = hist[3]; d = hist[4]; e = hist[5];
    f = data[0];
    data[0] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
    for (i = 4; i < length; i += 4) {
        a = c; b = d; c = e; d = f;
        e = data[i-2]; f = data[i];
        data[i/2] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
    }
    hist[0]=a; hist[1]=b; hist[2]=c; hist[3]=d; hist[4]=e; hist[5]=f;
}

#define CIC_TABLE_MAX2 10
static int cic_9_tables2[][10] = {
    {0,},
    {9, -156,  -97, 2798, -15489, 61019, -15489, 2798,  -97, -156},
    {9, -128, -568, 5593, -24125, 74126, -24125, 5593, -568, -128},
    {9, -129, -639, 6187, -26281, 77511, -26281, 6187, -639, -129},
    {9, -122, -612, 6082, -26353, 77818, -26353, 6082, -612, -122},
    {9, -120, -602, 6015, -26269, 77757, -26269, 6015, -602, -120},
    {9, -120, -582, 5951, -26128, 77542, -26128, 5951, -582, -120},
    {9, -119, -580, 5931, -26094, 77505, -26094, 5931, -580, -119},
    {9, -119, -578, 5921, -26077, 77484, -26077, 5921, -578, -119},
    {9, -119, -577, 5917, -26067, 77473, -26067, 5917, -577, -119},
    {9, -199, -362, 5303, -25505, 77489, -25505, 5303, -362, -199},
};

static void generic_fir2(int16_t *data, int length, int *fir, int16_t *hist)
{
    int d, temp, sum;
    for (d = 0; d < length; d += 2) {
        temp = data[d];
        sum  = 0;
        sum += (hist[0] + hist[8]) * fir[1];
        sum += (hist[1] + hist[7]) * fir[2];
        sum += (hist[2] + hist[6]) * fir[3];
        sum += (hist[3] + hist[5]) * fir[4];
        sum +=             hist[4] * fir[5];
        data[d] = sum >> 15;
        hist[0]=hist[1]; hist[1]=hist[2]; hist[2]=hist[3];
        hist[3]=hist[4]; hist[4]=hist[5]; hist[5]=hist[6];
        hist[6]=hist[7]; hist[7]=hist[8]; hist[8]=temp;
    }
}

static void multiply2(int ar, int aj, int br, int bj, int *cr, int *cj)
{
    *cr = ar*br - aj*bj;
    *cj = aj*br + ar*bj;
}

static int polar_discriminant2(int ar, int aj, int br, int bj)
{
    int cr, cj;
    double angle;
    multiply2(ar, aj, br, -bj, &cr, &cj);
    angle = atan2((double)cj, (double)cr);
    return (int)(angle / 3.14159 * (1<<14));
}

static int fast_atan2_2(int y, int x)
{
    int yabs, angle;
    int pi4 = (1<<12), pi34 = 3*(1<<12);
    if (x == 0 && y == 0) return 0;
    yabs = y;
    if (yabs < 0) yabs = -yabs;
    if (x >= 0)
        angle = pi4  - pi4 * (x-yabs) / (x+yabs);
    else
        angle = pi34 - pi4 * (x+yabs) / (yabs-x);
    return (y < 0) ? -angle : angle;
}

static int polar_disc_fast2(int ar, int aj, int br, int bj)
{
    int cr, cj;
    multiply2(ar, aj, br, -bj, &cr, &cj);
    return fast_atan2_2(cj, cr);
}

static int atan_lut_init2(void)
{
    int i;
    atan_lut2 = static_cast<int*>(malloc(atan_lut_size2 * sizeof(int)));
    for (i = 0; i < atan_lut_size2; i++)
        atan_lut2[i] = (int)(atan((double)i / (1<<atan_lut_coef2)) / 3.14159 * (1<<14));
    return 0;
}

static int polar_disc_lut2(int ar, int aj, int br, int bj)
{
    int cr, cj, x, x_abs;
    multiply2(ar, aj, br, -bj, &cr, &cj);
    if (cr == 0 || cj == 0) {
        if (cr == 0 && cj == 0)  return 0;
        if (cr == 0 && cj > 0)   return 1 << 13;
        if (cr == 0 && cj < 0)   return -(1 << 13);
        if (cj == 0 && cr > 0)   return 0;
        if (cj == 0 && cr < 0)   return 1 << 14;
    }
    x = (cj << atan_lut_coef2) / cr;
    x_abs = abs(x);
    if (x_abs >= atan_lut_size2)
        return (cj > 0) ? 1<<13 : -(1<<13);
    if (x > 0)
        return (cj > 0) ? atan_lut2[x] : atan_lut2[x] - (1<<14);
    else
        return (cj > 0) ? (1<<14) - atan_lut2[-x] : -atan_lut2[-x];
    return 0;
}

static void fm_demod2(struct demod_state2 *fm)
{
    int i, pcm;
    int16_t *lp = fm->lowpassed;
    pcm = polar_discriminant2(lp[0], lp[1], fm->pre_r, fm->pre_j);
    fm->result[0] = (int16_t)pcm;
    for (i = 2; i < (fm->lp_len-1); i += 2) {
        switch (fm->custom_atan) {
        case 0: pcm = polar_discriminant2(lp[i], lp[i+1], lp[i-2], lp[i-1]); break;
        case 1: pcm = polar_disc_fast2(lp[i], lp[i+1], lp[i-2], lp[i-1]);    break;
        case 2: pcm = polar_disc_lut2(lp[i], lp[i+1], lp[i-2], lp[i-1]);     break;
        }
        fm->result[i/2] = (int16_t)pcm;
    }
    fm->pre_r = lp[fm->lp_len - 2];
    fm->pre_j = lp[fm->lp_len - 1];
    fm->result_len = fm->lp_len / 2;
}

static void deemph_filter2(struct demod_state2 *fm)
{
    static int avg;
    int i, d;
    for (i = 0; i < fm->result_len; i++) {
        d = fm->result[i] - avg;
        if (d > 0) avg += (d + fm->deemph_a/2) / fm->deemph_a;
        else       avg += (d - fm->deemph_a/2) / fm->deemph_a;
        fm->result[i] = (int16_t)avg;
    }
}

static void dc_block_filter2(struct demod_state2 *fm)
{
    int i, avg;
    int64_t sum = 0;
    for (i = 0; i < fm->result_len; i++) sum += fm->result[i];
    avg = sum / fm->result_len;
    avg = (avg + fm->dc_avg * 9) / 10;
    for (i = 0; i < fm->result_len; i++) fm->result[i] -= avg;
    fm->dc_avg = avg;
}

static long int rms2(int16_t *samples, int len, int step)
{
    int i;
    long int rms_val;
    int64_t p = 0, t = 0;
    int64_t s;
    for (i = 0; i < len; i += step) {
        s = samples[i];
        t += s;
        p += s * s;
    }
    double dc  = (double)(t * (int64_t)step) / (double)len;
    double err = (double)t * 2.0 * dc - dc * dc * (double)len;
    double val = ((double)p - err) / (double)len;
    if (val < 0.0) val = 0.0;
    rms_val = (long int)sqrt(val);
    if (rms_val < 0) rms_val = 999;
    return rms_val;
}

static void full_demod2(struct demod_state2 *d)
{
    int i, ds_p;
    ds_p = d->downsample_passes;
    if (ds_p) {
        for (i = 0; i < ds_p; i++) {
            fifth_order2(d->lowpassed,   (d->lp_len >> i),     d->lp_i_hist[i]);
            fifth_order2(d->lowpassed+1, (d->lp_len >> i) - 1, d->lp_q_hist[i]);
        }
        d->lp_len = d->lp_len >> ds_p;
        if (d->comp_fir_size == 9 && ds_p <= CIC_TABLE_MAX2) {
            generic_fir2(d->lowpassed,   d->lp_len,     cic_9_tables2[ds_p], d->droop_i_hist);
            generic_fir2(d->lowpassed+1, d->lp_len - 1, cic_9_tables2[ds_p], d->droop_q_hist);
        }
    } else {
        low_pass2(d);
    }
    if (d->squelch_level) {
        int sr = (int)rms2(d->lowpassed, d->lp_len, 1);
        if (sr < d->squelch_level) {
            d->squelch_hits++;
            for (i = 0; i < d->lp_len; i++) d->lowpassed[i] = 0;
        } else {
            d->squelch_hits = 0;
        }
    }
    d->mode_demod(d);
    if (d->post_downsample > 1)
        d->result_len = low_pass_simple2(d->result, d->result_len, d->post_downsample);
    if (d->deemph)    deemph_filter2(d);
    if (d->dc_block)  dc_block_filter2(d);
    if (d->rate_out2 > 0) low_pass_real2(d);
}

/* ------------------------------------------------------------------ */
/* Thread callbacks                                                    */
/* ------------------------------------------------------------------ */

static void rtlsdr_callback2(unsigned char *buf, uint32_t len, void *ctx)
{
    int i;
    struct dongle_state2 *s = static_cast<dongle_state2*>(ctx);
    struct demod_state2  *d = s->demod_target;
    if (exitflag) return;
    if (!ctx)     return;
    if (s->mute) {
        for (i = 0; i < s->mute; i++) buf[i] = 127;
        s->mute = 0;
    }
    if (!s->offset_tuning) rotate_90_2(buf, len);
    for (i = 0; i < (int)len; i++) s->buf16[i] = (int16_t)buf[i] - 127;
    pthread_rwlock_wrlock(&d->rw);
    memcpy(d->lowpassed, s->buf16, 2*len);
    d->lp_len = len;
    pthread_rwlock_unlock(&d->rw);
    safe_cond_signal2(&d->ready, &d->ready_m);
}

static void *dongle_thread_fn2(void *arg)
{
    struct dongle_state2 *s = static_cast<dongle_state2*>(arg);
    rtlsdr_read_async(s->dev, rtlsdr_callback2, s, 0, s->buf_len);
    return 0;
}

static void *demod_thread_fn2(void *arg)
{
    struct demod_state2  *d = static_cast<demod_state2*>(arg);
    struct output_state2 *o = d->output_target;
    while (!exitflag) {
        safe_cond_wait2(&d->ready, &d->ready_m);
        pthread_rwlock_wrlock(&d->rw);
        full_demod2(d);
        pthread_rwlock_unlock(&d->rw);
        if (d->exit_flag) { exitflag = 1; }
        if (d->squelch_level && d->squelch_hits > d->conseq_squelch) {
            d->squelch_hits = d->conseq_squelch + 1;
            continue;
        }
        pthread_rwlock_wrlock(&o->rw);
        for (int i = 0; i < d->result_len; i++)
            for (int j = 0; j < bandwidth_multiplier2; j++)
                o->queue.push(d->result[i]);
        pthread_rwlock_unlock(&o->rw);
        safe_cond_signal2(&o->ready, &o->ready_m);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Setup helpers                                                       */
/* ------------------------------------------------------------------ */

static int nearest_gain2(rtlsdr_dev_t *dev, int target_gain)
{
    int i, r, err1, err2, count, nearest;
    int *gains;
    r = rtlsdr_set_tuner_gain_mode(dev, 1);
    if (r < 0) { fprintf(stderr, "WARNING: VC dongle failed to enable manual gain.\n"); return r; }
    count = rtlsdr_get_tuner_gains(dev, NULL);
    if (count <= 0) return 0;
    gains = static_cast<int*>(malloc(sizeof(int) * count));
    count = rtlsdr_get_tuner_gains(dev, gains);
    nearest = gains[0];
    for (i = 0; i < count; i++) {
        err1 = abs(target_gain - nearest);
        err2 = abs(target_gain - gains[i]);
        if (err2 < err1) nearest = gains[i];
    }
    free(gains);
    return nearest;
}

static void optimal_settings2(int freq, int rate)
{
    UNUSED(rate);
    int capture_freq, capture_rate;
    demod2.downsample = (1000000 / demod2.rate_in) + 1;
    if (demod2.downsample_passes) {
        demod2.downsample_passes = (int)log2(demod2.downsample) + 1;
        demod2.downsample = 1 << demod2.downsample_passes;
    }
    capture_freq = freq;
    capture_rate = demod2.downsample * demod2.rate_in;
    if (!dongle2.offset_tuning) capture_freq = freq + capture_rate / 4;
    demod2.output_scale = (1<<15) / (128 * demod2.downsample);
    if (demod2.output_scale < 1) demod2.output_scale = 1;
    if (demod2.mode_demod == &fm_demod2) demod2.output_scale = 1;
    dongle2.freq = (uint32_t)capture_freq;
    dongle2.rate = (uint32_t)capture_rate;
}

static void demod2_init_ro2(struct demod_state2 *s)
{
    s->rate_in = rtl_bandwidth2_hz;
    s->rate_out = rtl_bandwidth2_hz;
    s->squelch_level = 0;
    s->conseq_squelch = 10;
    s->terminate_on_squelch = 0;
    s->squelch_hits = 11;
    s->downsample_passes = 0;
    s->comp_fir_size = 0;
    s->prev_index = 0;
    s->post_downsample = 1;
    s->custom_atan = 0;
    s->deemph = 0;
    s->rate_out2 = rtl_bandwidth2_hz;
    s->mode_demod = &fm_demod2;
    s->pre_j = s->pre_r = s->now_r = s->now_j = 0;
    s->prev_lpr_index = 0;
    s->deemph_a = 0;
    s->now_lpr = 0;
    s->dc_block = 1;
    s->dc_avg = 0;
    pthread_rwlock_init(&s->rw, NULL);
    pthread_cond_init(&s->ready, NULL);
    pthread_mutex_init(&s->ready_m, NULL);
    s->output_target = &output2;
}

static void demod2_init_analog(struct demod_state2 *s)
{
    s->rate_in = rtl_bandwidth2_hz;
    s->rate_out = rtl_bandwidth2_hz;
    s->squelch_level = 0;
    s->conseq_squelch = 10;
    s->terminate_on_squelch = 0;
    s->squelch_hits = 11;
    s->downsample_passes = 1;
    s->comp_fir_size = 0;
    s->prev_index = 0;
    s->post_downsample = 1;
    s->custom_atan = 0;
    s->deemph = 1;
    s->rate_out2 = rtl_bandwidth2_hz;
    s->mode_demod = &fm_demod2;
    s->pre_j = s->pre_r = s->now_r = s->now_j = 0;
    s->prev_lpr_index = 0;
    s->deemph_a = 0;
    s->now_lpr = 0;
    s->dc_block = 1;
    s->dc_avg = 0;
    pthread_rwlock_init(&s->rw, NULL);
    pthread_cond_init(&s->ready, NULL);
    pthread_mutex_init(&s->ready_m, NULL);
    s->output_target = &output2;
}

static void demod2_init(struct demod_state2 *s)
{
    s->rate_in = rtl_bandwidth2_hz;
    s->rate_out = rtl_bandwidth2_hz;
    s->squelch_level = 0;
    s->conseq_squelch = 10;
    s->terminate_on_squelch = 0;
    s->squelch_hits = 11;
    s->downsample_passes = 0;
    s->comp_fir_size = 0;
    s->prev_index = 0;
    s->post_downsample = 1;
    s->custom_atan = 0;
    s->deemph = 0;
    s->rate_out2 = -1;
    s->mode_demod = &fm_demod2;
    s->pre_j = s->pre_r = s->now_r = s->now_j = 0;
    s->prev_lpr_index = 0;
    s->deemph_a = 0;
    s->now_lpr = 0;
    s->dc_block = 1;
    s->dc_avg = 0;
    pthread_rwlock_init(&s->rw, NULL);
    pthread_cond_init(&s->ready, NULL);
    pthread_mutex_init(&s->ready_m, NULL);
    s->output_target = &output2;
}

static void demod2_cleanup(struct demod_state2 *s)
{
    pthread_rwlock_destroy(&s->rw);
    pthread_cond_destroy(&s->ready);
    pthread_mutex_destroy(&s->ready_m);
}

static void output2_init(struct output_state2 *s)
{
    s->rate = rtl_bandwidth2_hz;
    pthread_rwlock_init(&s->rw, NULL);
    pthread_cond_init(&s->ready, NULL);
    pthread_mutex_init(&s->ready_m, NULL);
}

static void output2_cleanup(struct output_state2 *s)
{
    pthread_rwlock_destroy(&s->rw);
    pthread_cond_destroy(&s->ready);
    pthread_mutex_destroy(&s->ready_m);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void open_rtlsdr_stream2(dsd_opts *opts)
{
    int r;

    rtl_bandwidth2_hz      = opts->rtl_bandwidth2 * 1000;
    bandwidth_multiplier2  = bandwidth_divisor2 / rtl_bandwidth2_hz;

    /* zero out structs */
    /* Zero only the C-struct members (dongle2, demod2 are POD).
     * output2 contains a std::queue — never memset a C++ container. */
    memset(&dongle2, 0, sizeof(dongle2));
    memset(&demod2,  0, sizeof(demod2));
    rtl_clean_queue2();   /* empty any leftover samples if re-opened */
    output2.rate = 0;

    /* dongle defaults */
    dongle2.rate         = (uint32_t)rtl_bandwidth2_hz;
    dongle2.gain         = AUTO_GAIN2;
    dongle2.mute         = 0;
    dongle2.direct_sampling = 0;
    dongle2.offset_tuning   = 0;
    dongle2.demod_target    = &demod2;

    /* demod init — match the CC dongle's demod settings */
    if (opts->frame_p25p1 == 1 || opts->frame_p25p2 == 1 || opts->frame_provoice == 1)
        demod2_init_ro2(&demod2);
    else if (opts->analog_only == 1)
        demod2_init_analog(&demod2);
    else
        demod2_init(&demod2);

    output2_init(&output2);

    /* initial frequency — start on CC freq so dongle is warm when first grant arrives */
    uint32_t init_freq = (opts->rtlsdr_center_freq2 > 0)
                         ? opts->rtlsdr_center_freq2
                         : opts->rtlsdr_center_freq;

    if (opts->rtlsdr_ppm_error2 != 0)
        dongle2.ppm_error = opts->rtlsdr_ppm_error2;

    dongle2.dev_index = opts->rtl_dev_index2;

    fprintf(stderr, "VC dongle: device=%d init_freq=%u bandwidth=%d Hz gain=%d ppm=%d\n",
            dongle2.dev_index, init_freq, rtl_bandwidth2_hz,
            opts->rtl_gain_value2, opts->rtlsdr_ppm_error2);

    demod2.rate_in *= demod2.post_downsample;
    if (!output2.rate) output2.rate = demod2.rate_out;

    ACTUAL_BUF_LENGTH2 = lcm_post2[demod2.post_downsample] * DEFAULT_BUF_LENGTH2;

    r = rtlsdr_open(&dongle2.dev, (uint32_t)dongle2.dev_index);
    if (r < 0) {
        fprintf(stderr, "Failed to open VC dongle (device %d).\n", dongle2.dev_index);
        exit(1);
    }
    fprintf(stderr, "VC dongle opened: device index %d\n", dongle2.dev_index);

    if (demod2.deemph)
        demod2.deemph_a = (int)round(1.0 / (1.0 - exp(-1.0 / (demod2.rate_out * 75e-6))));

    if (opts->rtl_gain_value2 > 0) {
        dongle2.gain = opts->rtl_gain_value2 * 10;
        dongle2.gain = nearest_gain2(dongle2.dev, dongle2.gain);
        rtlsdr_set_tuner_gain_mode(dongle2.dev, 1);
        rtlsdr_set_tuner_gain(dongle2.dev, dongle2.gain);
        fprintf(stderr, "VC dongle gain set to %.2f dB\n", dongle2.gain / 10.0);
    } else {
        rtlsdr_set_tuner_gain_mode(dongle2.dev, 0);
        fprintf(stderr, "VC dongle: auto gain\n");
    }

    rtlsdr_set_freq_correction(dongle2.dev, dongle2.ppm_error);

    /* tune to initial frequency */
    optimal_settings2((int)init_freq, demod2.rate_in);
    rtlsdr_set_center_freq(dongle2.dev, dongle2.freq);
    rtlsdr_set_sample_rate(dongle2.dev, dongle2.rate);
    rtlsdr_reset_buffer(dongle2.dev);

    /* start threads */
    pthread_create(&demod2.thread,  NULL, demod_thread_fn2,  (void*)(&demod2));
    pthread_create(&dongle2.thread, NULL, dongle_thread_fn2, (void*)(&dongle2));

    /* initialize LUT if not done yet */
    if (!atan_lut2) atan_lut_init2();
}

void cleanup_rtlsdr_stream2(void)
{
    fprintf(stderr, "VC dongle: cleaning up...\n");
    rtlsdr_cancel_async(dongle2.dev);
    pthread_join(dongle2.thread, NULL);
    safe_cond_signal2(&demod2.ready, &demod2.ready_m);
    pthread_join(demod2.thread, NULL);
    safe_cond_signal2(&output2.ready, &output2.ready_m);

    demod2_cleanup(&demod2);
    output2_cleanup(&output2);

    rtlsdr_close(dongle2.dev);

    if (atan_lut2) { free(atan_lut2); atan_lut2 = NULL; }
}

int get_rtlsdr_sample2(int16_t *sample, dsd_opts *opts, dsd_state *state)
{
    UNUSED(opts);
    UNUSED(state);

    for (;;)
    {
        /* Read-lock guards empty() — callback pushes under write lock, so calling
           empty() without a lock is a data race that corrupts std::deque internals. */
        pthread_rwlock_rdlock(&output2.rw);
        bool is_empty = output2.queue.empty();
        pthread_rwlock_unlock(&output2.rw);

        while (is_empty)
        {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += (long)(10e6);
            pthread_mutex_lock(&output2.ready_m);
            pthread_cond_timedwait(&output2.ready, &output2.ready_m, &ts);
            pthread_mutex_unlock(&output2.ready_m);
            if (exitflag) return -1;

            pthread_rwlock_rdlock(&output2.rw);
            is_empty = output2.queue.empty();
            pthread_rwlock_unlock(&output2.rw);
        }

        pthread_rwlock_wrlock(&output2.rw);
        if (!output2.queue.empty()) /* re-check: rtl_dev_tune2 may have cleared queue */
        {
            *sample = output2.queue.front();
            output2.queue.pop();
            pthread_rwlock_unlock(&output2.rw);
            return 0;
        }
        pthread_rwlock_unlock(&output2.rw);
    }
}

/* Tune the VC dongle and mark the main loop to read from it. */
void rtl_dev_tune2(dsd_opts *opts, long int frequency)
{
    int r;
    if (opts->payload == 1)
        fprintf(stderr, "\nVC dongle tuning to %ld Hz.", frequency);

    opts->rtlsdr_center_freq2 = (uint32_t)frequency;
    optimal_settings2((int)frequency, demod2.rate_in);

    if (opts->payload == 1)
        fprintf(stderr, " (Center: %u Hz.)\n", dongle2.freq);

    r = rtlsdr_set_center_freq(dongle2.dev, dongle2.freq);
    if (r < 0)
        fprintf(stderr, " WARNING: VC dongle failed to set center freq %u\n", dongle2.freq);

    /* flush stale samples so decoder sees the new channel immediately */
    pthread_rwlock_wrlock(&output2.rw);
    rtl_clean_queue2();
    pthread_rwlock_unlock(&output2.rw);

    opts->rtl_vc_active = 1;
}

long int rtl_return_rms2(void)
{
    static long int prev_rms2 = 0;
    long int sr = rms2(demod2.lowpassed, 512, 1);
    sr = (prev_rms2 * 3 + sr) / 4;
    prev_rms2 = sr;
    return sr;
}

void rtl_clean_queue2(void)
{
    std::queue<int16_t> empty;
    std::swap(output2.queue, empty);
}
