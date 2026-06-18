/** @file
    Pulse detection functions, FSK pulse detector.

    Copyright (C) 2015 Tommy Vestermark
    Copyright (C) 2019 Benjamin Larsson.
    Copyright (C) 2022 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "pulse_detect_fsk.h"
#include "c_util.h" // for MIN(), MAX()
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FSK adaptive frequency estimator constants
#define FSK_DEFAULT_FM_DELTA 6000       // Default estimate for frequency delta
#define FSK_EST_SLOW        64          // Constant for slowness of FSK estimators
#define FSK_EST_FAST        16          // Constant for slowness of FSK estimators

// FSK min/max slicer robustness constants
#define FSK_MINMAX_SLEW_STEP       350     // Max rail movement per sample; bounds how far an isolated noise excursion can drag the window
#define FSK_MINMAX_MIN_PULSE       3       // Pulses shorter than this many samples are rewound as glitches
#define FSK_MINMAX_MEDIAN_MIN_RATE 500000  // Apply the FM median glitch filter at/above this sample rate (Hz)

void pulse_detect_fsk_init(pulse_detect_fsk_t *s)
{
    *s              = (pulse_detect_fsk_t){0};
    s->var_test_max = INT16_MIN;
    s->var_test_min = INT16_MAX;
    s->skip_samples = 40;
}

void pulse_detect_fsk_classic(pulse_detect_fsk_t *s, int16_t fm_n, pulse_data_t *fsk_pulses)
{
    int const fm_f1_delta = abs(fm_n - s->fm_f1_est); // Get delta from F1 frequency estimate
    int const fm_f2_delta = abs(fm_n - s->fm_f2_est); // Get delta from F2 frequency estimate
    s->fsk_pulse_length += 1;

    switch(s->fsk_state) {
        case PD_FSK_STATE_INIT:        // Initial frequency - High or low?
            // Initial samples?
            if (s->fsk_pulse_length < PD_MIN_PULSE_SAMPLES) {
                s->fm_f1_est = s->fm_f1_est/2 + fm_n/2;        // Quick initial estimator
            }
            // Above default frequency delta?
            else if (fm_f1_delta > (FSK_DEFAULT_FM_DELTA/2)) {
                // Positive frequency delta - Initial frequency was low (gap)
                if (fm_n > s->fm_f1_est) {
                    s->fsk_state = PD_FSK_STATE_FH;
                    s->fm_f2_est = s->fm_f1_est;    // Switch estimates
                    s->fm_f1_est = fm_n;            // Prime F1 estimate
                    fsk_pulses->pulse[0] = 0;        // Initial frequency was a gap...
                    fsk_pulses->gap[0] = s->fsk_pulse_length;        // Store gap width
                    fsk_pulses->num_pulses += 1;
                    s->fsk_pulse_length = 0;
                }
                // Negative Frequency delta - Initial frequency was high (pulse)
                else {
                    s->fsk_state = PD_FSK_STATE_FL;
                    s->fm_f2_est = fm_n;    // Prime F2 estimate
                    fsk_pulses->pulse[0] = s->fsk_pulse_length;    // Store pulse width
                    s->fsk_pulse_length = 0;
                }
            }
            // Still below threshold
            else {
                s->fm_f1_est += fm_n/FSK_EST_FAST - s->fm_f1_est/FSK_EST_FAST;    // Fast estimator
            }
            break;
        case PD_FSK_STATE_FH:        // Pulse high at F1 frequency
            // Closer to F2 than F1?
            if (fm_f1_delta > fm_f2_delta) {
                s->fsk_state = PD_FSK_STATE_FL;
                // Store if pulse is not too short (suppress spurious)
                if (s->fsk_pulse_length >= PD_MIN_PULSE_SAMPLES) {
                    fsk_pulses->pulse[fsk_pulses->num_pulses] = s->fsk_pulse_length;    // Store pulse width
                    s->fsk_pulse_length = 0;
                }
                // Else rewind to last gap
                else {
                    s->fsk_pulse_length += fsk_pulses->gap[fsk_pulses->num_pulses-1];    // Restore counter
                    fsk_pulses->num_pulses -= 1;        // Rewind one pulse
                    // Are we back to initial frequency? (Was initial frequency a gap?)
                    if ((fsk_pulses->num_pulses == 0) && (fsk_pulses->pulse[0] == 0)) {
                        s->fm_f1_est = s->fm_f2_est;    // Switch back estimates
                        s->fsk_state = PD_FSK_STATE_INIT;
                    }
                }
            }
            // Still below threshold
            else {
                if (fm_n > s->fm_f1_est) {
                    s->fm_f1_est += fm_n/FSK_EST_FAST - s->fm_f1_est/FSK_EST_FAST;    // Fast estimator
                } else {
                    s->fm_f1_est += fm_n/FSK_EST_SLOW - s->fm_f1_est/FSK_EST_SLOW;    // Slow estimator
                }
            }
            break;
        case PD_FSK_STATE_FL:        // Pulse gap at F2 frequency
            // Freq closer to F1 than F2 ?
            if (fm_f2_delta > fm_f1_delta) {
                s->fsk_state = PD_FSK_STATE_FH;
                // Store if pulse is not too short (suppress spurious)
                if (s->fsk_pulse_length >= PD_MIN_PULSE_SAMPLES) {
                    fsk_pulses->gap[fsk_pulses->num_pulses] = s->fsk_pulse_length;    // Store gap width
                    fsk_pulses->num_pulses += 1;    // Go to next pulse
                    s->fsk_pulse_length = 0;
                    // When pulse buffer is full go to error state
                    if (fsk_pulses->num_pulses >= PD_MAX_PULSES) {
                        //fprintf(stderr, "pulse_detect_fsk_classic(): Maximum number of pulses reached!\n");
                        //s->fsk_state = PD_FSK_STATE_ERROR;
                        // TODO: workaround, specifically for the Inkbird-ITH20R: free some of the buffer
                        pulse_data_shift(fsk_pulses);
                    }
                }
                // Else rewind to last pulse
                else {
                    s->fsk_pulse_length += fsk_pulses->pulse[fsk_pulses->num_pulses];    // Restore counter
                    // Are we back to initial frequency?
                    if (fsk_pulses->num_pulses == 0) {
                        s->fsk_state = PD_FSK_STATE_INIT;
                    }
                }
            }
            // Still below threshold
            else {
                if (fm_n < s->fm_f2_est) {
                    s->fm_f2_est += fm_n/FSK_EST_FAST - s->fm_f2_est/FSK_EST_FAST;    // Fast estimator
                } else {
                    s->fm_f2_est += fm_n/FSK_EST_SLOW - s->fm_f2_est/FSK_EST_SLOW;    // Slow estimator
                }
            }
            break;
        case PD_FSK_STATE_ERROR:        // Stay here until cleared
            break;
        default:
            fprintf(stderr, "pulse_detect_fsk_classic(): Unknown FSK state!!\n");
            s->fsk_state = PD_FSK_STATE_ERROR;
    } // switch(s->fsk_state)
}

void pulse_detect_fsk_wrap_up(pulse_detect_fsk_t *s, pulse_data_t *fsk_pulses)
{
    if (fsk_pulses->num_pulses < PD_MAX_PULSES) { // Avoid overflow
        s->fsk_pulse_length += 1;
        if (s->fsk_state == PD_FSK_STATE_FH) {
            fsk_pulses->pulse[fsk_pulses->num_pulses] = s->fsk_pulse_length; // Store last pulse
            fsk_pulses->gap[fsk_pulses->num_pulses]   = 0;                   // Zero gap at end
        }
        else {
            fsk_pulses->gap[fsk_pulses->num_pulses] = s->fsk_pulse_length; // Store last gap
        }
        fsk_pulses->num_pulses += 1;
    }
}

/// Median of three values; used as a glitch filter on the FM stream.
static int16_t median3(int16_t a, int16_t b, int16_t c)
{
    if (a > b) { int16_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;
}

void pulse_detect_fsk_minmax(pulse_detect_fsk_t *s, int16_t fm_n, pulse_data_t *fsk_pulses)
{
    int16_t mid = 0;

    // At high sample rates the FM discriminator output carries single-sample
    // noise glitches (a bit spans many samples, and the FM low-pass is wide
    // relative to the bitrate). On a weak signal these glitches cross the
    // slicing band and shatter the bit framing into sub-bit chatter. A
    // 3-sample median removes them without the state-machine side effects of
    // rewinding committed pulses. It is a sample-domain filter for a
    // sample-domain artifact, so it is gated on the sample rate: at low rates
    // a bit is only a handful of samples wide and the median's one-sample edge
    // shifts distort real pulses instead.
    unsigned int min_pulse = FSK_MINMAX_MIN_PULSE;
    if (fsk_pulses->sample_rate < FSK_MINMAX_MEDIAN_MIN_RATE) {
        // Below the median gate the FM stream keeps its single-sample glitches,
        // so the rewind must cover slightly wider chatter. Real pulses at these
        // rates are no narrower than ~7 samples, so 4 is still safely sub-bit.
        min_pulse = 4;
    }
    if (fsk_pulses->sample_rate >= FSK_MINMAX_MEDIAN_MIN_RATE) {
        int16_t raw = fm_n;
        fm_n         = median3(s->fm_hist[1], s->fm_hist[0], raw);
        s->fm_hist[1] = s->fm_hist[0];
        s->fm_hist[0] = raw;
    }

    /* Skip a few samples in the beginning, need for framing
     * otherwise the min/max trackers won't converge properly
     */
    if (!s->skip_samples) {
        // Running min/max of the FSK deviation over the package (reset per
        // package in pulse_detect_fsk_init()), tracked without decay (a fixed
        // per-sample step toward mid is not scaled to the sample rate and, at
        // high rates, collapses the window faster than one bit lasts). The raw
        // running min/max is unsafe, though: the FM discriminator emits
        // full-scale random output whenever the carrier fades or at burst
        // edges, and a single such excursion would pin a rail far outside the
        // true FSK deviation, dragging mid off-centre and inflating
        // hyst=(max-min)/8 toward the whole mark-space separation -- burying
        // one FSK level in the dead band and shattering the bit framing on
        // weak signals. Noise magnitude overlaps real wide-deviation signals,
        // so the rails are slew-rate limited instead of clamped: a rail moves
        // toward a new extreme by at most a bounded step per sample. A
        // sustained real level walks the rail out within a fraction of a bit,
        // while an isolated carrier-fade excursion moves it only a bounded
        // amount and the window stays near the true deviation.
        if (s->var_test_max == INT16_MIN) {
            s->var_test_max = fm_n; // First tracked sample seeds the window
        }
        if (s->var_test_min == INT16_MAX) {
            s->var_test_min = fm_n;
        }
        if (fm_n > s->var_test_max) {
            int step = fm_n - s->var_test_max;
            s->var_test_max += (step > FSK_MINMAX_SLEW_STEP) ? FSK_MINMAX_SLEW_STEP : step;
        }
        if (fm_n < s->var_test_min) {
            int step = s->var_test_min - fm_n;
            s->var_test_min -= (step > FSK_MINMAX_SLEW_STEP) ? FSK_MINMAX_SLEW_STEP : step;
        }
        mid = (s->var_test_max + s->var_test_min) / 2;
        // Hysteresis band around the slicing level, proportional to the tracked
        // FSK deviation, so a noisy weak signal doesn't produce spurious
        // mid-crossings that shatter the bit framing. Self-scales per device.
        int16_t const hyst = (s->var_test_max - s->var_test_min) / 8;

        s->fsk_pulse_length += 1;
        switch(s->fsk_state) {
            case PD_FSK_STATE_INIT:
                if (fm_n > mid) {
                    s->fsk_state = PD_FSK_STATE_FH;
                }
                if (fm_n <= mid) {
                    s->fsk_state = PD_FSK_STATE_FL;
                }
                break;
            case PD_FSK_STATE_FH:
                if (fm_n < mid - hyst) {
                    s->fsk_state = PD_FSK_STATE_FL;
                    // Suppress spurious sub-bit pulses, mirroring the filter in
                    // pulse_detect_fsk_classic() but with a tighter bound: only
                    // glitches that survived the median (2 samples or less) are
                    // rewound into the surrounding gap. A larger bound corrupts
                    // devices whose real pulses are only a few samples wide.
                    if (s->fsk_pulse_length >= min_pulse) {
                        fsk_pulses->pulse[fsk_pulses->num_pulses] = s->fsk_pulse_length;
                        s->fsk_pulse_length = 0;
                    }
                    else if (fsk_pulses->num_pulses > 0) {
                        s->fsk_pulse_length += fsk_pulses->gap[fsk_pulses->num_pulses - 1]; // Restore counter
                        fsk_pulses->num_pulses -= 1;                                        // Rewind one pulse
                    }
                }
                s->fm_f2_est += fm_n / FSK_EST_SLOW - s->fm_f2_est / FSK_EST_SLOW; // Slow estimator
                break;
            case PD_FSK_STATE_FL:
                if (fm_n > mid + hyst) {
                    s->fsk_state = PD_FSK_STATE_FH;
                    // Symmetric spurious-pulse suppression (see PD_FSK_STATE_FH)
                    if (s->fsk_pulse_length >= min_pulse) {
                        fsk_pulses->gap[fsk_pulses->num_pulses] = s->fsk_pulse_length;
                        fsk_pulses->num_pulses += 1;
                        s->fsk_pulse_length = 0;
                        // When pulse buffer is full go to error state
                        if (fsk_pulses->num_pulses >= PD_MAX_PULSES) {
                            //fprintf(stderr, "pulse_detect_fsk_minmax(): Maximum number of pulses reached!\n");
                            //s->fsk_state = PD_FSK_STATE_ERROR;
                            // TODO: workaround, specifically for the Inkbird-ITH20R: free some of the buffer
                            pulse_data_shift(fsk_pulses);
                        }
                    }
                    else {
                        s->fsk_pulse_length += fsk_pulses->pulse[fsk_pulses->num_pulses]; // Restore counter
                    }
                }
                s->fm_f1_est += fm_n / FSK_EST_SLOW - s->fm_f1_est / FSK_EST_SLOW; // Slow estimator
                break;
            case PD_FSK_STATE_ERROR:        // Stay here until cleared
                break;
            default:
                fprintf(stderr, "pulse_detect_fsk_minmax(): Unknown FSK state!!\n");
                s->fsk_state = PD_FSK_STATE_ERROR;
                break;
        }
    }
    if (s->skip_samples > 0) {
        s->skip_samples -= 1;
    }
}
