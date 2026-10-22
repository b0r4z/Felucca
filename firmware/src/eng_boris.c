/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Boris Zec. Built on Felucca (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* BORIS ("Sample by Boris"): a mono sample player for the step sequencer, after the idea of a Reaktor
 * ensemble ("Sample" by Dino Sabatini). Nothing of that ensemble is copied: the signal chain is rebuilt
 * from a description of it.
 *
 *   sequencer step (note = pitch, C4 = original speed)
 *     -> SAMPLER: mono, every note restarts the sample at START (0..127: 128 points across the material)
 *     -> WRAPPER: wraparound distortion; WRAP = gain (exponential, up to ~ +48 dB) before the wrap,
 *                 its symmetry moved by the ENVELOPE FOLLOWER (FOLW) and the LFO (the track's LFO -> SHP),
 *                 a DC blocker after it
 *     -> SAFETY LIMITER: a soft clip, always on (keeps the wrapper's peaks out of the filter)
 *     -> FOLLOWER FILTER: Moog-style ladder, 2 or 4 poles (POLE), CUT and RES; the follower (FOLW) and the
 *                 track's LFO / ENV -> FLT open it
 *     -> the track's ADSR and LEVEL; DUB DELAY and the master compressor are Felucca's (the track's DLY send,
 *        MENU > the global delay, the master)
 *
 * Material: SLICE's sources (eng_slice.c: BREAK, USR1..3 from the web editor, PIANO) and their tables of
 * ADPCM decoder states at 128 grid points, so START enters the sample exactly at one of them.
 * Missing material plays the sine (eng_sample.c smp_sine), as SAMPLE and SLICE do.
 *
 * Voice state: ph[0] position, ph[1] fraction Q16, s[0] predictor, s[1] step index, s[2] / s[3] previous /
 * current sample, s[4] source | segment << 3, s[6] 0 to prime, 1 playing, 2 ended, 4 the sine, s[7] the sine's
 * low-pass. The chain's state (follower, DC blocker, ladder) is per voice in the part's engine memory. */

#define BRS_SINE 4                    /* s[6]: no material, the sine */
#define BRS_FADE 64u                  /* samples faded out before the material's end */
#define BRS_ROOT (60 * 16)            /* C4 plays the material at its own speed */

typedef struct {
    int32_t env;                      /* envelope follower, Q15 */
    int32_t hx, hy;                   /* DC blocker: last input, last output */
    int32_t y[4];                     /* ladder stages, Q15 */
} brs_ch_t;
typedef brs_ch_t brs_mem_t[NVOICE];
ENG_MEM_FITS("BORIS", sizeof(brs_mem_t));

static const char *const N_BRS_POLE[] = {"2P", "4P"};

static brs_ch_t *brs_ch(track_t *t, voice_t *v)
{
    brs_ch_t *c = (brs_ch_t *)ENG_MEM((uint32_t)(t - trk) % NPART);
    return &c[(uint32_t)(v - t->v) % NVOICE];
}
static uint32_t brs_src_of(const int16_t *p) { return (uint32_t)clamp(p[P_E0], 0, SLC_NSRC - 1); }

static void boris_note_on(track_t *t, voice_t *v)
{
    const int16_t *p = t->p;
    uint32_t src = brs_src_of(p), k = (uint32_t)clamp(p[P_E1], 0, SLC_GRID - 1);
    const slc_src_t *s = slc_get(src);
    brs_ch_t *c = brs_ch(t, v);
    c->hx = c->hy = 0;                /* a fresh voice: the chain starts empty (the follower too) */
    c->env = 0;
    c->y[0] = c->y[1] = c->y[2] = c->y[3] = 0;
    v->ph[1] = 0;
    v->s[2] = v->s[3] = 0;
    v->s[7] = 0;
    if (!s) {                         /* no material: the sine */
        v->ph[0] = 0;
        v->s[6] = BRS_SINE;
        return;
    }
    {
        slc_dec_t d;
        slc_dec_at(&d, slc_gpos(s, k), s->grid[k]);   /* START: one of the 128 grid points, exact */
        v->ph[0] = d.pos;
        v->s[0] = d.pred;
        v->s[1] = d.idx;
        v->s[4] = (int32_t)(src | d.seg << 3);
    }
    v->s[6] = 0;
}

/* one-pole coefficient of the ladder for cutoff c (0..127 << 8): g = w / (1 + w), w = 2 pi f / FS, Q15 */
static int32_t brs_g(int32_t c)
{
    uint32_t inc = pitch_inc((uint32_t)(384 + ((c * 1663) >> 15)));   /* note 24 .. ~127.9 (32 Hz .. 12.5 kHz) */
    int32_t w = (int32_t)(((inc >> 16) * 25736u) >> 13);                 /* (inc / 2^32) * 2 pi, Q15 */
    return (int32_t)(((uint32_t)w << 15) / (uint32_t)(32768 + w));
}

static void boris_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    const int16_t *p = t->p;
    uint32_t pk = (uint32_t)v->s[4], i, frac = v->ph[1], stepq;
    const slc_src_t *s;
    brs_ch_t *c;
    slc_dec_t d;
    int32_t wrap = clamp(p[P_E3], 0, 127), folw = clamp(p[P_E4], 0, 127), pole4 = p[P_E7] != 0;
    int32_t gw, symlfo, cut, g, k, env, hx, hy, y0, y1, y2, y3;
    if (v->s[6] == BRS_SINE) {        /* no material: the sine at the note's pitch + TUNE */
        smp_sine(out, n, m, p[P_E2] * 16, 16000, &v->ph[0], &v->s[7]);
        return;
    }
    s = v->s[6] == 2 ? 0 : slc_get(pk & 7u);
    if (!s) {                         /* ended, or its slot was erased */
        v->active = 0;
        return;
    }
    c = brs_ch(t, v);
    stepq = (pow2_q16(clamp(m->pitch16 + p[P_E2] * 16 - BRS_ROOT, -1536, 576)) >> 8) * (s->rate >> 8);
    d.pos = v->ph[0];
    d.pred = v->s[0];
    d.idx = v->s[1];
    d.seg = (pk >> 3) & 15u;

    /* per block: the wrapper's gain (Q8, 2^(WRAP / 16): 1 .. ~240), its symmetry from the LFO (SHP) */
    gw = (int32_t)(pow2_q16(wrap * 12) >> 8);
    symlfo = (m->shape - (64 << 8)) * 2;                                  /* +-64 << 8 -> +-32768 */
    /* the filter: CUT + the track's modulation + the follower (its value at the block's start) */
    env = c->env;
    cut = clamp((p[P_E5] << 8) + m->cutoff + ((env * folw) >> 8), 0, 127 << 8);
    g = brs_g(cut);
    k = clamp(p[P_E6], 0, 127) * (pole4 ? 129 : 96);                     /* feedback, Q12: up to ~4.0 / ~3.0 */
    hx = c->hx;
    hy = c->hy;
    y0 = c->y[0];
    y1 = c->y[1];
    y2 = c->y[2];
    y3 = c->y[3];

    if (!v->s[6]) {                   /* prime the interpolator */
        if (d.pos >= s->len) {
            v->active = 0;
            return;
        }
        v->s[3] = slc_dec_next(s, &d);
        v->s[6] = 1;
    }
    for (i = 0; i < n; i++) {
        int32_t x, a, sym, w, fb;
        uint32_t rem;
        frac += stepq;
        while (frac >= 65536u) {
            frac -= 65536u;
            v->s[2] = v->s[3];
            if (d.pos >= s->len) {    /* the material's end */
                v->s[6] = 2;
                v->s[3] = 0;
                break;
            }
            v->s[3] = slc_dec_next(s, &d);
        }
        x = v->s[2] + (((v->s[3] - v->s[2]) * (int32_t)(frac >> 1)) >> 15);
        rem = d.pos < s->len ? s->len - d.pos : 0u;
        if (rem < BRS_FADE)           /* no click at the end */
            x = (x * (int32_t)rem) >> 6;

        /* envelope follower: fast up, ~45 ms down */
        a = x < 0 ? -x : x;
        env += a > env ? (a - env) >> 4 : (a - env) >> 11;

        /* WRAPPER: gain, symmetry (follower + LFO, scaled by WRAP: WRAP 0 is clean), wrap into 16 bits */
        if (wrap) {
            sym = clamp(symlfo + ((env * folw) >> 7), -32768, 32767);
            sym = (sym * wrap) >> 7;
            w = ((x * gw) >> 8) + sym;
            w = (int16_t)w;           /* the wrap: past full scale it comes in from the other side */
            x = w;
        }
        /* DC blocker (the symmetry offset), then the SAFETY LIMITER */
        hy = x - hx + ((hy * 32604) >> 15);
        hx = x;
        x = softclip(hy + (hy >> 2));

        /* FOLLOWER FILTER: ladder, feedback from the last stage in use, tanh at its input */
        fb = (((pole4 ? y3 : y1) >> 2) * k) >> 10;
        a = softclip(x - fb);
        y0 += mulq15(a - y0, g);
        y1 += mulq15(y0 - y1, g);
        if (pole4) {
            y2 += mulq15(y1 - y2, g);
            y3 += mulq15(y2 - y3, g);
            x = y3;
        } else
            x = y1;
        x += (x * k) >> 14;           /* some of the level resonance takes away */

        out[i] += voice_amp(clamp(x, -32767, 32767), m, i) << 1;
        if (v->s[6] == 2)
            break;
    }
    c->env = env;
    c->hx = hx;
    c->hy = hy;
    c->y[0] = y0;
    c->y[1] = y1;
    c->y[2] = y2;
    c->y[3] = y3;
    v->ph[0] = d.pos;
    v->ph[1] = frac;
    v->s[0] = d.pred;
    v->s[1] = d.idx;
    v->s[4] = (int32_t)((pk & 7u) | d.seg << 3);
}

/* factory sounds: all on BREAK (always there); with USR1..3 the same sounds play your own samples.
 * SRC START TUNE WRAP FOLW CUT RES POLE; ATK DEC SUS REL; ENV -> FLT; MONO; sends DIST CHOR DLY REV; pattern */
static const preset_t BORIS_PRESETS[] = {
    {"CLEAN", {0, 0, 0, 0, 0, 127, 0, 1}, {0, 127, 127, 40}, 0, 1, FX(0, 0, 0, 0), PAT(14)},
    {"WRAPDUB", {0, 0, 0, 48, 40, 80, 50, 1}, {0, 127, 127, 50}, 0, 1, FX(0, 0, 45, 10), PAT(14)},
    {"FOLLOW", {0, 32, 0, 30, 100, 40, 70, 1}, {0, 127, 127, 40}, 20, 1, FX(0, 0, 30, 0), PAT(14)},
    {"CRUSH2P", {0, 64, -12, 90, 60, 100, 90, 0}, {0, 127, 127, 30}, 0, 1, FX(0, 0, 20, 0), PAT(14)},
};

static const engine_t ENG_BORIS = {
    .name = "BORIS",
    .page_title = {"SMPL", "WRAP"},
    .edit = {
        {"SRC", F_ENUM, 0, SLC_NSRC - 1, 0, N_SLC_SRC, 0},
        {"START", F_INT, 0, 127, 0, 0, 0},
        {"TUNE", F_SEMI, -24, 24, 0, 0, 0},
        {"WRAP", F_INT, 0, 127, 0, 0, 0},
        {"FOLW", F_PCT, 0, 127, 0, 0, 0},
        {"CUT", F_INT, 0, 127, 127, 0, 0},
        {"RES", F_INT, 0, 127, 0, 0, 0},
        {"POLE", F_ENUM, 0, 1, 1, N_BRS_POLE, 0},
    },
    .presets = BORIS_PRESETS,
    .npresets = NELEM(BORIS_PRESETS),
    .note_on = boris_note_on,
    .render = boris_render,
    .knob = {P_E1, P_E3, P_E5, P_E6},
    .poly = 1,
    .sampled = 1,
    .mem = sizeof(brs_mem_t),
};
