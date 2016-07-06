/* a-eq
 * Copyright (C) 2016 Damien Zammit <damien@zamaudio.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <math.h>
#include <stdlib.h>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"

#define AEQ_URI	"urn:ardour:a-eq"
#define BANDS	6

typedef enum {
	AEQ_SHELFTOGL = 0,
	AEQ_FREQL,
	AEQ_GAINL,
	AEQ_FREQ1,
	AEQ_GAIN1,
	AEQ_BW1,
	AEQ_FREQ2,
	AEQ_GAIN2,
	AEQ_BW2,
	AEQ_FREQ3,
	AEQ_GAIN3,
	AEQ_BW3,
	AEQ_FREQ4,
	AEQ_GAIN4,
	AEQ_BW4,
	AEQ_SHELFTOGH,
	AEQ_FREQH,
	AEQ_GAINH,
	AEQ_MASTER,
	AEQ_FILTOGL,
	AEQ_FILTOG1,
	AEQ_FILTOG2,
	AEQ_FILTOG3,
	AEQ_FILTOG4,
	AEQ_FILTOGH,
	AEQ_INPUT,
	AEQ_OUTPUT,
} PortIndex;

struct linear_svf {
	double g, k;
	double a[3];
	double m[3];
	double s[2];
};

static void linear_svf_reset(struct linear_svf *self)
{
	self->s[0] = self->s[1] = 0.0;
}

typedef struct {
	float* shelftogl;
	float* shelftogh;
	float* f0[BANDS];
	float* g[BANDS];
	float* bw[BANDS];
	float* filtog[BANDS];
	float* master;

	float srate;

	float* input;
	float* output;
	struct linear_svf filter[BANDS];
} Aeq;

static LV2_Handle
instantiate(const LV2_Descriptor* descriptor,
            double rate,
            const char* bundle_path,
            const LV2_Feature* const* features)
{
	int i;
	Aeq* aeq = (Aeq*)malloc(sizeof(Aeq));
	aeq->srate = rate;
	
	for (i = 0; i < BANDS; i++)
		linear_svf_reset(&aeq->filter[i]);

	return (LV2_Handle)aeq;
}

static void
connect_port(LV2_Handle instance,
             uint32_t port,
             void* data)
{
	Aeq* aeq = (Aeq*)instance;

	switch ((PortIndex)port) {
	case AEQ_SHELFTOGL:
		aeq->shelftogl = (float*)data;
		break;
	case AEQ_FREQL:
		aeq->f0[0] = (float*)data;
		break;
	case AEQ_GAINL:
		aeq->g[0] = (float*)data;
		break;
	case AEQ_FREQ1:
		aeq->f0[1] = (float*)data;
		break;
	case AEQ_GAIN1:
		aeq->g[1] = (float*)data;
		break;
	case AEQ_BW1:
		aeq->bw[1] = (float*)data;
		break;
	case AEQ_FREQ2:
		aeq->f0[2] = (float*)data;
		break;
	case AEQ_GAIN2:
		aeq->g[2] = (float*)data;
		break;
	case AEQ_BW2:
		aeq->bw[2] = (float*)data;
		break;
	case AEQ_FREQ3:
		aeq->f0[3] = (float*)data;
		break;
	case AEQ_GAIN3:
		aeq->g[3] = (float*)data;
		break;
	case AEQ_BW3:
		aeq->bw[3] = (float*)data;
		break;
	case AEQ_FREQ4:
		aeq->f0[4] = (float*)data;
		break;
	case AEQ_GAIN4:
		aeq->g[4] = (float*)data;
		break;
	case AEQ_BW4:
		aeq->bw[4] = (float*)data;
		break;
	case AEQ_SHELFTOGH:
		aeq->shelftogh = (float*)data;
		break;
	case AEQ_FREQH:
		aeq->f0[5] = (float*)data;
		break;
	case AEQ_GAINH:
		aeq->g[5] = (float*)data;
		break;
	case AEQ_MASTER:
		aeq->master = (float*)data;
		break;
	case AEQ_FILTOGL:
		aeq->filtog[0] = (float*)data;
		break;
	case AEQ_FILTOG1:
		aeq->filtog[1] = (float*)data;
		break;
	case AEQ_FILTOG2:
		aeq->filtog[2] = (float*)data;
		break;
	case AEQ_FILTOG3:
		aeq->filtog[3] = (float*)data;
		break;
	case AEQ_FILTOG4:
		aeq->filtog[4] = (float*)data;
		break;
	case AEQ_FILTOGH:
		aeq->filtog[5] = (float*)data;
		break;
	case AEQ_INPUT:
		aeq->input = (float*)data;
		break;
	case AEQ_OUTPUT:
		aeq->output = (float*)data;
		break;
	}
}

static void
activate(LV2_Handle instance)
{
	int i;
	Aeq* aeq = (Aeq*)instance;

	for (i = 0; i < BANDS; i++)
		linear_svf_reset(&aeq->filter[i]);
}

// SVF filters
// http://www.cytomic.com/files/dsp/SvfLinearTrapOptimised2.pdf

static void linear_svf_set_hp(struct linear_svf *self, float sample_rate, float cutoff, float resonance)
{
	double f0 = (double)cutoff;
	double q = (double)resonance;
	double sr = (double)sample_rate;

	self->g = tan(M_PI * (f0 / sr));
	self->k = 1.0 / q;

	self->a[0] = 1.0 / (1.0 + self->g * (self->g + self->k));
	self->a[1] = self->g * self->a[0];
	self->a[2] = self->g * self->a[1];

	self->m[0] = 1.0;
	self->m[1] = -self->k;
	self->m[2] = -1.0;
}

static void linear_svf_set_lp(struct linear_svf *self, float sample_rate, float cutoff, float resonance)
{
	double f0 = (double)cutoff;
	double q = (double)resonance;
	double sr = (double)sample_rate;

	self->g = tan(M_PI * (f0 / sr));
	self->k = 1.0 / q;

	self->a[0] = 1.0 / (1.0 + self->g * (self->g + self->k));
	self->a[1] = self->g * self->a[0];
	self->a[2] = self->g * self->a[1];

	self->m[0] = 0.0;
	self->m[1] = 0.0;
	self->m[2] = 1.0;
}

static void linear_svf_set_peq(struct linear_svf *self, float gdb, float sample_rate, float cutoff, float bandwidth)
{
	double f0 = (double)cutoff;
	double q = (double)pow(2.0, 1.0 / bandwidth) / (pow(2.0, bandwidth) - 1.0);
	double sr = (double)sample_rate;
	double A = pow(10.0, gdb/40.0);

	self->g = tan(M_PI * (f0 / sr));
	self->k = 1.0 / (q * A);

	self->a[0] = 1.0 / (1.0 + self->g * (self->g + self->k));
	self->a[1] = self->g * self->a[0];
	self->a[2] = self->g * self->a[1];

	self->m[0] = 1.0;
	self->m[1] = self->k * (A * A - 1.0);
	self->m[2] = 0.0;
}

static void linear_svf_set_highshelf(struct linear_svf *self, float gdb, float sample_rate, float cutoff, float resonance)
{
	double f0 = (double)cutoff;
	double q = (double)resonance;
	double sr = (double)sample_rate;
	double A = pow(10.0, gdb/40.0);

	self->g = tan(M_PI * (f0 / sr));
	self->k = 1.0 / q;

	self->a[0] = 1.0 / (1.0 + self->g * (self->g + self->k));
	self->a[1] = self->g * self->a[0];
	self->a[2] = self->g * self->a[1];

	self->m[0] = A * A;
	self->m[1] = self->k * (1.0 - A) * A;
	self->m[2] = 1.0 - A * A;
}

static void linear_svf_set_lowshelf(struct linear_svf *self, float gdb, float sample_rate, float cutoff, float resonance)
{
	double f0 = (double)cutoff;
	double q = (double)resonance;
	double sr = (double)sample_rate;
	double A = pow(10.0, gdb/40.0);

	self->g = tan(M_PI * (f0 / sr));
	self->k = 1.0 / q;

	self->a[0] = 1.0 / (1.0 + self->g * (self->g + self->k));
	self->a[1] = self->g * self->a[0];
	self->a[2] = self->g * self->a[1];

	self->m[0] = 1.0;
	self->m[1] = self->k * (A - 1.0);
	self->m[2] = A * A - 1.0;
}

static float run_linear_svf(struct linear_svf *self, float in)
{
	double v[3];
	double din = (double)in;
	double out;

	v[2] = din - self->s[1];
	v[0] = (self->a[0] * self->s[0]) + (self->a[1] * v[2]);
	v[1] = self->s[1] + (self->a[1] * self->s[0]) + (self->a[2] * v[2]);

	self->s[0] = (2.0 * v[0]) - self->s[0];
	self->s[1] = (2.0 * v[1]) - self->s[1];

	out = (self->m[0] * din)
		+ (self->m[1] * v[0])
		+ (self->m[2] * v[1]);

	return (float)out;
}

static void
run(LV2_Handle instance, uint32_t n_samples)
{
	Aeq* aeq = (Aeq*)instance;

	const float* const input = aeq->input;
	float* const output = aeq->output;

	float srate = aeq->srate;
	float in0, out;
	uint32_t i, j;

	linear_svf_set_hp(&aeq->filter[0], srate, *(aeq->f0[0]), 0.7071068);
	linear_svf_set_peq(&aeq->filter[1], *(aeq->g[1]), srate, *(aeq->f0[1]), *(aeq->bw[1]));
	linear_svf_set_peq(&aeq->filter[2], *(aeq->g[2]), srate, *(aeq->f0[2]), *(aeq->bw[2]));
	linear_svf_set_peq(&aeq->filter[3], *(aeq->g[3]), srate, *(aeq->f0[3]), *(aeq->bw[3]));
	linear_svf_set_peq(&aeq->filter[4], *(aeq->g[4]), srate, *(aeq->f0[4]), *(aeq->bw[4]));

	linear_svf_set_lp(&aeq->filter[5], srate, *(aeq->f0[5]), 0.7071068);

	for (i = 0; i < n_samples; i++) {
		in0 = input[i];
		out = in0;
		for (j = 0; j < BANDS; j++) {
			out = run_linear_svf(&aeq->filter[j], out);
		}
		output[i] = out;
	}
}

static void
cleanup(LV2_Handle instance)
{
	free(instance);
}

const void*
extension_data(const char* uri)
{
	return NULL;
}

static const LV2_Descriptor descriptor = {
	AEQ_URI,
	instantiate,
	connect_port,
	activate,
	run,
	NULL,
	cleanup,
	extension_data
};

LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
	switch (index) {
	case 0:
		return &descriptor;
	default:
		return NULL;
	}
}
