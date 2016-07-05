/* a-filter
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

#define AFILTER_URI "urn:ardour:a-filter"

typedef enum {
	AFILTER_INPUT = 0,
	AFILTER_OUTPUT,

	AFILTER_CUTOFF,
	AFILTER_SLOPE,
} PortIndex;

struct linear_svf {
	double g, k;
	double a[3];
	double m[3];
	double s[4][2];
};

static void linear_svf_reset(struct linear_svf *self)
{
	int i;

	for (i = 0; i < 4; i++) {
		self->s[i][0] = self->s[i][1] = 0.0;
	}
}

typedef struct {
	float* input;
	float* output;

	float* f0;
	float* slope;

	float oldf0;
	float srate;

	struct linear_svf highpass;
} AFilter;

static LV2_Handle
instantiate(const LV2_Descriptor* descriptor,
            double rate,
            const char* bundle_path,
            const LV2_Feature* const* features)
{
	AFilter* afilter = (AFilter*)malloc(sizeof(AFilter));
	afilter->srate = rate;

	afilter->oldf0 = 0.f;

	linear_svf_reset(&afilter->highpass);

	return (LV2_Handle)afilter;
}

static void
connect_port(LV2_Handle instance,
             uint32_t port,
             void* data)
{
	AFilter* afilter = (AFilter*)instance;

	switch ((PortIndex)port) {
	case AFILTER_CUTOFF:
		afilter->f0 = (float*)data;
		break;
	case AFILTER_SLOPE:
		afilter->slope = (float*)data;
		break;
	case AFILTER_INPUT:
		afilter->input = (float*)data;
		break;
	case AFILTER_OUTPUT:
		afilter->output = (float*)data;
		break;
	}
}

static void
activate(LV2_Handle instance)
{
	AFilter* afilter = (AFilter*)instance;

	linear_svf_reset(&afilter->highpass);

	*(afilter->f0) = 160.0f;
	*(afilter->slope) = 12.0f;
}

/*
 * highpass SVF
 * http://www.cytomic.com/files/dsp/SvfLinearTrapOptimised2.pdf
 */
static void linear_svf_set_hp(struct linear_svf *self, float sample_rate, float cutoff, float resonance)
{
	double f0 = (double)cutoff;
	double q = (double)resonance;
	double sr = (double)sample_rate;

	self->g = tan(M_PI * (f0 / sr));
	self->k = 1.f / q;

	self->a[0] = 1.0 / (1.0 + self->g * (self->g + self->k));
	self->a[1] = self->g * self->a[0];
	self->a[2] = self->g * self->a[1];

	self->m[0] = 1.0;
	self->m[1] = -self->k;
	self->m[2] = -1.0;
}

static float run_linear_svf(struct linear_svf *self, int c, float in)
{
	double v[3];
	double din = (double)in;
	double out;

	v[2] = din - self->s[c][1];
	v[0] = (self->a[0] * self->s[c][0]) + (self->a[1] * v[2]);
	v[1] = self->s[c][1] + (self->a[1] * self->s[c][0]) + (self->a[2] * v[2]);

	self->s[c][0] = (2.0 * v[0]) - self->s[c][0];
	self->s[c][1] = (2.0 * v[1]) - self->s[c][1];

	out = (self->m[0] * din)
		+ (self->m[1] * v[0])
		+ (self->m[2] * v[1]);

	return (float)out;
}

static void
run(LV2_Handle instance, uint32_t n_samples)
{
	AFilter* afilter = (AFilter*)instance;

	const float* const input = afilter->input;
	float* const output = afilter->output;

	float srate = afilter->srate;
	int stacked = (int)(*(afilter->slope) / 12.f);
	float in0, out;
	uint32_t i, j;

	if (*(afilter->f0) != afilter->oldf0)
		linear_svf_set_hp(&afilter->highpass, srate, *(afilter->f0), 0.7071068);

	for (i = 0; i < n_samples; i++) {
		in0 = input[i];
		out = in0;
		for (j = 0; j < stacked; j++) {
			out = run_linear_svf(&afilter->highpass, j, out);
		}
		output[i] = out;
	}

	afilter->oldf0 = *(afilter->f0);
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
	AFILTER_URI,
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
