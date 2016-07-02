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
	AFILTER_BANDWIDTH,
} PortIndex;

struct linear_svf {
	float g, k;
	float a[3];
	float m[3];
	float s[2][2];
};

static void linear_svf_reset(struct linear_svf *self)
{
	self->s[0][0]     =
		self->s[0][1] =
		self->s[1][0] =
		self->s[1][1] = 0.f;
}

typedef struct {
	float* input;
	float* output;

	float* f0;
	float* bw;

	float oldf0;
	float oldbw;
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
	afilter->oldbw = 0.f;
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
	case AFILTER_BANDWIDTH:
		afilter->bw = (float*)data;
		break;
	case AFILTER_INPUT:
		afilter->input = (float*)data;
		break;
	case AFILTER_OUTPUT:
		afilter->output = (float*)data;
		break;
	}
}

// Force already-denormal float value to zero
static inline float
sanitize_denormal(float value) {
	if (!isnormal(value)) {
		value = 0.f;
	}
	return value;
}

static inline float
from_dB(float gdb) {
	return (exp(gdb/20.f*log(10.f)));
}

static inline float
to_dB(float g) {
	return (20.f*log10(g));
}

static void
activate(LV2_Handle instance)
{
	AFilter* afilter = (AFilter*)instance;

	linear_svf_reset(&afilter->highpass);

	*(afilter->f0) = 160.0f;
	*(afilter->bw) = 2.0f;
}

/*
 * highpass SVF
 * http://www.cytomic.com/files/dsp/SvfLinearTrapOptimised2.pdf
 */
static void linear_svf_set_hp(struct linear_svf *self, float sample_rate, float cutoff, float bandwidth)
{
	float resonance = powf(2.0, 1.0/bandwidth)/(powf(2.0, bandwidth) - 1.0);
	
	self->g = tanf(M_PI * (cutoff / sample_rate));
	self->k = 1.f / resonance;

	self->a[0] = 1.f / (1.f + self->g * (self->g + self->k));
	self->a[1] = self->g * self->a[0];
	self->a[2] = self->g * self->a[1];

	self->m[0] = 1.f;
	self->m[1] = -self->k;
	self->m[2] = -1.f;
}

static float run_linear_svf(struct linear_svf *self, int c, float in)
{
	float v[3];

	v[2] = in - self->s[c][1];
	v[0] = (self->a[0] * self->s[c][0]) + (self->a[1] * v[2]);
	v[1] = self->s[c][1] + (self->a[1] * self->s[c][0]) + (self->a[2] * v[2]);

	self->s[c][0] = (2.f * v[0]) - self->s[c][0];
	self->s[c][1] = (2.f * v[1]) - self->s[c][1];

	return
		(self->m[0] * in)
		+ (self->m[1] * v[0])
		+ (self->m[2] * v[1]);
}

static void
run(LV2_Handle instance, uint32_t n_samples)
{
	AFilter* afilter = (AFilter*)instance;

	const float* const input = afilter->input;
	float* const output = afilter->output;

	float srate = afilter->srate;
	int recalc = 0;
	float in0, out;
	uint32_t i;

	if (*(afilter->f0) != afilter->oldf0) {
		recalc = 1;
	}
	if (*(afilter->bw) != afilter->oldbw) {
		recalc = 1;
	}

	if (recalc)
		linear_svf_set_hp(&afilter->highpass, srate, *(afilter->f0), *(afilter->bw));

	for (i = 0; i < n_samples; i++) {
		in0 = input[i];
		out = run_linear_svf(&afilter->highpass, 0, in0);
		output[i] = out;
	}

	afilter->oldf0 = *(afilter->f0);
	afilter->oldbw = *(afilter->bw);
}

static void
deactivate(LV2_Handle instance)
{
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
	deactivate,
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
