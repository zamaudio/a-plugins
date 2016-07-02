/* a-comp
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

#define ACOMP_URI "urn:ardour:a-comp"

typedef enum {
	ACOMP_INPUT0 = 0,
	ACOMP_INPUT1,
	ACOMP_OUTPUT,

	ACOMP_ATTACK,
	ACOMP_RELEASE,
	ACOMP_KNEE,
	ACOMP_RATIO,
	ACOMP_THRESHOLD,
	ACOMP_MAKEUP,
	
	ACOMP_GAINR,
	ACOMP_OUTLEVEL,
	ACOMP_SIDECHAIN,
} PortIndex;


typedef struct {
	float* input0;
	float* input1;
	float* output;

	float* attack;
	float* release;
	float* knee;
	float* ratio;
	float* thresdb;
	float* makeup;

	float* gainr;
	float* outlevel;
	float* sidechain;

	float srate;
	float old_yl;
	float old_y1;
	float old_yg;
} AComp;

static LV2_Handle
instantiate(const LV2_Descriptor* descriptor,
            double rate,
            const char* bundle_path,
            const LV2_Feature* const* features)
{
	AComp* acomp = (AComp*)malloc(sizeof(AComp));
	acomp->srate = rate;

	acomp->old_yl=acomp->old_y1=acomp->old_yg=0.f;

	return (LV2_Handle)acomp;
}

static void
connect_port(LV2_Handle instance,
             uint32_t port,
             void* data)
{
	AComp* acomp = (AComp*)instance;

	switch ((PortIndex)port) {
	case ACOMP_ATTACK:
		acomp->attack = (float*)data;
		break;
	case ACOMP_RELEASE:
		acomp->release = (float*)data;
		break;
	case ACOMP_KNEE:
		acomp->knee = (float*)data;
		break;
	case ACOMP_RATIO:
		acomp->ratio = (float*)data;
		break;
	case ACOMP_THRESHOLD:
		acomp->thresdb = (float*)data;
		break;
	case ACOMP_MAKEUP:
		acomp->makeup = (float*)data;
		break;
	case ACOMP_GAINR:
		acomp->gainr = (float*)data;
		break;
	case ACOMP_OUTLEVEL:
		acomp->outlevel = (float*)data;
		break;
	case ACOMP_SIDECHAIN:
		acomp->sidechain = (float*)data;
		break;
	case ACOMP_INPUT0:
		acomp->input0 = (float*)data;
		break;
	case ACOMP_INPUT1:
		acomp->input1 = (float*)data;
		break;
	case ACOMP_OUTPUT:
		acomp->output = (float*)data;
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
	AComp* acomp = (AComp*)instance;

	*(acomp->gainr) = 0.0f;
	*(acomp->outlevel) = -45.0f;
	acomp->old_yl=acomp->old_y1=acomp->old_yg=0.f;
}

static void
run(LV2_Handle instance, uint32_t n_samples)
{
	AComp* acomp = (AComp*)instance;

	const float* const input0 = acomp->input0;
	const float* const input1 = acomp->input1;
	float* const output = acomp->output;

	float srate = acomp->srate;
	float width = (6.f * *(acomp->knee)) + 0.01;
	float cdb=0.f;
	float attack_coeff = exp(-1000.f/(*(acomp->attack) * srate));
	float release_coeff = exp(-1000.f/(*(acomp->release) * srate));

	float max = 0.f;
	float lgaininp = 0.f;
	float Lgain = 1.f;
	float Lxg, Lxl, Lyg, Lyl, Ly1;
	int usesidechain = (*(acomp->sidechain) < 0.5) ? 0 : 1;
	uint32_t i;
	float ingain;
	float in0;
	float in1;
	float ratio = *(acomp->ratio);
	float thresdb = *(acomp->thresdb);

	for (i = 0; i < n_samples; i++) {
		in0 = input0[i];
		in1 = input1[i];
		ingain = usesidechain ? in1 : in0;
		Lyg = 0.f;
		Lxg = (ingain==0.f) ? -160.f : to_dB(fabs(ingain));
		Lxg = sanitize_denormal(Lxg);

		Lyg = Lxg + (1.f/ratio-1.f)*(Lxg-thresdb+width/2.f)*(Lxg-thresdb+width/2.f)/(2.f*width);

		if (2.f*(Lxg-thresdb) < -width) {
			Lyg = Lxg;
		} else {
			Lyg = thresdb + (Lxg-thresdb)/ratio;
			Lyg = sanitize_denormal(Lyg);
		}

		Lxl = Lxg - Lyg;

		acomp->old_y1 = sanitize_denormal(acomp->old_y1);
		acomp->old_yl = sanitize_denormal(acomp->old_yl);
		Ly1 = fmaxf(Lxl, release_coeff * acomp->old_y1+(1.f-release_coeff)*Lxl);
		Lyl = attack_coeff * acomp->old_yl+(1.f-attack_coeff)*Ly1;
		Ly1 = sanitize_denormal(Ly1);
		Lyl = sanitize_denormal(Lyl);

		cdb = -Lyl;
		Lgain = from_dB(cdb);

		*(acomp->gainr) = Lyl;

		lgaininp = in0 * Lgain;
		output[i] = lgaininp * from_dB(*(acomp->makeup));

		max = (fabsf(output[i]) > max) ? fabsf(output[i]) : sanitize_denormal(max);

		acomp->old_yl = Lyl;
		acomp->old_y1 = Ly1;
		acomp->old_yg = Lyg;
	}
	*(acomp->outlevel) = (max == 0.f) ? -45.f : to_dB(max);
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
	ACOMP_URI,
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
