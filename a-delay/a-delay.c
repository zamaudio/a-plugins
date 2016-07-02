/* a-delay
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
#include <stdio.h>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/time/time.h"
#include "lv2/lv2plug.in/ns/ext/atom/forge.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"

#define ADELAY_URI "urn:ardour:a-delay"

// 8 seconds of delay at 96kHz
#define MAX_DELAY 768000

#ifndef M_PI
# define M_PI 3.1415926
#endif

typedef enum {
	ADELAY_INPUT = 0,
	ADELAY_OUTPUT,

	ADELAY_BPM,

	ADELAY_INV,
	ADELAY_SYNC,
	ADELAY_TIME,
	ADELAY_DIVISOR,
	ADELAY_WETDRY,
	ADELAY_LPF,
	ADELAY_GAIN,
	
	ADELAY_DELAYTIME,
} PortIndex;


typedef struct {
	LV2_URID atom_Blank;
	LV2_URID atom_Sequence;
	LV2_URID atom_Float;
	LV2_URID atom_Long;
	LV2_URID time_beatsPerMinute;
	LV2_URID time_beatUnit;
	LV2_URID time_Position;
} DelayURIs;

typedef struct {
	float* input;
	float* output;

	const LV2_Atom_Sequence* atombpm;

	float* inv;
	float* sync;
	float* time;
	float* divisor;
	float* wetdry;
	float* lpf;
	float* gain;
	
	float* delaytime;

	float srate;
	float bpm;
	float beatunit;
	int beatuniti;
	int bpmvalid;

	uint32_t posz;
	float tap[2];
	float z[MAX_DELAY];
	int active;
	int next;
	float fbstate;
	float lpfold;
	float divisorold;
	float gainold;
	float invertold;
	float timeold;
	float delaytimeold;
	float syncold;
	float wetdryold;
	float delaysamplesold;

	float A0, A1, A2, A3, A4, A5;
	float B0, B1, B2, B3, B4, B5;
	float state[4];

	DelayURIs uris;
} ADelay;

static LV2_Handle
instantiate(const LV2_Descriptor* descriptor,
            double rate,
            const char* bundle_path,
            const LV2_Feature* const* features)
{
	ADelay* adelay = (ADelay*)malloc(sizeof(ADelay));
	adelay->srate = rate;

	adelay->bpmvalid = 0;

	return (LV2_Handle)adelay;
}

static void
connect_port(LV2_Handle instance,
             uint32_t port,
             void* data)
{
	ADelay* adelay = (ADelay*)instance;

	switch ((PortIndex)port) {
	case ADELAY_INPUT:
		adelay->input = (float*)data;
		break;
	case ADELAY_OUTPUT:
		adelay->output = (float*)data;
		break;
	case ADELAY_BPM:
		adelay->atombpm = (const LV2_Atom_Sequence*)data;
		break;
	case ADELAY_INV:
		adelay->inv = (float*)data;
		break;
	case ADELAY_SYNC:
		adelay->sync = (float*)data;
		break;
	case ADELAY_TIME:
		adelay->time = (float*)data;
		break;
	case ADELAY_DIVISOR:
		adelay->divisor = (float*)data;
		break;
	case ADELAY_WETDRY:
		adelay->wetdry = (float*)data;
		break;
	case ADELAY_LPF:
		adelay->lpf = (float*)data;
		break;
	case ADELAY_GAIN:
		adelay->gain = (float*)data;
		break;
	case ADELAY_DELAYTIME:
		adelay->delaytime = (float*)data;
		break;
	}
}

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

static void clearfilter(LV2_Handle instance)
{
	ADelay* adelay = (ADelay*)instance;

	adelay->state[0] = adelay->state[1] =
		adelay->state[2] = adelay->state[3] = 0.f;
}

static void
activate(LV2_Handle instance)
{
	ADelay* adelay = (ADelay*)instance;

	int i;
	for (i = 0; i < MAX_DELAY; i++) {
		adelay->z[i] = 0.f;
	}
	adelay->posz = 0;
	adelay->tap[0] = 0;
	adelay->tap[1] = 0;
	adelay->active = 0;
	adelay->next = 1;
	adelay->fbstate = 0.f;

	clearfilter(adelay);

	adelay->lpfold = 0.f;
	adelay->divisorold = 0.f;
	adelay->gainold = 0.f;
	adelay->invertold = 0.f;
	adelay->timeold = 0.f;
	adelay->delaytimeold = 0.f;
	adelay->syncold = 0.f;
	adelay->wetdryold = 0.f;
	adelay->delaysamplesold = 1.f;
}

static void lpfRbj(LV2_Handle instance, float fc, float srate)
{
	ADelay* adelay = (ADelay*)instance;

	float w0, alpha, cw, sw, q;
	q = 0.707;
	w0 = (2. * M_PI * fc / srate);
	sw = sin(w0);
	cw = cos(w0);
	alpha = sw / (2. * q);

	adelay->A0 = 1. + alpha;
	adelay->A1 = -2. * cw;
	adelay->A2 = 1. - alpha;
	adelay->B0 = (1. - cw) / 2.;
	adelay->B1 = (1. - cw);
	adelay->B2 = adelay->B0;

	adelay->A3 = 1. + alpha;
	adelay->A4 = -2. * cw;
	adelay->A5 = 1. - alpha;
	adelay->B3 = (1. - cw) / 2.;
	adelay->B4 = (1. - cw);
	adelay->B5 = adelay->B3;
}

static float runfilter(LV2_Handle instance, float in)
{
	ADelay* a = (ADelay*)instance;

	float out;
	in = sanitize_denormal(in);

	out = a->B0/a->A0*in + a->B1/a->A0*a->state[0] + a->B2/a->A0*a->state[1]
			-a->A1/a->A0*a->state[2] - a->A2/a->A0*a->state[3] + 1e-20;

	a->state[1] = a->state[0];
	a->state[0] = in;
	a->state[3] = a->state[2];
	a->state[2] = out;
	return out;
}

static void
update_bpm(ADelay* self, const LV2_Atom_Object* obj)
{
	const DelayURIs* uris = &self->uris;

	// Received new transport position/speed
	LV2_Atom *beatunit = NULL, *bpm = NULL;
	lv2_atom_object_get(obj,
	                    uris->time_beatUnit, &beatunit,
	                    uris->time_beatsPerMinute, &bpm,
	                    NULL);
	if (bpm && bpm->type == uris->atom_Float) {
		// Tempo changed, update BPM
		self->bpm = ((LV2_Atom_Float*)bpm)->body;
	}
	if (beatunit && beatunit->type == uris->atom_Float) {
		// Time signature changed, update beatunit (float)
		self->beatunit = ((LV2_Atom_Float*)beatunit)->body;
		self->beatuniti = (int)self->beatunit;
	}
	if (beatunit && beatunit->type == uris->atom_Long) {
		// Time signature changed, update beatunit (int)
		self->beatuniti = ((LV2_Atom_Long*)beatunit)->body;
		self->beatunit = (float)self->beatuniti;
	}
	self->bpmvalid = 1;
}

static void
run(LV2_Handle instance, uint32_t n_samples)
{
	ADelay* adelay = (ADelay*)instance;

	const float* const input = adelay->input;
	float* const output = adelay->output;

	float srate = adelay->srate;

	uint32_t i;
	float in;
	float bpm = adelay->bpm;
	int delaysamples;
	unsigned int tmp;
	float inv;
	float xfade;
	int recalc;
	if (*(adelay->inv) < 0.5) {
		inv = -1.f;
	} else {
		inv = 1.f;
	}
	
	recalc = 0;
	*(adelay->delaytime) = *(adelay->time);
	if (adelay->bpmvalid) {
		if (*(adelay->sync) > 0.5f) {
			*(adelay->delaytime) = adelay->beatunit * 1000.f * 60.f / (bpm * powf(2., *(adelay->divisor) - 1.));
		}
	}
	delaysamples = (int)(*(adelay->delaytime) * srate) / 1000;
	
	if (*(adelay->lpf) != adelay->lpfold) {
		lpfRbj(adelay, *(adelay->lpf), srate);
	}
	if (*(adelay->divisor) != adelay->divisorold) {
		recalc = 1;
	}
	if (*(adelay->gain) != adelay->gainold) {
		recalc = 1;
	}
	if (*(adelay->inv) != adelay->invertold) {
		recalc = 1;
	}
	if (*(adelay->time) != adelay->timeold) {
		recalc = 1;
	}
	if (*(adelay->sync) != adelay->syncold) {
		recalc = 1;
	}
	
	if (recalc) {
		adelay->tap[adelay->next] = delaysamples;
	}

	xfade = 0.f;
	for (i = 0; i < n_samples; i++) {
		in = input[i];
		adelay->z[adelay->posz] = in; // + feedb / 100. * fbstate;
		adelay->fbstate = 0.f;
		int p = adelay->posz - adelay->tap[adelay->active]; // active line
		if (p<0) p += MAX_DELAY;
		adelay->fbstate += adelay->z[p];
		
		if (recalc) {
			xfade += 1.0f / (float)n_samples;
			adelay->fbstate *= (1.-xfade);
			int p = adelay->posz - adelay->tap[adelay->next]; // next line
			if (p<0) p += MAX_DELAY;
			adelay->fbstate += adelay->z[p] * xfade;
		}
		output[i] = from_dB(*(adelay->gain)) * ((100.-*(adelay->wetdry)) / 100. * in + *(adelay->wetdry) / 100. * -inv * runfilter(adelay, adelay->fbstate));
		if (++(adelay->posz) >= MAX_DELAY) {
			adelay->posz = 0;
		}
	}
	adelay->lpfold = *(adelay->lpf);
	adelay->divisorold = *(adelay->divisor);
	adelay->gainold = *(adelay->gain);
	adelay->invertold = *(adelay->inv);
	adelay->timeold = *(adelay->time);
	adelay->syncold = *(adelay->sync);
	adelay->wetdryold = *(adelay->wetdry);
	adelay->delaytimeold = *(adelay->delaytime);
	adelay->delaysamplesold = delaysamples;
	if (recalc) {
		tmp = adelay->active;
		adelay->active = adelay->next;
		adelay->next = tmp;
	}
	
	if (adelay->atombpm) {
		LV2_Atom_Event* ev = lv2_atom_sequence_begin(&(adelay->atombpm)->body);
		while(!lv2_atom_sequence_is_end(&(adelay->atombpm)->body, (adelay->atombpm)->atom.size, ev)) {
			if (ev->body.type == adelay->uris.atom_Blank) {
				const LV2_Atom_Object* obj = (LV2_Atom_Object*)&ev->body;
				if (obj->body.otype == adelay->uris.time_Position) {
					update_bpm(adelay, obj);
				}
			}
			ev = lv2_atom_sequence_next(ev);
		}
	}
	//printf("bpmvalid=%d\n", adelay->bpmvalid);
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
	ADELAY_URI,
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
