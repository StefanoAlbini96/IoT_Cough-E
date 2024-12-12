#ifndef _FILTERING_H_
#define _FILTERING_H_

void linear_filer(float *sig, int len, const float *b, const float *a, float *zi, float *res);
void filtfilt(const float *sig, int len, const float *b, const float* a, const float *zi, float *res);


// High-pass filter state
typedef struct {
    float alpha;       // Filter coefficient
    float prev_input;  // Previous input value
    float prev_output; // Previous output value
} HighPassFilter;

void init_high_pass_filter(HighPassFilter *filter, float sample_rate, float cutoff_freq);
float apply_high_pass_filter(HighPassFilter *filter, float input);

#endif