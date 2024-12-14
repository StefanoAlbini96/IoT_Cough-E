#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <launcher.h>

#include <fsm_control.h>
#include <feature_extraction.h>
#include <audio_features.h>
#include <imu_features.h>
#include <postprocessing.h>
#include <filtering.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "versa_ble.h"
#include "app_data.h"

LOG_MODULE_REGISTER(launcher, LOG_LEVEL_INF);

#define IMU_ARRAY_SIZE 50       // 0.5 sec at 100 Hz
#define AUD_ARRAY_SIZE 6400     // 0.8 sec at 8 kHz

int N_aud_data = 0;
bool AUD_ready_to_proc = false;


#define IMU_BUF_SIZE    100
static float circ_buffer_imu[IMU_BUF_SIZE][6];
static int write_index_imu = 0;
static int read_index_imu = 0;
static int data_count_imu = 0;

static struct k_mutex imu_buffer_mutex;
K_SEM_DEFINE(imu_sem, 0, 1);

float processing_imu_wind[IMU_ARRAY_SIZE][6];



#define AUD_BUF_SIZE    13000
static float circ_buffer_aud[AUD_BUF_SIZE];
static int write_index_aud = 0;
static int read_index_aud = 0;
static int data_count_aud = 0;

static struct k_mutex aud_buffer_mutex;
K_SEM_DEFINE(aud_sem, 0, 1);

float processing_aud_wind[AUD_ARRAY_SIZE];



#define DOWNSAMPLE_FACTOR 32
#define DOWNSAMPLED_WIND_SIZE ((AUD_ARRAY_SIZE) / (DOWNSAMPLE_FACTOR))
float downsampled_aud[7 * DOWNSAMPLED_WIND_SIZE];    // Accomodate up to 3 windows
uint16_t downsample_idx = 0;    // Indices the next free space (how many have been added)


// Set to true when the execution took place.
// Useful to trigger the FSM update only when needed
bool executed = false;  

bool ready = false;


// K_THREAD_STACK_DEFINE(app_thread_stack, 4098);
K_THREAD_STACK_DEFINE(app_thread_stack, 10000);
struct k_thread app_thread;


K_THREAD_STACK_DEFINE(app_data_thread_stack, 15000);
struct k_thread app_data_thread;


void downsample(const float *input, int input_length, float *output) {

    uint16_t out_len = 0;

    if (DOWNSAMPLE_FACTOR <= 0 || input_length <= 0) {
        return;
    }

    // Calculate the length of the downsampled array
    out_len = (input_length + DOWNSAMPLE_FACTOR - 1) / DOWNSAMPLE_FACTOR; // Ceiling division for safety

    // Perform downsampling
    for (int i = 0; i < out_len; i++) {
        output[i] = input[i * DOWNSAMPLE_FACTOR];
        downsample_idx++;
    }
}


void data_thread(){

    LOG_INF("RUNNING DATA THREAD!\n");

    int N_imu_data = 0;
    int N_aud_data = 0;

    uint8_t res = 0;

    float new_aud_datum = 0.0;

    while(1){
        // Get data from T5838
        uint8_t *new_data_aud = (uint8_t*)k_malloc(240 * sizeof(uint8_t));

        if(new_data_aud != NULL){

            res = get_t5838_data_from_fifo(new_data_aud); // Gets 10 samples of 13 bytes each (index included)
            
            if((res != NULL)){
                
                // LOG_INF("NEW DATA ITER!");

                if (k_mutex_lock(&aud_buffer_mutex, K_FOREVER) == 0){
                    // LOG_INF("Data MUTEX lock!");
                } else {
                    // LOG_INF("Data MUTEX lock failed!");
                }

                for(int i=0; i<120; i++){
                    new_aud_datum = (float)((int16_t)((new_data_aud[(i*2)+1] << 8) | new_data_aud[(i*2)]));
                    circ_buffer_aud[write_index_aud] = new_aud_datum;

                    write_index_aud = (write_index_aud + 1) % AUD_BUF_SIZE;

                    if (data_count_aud < AUD_BUF_SIZE) {
                        data_count_aud++;
                    } else {
                        // Overwrite oldest data (move read pointer)
                        read_index_aud = (read_index_aud + 1) % AUD_BUF_SIZE;
                    }
                }

                // Signal the consumer if enough data is ready
                if (data_count_aud >= WINDOW_SAMP_AUDIO) {
                    if (k_sem_count_get(&aud_sem) == 0){
                        k_sem_give(&aud_sem);
                        k_msleep(10);
                    }
                }
                // LOG_INF("Got AUD DATA");

                k_mutex_unlock(&aud_buffer_mutex);
            }

            k_free(new_data_aud);
        }

    }

}


void iot_cough_E(){

    // These two arrays contain the indexes of the features that are going to be extracted
    int16_t *indexes_audio_f = (int16_t*)k_malloc(N_AUDIO_FEATURES * sizeof(int16_t));

    int16_t idx = 0;
    for(int16_t i=0; i<Number_AUDIO_Features; i++){
        if(audio_features_selector[i] == 1){
            indexes_audio_f[idx] = i;
            idx++;
        }
    }

    ////    AUDIO FEATURES    ////
    // Array for containing the audio features values. The order is the same as of the
    // features families enum
    float  *audio_feature_array = (float*)k_malloc(Number_AUDIO_Features * sizeof(float));
    memset(audio_feature_array, 0.0, Number_AUDIO_Features);

    // Array for the features set of the audio model
    float* features_audio_model = (float*)k_malloc(TOT_FEATURES_AUDIO_MODEL_AUDIO * sizeof(float));

    float audio_proba = 0.0;


    // Postprocessing arrays and variables
    uint16_t *starts = (uint16_t*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(uint16_t));
    uint16_t *ends = (uint16_t*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(uint16_t));
    uint16_t *locs = (uint16_t*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(uint16_t));
    float *peaks = (float*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(float));

    // Number of peaks found from last output
    uint16_t n_peaks = 0;

    // Number of peaks found in the current window
    uint16_t new_added = 0;

    // Confidence of the model per each peak found
    float *audio_confidence = (float*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(float*));

    // Index of the start of the current window (depending on the model to use, it indexes the AUDIO or the IMU signal)
    uint32_t idx_start_window = 0;

    // Number of peaks for which the model confidence is above the threshold
    uint16_t n_idxs_above_th = 0;    


    float sample_rate = 8000.0f; // Sampling rate in Hz
    float cutoff_freq = 200.0f; // Cutoff frequency in Hz

    HighPassFilter filter;
    init_high_pass_filter(&filter, sample_rate, cutoff_freq);

    init_state();

    while(1){

        k_sem_take(&aud_sem, K_FOREVER);
        k_mutex_lock(&aud_buffer_mutex, K_FOREVER);
        
        LOG_INF("AUD proc");
        
        // Get a window from the circular buffer
        for(int i=0; i<WINDOW_SAMP_AUDIO; i++){
            processing_aud_wind[i] = circ_buffer_aud[(read_index_aud + i) % AUD_BUF_SIZE];
        }
        read_index_aud = (read_index_aud + AUDIO_STEP) % AUD_BUF_SIZE;
        data_count_aud -= AUDIO_STEP;

        for (size_t i = 0; i < WINDOW_SAMP_AUDIO; ++i) {
            processing_aud_wind[i] = apply_high_pass_filter(&filter, processing_aud_wind[i]);
        }

        // Normalization
        vect_mean_std_norm(processing_aud_wind, WINDOW_SAMP_AUDIO);

        downsample(&processing_aud_wind[WINDOW_SAMP_AUDIO/2], WINDOW_SAMP_AUDIO/2, &downsampled_aud[downsample_idx]);

        // Extract AUDIO features
        audio_features(audio_features_selector, &processing_aud_wind[0], WINDOW_SAMP_AUDIO, AUDIO_FS, audio_feature_array);

        // Fill the array of fifeatures_imu_modelnal audio features to feed into the AUDIO model
        for(int16_t j=0; j<N_AUDIO_FEATURES; j++){
            features_audio_model[j] = audio_feature_array[indexes_audio_f[j]];
        }
        if(audio_bio_feats_selector[0] == 1){
            features_audio_model[N_AUDIO_FEATURES] = gender;
        }
        if(audio_bio_feats_selector[1] == 1){
            features_audio_model[N_AUDIO_FEATURES+1] = bmi;
        }

        audio_proba = audio_predict(features_audio_model);
        LOG_INF("AUD prob: %f", audio_proba);

        // Update the output of the FSM
        if(audio_proba >= AUDIO_TH){
            fsm_state.model_cls_out = COUGH_OUT;
        } else {
            fsm_state.model_cls_out = NON_COUGH_OUT;
        }

        // Identify the peaks   
        _get_cough_peaks(&processing_aud_wind[0], WINDOW_SAMP_AUDIO, AUDIO_FS, &starts[n_peaks], &ends[n_peaks], &locs[n_peaks], &peaks[n_peaks], &new_added);

        // Readjust the indexes for the position of the current window (to get absolute index)
        for(uint16_t j=0; j<new_added; j++){
            starts[n_peaks+j] += idx_start_window*AUDIO_STEP;
            ends[n_peaks+j] += idx_start_window*AUDIO_STEP;
            locs[n_peaks+j] += (idx_start_window*AUDIO_STEP);
            audio_confidence[n_peaks+j] = audio_proba;
        }
        n_peaks += new_added;


        k_mutex_unlock(&aud_buffer_mutex);

        update();

        if(check_postprocessing()){

            uint16_t n_peaks_final = 0;

            if(n_peaks > 0){
                // Keeps track of the indexes of the peaks for which the model confidence is above the threshold 
                uint16_t *idxs_above_th = (uint16_t*)k_malloc(n_peaks * sizeof(uint16_t));


                for(uint16_t i=0; i<n_peaks; i++){
                    if(audio_confidence[i] >= AUDIO_TH){
                        idxs_above_th[n_idxs_above_th] = i;
                        n_idxs_above_th++;
                    }
                }

                uint16_t *final_starts = (uint16_t*)k_malloc(n_idxs_above_th * sizeof(uint16_t));
                uint16_t *final_ends = (uint16_t*)k_malloc(n_idxs_above_th * sizeof(uint16_t));
                uint16_t *above_locs = (uint16_t*)k_malloc(n_idxs_above_th * sizeof(uint16_t));
                float *above_peaks = (float*)k_malloc(n_idxs_above_th * sizeof(float));


                for(uint16_t i=0; i<n_idxs_above_th; i++){
                    final_starts[i] = starts[idxs_above_th[i]];
                    final_ends[i] = ends[idxs_above_th[i]];
                    above_locs[i] = locs[idxs_above_th[i]];
                    above_peaks[i] = peaks[idxs_above_th[i]];
                }

                n_peaks_final = _clean_cough_segments(final_starts, final_ends, above_locs, above_peaks, n_idxs_above_th, AUDIO_FS);

                k_free(idxs_above_th);
                k_free(final_starts);
                k_free(final_ends);
                k_free(above_locs);
                k_free(above_peaks);
            }
            
            // ble_receive_final_data(&n_peaks_final);
            send_aud_wind_ble(downsampled_aud, downsample_idx, n_peaks_final);

            // Reset postprocessing variables to their default value
            n_peaks = 0;

            n_idxs_above_th = 0;
            
            downsample_idx = 0;
        }
    }
    
    k_free(indexes_audio_f);

    k_free(audio_feature_array);

    k_free(features_audio_model);

    k_free(starts);
    k_free(ends);
    k_free(locs);
    k_free(peaks);

    k_free(audio_confidence);

}


void start_coughE_thread(){

    k_mutex_init(&aud_buffer_mutex);
    k_sem_init(&aud_sem, 0, 1);

    k_tid_t app_thread_id = k_thread_create(&app_thread, app_thread_stack, K_THREAD_STACK_SIZEOF(app_thread_stack),
                                        iot_cough_E, NULL, NULL, NULL, 11, 0, K_NO_WAIT);

    k_tid_t app_data_thread_id = k_thread_create(&app_data_thread, app_data_thread_stack, K_THREAD_STACK_SIZEOF(app_data_thread_stack),
                                        data_thread, NULL, NULL, NULL, 3, 0, K_NO_WAIT);

    LOG_INF("COUGH-E thread initialized!\n");
}
