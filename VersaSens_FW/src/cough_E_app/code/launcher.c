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

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "versa_ble.h"
#include "app_data.h"


#define IMU_ARRAY_SIZE 50       // 0.5 sec at 100 Hz
#define AUD_ARRAY_SIZE 6400     // 0.8 sec at 8 kHz

// Global array, stored in RAM
float imu_data_array[IMU_ARRAY_SIZE][6];
float imu_data_array_to_proc[IMU_ARRAY_SIZE][6];
int N_imu_data = 0;
bool IMU_ready_to_proc = false;

// float aud_data_array[AUD_ARRAY_SIZE];
int N_aud_data = 0;
bool AUD_ready_to_proc = false;

// Set to true when the execution took place.
// Useful to trigger the FSM update only when needed
bool executed = false;  


LOG_MODULE_REGISTER(launcher, LOG_LEVEL_INF);

// K_THREAD_STACK_DEFINE(app_thread_stack, 4098);
K_THREAD_STACK_DEFINE(app_thread_stack, 4098);
struct k_thread app_thread;


K_THREAD_STACK_DEFINE(app_data_thread_stack, 2048);
struct k_thread app_data_thread;

void cough_E(){
    uint8_t dbg;

    // k_msleep(1000);

    // LOG_INF("COUGH-E RUNNING!\n");

    // These two arrays contain the indexes of the features that are going to be extracted
    int16_t *indexes_audio_f = (int16_t*)k_malloc(N_AUDIO_FEATURES * sizeof(int16_t));
    int8_t *indexes_imu_f = (int8_t*)k_malloc(N_IMU_FEATURES * sizeof(int8_t));

    int16_t idx = 0;
    for(int16_t i=0; i<Number_AUDIO_Features; i++){
        if(audio_features_selector[i] == 1){
            indexes_audio_f[idx] = i;
            idx++;
        }
    }
    idx = 0;
    for(int8_t i=0; i<Number_IMU_Features; i++){
        if(imu_features_selector[i] == 1){
            indexes_imu_f[idx] = i;
            idx++;
        }
    }

    ////    AUDIO FEATURES    ////
    // Array for containing the audio features values. The order is the same as of the
    // features families enum
    float  *audio_feature_array = (float*)k_malloc(Number_AUDIO_Features * sizeof(float));
    memset(audio_feature_array, 0.0, Number_AUDIO_Features);


    ////    IMU FEATURES    ////
    float *imu_feature_array = (float*)k_malloc(Number_IMU_Features * sizeof(float)); // To store all the possible IMU features
    memset(imu_feature_array, 0.0, Number_IMU_Features);

    // Array for the features set of the audio model
    float* features_audio_model = (float*)k_malloc(TOT_FEATURES_AUDIO_MODEL_AUDIO * sizeof(float));

    float audio_proba = 0.0;

    // Array for the features set of the imu model
    float* features_imu_model = (float*)k_malloc(TOT_FEATURES_IMU_MODEL_IMU * sizeof(float));

    float imu_proba = 0.0;

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

    int debug_cnt = 0;

    init_state();

    // Looping through the windows
    while(1){

        // idx_start_window = get_idx_window();

        if(fsm_state.model == IMU_MODEL){

            // if(idx_start_window >= IMU_LEN){
            //     // break;
            //     init_state();
            //     idx_start_window = get_idx_window();
            // }

            // Start processing only if there are enough samples
            if(IMU_ready_to_proc){

                LOG_INF("IMU exec!\n");
                IMU_ready_to_proc = false;
                
                dbg = 100;
                ble_receive_final_data(&dbg);

                // Extract IMU features
                // imu_features(imu_features_selector, &imu_in[idx_start_window], WINDOW_SAMP_IMU, imu_feature_array);
                imu_features(imu_features_selector, &imu_data_array_to_proc[0], WINDOW_SAMP_IMU, imu_feature_array);

                dbg = 101;
                ble_receive_final_data(&dbg);

                // Fill the array of final imu features to feed into the IMU model
                for(int16_t j=0; j<N_IMU_FEATURES; j++){
                    features_imu_model[j] = imu_feature_array[indexes_imu_f[j]];
                }
                if(imu_bio_feats_selector[0] == 1){
                    features_imu_model[N_IMU_FEATURES] = gender;
                }
                if(imu_bio_feats_selector[1] == 1){
                    features_imu_model[N_IMU_FEATURES+1] = bmi;
                }

                // Predict with the IMU model
                imu_proba = imu_predict(features_imu_model);

                dbg = 102;
                ble_receive_final_data(&dbg);

                // Update the output of the FSM
                if(imu_proba>=IMU_TH){
                    fsm_state.model_cls_out = COUGH_OUT;
                } else {
                    fsm_state.model_cls_out = NON_COUGH_OUT;
                }

                executed = true;
            }
        }
        else { 

            // if(idx_start_window >= AUDIO_LEN){
            //     // break;
            //     init_state();
            //     idx_start_window = get_idx_window();
            // }
            
            if(AUD_ready_to_proc){
                // Extract AUDIO features
                // audio_features(audio_features_selector, &aud_data_array[0], WINDOW_SAMP_AUDIO, AUDIO_FS, audio_feature_array);

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

                // Update the output of the FSM
                if(audio_proba >= AUDIO_TH){
                    fsm_state.model_cls_out = COUGH_OUT;
                } else {
                    fsm_state.model_cls_out = NON_COUGH_OUT;
                }

                // Identify the peaks   
                // _get_cough_peaks(&aud_data_array[0], WINDOW_SAMP_AUDIO, AUDIO_FS, &starts[n_peaks], &ends[n_peaks], &locs[n_peaks], &peaks[n_peaks], &new_added);

                // Readjust the indexes for the position of the current window (to get absolute index)
                for(uint16_t j=0; j<new_added; j++){
                    starts[n_peaks+j] += idx_start_window*AUDIO_STEP;
                    ends[n_peaks+j] += idx_start_window*AUDIO_STEP;
                    locs[n_peaks+j] += (idx_start_window*AUDIO_STEP);
                    audio_confidence[n_peaks+j] = audio_proba;
                }
                n_peaks += new_added;

                executed = true;
            }
        }

        if(executed){
            update();
            executed = false;
        }

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

                // TODO: stai passando due volte gli stessi parametri!
                n_peaks_final = _clean_cough_segments(final_starts, final_ends, above_locs, above_peaks, n_idxs_above_th, AUDIO_FS);

                k_free(idxs_above_th);
                k_free(final_starts);
                k_free(final_ends);
                k_free(above_locs);
                k_free(above_peaks);
            }
            
            // printf("N_PEAKS FINAL: %d\n", n_peaks_final);
            // LOG_INF("N_PEAKS: %d\n", n_peaks_final);
            // uint8_t final_data[5] = {n_peaks_final, n_peaks_final, n_peaks_final, n_peaks_final, n_peaks_final}; 
            // receive_sensor_data(final_data, (size_t)5);
            // receive_sensor_data(&n_peaks_final, (size_t)1);
            ble_receive_final_data(&n_peaks_final);

            // Reset postprocessing variables to their default value
            n_peaks = 0;

            n_idxs_above_th = 0;
        }

        debug_cnt++;

    }


    k_free(indexes_audio_f);
    k_free(indexes_imu_f);

    k_free(audio_feature_array);
    k_free(imu_feature_array);

    k_free(features_audio_model);
    k_free(features_imu_model);

    k_free(starts);
    k_free(ends);
    k_free(locs);
    k_free(peaks);

    k_free(audio_confidence);

}




void data_thread(){

    LOG_INF("RUNNING DATA THREAD!\n");

    int N_imu_data = 0;
    int N_aud_data = 0;

    uint8_t res = 0;

    while(1){


        if(N_imu_data < IMU_ARRAY_SIZE){
            // Get data from BNO086
            uint8_t *new_data = (uint8_t*)k_malloc(13*10 * sizeof(uint8_t));

            if(new_data != NULL){

                res = get_bno086_data_from_fifo(new_data); // Gets 10 samples of 13 bytes each (index included)
                
                if((res != NULL)){

                    LOG_INF("TOOK DATA (bno_foo)!");

                    for(int i=0; i<10; i++){
                        imu_data_array[N_imu_data+i][0] = (float)(uint16_t)((new_data[(i*10)+2] << 8) | (new_data[(i*10)+1]));
                        imu_data_array[N_imu_data+i][1] = (float)(uint16_t)((new_data[(i*10)+4] << 8) | (new_data[(i*10)+3]));
                        imu_data_array[N_imu_data+i][2] = (float)(uint16_t)((new_data[(i*10)+6] << 8) | (new_data[(i*10)+5]));
                        imu_data_array[N_imu_data+i][3] = (float)(uint16_t)((new_data[(i*10)+8] << 8) | (new_data[(i*10)+7]));
                        imu_data_array[N_imu_data+i][4] = (float)(uint16_t)((new_data[(i*10)+10] << 8) | (new_data[(i*10)+9]));
                        imu_data_array[N_imu_data+i][5] = (float)(uint16_t)((new_data[(i*10)+12] << 8) | (new_data[(i*10)+11]));
                    }
                    N_imu_data += 10;
                    LOG_INF("N IMU DATA: %d\n", N_imu_data);
                }

                k_free(new_data);
            }
        } else {
            LOG_INF("IMU DATA ready for proc!\n");
            memcpy(&imu_data_array_to_proc, &imu_data_array, IMU_ARRAY_SIZE*6*sizeof(float));
            N_imu_data = 0;
            IMU_ready_to_proc = true;
        }

    }

}


void start_coughE_thread(){
    k_tid_t app_thread_id = k_thread_create(&app_thread, app_thread_stack, K_THREAD_STACK_SIZEOF(app_thread_stack),
                                        cough_E, NULL, NULL, NULL, 11, 0, K_NO_WAIT);

    k_tid_t app_data_thread_id = k_thread_create(&app_data_thread, app_data_thread_stack, K_THREAD_STACK_SIZEOF(app_data_thread_stack),
                                        data_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);

    LOG_INF("COUGH-E thread initialized!\n");
}
