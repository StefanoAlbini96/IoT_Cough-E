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

LOG_MODULE_REGISTER(launcher, LOG_LEVEL_INF);

#define IMU_ARRAY_SIZE 50       // 0.5 sec at 100 Hz
#define AUD_ARRAY_SIZE 6400     // 0.8 sec at 8 kHz

// // Global array, stored in RAM
// float imu_data_array[IMU_ARRAY_SIZE][6];
// float imu_data_array_to_proc[IMU_ARRAY_SIZE][6];
// int N_imu_data = 0;
// bool IMU_ready_to_proc = false;

// float aud_data_array[AUD_ARRAY_SIZE];
int N_aud_data = 0;
bool AUD_ready_to_proc = false;


// #define IMU_BUF_SIZE    100
// static float circ_buffer_imu[IMU_BUF_SIZE][6];
// static int write_index_imu = 0;
// static int read_index_imu = 0;
// static int data_count_imu = 0;

// static struct k_mutex imu_buffer_mutex;
// K_SEM_DEFINE(imu_sem, 0, 1);

// float processing_imu_wind[IMU_ARRAY_SIZE][6];



#define AUD_BUF_SIZE    7000
static float circ_buffer_aud[AUD_BUF_SIZE];
static int write_index_aud = 0;
static int read_index_aud = 0;
static int data_count_aud = 0;

static struct k_mutex aud_buffer_mutex;
K_SEM_DEFINE(aud_sem, 0, 1);

float processing_aud_wind[AUD_ARRAY_SIZE];


// Set to true when the execution took place.
// Useful to trigger the FSM update only when needed
bool executed = false;  

bool ready = false;


// K_THREAD_STACK_DEFINE(app_thread_stack, 4098);
K_THREAD_STACK_DEFINE(app_thread_stack, 4098);
struct k_thread app_thread;


K_THREAD_STACK_DEFINE(app_data_thread_stack, 20000);
struct k_thread app_data_thread;


// void cough_E(){
//     uint8_t dbg;

//     // k_msleep(1000);

//     // LOG_INF("COUGH-E RUNNING!\n");

//     // These two arrays contain the indexes of the features that are going to be extracted
//     int16_t *indexes_audio_f = (int16_t*)k_malloc(N_AUDIO_FEATURES * sizeof(int16_t));
//     int8_t *indexes_imu_f = (int8_t*)k_malloc(N_IMU_FEATURES * sizeof(int8_t));

//     int16_t idx = 0;
//     for(int16_t i=0; i<Number_AUDIO_Features; i++){
//         if(audio_features_selector[i] == 1){
//             indexes_audio_f[idx] = i;
//             idx++;
//         }
//     }
//     idx = 0;
//     for(int8_t i=0; i<Number_IMU_Features; i++){
//         if(imu_features_selector[i] == 1){
//             indexes_imu_f[idx] = i;
//             idx++;
//         }
//     }

//     ////    AUDIO FEATURES    ////
//     // Array for containing the audio features values. The order is the same as of the
//     // features families enum
//     float  *audio_feature_array = (float*)k_malloc(Number_AUDIO_Features * sizeof(float));
//     memset(audio_feature_array, 0.0, Number_AUDIO_Features);


//     ////    IMU FEATURES    ////
//     float *imu_feature_array = (float*)k_malloc(Number_IMU_Features * sizeof(float)); // To store all the possible IMU features
//     memset(imu_feature_array, 0.0, Number_IMU_Features);

//     // Array for the features set of the audio model
//     float* features_audio_model = (float*)k_malloc(TOT_FEATURES_AUDIO_MODEL_AUDIO * sizeof(float));

//     float audio_proba = 0.0;

//     // Array for the features set of the imu model
//     float* features_imu_model = (float*)k_malloc(TOT_FEATURES_IMU_MODEL_IMU * sizeof(float));

//     float imu_proba = 0.0;

//     // Postprocessing arrays and variables
//     uint16_t *starts = (uint16_t*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(uint16_t));
//     uint16_t *ends = (uint16_t*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(uint16_t));
//     uint16_t *locs = (uint16_t*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(uint16_t));
//     float *peaks = (float*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(float));

//     // Number of peaks found from last output
//     uint16_t n_peaks = 0;

//     // Number of peaks found in the current window
//     uint16_t new_added = 0;

//     // Confidence of the model per each peak found
//     float *audio_confidence = (float*)k_malloc(MAX_PEAKS_EXPECTED * sizeof(float*));

//     // Index of the start of the current window (depending on the model to use, it indexes the AUDIO or the IMU signal)
//     uint32_t idx_start_window = 0;

//     // Number of peaks for which the model confidence is above the threshold
//     uint16_t n_idxs_above_th = 0;

//     int debug_cnt = 0;

//     init_state();

//     // Looping through the windows
//     while(1){

//         // idx_start_window = get_idx_window();

//         if(fsm_state.model == IMU_MODEL){

//             // if(idx_start_window >= IMU_LEN){
//             //     // break;
//             //     init_state();
//             //     idx_start_window = get_idx_window();
//             // }

//             // Wait for enough data
//             // k_sem_take(&imu_sem, K_FOREVER);
//             // k_mutex_lock(&imu_buffer_mutex, K_FOREVER);

//             // // Start processing only if there are enough samples
//             // // if(data_count_imu >= WINDOW_SAMP_IMU){
//             //     // Copy data from circular buffer to the processing window
//             // // for (int i = 0; i < WINDOW_SAMP_IMU; i++) {
//             // //     processing_imu_wind[i][0] = circ_buffer_imu[(read_index_imu + i) % IMU_BUF_SIZE][0];
//             // //     processing_imu_wind[i][1] = circ_buffer_imu[(read_index_imu + i) % IMU_BUF_SIZE][1];
//             // //     processing_imu_wind[i][2] = circ_buffer_imu[(read_index_imu + i) % IMU_BUF_SIZE][2];
//             // //     processing_imu_wind[i][3] = circ_buffer_imu[(read_index_imu + i) % IMU_BUF_SIZE][3];
//             // //     processing_imu_wind[i][4] = circ_buffer_imu[(read_index_imu + i) % IMU_BUF_SIZE][4];
//             // //     processing_imu_wind[i][5] = circ_buffer_imu[(read_index_imu + i) % IMU_BUF_SIZE][5];
//             // // }
//             // // Move read pointer to overlap position
//             // read_index_imu = (read_index_imu + IMU_STEP) % IMU_BUF_SIZE;
//             // data_count_imu -= IMU_STEP;
            
//             // k_mutex_unlock(&imu_buffer_mutex);

//             // // dbg = 100;
//             // // ble_receive_final_data(&dbg);

//             // // Extract IMU features
//             // // imu_features(imu_features_selector, &imu_in[idx_start_window], WINDOW_SAMP_IMU, imu_feature_array);
//             // // imu_features(imu_features_selector, &processing_imu_wind[0], WINDOW_SAMP_IMU, imu_feature_array);

//             // // dbg = 101;
//             // // ble_receive_final_data(&dbg);

//             // // Fill the array of final imu features to feed into the IMU model
//             // for(int16_t j=0; j<N_IMU_FEATURES; j++){
//             //     features_imu_model[j] = imu_feature_array[indexes_imu_f[j]];
//             // }
//             // if(imu_bio_feats_selector[0] == 1){
//             //     features_imu_model[N_IMU_FEATURES] = gender;
//             // }
//             // if(imu_bio_feats_selector[1] == 1){
//             //     features_imu_model[N_IMU_FEATURES+1] = bmi;
//             // }

//             // // Predict with the IMU model
//             // imu_proba = imu_predict(features_imu_model);

//             // // dbg = 102;
//             // // ble_receive_final_data(&dbg);

//             // // Update the output of the FSM
//             // if(imu_proba>=IMU_TH){
//             //     fsm_state.model_cls_out = COUGH_OUT;
//             // } else {
//             //     fsm_state.model_cls_out = NON_COUGH_OUT;
//             // }

//             // executed = true;
//             // } else {
//             //     k_mutex_unlock(&imu_buffer_mutex);
//             //     k_msleep(100);
//             // }
//         }
//         else { // AUDIO MODEL
        
        
//             LOG_INF("APP SEM: %d", k_sem_count_get(&aud_sem));
//             if(k_sem_take(&aud_sem, K_FOREVER) == 0) {
//                 LOG_INF("APP SEM taken successfully!");
//                 LOG_INF("SEM coun: %d", k_sem_count_get(&aud_sem));

//             } else {
//                 LOG_INF("APP SEM take failed!\n");
//             }

//             if(k_mutex_lock(&aud_buffer_mutex, K_FOREVER) == 0){
//                 LOG_INF("App MUTEX locked!");
//             } else {
//                 LOG_INF("App MUTEX locked failed!");
//             }
            
//             // Get a window from the circular buffer
//             for(int i=0; i<WINDOW_SAMP_AUDIO; i++){
//                 processing_aud_wind[i] = circ_buffer_aud[(read_index_aud + i) % AUD_BUF_SIZE];
//             }

//             // Move read pointer to overlap position
//             read_index_aud = (read_index_aud + AUDIO_STEP) % AUD_BUF_SIZE;
//             data_count_aud -= AUDIO_STEP;

//             k_mutex_unlock(&aud_buffer_mutex);
//             LOG_INF("App MUTEX unlock!");

//             // int dbg = 100;
//             // ble_receive_final_data(&dbg);

//             // Extract AUDIO features
//             audio_features(audio_features_selector, &processing_aud_wind[0], WINDOW_SAMP_AUDIO, AUDIO_FS, audio_feature_array);

//             // Fill the array of fifeatures_imu_modelnal audio features to feed into the AUDIO model
//             for(int16_t j=0; j<N_AUDIO_FEATURES; j++){
//                 features_audio_model[j] = audio_feature_array[indexes_audio_f[j]];
//             }
//             if(audio_bio_feats_selector[0] == 1){
//                 features_audio_model[N_AUDIO_FEATURES] = gender;
//             }
//             if(audio_bio_feats_selector[1] == 1){
//                 features_audio_model[N_AUDIO_FEATURES+1] = bmi;
//             }

//             audio_proba = audio_predict(features_audio_model);

//             // Update the output of the FSM
//             if(audio_proba >= AUDIO_TH){
//                 fsm_state.model_cls_out = COUGH_OUT;
//             } else {
//                 fsm_state.model_cls_out = NON_COUGH_OUT;
//             }

//             // Identify the peaks   
//             _get_cough_peaks(&processing_aud_wind[0], WINDOW_SAMP_AUDIO, AUDIO_FS, &starts[n_peaks], &ends[n_peaks], &locs[n_peaks], &peaks[n_peaks], &new_added);

//             // Readjust the indexes for the position of the current window (to get absolute index)
//             for(uint16_t j=0; j<new_added; j++){
//                 starts[n_peaks+j] += idx_start_window*AUDIO_STEP;
//                 ends[n_peaks+j] += idx_start_window*AUDIO_STEP;
//                 locs[n_peaks+j] += (idx_start_window*AUDIO_STEP);
//                 audio_confidence[n_peaks+j] = audio_proba;
//             }
//             n_peaks += new_added;

//             executed = true;
//         }

//         if(executed){
//             // dbg = 200;
//             // ble_receive_final_data(&dbg);
//             update();
//             executed = false;
//         }

//         if(check_postprocessing()){
//             dbg = 202;
//             ble_receive_final_data(&dbg);

//             uint16_t n_peaks_final = 0;

//             if(n_peaks > 0){
//                 // Keeps track of the indexes of the peaks for which the model confidence is above the threshold 
//                 uint16_t *idxs_above_th = (uint16_t*)k_malloc(n_peaks * sizeof(uint16_t));


//                 for(uint16_t i=0; i<n_peaks; i++){
//                     if(audio_confidence[i] >= AUDIO_TH){
//                         idxs_above_th[n_idxs_above_th] = i;
//                         n_idxs_above_th++;
//                     }
//                 }

//                 uint16_t *final_starts = (uint16_t*)k_malloc(n_idxs_above_th * sizeof(uint16_t));
//                 uint16_t *final_ends = (uint16_t*)k_malloc(n_idxs_above_th * sizeof(uint16_t));
//                 uint16_t *above_locs = (uint16_t*)k_malloc(n_idxs_above_th * sizeof(uint16_t));
//                 float *above_peaks = (float*)k_malloc(n_idxs_above_th * sizeof(float));


//                 for(uint16_t i=0; i<n_idxs_above_th; i++){
//                     final_starts[i] = starts[idxs_above_th[i]];
//                     final_ends[i] = ends[idxs_above_th[i]];
//                     above_locs[i] = locs[idxs_above_th[i]];
//                     above_peaks[i] = peaks[idxs_above_th[i]];
//                 }

//                 // TODO: stai passando due volte gli stessi parametri!
//                 n_peaks_final = _clean_cough_segments(final_starts, final_ends, above_locs, above_peaks, n_idxs_above_th, AUDIO_FS);

//                 k_free(idxs_above_th);
//                 k_free(final_starts);
//                 k_free(final_ends);
//                 k_free(above_locs);
//                 k_free(above_peaks);
//             }
            
//             // printf("N_PEAKS FINAL: %d\n", n_peaks_final);
//             // LOG_INF("N_PEAKS: %d\n", n_peaks_final);
//             // uint8_t final_data[5] = {n_peaks_final, n_peaks_final, n_peaks_final, n_peaks_final, n_peaks_final}; 
//             // receive_sensor_data(final_data, (size_t)5);
//             // receive_sensor_data(&n_peaks_final, (size_t)1);
//             ble_receive_final_data(&n_peaks_final);

//             // Reset postprocessing variables to their default value
//             n_peaks = 0;

//             n_idxs_above_th = 0;
//         }

//         debug_cnt++;

//     }


//     k_free(indexes_audio_f);
//     k_free(indexes_imu_f);

//     k_free(audio_feature_array);
//     k_free(imu_feature_array);

//     k_free(features_audio_model);
//     k_free(features_imu_model);

//     k_free(starts);
//     k_free(ends);
//     k_free(locs);
//     k_free(peaks);

//     k_free(audio_confidence);

// }




void data_thread(){

    LOG_INF("RUNNING DATA THREAD!\n");

    int N_imu_data = 0;
    int N_aud_data = 0;

    uint8_t res = 0;

    while(1){

        // Get data from BNO086
        // uint8_t *new_data = (uint8_t*)k_malloc(13*10 * sizeof(uint8_t));

        // if(new_data != NULL){

        //     res = get_bno086_data_from_fifo(new_data); // Gets 10 samples of 13 bytes each (index included)
            
        //     if((res != NULL)){

        //         LOG_INF("TOOK DATA (bno_foo)!");

        //         int dbg = 161;
        //         ble_receive_final_data(&dbg);

        //         // k_mutex_lock(&imu_buffer_mutex, K_FOREVER);

        //         for(int i=0; i<10; i++){
        //             circ_buffer_imu[write_index_imu][0] = (float)(uint16_t)((new_data[(i*10)+2] << 8) | (new_data[(i*10)+1]));
        //             circ_buffer_imu[write_index_imu][1] = (float)(uint16_t)((new_data[(i*10)+4] << 8) | (new_data[(i*10)+3]));
        //             circ_buffer_imu[write_index_imu][2] = (float)(uint16_t)((new_data[(i*10)+6] << 8) | (new_data[(i*10)+5]));
        //             circ_buffer_imu[write_index_imu][3] = (float)(uint16_t)((new_data[(i*10)+8] << 8) | (new_data[(i*10)+7]));
        //             circ_buffer_imu[write_index_imu][4] = (float)(uint16_t)((new_data[(i*10)+10] << 8) | (new_data[(i*10)+9]));
        //             circ_buffer_imu[write_index_imu][5] = (float)(uint16_t)((new_data[(i*10)+12] << 8) | (new_data[(i*10)+11]));

        //             write_index_imu = (write_index_imu + 1) % IMU_BUF_SIZE;

        //             if (data_count_imu < IMU_BUF_SIZE) {
        //                 data_count_imu++;
        //             } else {
        //                 // Overwrite oldest data (move read pointer)
        //                 read_index_imu = (read_index_imu + 1) % IMU_BUF_SIZE;
        //             }

        //             // Signal the consumer if enough data is ready
        //             // if (data_count_imu >= WINDOW_SAMP_IMU) {
        //             //     k_sem_give(&imu_sem);
        //             // }
        //         }

        //         // k_mutex_unlock(&imu_buffer_mutex);
        //         // LOG_INF("N IMU DATA: %d\n", N_imu_data);
        //     }

        //     k_free(new_data);

        // //     }
        // // } else {
        // //     LOG_INF("IMU DATA ready for proc!\n");
        // //     memcpy(&imu_data_array_to_proc, &imu_data_array, IMU_ARRAY_SIZE*6*sizeof(float));
        // //     N_imu_data = 0;
        // //     IMU_ready_to_proc = true;
        // }

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
                    circ_buffer_aud[write_index_aud] = (float)((new_data_aud[(i*2)+1] << 8) | new_data_aud[(i*2)]);

                    write_index_aud = (write_index_aud + 1) % AUD_BUF_SIZE;

                    if (data_count_aud < AUD_BUF_SIZE) {
                        data_count_aud++;
                    } else {
                        // Overwrite oldest data (move read pointer)
                        read_index_aud = (read_index_aud + 1) % AUD_BUF_SIZE;
                    }
                }

                LOG_INF("Data count: %d\n", data_count_aud);
                // Signal the consumer if enough data is ready
                // LOG_INF("DAT SEM: %d", k_sem_count_get(&aud_sem));
                if (data_count_aud >= WINDOW_SAMP_AUDIO) {
                    if (k_sem_count_get(&aud_sem) == 0){
                        k_sem_give(&aud_sem);
                        // LOG_INF("Data SEM give!");
                        // LOG_INF("SEM coun: %d", k_sem_count_get(&aud_sem));
                    }
                }

                // if (data_count_aud >= WINDOW_SAMP_AUDIO){
                //     ready = true;
                // }

                k_mutex_unlock(&aud_buffer_mutex);
                // LOG_INF("Data MUTEX unlock!\n");
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


    init_state();

    while(1){
        k_sem_take(&aud_sem, K_FOREVER);
        LOG_INF("SEM TAKEN!");
        k_mutex_lock(&aud_buffer_mutex, K_FOREVER);

        // LOG_INF("%f", circ_buffer_aud[read_index_aud]);
        // int t = 100;
        // ble_receive_final_data(&t);
        
        
        // Get a window from the circular buffer
        for(int i=0; i<WINDOW_SAMP_AUDIO; i++){
            processing_aud_wind[i] = circ_buffer_aud[(read_index_aud + i) % AUD_BUF_SIZE];
        }
        read_index_aud = (read_index_aud + AUDIO_STEP) % AUD_BUF_SIZE;
        data_count_aud -= AUDIO_STEP;

        // Extract AUDIO features
        audio_features(audio_features_selector, &processing_aud_wind[0], WINDOW_SAMP_AUDIO, AUDIO_FS, audio_feature_array);
        LOG_INF("Feats!");

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
        LOG_INF("Predict --> proba: %f", audio_proba);

        // Update the output of the FSM
        if(audio_proba >= AUDIO_TH){
            fsm_state.model_cls_out = COUGH_OUT;
        } else {
            fsm_state.model_cls_out = NON_COUGH_OUT;
        }

        // Identify the peaks   
        _get_cough_peaks(&processing_aud_wind[0], WINDOW_SAMP_AUDIO, AUDIO_FS, &starts[n_peaks], &ends[n_peaks], &locs[n_peaks], &peaks[n_peaks], &new_added);
        LOG_INF("Cough peaks!");

        // Readjust the indexes for the position of the current window (to get absolute index)
        for(uint16_t j=0; j<new_added; j++){
            starts[n_peaks+j] += idx_start_window*AUDIO_STEP;
            ends[n_peaks+j] += idx_start_window*AUDIO_STEP;
            locs[n_peaks+j] += (idx_start_window*AUDIO_STEP);
            audio_confidence[n_peaks+j] = audio_proba;
        }
        n_peaks += new_added;


        update();
        LOG_INF("Time last: %f\n", fsm_state.time_from_last_out);

        if(check_postprocessing()){
            LOG_INF("SEM TAKEN!");

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
            
            ble_receive_final_data(&n_peaks_final);

            // Reset postprocessing variables to their default value
            n_peaks = 0;

            n_idxs_above_th = 0;
        }

        k_mutex_unlock(&aud_buffer_mutex);
    }
}


void start_coughE_thread(){

    k_mutex_init(&aud_buffer_mutex);
    k_sem_init(&aud_sem, 0, 1);

    // k_tid_t app_thread_id = k_thread_create(&app_thread, app_thread_stack, K_THREAD_STACK_SIZEOF(app_thread_stack),
    //                                     cough_E, NULL, NULL, NULL, 11, 0, K_NO_WAIT);
    k_tid_t app_thread_id = k_thread_create(&app_thread, app_thread_stack, K_THREAD_STACK_SIZEOF(app_thread_stack),
                                        iot_cough_E, NULL, NULL, NULL, 11, 0, K_NO_WAIT);

    k_tid_t app_data_thread_id = k_thread_create(&app_data_thread, app_data_thread_stack, K_THREAD_STACK_SIZEOF(app_data_thread_stack),
                                        data_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);

    LOG_INF("COUGH-E thread initialized!\n");
}
