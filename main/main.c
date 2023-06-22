#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_peripherals.h"
#include "esp_log.h"
#include "board.h"
#include "audio_idf_version.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "algorithm_stream.h"
#include "i2s_stream.h"
#include "audio_event_iface.h"

static const char *TAG = "AUDIFONO2";

/* Debug original input data for AEC feature*/
// #define DEBUG_ALGO_INPUT

#define I2S_SAMPLE_RATE     8000
#if CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
#define I2S_CHANNELS        I2S_CHANNEL_FMT_RIGHT_LEFT
#else
#define I2S_CHANNELS        I2S_CHANNEL_FMT_ONLY_LEFT
#endif
#define I2S_BITS            CODEC_ADC_BITS_PER_SAMPLE

/* The AEC internal buffering mechanism requires that the recording signal
   is delayed by around 0 - 10 ms compared to the corresponding reference (playback) signal. */
#define DEFAULT_REF_DELAY_MS    0
#define ESP_RING_BUFFER_SIZE    256

void app_main(void)
{
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_reader,i2s_stream_writer; //writer puerto 0

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);


    // Setup audio codec
    ESP_LOGI(TAG, "[1.0] Start codec chip");

    // audio_board_handle_t board_handle = (audio_board_handle_t) audio_calloc(1, sizeof(struct audio_board_handle));
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    board_handle->adc_hal = audio_board_adc_init();
    

    ////// FUNCTIOOOOOOOOON
    ESP_LOGI(TAG, "[1.1] Initialize recorder pipeline");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);



    ESP_LOGI(TAG, "[1.2] Create audio elements for pipeline");
    
    i2s_stream_cfg_t i2s_read_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_read_cfg.type = AUDIO_STREAM_READER; // PORT 1 LECTOR
    i2s_read_cfg.i2s_port = 1;
    i2s_stream_reader = i2s_stream_init(&i2s_read_cfg);
    //mem_assert(i2s_stream_reader);
    //audio_element_set_music_info(i2s_stream_reader, 48000, 2, 16);


    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_read_cfg.i2s_port = 0;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);
    //mem_assert(i2s_stream_writer);
    //audio_element_set_music_info(i2s_stream_writer, 48000, 2, 16);


    ESP_LOGI(TAG, "[1.3] Register audio elements to  pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s_read");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s_write");

    ESP_LOGI(TAG, "[2] Link it together [codec_chip]-->i2s_stream_reader-->i2s_stream_writer-->[codec_chip]");
    const char *link_tag[2] = {"i2s_read", "i2s_write"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);


    ESP_LOGI(TAG, "[ 3 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    audio_pipeline_set_listener(pipeline, evt); //AÑADIDO POR HERENCIA DE ALGORITHM

    ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);
    
    ESP_LOGI(TAG, "[ 6 ] Listen for all pipeline events");
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }
        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            ESP_LOGW(TAG, "[ * ] Stop event received");
            break;
        }
    }


    ESP_LOGI(TAG, "[ 7 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, i2s_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    


    


    

 
//     // INICIALIZAMOS EL ADC

//     board_handle->adc_hal = audio_board_adc_init();





//     ESP_LOGI(TAG, "[3.1] Create algorithm stream for aec");
//     algorithm_stream_cfg_t algo_config = ALGORITHM_STREAM_CFG_DEFAULT();

// #if !RECORD_HARDWARE_AEC //Lyra T mini =1
//     algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE2; //por ende es type1
//     /*!< Type 1 is default used by mini-board, the reference signal and the recording signal are respectively read in from the left channel and the right channel of the same I2S */
// #endif

// #if CONFIG_ESP_LYRAT_MINI_V1_1_BOARD //True
//     algo_config.ref_linear_factor = 3; /*!< The linear amplication factor of reference signal */
// #endif

// #ifdef DEBUG_ALGO_INPUT //para el debug, pero está desactivado, no se ejecuta este condicional
//     algo_config.debug_input = true;
// #endif

// #if (CONFIG_ESP_LYRAT_MINI_V1_1_BOARD || CONFIG_ESP32_S3_KORVO2_V3_BOARD)
//     algo_config.swap_ch = true; /*!< Swap left and right channels */
// #endif

}