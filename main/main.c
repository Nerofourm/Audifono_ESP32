#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_peripherals.h"
#include "esp_log.h"
#include "board.h"
#include "driver/i2s.h"
#include "audio_idf_version.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "algorithm_stream.h"
#include "i2s_stream.h"


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

static esp_err_t i2s_driver_init(i2s_port_t port, i2s_channel_fmt_t channels, i2s_bits_per_sample_t bits)
{
    i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = bits,
        .channel_format = channels,
#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 3, 0))
        .communication_format = I2S_COMM_FORMAT_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#endif
        .tx_desc_auto_clear = true,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
    };

    i2s_driver_install(port, &i2s_cfg, 0, NULL);
    board_i2s_pin_t board_i2s_pin = {0};
    i2s_pin_config_t i2s_pin_cfg;
    get_i2s_pins(port, &board_i2s_pin);
    i2s_pin_cfg.bck_io_num = board_i2s_pin.bck_io_num;
    i2s_pin_cfg.ws_io_num = board_i2s_pin.ws_io_num;
    i2s_pin_cfg.data_out_num = board_i2s_pin.data_out_num;
    i2s_pin_cfg.data_in_num = board_i2s_pin.data_in_num;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    i2s_pin_cfg.mck_io_num = board_i2s_pin.mck_io_num;
#endif
    i2s_set_pin(port, &i2s_pin_cfg);

    return ESP_OK;
}


static int i2s_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    size_t bytes_read = 0;

    int ret = i2s_read(CODEC_ADC_I2S_PORT, buf, len, &bytes_read, wait_time);
    if (ret == ESP_OK) {
#if (CONFIG_IDF_TARGET_ESP32 && !RECORD_HARDWARE_AEC)
        algorithm_mono_fix((uint8_t *)buf, bytes_read);
        /* En resumen, el código intercambia los canales de un búfer de audio estéreo para corregir el orden de los canales.*/
#endif
    } else {
        ESP_LOGE(TAG, "i2s read failed");
    }
    return bytes_read;
}


void app_main(void)
{
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_writer;

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);


    ESP_LOGI(TAG, "[1.0] Start codec chip");

    /*
    INICIALIZAMOS EL DRIVER QUE CONECTA EL CODEC ES8311 CON EL ESP32
    */
    i2s_driver_init(I2S_NUM_0, I2S_CHANNELS, I2S_BITS);

    /*
    CONFIGURAMOS E INICIALIZA EL CODEC ES8311
    */
    audio_board_handle_t board_handle = (audio_board_handle_t) audio_calloc(1, sizeof(struct audio_board_handle));
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_codec_cfg.i2s_iface.samples = AUDIO_HAL_08K_SAMPLES; // aqui le baja de 48k a 8k.....
    board_handle->audio_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8311_DEFAULT_HANDLE);

    /*
    INICIALIZAMOS EL ADC
    */
    board_handle->adc_hal = audio_board_adc_init();

    /*
    INICIALIZAMOS EL DRIVER I2S QUE CONECTA EL ADC ES7243 CON EL ESP32
    */
    i2s_driver_init(I2S_NUM_1, I2S_CHANNELS, I2S_BITS);

/////////////////////////////////////////////
/////////////////////////////////////////////

    /*
    INICIALIZAMOS LA PIPELINE
    */
    ESP_LOGI(TAG, "[3.0] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline); //para  asegurar que hay espacio para el pipeline OJO PASSTHRU NO LO TIENE

    
    ESP_LOGI(TAG, "[3.1] Create algorithm stream for aec");
    algorithm_stream_cfg_t algo_config = ALGORITHM_STREAM_CFG_DEFAULT();

#if !RECORD_HARDWARE_AEC //Lyra T mini =1
    algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE2; //por ende es type1
    /*!< Type 1 is default used by mini-board, the reference signal and the recording signal are respectively read in from the left channel and the right channel of the same I2S */
#endif

#if CONFIG_ESP_LYRAT_MINI_V1_1_BOARD //True
    algo_config.ref_linear_factor = 3; /*!< The linear amplication factor of reference signal */
#endif

#ifdef DEBUG_ALGO_INPUT //para el debug, pero está desactivado, no se ejecuta este condicional
    algo_config.debug_input = true;
#endif

#if (CONFIG_ESP_LYRAT_MINI_V1_1_BOARD || CONFIG_ESP32_S3_KORVO2_V3_BOARD)
    algo_config.swap_ch = true; /*!< Swap left and right channels */
#endif
/*
CONFIGURACIÓN DEL PRIMER ELEMENTO DEL PIPELINE
*/
    algo_config.sample_rate = I2S_SAMPLE_RATE;  /*!< The sampling rate of the input PCM (in Hz) */
    algo_config.out_rb_size = ESP_RING_BUFFER_SIZE; /*!< Size of output ringbuffer */
    audio_element_handle_t element_algo = algo_stream_init(&algo_config); // Crea un elemento del tipo audio_element_handle_t con la función algo_stream_init con la configuración que tiene en su interior
    audio_element_set_music_info(element_algo, I2S_SAMPLE_RATE, 1, ALGORITHM_STREAM_DEFAULT_SAMPLE_BIT); 
    //esp_err_t audio_element_set_music_info(audio_element_handle_t el, int sample_rates, int channels, int bits)
    audio_element_set_read_cb(element_algo, i2s_read_cb, NULL);
    audio_element_set_input_timeout(element_algo, portMAX_DELAY);
    //#define portMAX_DELAY ( TickType_t ) 0xffffffffUL


    /*
    DEFINIMOS EL 2DO ELEMENTO DE LA PIPELINE
    */

    ESP_LOGI(TAG, "[3.1] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    /*
    REGISTRAMOS LOS ELEMENTOS A LA PIPELINE
    */
    ESP_LOGI(TAG, "[3.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, element_algo, "algo");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s_write");

    /*
    UNIR LOS ELEMENTOS
    */
    ESP_LOGI(TAG, "[3.5] Link it together [codec_chip]-->algorithm-->i2s_stream_writer-->[codec_chip]");
    const char *link_tag[2] = {"algo", "i2s_write"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    /*
    ESTABLECEMOS AL MIRON
    */
    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);
    
    /*
    INICIAMOS LA PIPELINE
    */

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

    audio_pipeline_unregister(pipeline, element_algo);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Stop all periph before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    esp_periph_set_destroy(set);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(element_algo);
    audio_element_deinit(i2s_stream_writer);
}