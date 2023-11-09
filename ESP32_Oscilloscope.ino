#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <soc/syscon_reg.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "esp_adc_cal.h"
#include "filters.h"

/*rotary encoder start*/
#include <ESP32RotaryEncoder.h>

#define DO_ENCODER_VCC -1
#define DI_ENCODER_SW  32 // connected to 32 but not handled by rotary encoder library
#define DI_ENCODER_B   15 // DT
#define DI_ENCODER_A   35 // CLK

const uint8_t LONG_PRESS = 500; // push button long press duration

RotaryEncoder rotaryEncoder( DI_ENCODER_A, DI_ENCODER_B, DI_ENCODER_SW, DO_ENCODER_VCC );
/*rotary encoder end*/

//#define DEBUG_SERIAL
//#define DEBUG_BUFF
#define DELAY 1000

// Width and height of sprite
#define WIDTH  240
#define HEIGHT 280

#define ADC_CHANNEL   ADC1_CHANNEL_5  // GPIO33
#define NUM_SAMPLES   1000            // number of samples
#define I2S_NUM         (0)
#define BUFF_SIZE 50000
#define B_MULT BUFF_SIZE/NUM_SAMPLES

TFT_eSPI    tft = TFT_eSPI();         // Declare object "tft"

TFT_eSprite spr = TFT_eSprite(&tft);  // Declare Sprite object "spr" with pointer to "tft" object

esp_adc_cal_characteristics_t adc_chars;

TaskHandle_t task_menu;
TaskHandle_t task_adc;

float v_div = 825;
float s_div = 10;
float offset = 0;
float toffset = 0;
uint8_t current_filter = 1;

//options handler
enum Option {
  None,
  Autoscale,
  Vdiv,
  Sdiv,
  Offset,
  TOffset,
  Filter,
  Stop,
  Mode,
  Single,
  Clear,
  Reset,
  Probe,
  UpdateF,
  Cursor1,
  Cursor2
};

int8_t volts_index = 0;

int8_t tscale_index = 0;

uint8_t opt = None;

bool menu = false;
bool info = true;
bool set_value  = false;

float RATE = 1000; //in ksps --> 1000 = 1Msps

bool auto_scale = false;

bool full_pix = true;

bool stop = false;

bool stop_change = false;

uint16_t i2s_buff[BUFF_SIZE];

bool single_trigger = false;
bool data_trigger = false;

bool updating_screen = false;
bool new_data = false;
bool menu_action = false;
uint8_t digital_wave_option = 0; //0-auto | 1-analog | 2-digital data (SERIAL/SPI/I2C/etc)
int btnok,btnpl,btnmn,btnbk;
void IRAM_ATTR btok()
{
  btnok = 1;
}
void IRAM_ATTR btplus()
{
  btnpl = 1;
}
void IRAM_ATTR btminus()
{
  btnmn = 1;
}
void IRAM_ATTR btback()
{
  btnbk = 1;
}
void setup() {

/*rotary encoder start*/
  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
  rotaryEncoder.setBoundaries( -1, 1, false ); // reduces rotary encoder motion from min/max to up/down in endless loop
  rotaryEncoder.onTurned( &knobCallback );
  rotaryEncoder.onPressed( &buttonCallback );
  rotaryEncoder.begin();
/*rotary encoder end*/

  Serial.begin(115200);

  configure_i2s(1000000);

  setup_screen();

  characterize_adc();
#ifdef DEBUG_BUF
  debug_buffer();
#endif

  xTaskCreatePinnedToCore(
    core0_task,
    "menu_handle",
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &task_menu,  /* Task handle. */
    0); /* Core where the task should run */

  xTaskCreatePinnedToCore(
    core1_task,
    "adc_handle",
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    3,  /* Priority of the task */
    &task_adc,  /* Task handle. */
    1); /* Core where the task should run */
}


void core0_task( void * pvParameters ) {

  (void) pvParameters;

  for (;;) {
    menu_handler();

    if (new_data || menu_action) {
      new_data = false;
      menu_action = false;

      updating_screen = true;
      update_screen(i2s_buff, RATE);
      updating_screen = false;
      vTaskDelay(pdMS_TO_TICKS(10));
      Serial.println("CORE0");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

}

void core1_task( void * pvParameters ) {

  (void) pvParameters;

  for (;;) {
    if (!single_trigger) {
      while (updating_screen) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      if (!stop) {
        if (stop_change) {
          i2s_adc_enable(I2S_NUM_0);
          stop_change = false;
        }
        ADC_Sampling(i2s_buff);
        new_data = true;
      }
      else {
        if (!stop_change) {
          i2s_adc_disable(I2S_NUM_0);
          i2s_zero_dma_buffer(I2S_NUM_0);
          stop_change = true;
        }
      }
      Serial.println("CORE1");
      vTaskDelay(pdMS_TO_TICKS(300));
    }
    else {
      float old_mean = 0;
      while (single_trigger) {
        stop = true;
        ADC_Sampling(i2s_buff);
        float mean = 0;
        float max_v, min_v;
        peak_mean(i2s_buff, BUFF_SIZE, &max_v, &min_v, &mean);

        //signal captured (pp > 0.4V || changing mean > 0.2V) -> DATA ANALYSIS
        if ((old_mean != 0 && fabs(mean - old_mean) > 0.2) || to_voltage(max_v) - to_voltage(min_v) > 0.05) {
          float freq = 0;
          float period = 0;
          uint32_t trigger0 = 0;
          uint32_t trigger1 = 0;

          //if analog mode OR auto mode and wave recognized as analog
          bool digital_data = !false;
          if (digital_wave_option == 1) {
            trigger_freq_analog(i2s_buff, RATE, mean, max_v, min_v, &freq, &period, &trigger0, &trigger1);
          }
          else if (digital_wave_option == 0) {
            digital_data = digital_analog(i2s_buff, max_v, min_v);
            if (!digital_data) {
              trigger_freq_analog(i2s_buff, RATE, mean, max_v, min_v, &freq, &period, &trigger0, &trigger1);
            }
            else {
              trigger_freq_digital(i2s_buff, RATE, mean, max_v, min_v, &freq, &period, &trigger0);
            }
          }
          else {
            trigger_freq_digital(i2s_buff, RATE, mean, max_v, min_v, &freq, &period, &trigger0);
          }

          single_trigger = false;
          new_data = true;
          Serial.println("Single GOT");
          //return to normal execution in stop mode
        }

        vTaskDelay(pdMS_TO_TICKS(1));   //time for the other task to start (low priorit)

      }
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }
}

/*rotary encoder start*/
void knobCallback( int value )
{
  if(rotaryEncoder.getEncoderValue() == 1){
    //Serial.println("up");	// for debugging
    btnpl = 1;
  }
  else if(rotaryEncoder.getEncoderValue() == -1){
    //Serial.println("up");	// for debugging
    btnmn = 1;
  }
  else {
    btnpl = 0;
    btnmn = 0;
  }
  //Serial.printf( "Value: %i\n", value );	// for debugging
}

void buttonShortPress()
{
  btnbk = 0;
  btnok = 1;
	// Serial.println( "boop!" ); // for debugging
}

void buttonLongPress()
{
  btnok = 0;
  btnbk = 1;
  // Serial.println( "BOOOOOOOOOOP!" ); // for debugging
}

void buttonCallback( unsigned long duration )
{
	if( duration > LONG_PRESS )
	{
		buttonLongPress();
	}
	else
	{
		buttonShortPress();
	}
}
/*rotary encoder end*/

void loop() {}
