#include <Arduino.h>
#include "FreeRTOS.h"
//#include <Arduino_FreeRTOS.h>
#include "task.h"
#include <avr/io.h>
#include <Wire.h>

#define IS_CLOSE_ENOUGH(a, b, eps) \
        (((a) > (b) ? (a) - (b) : (b) - (a)) <= (eps))

#define MAX_COMBINATION_SIZE 10
#define SLAVE_ADDRESS 0x08

volatile static uint8_t saved_combination[MAX_COMBINATION_SIZE];
volatile static uint8_t is_password_set = 0;
volatile bool silence_buzzer = false;
volatile bool use_hardware_config = true;

typedef struct {
    uint8_t attempts_count;
    uint8_t failed_count;
    uint8_t success_count;
    uint8_t alarm_count;
    uint8_t current_code_length;
    uint16_t current_delay_ms;
} SecurityLogs;

volatile SecurityLogs logs = {0, 0, 0, 0, 4, 500};

static void vUpdateCode(void* pvParameters);
static void vGreenBlinkLed(void* pvParameters);
static void vSecurityCheck(void* pvParameters);
void receiveEvent(int howMany);
void requestEvent();

const uint8_t redLed   = _BV(PD2);
const uint8_t greenLed = _BV(PD3);
const uint8_t resetBtn = _BV(PD4);
const uint8_t buzzerPin = _BV(PD6);
const uint8_t ultraPin  = _BV(PD7);

// -- Low level functions --


void init_adc()
{
    ADMUX = (1 << REFS0);
    ADMUX &= 0xF0;
    ADCSRA |= (1 << ADEN ) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
/*
uint16_t read_rotary()
{
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}*/

uint16_t read_adc(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint8_t to_code_digit(uint16_t value)
{
    if (value < 0) value = 0;
    if (value > 1000) value = 1000;

    return (value * 9) / 1000;
}

long map_value(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void)
{
    DDRD |= (redLed | greenLed | buzzerPin); // PD2 and PD3 as outputs
    DDRD &= ~resetBtn;
    PORTD &= ~resetBtn;
    init_adc();
    Serial.begin(9600);

    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    // Create task #1
    TaskHandle_t codeUpdate_handle;
    xTaskCreate                     //documented here: https://www.freertos.org/a00125.html
    (
        vUpdateCode,                //pointer function to the handler
        (const char*)"updateCode",  //naming the task
        configMINIMAL_STACK_SIZE,   //stack size
        NULL,                       //parameters of the handler
        1U,                         //priority
        &codeUpdate_handle          //address of task handler
    );

    // Create task #2
    // This task is completely useless
    TaskHandle_t greenBlink_handle;
    xTaskCreate
    (
        vGreenBlinkLed,
        (const char*)"greenBlink",
        192,
        NULL,
        1U,
        &greenBlink_handle
    );
    xTaskCreate(
        vSecurityCheck,
        "SecurityTask",
        128,
        NULL,
        2U,
        NULL
        );

    // Start scheduler.
    vTaskStartScheduler();

    return 0;
}

void requestEvent() {
    Wire.write((uint8_t*)&logs, sizeof(logs));
}

/**************************************************************************//**
 * \fn static void vGreenBlinkLed(void* pvParameters)
 *
 * \brief toggle the green led.
 *
 * \param[in]   pvParameters
 ******************************************************************************/
void receiveEvent(int howMany) {
    if (Wire.available()) {
        char command = Wire.read();

        if (command == 'U') { // UNLOCK
            is_password_set = 0;
            silence_buzzer = false;
            PORTD &= ~redLed;
            PORTD &= ~buzzerPin;
            //Serial.println(F("[I2C] Unlock"));
        }
        else if (command == 'S') { // SILENCE
            silence_buzzer = true;
            PORTD &= ~buzzerPin;
          //  Serial.println(F("[I2C] Silence buzzer"));
        }
        else if (command == 'R') {
            logs.attempts_count = 0;
            logs.failed_count = 0;
            logs.success_count = 0;
            logs.alarm_count = 0;
        }
        else if (command == 'D') { // DELAY
            if (Wire.available() >= 2) {
                uint8_t lowByte = Wire.read();
                uint8_t highByte = Wire.read();
                logs.current_delay_ms = (highByte << 8) | lowByte;
                use_hardware_config = false;
            }
        }
        else if (command == 'L') { // LENGTH
            if (Wire.available() >= 1) {
                logs.current_code_length = Wire.read();
                use_hardware_config = false;
                is_password_set = 0;
                PORTD &= ~redLed;
                silence_buzzer = false;
            }
        }
        else if (command == 'C') { // SET CODE
            int codeSize = Wire.available();

            if (codeSize > 0 && codeSize <= MAX_COMBINATION_SIZE) {
                logs.current_code_length = codeSize;

                for (int i = 0; i < codeSize; i++) {
                    saved_combination[i] = Wire.read();
                }

                is_password_set = 1;
                silence_buzzer = false;
                use_hardware_config = false;
                PORTD |= redLed;
            } else {
                while(Wire.available()) Wire.read();
            }
        }
    }
}

static void vGreenBlinkLed(void* pvParameters)
{
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
        PORTD ^= greenLed; //PD3 on the micro controller is linked to D3 on the shield
        vTaskDelayUntil(&xLastWakeUpTime, 2000/portTICK_PERIOD_MS);  //passive Delay
    }
}

static void vSecurityCheck(void* pvParameters) {
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    uint8_t alarm_is_ringing = 0;
    TickType_t alarm_start_time = 0;

    while(1) {


        if (is_password_set == 1 && !silence_buzzer) {
            long duration, cm;

            DDRD |= ultraPin;
            PORTD &= ~ultraPin;
            delayMicroseconds(2);
            PORTD |= ultraPin;
            delayMicroseconds(10);
            PORTD &= ~ultraPin;
            DDRD &= ~ultraPin;
            duration = pulseIn(7, HIGH, 15000);
            cm = duration / 58;

            if (cm > 10 && cm > 0) {
                if (alarm_is_ringing == 0) {
                    alarm_is_ringing = 1;
                    taskENTER_CRITICAL();
                    if(logs.alarm_count < 255)
                        logs.alarm_count++;
                    taskEXIT_CRITICAL();
                    alarm_start_time = xTaskGetTickCount();
                    PORTD |= buzzerPin;
                    Serial.println(F("[ALARM] Intrusion detected!"));
                }
                else {
                    if ((xTaskGetTickCount() - alarm_start_time) > pdMS_TO_TICKS(3000)) {
                        PORTD &= ~buzzerPin;
                    } else {
                        PORTD |= buzzerPin;
                    }
                }
            } else {
                alarm_is_ringing = 0;
                PORTD &= ~buzzerPin;
            }
        } else {
            PORTD &= ~buzzerPin;
            alarm_is_ringing = 0;
        }
        vTaskDelayUntil(&xLastWakeUpTime, 150 / portTICK_PERIOD_MS);
    }
}

static void vUpdateCode(void* pvParameters) {
    // Timestamps
    TickType_t xLastWakeUpTime, xLastCodeChangeTime;
    xLastWakeUpTime = xTaskGetTickCount();
    xLastCodeChangeTime = xLastWakeUpTime;

    // Digit change logic
    uint8_t new_value = 0;
    uint16_t last_digit_read;
    last_digit_read = read_adc(0);
    //current_digit_read = last_digit_read;
    uint8_t combination[MAX_COMBINATION_SIZE] = {0};
    uint8_t index = 0;
    uint16_t last_raw_len_read = 0;
    uint16_t last_raw_delay_read = 0;
  //  uint8_t active_length = 4;
    //uint16_t active_delay = 500;
    last_raw_len_read = read_adc(1);
    last_raw_delay_read = read_adc(2);

    Serial.println(F("Enter combination:"));

    while (1) {
        TickType_t now = xTaskGetTickCount();
        uint16_t raw_len = read_adc(1);
        uint16_t raw_delay = read_adc(2);

        if ( abs(raw_len - last_raw_len_read) > 20 || abs(raw_delay - last_raw_delay_read) > 20 ) {
            use_hardware_config = true;
            last_raw_len_read = raw_len;
            last_raw_delay_read = raw_delay;
            Serial.println(F("Hardware Config Override"));
            Serial.print(F("Config Hard: Length="));
            Serial.println(logs.current_code_length);
            Serial.print(F("Config Hard: Delay="));
            Serial.println(logs.current_delay_ms);
        }
        if (use_hardware_config) {
            uint8_t new_len_mapped = map_value(raw_len, 0, 1023, 1, 10);
            uint16_t new_delay_mapped = map_value(raw_delay, 0, 1023, 200, 2000);

            if (new_len_mapped != logs.current_code_length) {
                taskENTER_CRITICAL();
                logs.current_code_length = new_len_mapped;
                taskEXIT_CRITICAL();
                index = 0;
                Serial.print(F("Config Hard: Length="));
                Serial.println(new_len_mapped);
            }

            if (new_delay_mapped != logs.current_delay_ms) {
                taskENTER_CRITICAL();
                logs.current_delay_ms = new_delay_mapped;
                taskEXIT_CRITICAL();
            }
        }

        uint8_t active_length = logs.current_code_length;
        uint16_t active_delay = logs.current_delay_ms;

        //reset combination input
        if ( (PIND & resetBtn) ) {
            index = 0;
            Serial.println(F("--- Combination input reset ---"));
            vTaskDelay(pdMS_TO_TICKS(200));
            PORTD |= redLed; vTaskDelay(pdMS_TO_TICKS(100));
            PORTD &= ~redLed;

            while(PIND & resetBtn) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
       // uint8_t new_len = map_value(raw_len, 0, 1023, 1, 10);
        uint16_t current_digit_read = read_adc(0);
        // reset combination when length is changed
     /*   if (new_len != active_length) {
            active_length = new_len;
            taskENTER_CRITICAL();
            logs.current_code_length = active_length;
            taskEXIT_CRITICAL();
            index = 0;
            Serial.print(F("--- New code length: "));
            Serial.println(active_length);
        }
*/
        // delay change
        /*
        active_delay = map_value(raw_delay, 0, 1023, 200, 2000);
        taskENTER_CRITICAL();
        logs.current_delay_ms = active_delay;
        taskEXIT_CRITICAL();
        */

        //debounce
        if (!IS_CLOSE_ENOUGH(current_digit_read, last_digit_read, 10)) {
            last_digit_read = current_digit_read;
            new_value = 1;
            xLastCodeChangeTime = now;
        }

        //validate input if stable for 500ms
        if (new_value && now - xLastCodeChangeTime >= pdMS_TO_TICKS(active_delay)) {
            new_value = 0;
            uint8_t digit = to_code_digit(current_digit_read);
            Serial.print(F("Registering new input: "));
            Serial.println(digit);
            combination[index++] = digit;

            if (index >= active_length) {
                if (is_password_set == 0) {
                    for(uint8_t i = 0; i < active_length; i++) {
                        saved_combination[i] = combination[i];
                    }
                    is_password_set = 1;
                    silence_buzzer = false;
                    Serial.print(F("Combination saved: "));
                    for(uint8_t i=0; i<active_length; i++)
                        Serial.print(saved_combination[i]);
                    Serial.println(F("Locking"));
                    PORTD |= redLed;
                }
                else {
                    taskENTER_CRITICAL();
                    if(logs.attempts_count < 255)
                        logs.attempts_count++;
                    taskEXIT_CRITICAL();
                    uint8_t code_is_correct = 1;
                    for (uint8_t i = 0; i < active_length; i++) {
                        if (combination[i] != saved_combination[i]){
                            code_is_correct = 0;
                            break;
                        }
                    }
                    if (code_is_correct) {
                        Serial.println(F(">> CODE CORRECT ! Acces autorise."));
                        taskENTER_CRITICAL();
                        if(logs.success_count < 255)
                            logs.success_count++;
                        taskEXIT_CRITICAL();
                        is_password_set = 0;
                        PORTD &= ~redLed;
                        //PORTD &= ~buzzerPin;
                    }
                    else {
                        taskENTER_CRITICAL();
                        if(logs.failed_count < 255)
                            logs.failed_count++;
                        taskEXIT_CRITICAL();
                        Serial.println(F(">> CODE INCORRECT ! Acces refuse."));
                      //  PORTD &= ~redLed; vTaskDelay(100); PORTD |= redLed;
                    }
                }
                index = 0;
            }
        }

        vTaskDelayUntil(&xLastWakeUpTime, 80 / portTICK_PERIOD_MS);
    }
}

void loop(){}
