#include <Arduino.h>
//#include "FreeRTOS.h"
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include <avr/io.h>
#include <Wire.h>

#define IS_CLOSE_ENOUGH(a, b, eps) \
        (((a) > (b) ? (a) - (b) : (b) - (a)) <= (eps))

#define COMBINATION_SIZE 4
#define SLAVE_ADDRESS 0x08

volatile static uint8_t saved_combination[COMBINATION_SIZE];
volatile static uint8_t is_password_set = 0;
volatile bool silence_buzzer = false;

static void vUpdateCode(void* pvParameters);
static void vGreenBlinkLed(void* pvParameters);
static void vSecurityCheck(void* pvParameters);
void receiveEvent(int howMany);

const uint8_t redLed   = _BV(PD2);
const uint8_t greenLed = _BV(PD3);
const uint8_t buzzerPin = _BV(PD6);
const uint8_t ultraPin  = _BV(PD7);

void init_rotary()
{
    ADMUX = (1 << REFS0);
    ADMUX &= 0xF0;
    ADCSRA |= (1 << ADEN ) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t read_rotary()
{
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

int main(void)
{
    DDRD |= (redLed | greenLed | buzzerPin); // PD2 and PD3 as outputs
    init_rotary();
    Serial.begin(9600);

    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);

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
        configMINIMAL_STACK_SIZE,
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
          //  Serial.println(F("[I2C] Unlock"));
        }
        else if (command == 'S') { // SILENCE
            silence_buzzer = true;
            PORTD &= ~buzzerPin;  // Couper Buzzer immediatement
          //  Serial.println(F("[I2C] Silence buzzer"));
        }
        else if (command == 'C') { // SET CODE
            if (Wire.available() >= COMBINATION_SIZE) {
                for (int i = 0; i < COMBINATION_SIZE; i++) {
                    saved_combination[i] = Wire.read();
                }
                is_password_set = 1;
                silence_buzzer = false;
                PORTD |= redLed;
                //Serial.println(F("[I2C] New combination set: "));
              //  for (int i = 0; i < COMBINATION_SIZE; i++) {
                  //  Serial.print(saved_combination[i]);
                //}
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
            PORTD &= ~ultraPin; delayMicroseconds(2);
            PORTD |= ultraPin;  delayMicroseconds(10);
            PORTD &= ~ultraPin;

            DDRD &= ~ultraPin;
            duration = pulseIn(7, HIGH, 15000);
            cm = duration / 58;

            if (cm > 10 && cm > 0) {
                if (alarm_is_ringing == 0) {
                    alarm_is_ringing = 1;
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
    uint16_t last_digit_read, current_digit_read;
    last_digit_read = read_rotary();
    //current_digit_read = last_digit_read;

    // Code verification logic
    uint8_t combination[COMBINATION_SIZE] = {0};
    uint8_t index = 0;
    Serial.println(F("Enter combination:"));

    while (1) {
        TickType_t now = xTaskGetTickCount();
        current_digit_read = read_rotary();
        //debounce
        if (!IS_CLOSE_ENOUGH(current_digit_read, last_digit_read, 10)) {
            last_digit_read = current_digit_read;
            new_value = 1;
            xLastCodeChangeTime = now;
        }


        //validate input if stable for 500ms
        if (new_value && now - xLastCodeChangeTime >= pdMS_TO_TICKS(500)) {
            new_value = 0;
            uint8_t digit = to_code_digit(current_digit_read);
            Serial.print(F("Registering new input: "));
            Serial.println(digit);
            combination[index++] = digit;

            if (index >= COMBINATION_SIZE) {
                if (is_password_set == 0) {
                    for(uint8_t i = 0; i < COMBINATION_SIZE; i++) {
                        saved_combination[i] = combination[i];
                    }
                    is_password_set = 1;
                    silence_buzzer = false;
                    Serial.print(F("Combination saved: "));
                    for(uint8_t i=0; i<COMBINATION_SIZE; i++) Serial.print(saved_combination[i]);
                    Serial.println(F("Locking"));

                    PORTD |= redLed;
                }
                else {
                    uint8_t code_is_correct = 1;
                    for (uint8_t i = 0; i < COMBINATION_SIZE; i++) {
                        if (combination[i] != saved_combination[i]){
                            code_is_correct = 0;
                            break;
                        }
                    }
                    if (code_is_correct) {
                        Serial.println(F(">> CODE CORRECT ! Acces autorise."));
                        is_password_set = 0;
                        PORTD &= ~redLed;
                        //PORTD &= ~buzzerPin;
                    }
                    else {
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
