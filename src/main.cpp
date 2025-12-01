#include "FreeRTOS.h"
#include "Arduino.h"
#include "task.h"

#include <avr/io.h>

#define IS_CLOSE_ENOUGH(a, b, eps) \
        (((a) > (b) ? (a) - (b) : (b) - (a)) <= (eps))

#define CORRECT_COMBINATION 0x9909
#define COMBINATION_SIZE 4

static void vUpdateCode(void* pvParameters);
static void vGreenBlinkLed(void* pvParameters);

const uint8_t redLed   = _BV(PD2);
const uint8_t greenLed = _BV(PD3);

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
    DDRD |= (redLed | greenLed); // PD2 and PD3 as outputs
    init_rotary();
    Serial.begin(9600);

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

 static void vGreenBlinkLed(void* pvParameters)
{
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
        PORTD ^= greenLed; //PD3 on the micro controller is linked to D3 on the shield
        vTaskDelayUntil(&xLastWakeUpTime, 2000/portTICK_PERIOD_MS);  //passive Delay
    }
}

static void vUpdateCode(void* pvParameters)
{
    // Timestamps
    TickType_t xLastWakeUpTime, xLastCodeChangeTime;
    xLastWakeUpTime = xTaskGetTickCount();
    xLastCodeChangeTime = xLastWakeUpTime;

    // Digit change logic
    uint8_t new_value = 0;
    uint16_t last_digit_read, current_digit_read;
    last_digit_read = read_rotary();
    current_digit_read = last_digit_read;

    // Code verification logic
    uint8_t combination[COMBINATION_SIZE] = {0};
    uint8_t index = 0;

    while (1)
    {
        TickType_t now = xTaskGetTickCount();

        current_digit_read = read_rotary();
        if (!IS_CLOSE_ENOUGH(current_digit_read, last_digit_read, 10))
        {
            last_digit_read = current_digit_read;
            new_value = 1;
            xLastCodeChangeTime = now;
        }

        if (new_value && now - xLastCodeChangeTime >= pdMS_TO_TICKS(500))
        {
            new_value = 0;
            Serial.print("Registering new input: ");
            Serial.println(to_code_digit(current_digit_read));
            combination[index++] = to_code_digit(current_digit_read);
            if (index >= COMBINATION_SIZE)
            {
                for (uint8_t i = 0; i < COMBINATION_SIZE; i++)
                {
                    uint8_t expected = (CORRECT_COMBINATION >> (4 * i)) & 0xF;
                    if (expected != combination[COMBINATION_SIZE - i - 1])
                    {
                        Serial.println("Code is incorrect");
                        break;
                    }

                }
                Serial.println("Code is correct");

                index = 0;
            }
        }

        vTaskDelayUntil(&xLastWakeUpTime, 80 / portTICK_PERIOD_MS);
    }
}

