#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include "timers.h"
#include "semphr.h"
#include <avr/io.h>

const uint8_t redLedMask   = _BV(PD2);
const uint8_t greenLedMask = _BV(PD3);
const uint8_t buttonMask   = _BV(PD4);
enum SafeState {
    ETAT_CONFIGURATION,
    ETAT_VERROUILLE,
    ETAT_SUCCES,
    ETAT_ECHEC
};

static void vRotaryReader(void* pvParameters);

void setup() {

    Serial.begin(9600);

    DDRD |= (redLedMask | greenLedMask);

    DDRD &= ~buttonMask;

    PORTD |= buttonMask;

    xTaskCreate(
        vRotaryReader,
        "rotaryReader",
        128,
        NULL,
        1,
        NULL
    );

    vTaskStartScheduler();
}

void loop() {
}

static void vRotaryReader(void* pvParameters) {
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
    SafeState currentState = ETAT_CONFIGURATION;

    int lastValue = -1;
    int savedValue0 = 0;
    int savedValue1 = 0;
    int savedValue2 = 0;
    bool isSaved = false;
    int lastButtonState = 1;
    int currentButtonState;

    while (1) {
        int currentValue0 = analogRead(A0);
        int currentValue1 = analogRead(A1);
        int currentValue2 = analogRead(A2);


        currentButtonState = (PIND & buttonMask) ? 1 : 0;

        if (lastButtonState == 1 && currentButtonState == 0) {
            isSaved = !isSaved;

            if (isSaved) {
                savedValue0 = currentValue0;
                savedValue1 = currentValue1;
                savedValue2 = currentValue2;
                Serial.println("-------------------------");
                Serial.print(">>> COMBINAISON ENREGISTREE : ");
                Serial.println(savedValue0);
                Serial.print("                        ");
                Serial.println(savedValue1);
                Serial.print("                        ");
                Serial.println(savedValue2);
                Serial.println("-------------------------");
            } else {
                Serial.println(">>> Retour au mode lecture...");
            }
        }
        lastButtonState = currentButtonState;
        if (isSaved) {
            PORTD |= greenLedMask;
            PORTD &= ~redLedMask;
        }
        else {
            PORTD |= redLedMask;
            PORTD &= ~greenLedMask;

            if (abs(currentValue1 - lastValue) > 1) {
                Serial.print("Valeur en direct : ");
                Serial.println(currentValue1);
                lastValue = currentValue1;
            }
        }

        vTaskDelayUntil(&xLastWakeUpTime, xFrequency);
    }
}
