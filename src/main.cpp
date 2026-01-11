/**
* @file main.cpp
* @brief Système de coffre-fort connecté basé sur FreeRTOS pour Arduino Uno.
* @author equiepe coffre
* @date 2025-2026
*
* Ce système gère :
* - Un verrouillage par code (potentiomètre/ADC).
* - Une détection d'intrusion par ultrasons.
* - Une communication I2C (Esclave 0x08).
* - Une persistance des données en EEPROM.
* - Un OS temps réel (FreeRTOS) pour gérer la concurrence.
*/

#include <Arduino.h>
//#include "FreeRTOS.h"
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <Grove_LED_Bar.h>
#include <semphr.h>

// =================================================================================
// CONFIGURATION & CONSTANTES
// =================================================================================

// --- Paramètres I2C ---
#define SLAVE_ADDRESS       0x08    /// Adresse I2C de cet appareil

// --- Paramètres du Code Secret ---
#define MAX_COMBINATION_SIZE 10     /// Taille maximale du code secret
#define ADC_DEADBAND         20     /// Seuil de variation ADC pour ignorer le bruit
#define ADC_DEBOUNCE_EPS     10     /// Marge d erreur pour stabiliser la lecture du chiffre
#define CODE_INPUT_DELAY_MS  500    /// Delai par defaut pour valider un chiffre

// --- Paramètres Ultrasons ---
#define ULTRASONIC_PIN_NUM   7      /// Numero de pin Arduino pour pulseIn()
#define SPEED_OF_SOUND_DIV   58     /// Diviseur pour convertir µs en cm
#define ALARM_TRIGGER_DIST   10     /// Distance (cm) declenchant l alarme
#define ALARM_DURATION_SEC   2      /// Duree de l alarme en secondes

// --- Mapping Memoire EEPROM ---
#define EEPROM_MAGIC_ADDR    0      /// Adresse du marqueur d initialisation
#define EEPROM_MAGIC_VAL     0x66   /// Valeur arbitraire prouvant que l EEPROM est init
#define EEPROM_LOGS_ADDR     10     /// Adresse de debut de la structure Logs
#define EEPROM_STATE_ADDR    50     /// Adresse de l etat (Verrouille/Ouvert)
#define EEPROM_CODE_ADDR     60     /// Adresse de debut du tableau du code secret

// --- Macros Utilitaires ---
// Macro pour vérifier si deux valeurs sont proches (pour le débruitage)
#define IS_CLOSE_ENOUGH(a, b, eps) (((a) > (b) ? (a) - (b) : (b) - (a)) <= (eps))

// =================================================================================
// STRUCTURES DE DONNÉES
// =================================================================================

/**
 * @struct SecurityLogs
 * @brief Structure stockant l historique et la configuration.
 * Taille totale : 7 octets
 */
typedef struct {
    uint8_t attempts_count;       /// Nombre total de tentatives d ouverture
    uint8_t failed_count;         /// Nombre de codes errones
    uint8_t success_count;        /// Nombre d ouvertures reussies
    uint8_t alarm_count;          /// Nombre de declenchements d alarme
    uint8_t current_code_length;  /// Longueur actuelle du code
    uint16_t current_delay_ms;    /// Delai de validation de saisie (ms)
} SecurityLogs;

// =================================================================================
// VARIABLES GLOBALES
// =================================================================================

// Donnees persistantes (chargees depuis l'EEPROM)
volatile static uint8_t saved_combination[MAX_COMBINATION_SIZE];
volatile static uint8_t is_password_set = 0;
volatile SecurityLogs logs = {0, 0, 0, 0, 4, 500};

// Drapeaux de controle
volatile bool silence_buzzer = false;        /// Coupe le son temporairement
volatile bool use_hardware_config = true;    /// Si true, les potentiomètres contrôlent la config
volatile bool g_request_save_eeprom = false; /// Flag pour demander une sauvegarde différée

// Objets OS et Periphériques
SemaphoreHandle_t xLogMutex;  /// Mutex pour protéger l accès à la structure logs
Grove_LED_Bar bar(9, 8, 0);   /// Barre LED

// Mapping des Pins (Manipulation directe des Ports pour la vitesse)
const uint8_t redLed    = _BV(PD2); ///< LED Rouge sur D2
const uint8_t greenLed  = _BV(PD3); ///< LED Verte sur D3
const uint8_t resetBtn  = _BV(PD4); ///< Bouton Reset sur D4
const uint8_t buzzerPin = _BV(PD6); ///< Buzzer sur D6
const uint8_t ultraPin  = _BV(PD7); ///< Ultrason sur D7

// =================================================================================
// PROTOTYPES DE FONCTIONS
// =================================================================================

// Tâches FreeRTOS
static void vUpdateCode(void* pvParameters);
static void vGreenBlinkLed(void* pvParameters);
static void vSecurityCheck(void* pvParameters);

// Gestion I2C
void receiveEvent(int howMany);
void requestEvent();

// Gestion Mémoire
void save_to_eeprom();
void load_from_eeprom();
uint8_t eeprom_read_byte_raw(uint16_t uiAddress);
void eeprom_update_byte_raw(uint16_t uiAddress, uint8_t ucData);
void eeprom_write_block(uint16_t addr, void* data, uint16_t len);
void eeprom_read_block(uint16_t addr, void* data, uint16_t len);

// Utilitaires
void init_adc();
uint16_t read_adc(uint8_t channel);
uint8_t to_code_digit(uint16_t value);
long map_value(long x, long in_min, long in_max, long out_min, long out_max);

// =================================================================================
// MAIN & SETUP
// =================================================================================
int main(void)
{
    //Configuration des entrées/sorties
    DDRD |= (redLed | greenLed | buzzerPin); // Outputs
    DDRD &= ~resetBtn;                       // Input
    PORTD &= ~resetBtn;                      // Pull-up

    //Initialisation
    init_adc();
    Serial.begin(9600);
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    bar.begin();
    bar.setLevel(0);

    xLogMutex = xSemaphoreCreateMutex();
    if (xLogMutex != NULL) {
        Serial.println(F("[SYSTEM] Mutex initialise."));
    }

    // Chargement des donnees
    load_from_eeprom();

    // etat initial LED Rouge
    if (is_password_set)
        PORTD |= redLed;
    else
        PORTD &= ~redLed;

    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    bar.begin();
    bar.setLevel(0);

    // Tâche Interface Utilisateur (Code, Potentiomètres)
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
    // TaskHandle_t greenBlink_handle;
    // xTaskCreate
    // (
    //     vGreenBlinkLed,
    //     (const char*)"greenBlink",
    //     192,
    //     NULL,
    //     1U,
    //     &greenBlink_handle
    // );

    // Tâche Sécurité (Alarme Ultrasons)
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
void loop(){}


// =================================================================================
//  GESTION I2C
// =================================================================================
/**
 * @brief Appelé par le Maître I2C pour récupérer les logs.
 * @note Cette fonction s'exécute dans un contexte d'interruption (ISR).
 */
void requestEvent() {
    Wire.write((uint8_t*)&logs, sizeof(logs));
}

/**
 * @brief Appelé quand le Maître I2C envoie des commandes.
 * @param howMany Nombre d'octets reçus.
 */
void receiveEvent(int howMany) {
    if (!Wire.available()) return;

    char command = Wire.read();
    bool data_changed = false;

    // 'U' -> UNLOCK (Déverrouillage forcé)
    if (command == 'U') {
        is_password_set = 0;
        silence_buzzer = false;
        PORTD &= ~redLed;
        PORTD &= ~buzzerPin;
        bar.setLevel(0);
        data_changed = true;
    }
    // 'S' -> SILENCE (Couper le buzzer)
    else if (command == 'S') {
        silence_buzzer = true;
        PORTD &= ~buzzerPin;
    }
    // 'R' -> RESET LOGS (Remise à zéro des compteurs)
    else if (command == 'R') {
        taskENTER_CRITICAL();
        logs.attempts_count = 0;
        logs.failed_count = 0;
        logs.success_count = 0;
        logs.alarm_count = 0;
        taskEXIT_CRITICAL();
        data_changed = true;
    }
    // 'D' -> DELAY (Configurer le délai de saisie)
    else if (command == 'D') {
        if (Wire.available() >= 2) {
            uint8_t lowByte = Wire.read();
            uint8_t highByte = Wire.read();
            taskENTER_CRITICAL();
            logs.current_delay_ms = (highByte << 8) | lowByte;
            taskEXIT_CRITICAL();
            use_hardware_config = false;
            data_changed = true;
        }
    }
    // 'L' -> LENGTH (Configurer la longueur du code)
    else if (command == 'L') {
        if (Wire.available() >= 1) {
            uint8_t newLen = Wire.read();
            if (is_password_set == 0) { // Uniquement si coffre ouvert
                taskENTER_CRITICAL();
                logs.current_code_length = newLen;
                taskEXIT_CRITICAL();
                use_hardware_config = false;
                bar.setLevel(0);
                data_changed = true;
            }
        }
    }
    // 'C' -> CODE (Definir un nouveau code via I2C)
    else if (command == 'C') {
        int codeSize = Wire.available();
        if (codeSize > 0 && codeSize <= MAX_COMBINATION_SIZE) {
            taskENTER_CRITICAL();
            logs.current_code_length = codeSize;
            taskEXIT_CRITICAL();

            for (int i = 0; i < codeSize; i++) {
                saved_combination[i] = Wire.read();
            }

            is_password_set = 1;
            silence_buzzer = false;
            use_hardware_config = false;
            PORTD |= redLed;
            bar.setLevel(10);
            data_changed = true;
        } else {
            while(Wire.available()) Wire.read();
        }
    }

    if (data_changed) {
        g_request_save_eeprom = true;
    }
}
// =================================================================================
// TASK FREERTOS
// =================================================================================

// static void vGreenBlinkLed(void* pvParameters)
// {
//     TickType_t xLastWakeUpTime = xTaskGetTickCount();
//     while (1)
//     {
//         PORTD ^= greenLed; //PD3 on the micro controller is linked to D3 on the shield
//         vTaskDelayUntil(&xLastWakeUpTime, 2000/portTICK_PERIOD_MS);  //passive Delay
//     }
// }

/**
 * @brief Tache de surveillance et alarme.
 * Gere le capteur ultrason et le buzzer.
 */
static void vSecurityCheck(void* pvParameters) {
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    uint8_t alarm_is_ringing = 0;
    TickType_t alarm_start_time = 0;

    while(1) {
        if (is_password_set == 1 && !silence_buzzer) {
            long duration, cm;

            // --- Séquence Trigger Ultrasons ---
            // On manipule les registres pour transformer la pin en Sortie puis Entrée
            DDRD |= ultraPin;
            PORTD &= ~ultraPin;
            delayMicroseconds(2);
            PORTD |= ultraPin;
            delayMicroseconds(10);
            PORTD &= ~ultraPin;
            DDRD &= ~ultraPin;

            duration = pulseIn(7, HIGH, 15000);
            cm = duration / 58;

            // --- Logique d'alarme ---
            if (cm > 10 && cm > 0) {
                if (alarm_is_ringing == 0) {
                    alarm_is_ringing = 1;
                    if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
                        if(logs.alarm_count < 255) logs.alarm_count++;
                        xSemaphoreGive(xLogMutex);
                    }
                    alarm_start_time = xTaskGetTickCount();
                    PORTD |= buzzerPin; // Buzzer ON
                    Serial.println(F("[ALARM] Intrusion detected!"));
                    bar.setLevel(10);
                    save_to_eeprom();
                }
                else {
                    // ALARME EN COURS
                    if ((xTaskGetTickCount() - alarm_start_time) > pdMS_TO_TICKS(3000)) {
                        PORTD &= ~buzzerPin;
                        bar.setLevel(0);
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

/**
 * @brief Tache Principale : Interface Utilisateur.
 * Gere la saisie du code, les potentiometres de config et la sauvegarde EEPROM.
 */
static void vUpdateCode(void* pvParameters) {
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    TickType_t xLastCodeChangeTime = xTaskGetTickCount();

    // Digit change logic
    uint8_t new_value = 0;
    uint16_t last_digit_read = read_adc(0);
    uint8_t combination[MAX_COMBINATION_SIZE] = {0};
    uint8_t index = 0;
    uint16_t last_raw_len_read = read_adc(1);
    uint16_t last_raw_delay_read = read_adc(2);

    Serial.println(F("Enter combination:"));

    while (1) {
        // --- GESTION SAUVEGARDE DIFFEREE ---
        if (g_request_save_eeprom == true) {
            g_request_save_eeprom = false;
            Serial.println(F(">> Sauvegarde EEPROM differee en cours..."));
            save_to_eeprom();
            Serial.println(F(">> Sauvegarde terminee."));
        }

        TickType_t now = xTaskGetTickCount();
        uint16_t raw_len = read_adc(1);   // Potentiometre Longueur code
        uint16_t raw_delay = read_adc(2); // Potentiometre Délai

        // ---  MODIFICATION VALEUR CAPTEUR ---
        if ( abs(raw_len - last_raw_len_read) > 20 || abs(raw_delay - last_raw_delay_read) > 20 ) {
            use_hardware_config = true;
            last_raw_len_read = raw_len;
            last_raw_delay_read = raw_delay;
            //DEBUG
            // Serial.println(F("Hardware Config Override"));
            // Serial.print(F("Config Hard: Length="));
            // Serial.println(logs.current_code_length);
            // Serial.print(F("Config Hard: Delay="));
            // Serial.println(logs.current_delay_ms);
        }
        if (use_hardware_config) {
            uint8_t new_len_mapped = map_value(raw_len, 0, 1023, 1, 10);
            uint16_t new_delay_mapped = map_value(raw_delay, 0, 1023, 200, 2000);

            // Changement de longueur du code
            if (new_len_mapped != logs.current_code_length) {
                if (is_password_set == 0) {
                    if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
                        logs.current_code_length = new_len_mapped;
                        xSemaphoreGive(xLogMutex);
                    }
                    index = 0;
                    bar.setLevel(0);
                    Serial.print(F("Config Hard: Length="));
                    Serial.println(new_len_mapped);
                    g_request_save_eeprom = true; // Utilisons le flag c'est plus propre
                } else {
                    Serial.println(F("Action refusee: Coffre verrouille !"));
                    use_hardware_config = false;
                }
            }

            // Changement de delai
            if (new_delay_mapped != logs.current_delay_ms) {
                if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
                    logs.current_delay_ms = new_delay_mapped;
                    xSemaphoreGive(xLogMutex);
                }
            }
        }

        // --- LECTURE DE LA CONFIG ACTUELLE ---
        uint8_t active_length;
        uint16_t active_delay;

        if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
            active_length = logs.current_code_length;
            active_delay = logs.current_delay_ms;
            xSemaphoreGive(xLogMutex);
        } else {
            // Valeurs par défaut si erreur
            active_length = 4;
            active_delay = 500;
        }

        // --- GESTION BOUTON RESET ---
        if ( (PIND & resetBtn) ) {
            index = 0;
            bar.setLevel(0);
            Serial.println(F("--- Combination input reset ---"));
            vTaskDelay(pdMS_TO_TICKS(200));
            PORTD |= redLed; vTaskDelay(pdMS_TO_TICKS(100));
            PORTD &= ~redLed;

            while(PIND & resetBtn) { //relachement
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        // --- LECTURE COMBINAISON ---
        uint16_t current_digit_read = read_adc(0);

        //debounce
        if (!IS_CLOSE_ENOUGH(current_digit_read, last_digit_read, 10)) {
            last_digit_read = current_digit_read;
            new_value = 1;
            xLastCodeChangeTime = now;
        }

        // Validation du chiffre (Si stable depuis 'active_delay' ms)
        if (new_value && (now - xLastCodeChangeTime >= pdMS_TO_TICKS(active_delay))) {
            new_value = 0;
            uint8_t digit = to_code_digit(current_digit_read);
            Serial.print(F("Chiffre saisi: ")); Serial.println(digit);

            combination[index++] = digit;
            bar.setLevel(index);

            // Si le code est complet
            if (index >= active_length) {
                // CAS A: VERROUILLAGE (Définition du nouveau mot de passe)
                if (is_password_set == 0) {
                    for(uint8_t i = 0; i < active_length; i++) {
                        saved_combination[i] = combination[i];
                    }
                    is_password_set = 1;
                    silence_buzzer = false;
                    Serial.println(F(">> VERROUILLAGE ACTIVE"));
                    PORTD |= redLed;
                    bar.setBits(0x3FF); // Allume tout
                    vTaskDelay(pdMS_TO_TICKS(200));
                    bar.setLevel(0);
                    g_request_save_eeprom = true;
                }
                // CAS B: TENTATIVE DE DÉVERROUILLAGE
                else {
                    // Enregistrement tentative
                    if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
                        if(logs.attempts_count < 255) logs.attempts_count++;
                        xSemaphoreGive(xLogMutex);
                    }

                    // Vérification combinaison
                    uint8_t code_is_correct = 1;
                    for (uint8_t i = 0; i < active_length; i++) {
                        if (combination[i] != saved_combination[i]){
                            code_is_correct = 0;
                            break;
                        }
                    }

                    if (code_is_correct) {
                        Serial.println(F(">> CODE CORRECT !"));
                        if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
                            if(logs.success_count < 255) logs.success_count++;
                            xSemaphoreGive(xLogMutex);
                        }
                        is_password_set = 0;
                        PORTD &= ~redLed;
                        // Animation d'ouverture
                        for(int i=0;i<=10;i++) {
                            bar.setLevel(i);
                            vTaskDelay(pdMS_TO_TICKS(30));
                        }
                        vTaskDelay(pdMS_TO_TICKS(200));
                        bar.setLevel(0);
                        g_request_save_eeprom = true;
                    }
                    else {
                        Serial.println(F(">> CODE INCORRECT !"));
                        if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
                            if(logs.failed_count < 255) logs.failed_count++;
                            xSemaphoreGive(xLogMutex);
                        }
                        // Animation d'erreur
                        bar.setLevel(0); vTaskDelay(pdMS_TO_TICKS(100));
                        bar.setLevel(active_length); vTaskDelay(pdMS_TO_TICKS(100));
                        bar.setLevel(0);
                        g_request_save_eeprom = true;
                    }
                }
                index = 0; // Reset index pour prochaine saisie
            }
        }
        vTaskDelayUntil(&xLastWakeUpTime, 80 / portTICK_PERIOD_MS);
    }
}

// =================================================================================
// FONCTIONS BAS NIVEAU & DRIVERS
// =================================================================================

/**
 * @brief Initialise le convertisseur Analogique-Numérique (ADC).
*/
void init_adc()
{
    ADMUX = (1 << REFS0);
    ADMUX &= 0xF0;
    ADCSRA |= (1 << ADEN ) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/**
 * @brief Lit une valeur analogique sur un canal donné.
 * @param channel Canal ADC (0 à 7).
 * @return Valeur 10 bits (0-1023).
 */
uint16_t read_adc(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

/**
 * @brief Convertit une valeur brute ADC (0-1023) en un chiffre (0-9).
 */
uint8_t to_code_digit(uint16_t value)
{
    if (value > 1000) value = 1000;
    return (value * 9) / 1000;
}

/**
 * @brief Remappe une valeur d'une plage à une autre (Equivalent map() Arduino).
 */
long map_value(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- Wrapper EEPROM Bas Niveau ---

/**
 * @brief Lecture bas niveau d'un octet en EEPROM.
 */
uint8_t eeprom_read_byte_raw(uint16_t uiAddress) {
    while(EECR & (1<<EEPE)); // Attend fin écriture précédente
    EEAR = uiAddress;
    EECR |= (1<<EERE);       // Déclenche lecture
    return EEDR;
}

/**
 * @brief Écriture bas niveau d'un octet en EEPROM (si valeur différente).
 * @note Protégé par taskENTER_CRITICAL car l'écriture est sensible au timing.
 */
void eeprom_update_byte_raw(uint16_t uiAddress, uint8_t ucData) {
    uint8_t currentVal = eeprom_read_byte_raw(uiAddress);
    if (currentVal == ucData) return; // Économise les cycles d'écriture

    taskENTER_CRITICAL();
    while(EECR & (1<<EEPE));
    EEAR = uiAddress;
    EEDR = ucData;
    EECR |= (1<<EEMPE); // Master Write Enable
    EECR |= (1<<EEPE);  // Write Enable
    taskEXIT_CRITICAL();
}

void eeprom_write_block(uint16_t addr, void* data, uint16_t len) {
    uint8_t* p = (uint8_t*)data;
    for(uint16_t i=0; i<len; i++) {
        eeprom_update_byte_raw(addr + i, *p++);
    }
}

void eeprom_read_block(uint16_t addr, void* data, uint16_t len) {
    uint8_t* p = (uint8_t*)data;
    for(uint16_t i=0; i<len; i++) {
        *p++ = eeprom_read_byte_raw(addr + i);
    }
}

/**
 * @brief Orchestre la sauvegarde complète des données du système.
 * Utilise le Mutex pour assurer une copie cohérente de la structure 'logs'.
 */
void save_to_eeprom() {
    if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {
        eeprom_write_block(EEPROM_LOGS_ADDR, (void*)&logs, sizeof(logs));
        xSemaphoreGive(xLogMutex);
    }

    eeprom_update_byte_raw(EEPROM_STATE_ADDR, is_password_set);
    for(int i=0; i<MAX_COMBINATION_SIZE; i++) {
        eeprom_update_byte_raw(EEPROM_CODE_ADDR + i, saved_combination[i]);
    }
}

/**
 * @brief Charge les données au démarrage ou initialise l'EEPROM si vierge.
 */
void load_from_eeprom() {
    // Vérification du "Magic Byte"
    if (eeprom_read_byte_raw(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
        Serial.println(F("EEPROM Vierge detectee -> Initialisation..."));

        logs.attempts_count = 0;
        logs.failed_count = 0;
        logs.success_count = 0;
        logs.alarm_count = 0;
        logs.current_code_length = 4;
        logs.current_delay_ms = 500;
        is_password_set = 0;

        eeprom_update_byte_raw(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
        eeprom_write_block(EEPROM_LOGS_ADDR, (void*)&logs, sizeof(logs));
        save_to_eeprom();
    } else {
        Serial.println(F("Donnees chargees depuis EEPROM."));
        eeprom_read_block(EEPROM_LOGS_ADDR, (void*)&logs, sizeof(logs));
        is_password_set = eeprom_read_byte_raw(EEPROM_STATE_ADDR);

        for(int i=0; i<MAX_COMBINATION_SIZE; i++) {
            saved_combination[i] = eeprom_read_byte_raw(EEPROM_CODE_ADDR + i);
        }
    }
}
