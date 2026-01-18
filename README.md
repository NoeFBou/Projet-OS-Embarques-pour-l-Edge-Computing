# Système de Coffre-Fort Connecté & Centralisé (IoT/Edge)

projet réalisé dans le cadre du cours de _Projet OS Embarques pour l’Edge Computing_ 

> Un système de sécurité pour l'hôtellerie combinant temps réel (FreeRTOS) et supervision centralisée (Raspberry Pi).

## Description

Ce projet implémente un système de coffre-fort intelligent conçu pour l'hôtellerie de luxe. Il utilise une architecture **Edge Computing** Maître/Esclave où :
1.  **Le Nœud Edge (Arduino Uno)** gère la sécurité critique en temps réel (verrouillage, alarmes, capteurs) via **FreeRTOS**.
2.  **Le Nœud Central (Raspberry Pi)** agit comme un superviseur permettant à la réception de l'hôtel de gérer les coffres à distance (déblocage d'urgence, logs, configuration).

### Fonctionnalités Clés
* **Temps Réel :** Détection d'intrusion par ultrasons (< 10cm) avec déclenchement immédiat de l'alarme (Priorité FreeRTOS).
* **Sécurité Physique :** Verrouillage par code numérique (taille et tolérance configurables).
* **Supervision Distante :** Communication I2C bidirectionnelle pour lire l'historique et piloter le coffre.
* **Persistance :** Sauvegarde de l'état, du code et des logs en EEPROM (résiste aux coupures de courant).
* **Hardware Override :** La configuration physique (potentiomètres) reste prioritaire sur la configuration logicielle.

---

## Architecture Matérielle

### Composants Requis
* **Microcontrôleur :** Arduino Uno (Esclave I2C - Adresse `0x08`)
* **Superviseur :** Raspberry Pi 3 (Maître I2C)
* **Capteurs :**
    * 1x Capteur Ultrasons HC-SR04 (Détection intrusion)
    * 3x Potentiomètres / Grove Rotary Angle (Saisie Code, Config Longueur, Config Délai)
    * 1x Bouton Poussoir (Reset saisie)
* **Actuateurs :**
    * 1x Grove LED Bar (Feedback visuel saisie)
    * 1x Buzzer (Alarme)
    * 1x LED Rouge (État Verrouillé)

### Câblage (Pinout Arduino)

| Composant | Pin Arduino | Description |
| :--- | :--- | :--- |
| **I2C SDA** | A4 | Communication vers Raspberry Pi (Pin 3) |
| **I2C SCL** | A5 | Communication vers Raspberry Pi (Pin 5) |
| **Potentiomètre (Code)** | A0 | Saisie des chiffres |
| **Potentiomètre (Taille)** | A1 | Configurer longueur du code (1-10) |
| **Potentiomètre (Délai)** | A2 | Configurer vitesse de saisie |
| **LED Rouge** | D2 | Indicateur état verrouillé |
| **Bouton Reset** | D4 | Effacer la saisie en cours |
| **Buzzer** | D6 | Alarme sonore |
| **Ultrasons (Signal)** | D7 | Trigger & Echo (Mode PulseIn) |
| **LED Bar (Data)** | D8 | Visualisation progression |
| **LED Bar (Clock)** | D9 | Visualisation progression |

---

## Architecture Logicielle

### 1. Firmware Arduino (`main.cpp`)
Le code repose sur **FreeRTOS** pour garantir le déterminisme des tâches critiques.
* **Tâche `vSecurityCheck` (Priorité Haute) :** Surveille le capteur ultrason en permanence. Déclenche l'alarme si le coffre est verrouillé et forcé.
* **Tâche `vUpdateCode` (Priorité Basse) :** Gère l'interface utilisateur (lecture ADC, debouncing, affichage LED).
* **ISR (Interruptions) :** Gère la communication I2C pour ne jamais bloquer le système.
* **Mutex (`xLogMutex`) :** Protège l'accès mémoire partagé (Logs) entre les tâches et l'I2C.

### 2. Manager Python (`chest_manager.py`)
Script d'administration CLI tournant sur le Raspberry Pi.
* Utilise la librairie `smbus` pour communiquer sur le bus `/dev/i2c-1`.
* Permet le déverrouillage d'urgence ('U'), le silence alarme ('S') et la lecture des logs binaires (`struct.unpack`).

---

## Installation & Utilisation

### Prérequis
1.  Installer **Arduino IDE** et les bibliothèques : `FreeRTOS`, `Grove_LED_Bar`.
2.  Activer l'I2C sur le Raspberry Pi (`sudo raspi-config`).
3.  Installer les dépendances Python : `sudo apt-get install python3-smbus`.

### Déploiement
1.  **Arduino :** Téléversez le fichier `main.cpp` sur l'Arduino Uno.
2.  **Câblage :** Reliez les masses (GND) de l'Arduino et du Raspberry Pi ensemble. Connectez SDA<->SDA et SCL<->SCL.
3.  **Raspberry Pi :** Lancez le script de gestion :
    ```bash
    python3 chest_manager.py
    ```

### Guide Utilisateur
1.  **Verrouiller :** Tournez le potentiomètre A0 pour choisir les chiffres. Une fois le nombre de chiffres atteint (défaut 4), le coffre se verrouille (LED Rouge ON).
2.  **Simuler une intrusion :** Coffre verrouillé, passez la main devant le capteur ultrason (< 10cm). Le buzzer sonne.
3.  **Administration :** Sur le terminal du Pi, tapez `1` pour déverrouiller d'urgence ou `5` pour voir les statistiques d'effraction.

---

## Protocole de Communication (I2C)

L'Arduino agit comme esclave à l'adresse **0x08**.

| Commande (Char) | Fonction |
| :--- | :--- |
| **'U'** | **Unlock** : Déverrouillage d'urgence |
| **'S'** | **Silence** : Arrêt du buzzer |
| **'R'** | **Reset** : Remise à zéro des logs |
| **'C'** + `[Bytes]` | **Code** : Force un nouveau code secret |
| **'L'** + `[Byte]` | **Length** : Change la longueur du code |

**Structure des Logs (Lecture 7 octets) :**
`[Tentatives (1B)] [Echecs (1B)] [Succès (1B)] [Alarmes (1B)] [Longueur Code (1B)] [Délai (2B)]`

---
*2025-2026 - Projet Universitaire*
