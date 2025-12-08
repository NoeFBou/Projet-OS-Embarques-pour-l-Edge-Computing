import smbus
import time
import sys

ARDUINO_ADDR = 0x08
I2C_BUS = 1

try:
    bus = smbus.SMBus(I2C_BUS)
except Exception as e:
    print(f"Erreur: Impossible d'ouvrir le bus I2C. {e}")
    sys.exit(1)

def send_unlock():
    try:
        bus.write_byte(ARDUINO_ADDR, ord('U'))
        print("-> Commande: DEVERROUILLER envoyée.")
    except Exception as e:
        print(f"Erreur de communication: {e}")

def send_silence():
    try:
        bus.write_byte(ARDUINO_ADDR, ord('S'))
        print("-> Commande: SILENCE BUZZER envoyée.")
    except Exception as e:
        print(f"Erreur de communication: {e}")

def send_new_code(code_list):
    if len(code_list) != 4:
        print("Erreur: Le code doit faire 4 chiffres.")
        return
    try:
        bus.write_i2c_block_data(ARDUINO_ADDR, ord('C'), code_list)
        print(f"-> Nouveau code {code_list} envoyé.")
    except Exception as e:
        print(f"Erreur de communication: {e}")

def main():
    print("=== CONTROLE DU COFFRE ===")
    while True:
        print("\n1. Deverrouiller (Unlock)")
        print("2. Couper le Buzzer (Silence)")
        print("3. Definir nouveau code")
        print("4. Quitter")

        choix = input("Choix: ")

        if choix == '1':
            send_unlock()
        elif choix == '2':
            send_silence()
        elif choix == '3':
            try:
                saisie = input("Entrez 4 chiffres (ex: 1234): ")
                if len(saisie) == 4 and saisie.isdigit():
                    liste = [int(c) for c in saisie]
                    send_new_code(liste)
                else:
                    print("Erreur: Il faut 4 chiffres.")
            except ValueError:
                print("Erreur de saisie.")
        elif choix == '4':
            break
        time.sleep(0.5)

if __name__ == "__main__":
    main()