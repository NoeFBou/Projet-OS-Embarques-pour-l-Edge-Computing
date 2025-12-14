import smbus
import time
import sys
import struct

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
def read_logs():
    try:
        data = bus.read_i2c_block_data(ARDUINO_ADDR, 0x00, 7)
        valeurs = struct.unpack('<BBBBBH', bytes(data))

        print("\n--- STATISTIQUES OPTIMISEES ---")
        print(f"Tentatives      : {valeurs[0]}")
        print(f"Echecs          : {valeurs[1]}")
        print(f"Succes          : {valeurs[2]}")
        print(f"Alarmes         : {valeurs[3]}")
        print("------------------------------")
        print(f"Longueur Code   : {valeurs[4]}")
        print(f"Vitesse Saisie  : {valeurs[5]} ms")
        print("------------------------------")

    except Exception as e:
        print(f"Erreur lecture logs: {e}")

def read_length_code():
    try:
        data = bus.read_i2c_block_data(ARDUINO_ADDR, 0x00, 7)
        valeurs = struct.unpack('<BBBBBH', bytes(data))
        return valeurs[4]
    except Exception as e:
        print(f"Erreur lecture longueur code: {e}")

def send_delay(delay):
    try:
        bytes_delay = list(struct.pack('<H', delay))
        bus.write_i2c_block_data(ARDUINO_ADDR, ord('D'), bytes_delay)
        print(f"-> Nouveau délai {delay} ms envoyé.")
    except Exception as e:
        print(f"Erreur: {e}")

def send_length_code(length):
    try:
        bus.write_byte_data(ARDUINO_ADDR, ord('L'), int(length))
        print(f"-> Nouvelle longueur {length} envoyée.")
    except Exception as e:
        print(f"Erreur: {e}")
def send_new_code(code_list):
    if len(code_list) > 10:
        print("Erreur: Le code ne peut pas dépasser 10 chiffres.")
        return
    try:
        bus.write_i2c_block_data(ARDUINO_ADDR, ord('C'), code_list)
        print(f"-> Nouveau code {code_list} envoyé.")
    except Exception as e:
        print(f"Erreur de communication: {e}")

def reset_logs():
    try:
        bus.write_byte(ARDUINO_ADDR, ord('R'))
        print("-> Logs Reset OK.")
    except Exception as e:
        print(f"Erreur: {e}")
def main():
    print("=== CONTROLE DU COFFRE ===")
    while True:
        print("\n1. Deverrouiller (Unlock)")
        print("2. Couper le Buzzer (Silence)")
        print("3. Definir nouveau code")
        print("4. Reset Stats")
        print("5. Lire Logs & Config")
        print("6. Configurer Vitesse (Delay)")
        print("7. Configurer Longueur")
        print("8. Quitter")

        choix = input("Choix: ")

        if choix == '1':
            send_unlock()
        elif choix == '2':
            send_silence()
        elif choix == '3':
            try:
                nr_chiffres = read_length_code()
                saisie = input("Enter new " + str(nr_chiffres) + " digit code: ")

                if len(saisie) == nr_chiffres and saisie.isdigit():
                    liste = [int(c) for c in saisie]
                    send_new_code(liste)
                else:
                    print("Erreur: Il faut entrer un code de " + str(nr_chiffres) + " chiffres.")
            except ValueError:
                print("Erreur de saisie.")
        elif choix == '4':
            reset_logs()
        elif choix == '5':
            read_logs()
        elif choix == '6':
            try:
                d = int(input("Nouveau délai (ms, 200-2000) : "))
                send_delay(d)
            except ValueError:
                print("Entrez un nombre entier.")
        elif choix == '7':
            try:
                l = int(input("Nouvelle longueur (1-10) : "))
                if 1 <= l <= 10:
                    send_length_code(l)
                else:
                    print("Entre 1 et 10 svp.")
            except ValueError:
                print("Entrez un nombre entier.")
        elif choix == '8':
            break
        time.sleep(0.5)

if __name__ == "__main__":
    main()