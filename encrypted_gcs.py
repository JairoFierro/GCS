#!/usr/bin/env python3
"""
GCS con soporte ASCON-128
Compatible con ArduPilot libraries/ascon/
"""

import struct
import time
from pymavlink import mavutil

# Importar ASCON oficial
try:
    from ascon import ascon_encrypt, ascon_decrypt
    print("[GCS] ✓ pyascon cargado")
except ImportError:
    print("[GCS] Error: instalar con 'pip install ascon'")
    exit(1)



# Key como el que tengo en c++
ASCON_KEY = bytes([
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
])

# Nonce base (será construido dinámicamente) este es el mismo que en c++
# En tu C++ usas: [ iv_boot(8) | sysid(1) | compid(1) | seq(1) | pad(5) ]
IV_BOOT = 0x1122334455667788  # Del C++

MAVLINK_IFLAG_ENCRYPTED = 0x02



def construct_nonce(iv_boot, sysid, compid, seq):
    """
    Construye el nonce de 16 bytes como en C++:
    [ iv_boot(8) | sysid(1) | compid(1) | seq(1) | padding(5) ]
    """
    nonce = bytearray(16)
    
    # iv_boot: 8 bytes (big endian)
    nonce[0:8] = struct.pack('>Q', iv_boot)
    
    # sysid: 1 byte
    nonce[8] = sysid
    
    # compid: 1 byte
    nonce[9] = compid
    
    # seq: 1 byte
    nonce[10] = seq
    
    # padding: 5 bytes (ceros)
    # nonce[11:16] ya son ceros por defecto
    
    return bytes(nonce)


# GCS CON DESCIFRADO

class EncryptedGCS:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        print(f"[GCS] Conectando a {connection_string}...")
        self.connection_string = connection_string
        self.mav = mavutil.mavlink_connection(connection_string)
        self.key = ASCON_KEY
        self.iv_boot = IV_BOOT
        
        print(f"[GCS] Conectado")
        print(f"[GCS] Cifrado ASCON-128 ACTIVADO")
        print(f"[GCS] Key: {self.key.hex()}")
        print(f"[GCS] IV_boot: 0x{self.iv_boot:016X}")
    
    def wait_heartbeat(self):
        """Espera el primer heartbeat"""
        print("[GCS] Esperando heartbeat del dron...")
        msg = self.mav.wait_heartbeat(timeout=10)
        if msg:
            print(f"[GCS] Heartbeat de sistema {msg.get_srcSystem()}")
            return True
        print("[GCS] Timeout esperando heartbeat")
        return False
    
    def decrypt_message(self, msg_bytes, sysid, compid, seq):
        """
        Descifra un mensaje MAVLink cifrado
        
        Args:
            msg_bytes: bytes del payload cifrado (incluye tag de 16 bytes)
            sysid: system ID del mensaje
            compid: component ID del mensaje
            seq: número de secuencia del mensaje
        
        Returns:
            bytes descifrados o None si falla
        """
        try:
            # Construir nonce igual que en C++
            nonce = construct_nonce(self.iv_boot, sysid, compid, seq)
            
            print(f"\n╔═══════════════════════════════════════════════╗")
            print(f"║     DESCIFRANDO MENSAJE                       ║")
            print(f"╠═══════════════════════════════════════════════╣")
            print(f"║ sysid: {sysid}")
            print(f"║ compid: {compid}")
            print(f"║ seq: {seq}")
            print(f"║ nonce: {nonce.hex()}")
            print(f"║ len cifrado: {len(msg_bytes)} bytes")
            print(f"║ primeros bytes: {msg_bytes[:min(8, len(msg_bytes))].hex()}")
            
            # Descifrar con ASCON-128
            plaintext = ascon_decrypt(
                key=self.key,
                nonce=nonce,
                associateddata=b'',  
                ciphertext=msg_bytes  
            )
            
            print(f"║ len descifrado: {len(plaintext)} bytes")
            print(f"║ primeros bytes: {plaintext[:min(8, len(plaintext))].hex()}")
            print(f"║ DESCIFRADO EXITOSO")

            
            return plaintext
            
        except Exception as e:
            print(f"ERROR AL DESCIFRAR: {e}")
            return None
    
    def receive_loop(self):
        """Loop principal de recepción"""
        print("\n[GCS] Iniciando loop de telemetría...")
        print("[GCS] Presiona Ctrl+C para salir")
        print("=" * 60)
        
        msg_count = 0
        encrypted_count = 0
        decrypted_count = 0
        
        try:
            while True:
                msg = self.mav.recv_match(blocking=True, timeout=1)
                
                if msg is None:
                    continue
                
                msg_count += 1
                msg_type = msg.get_type()
                
                # Verificar si está cifrado
                if hasattr(msg, '_header') and (msg._header.incompat_flags & MAVLINK_IFLAG_ENCRYPTED):
                    encrypted_count += 1
                    
                    # Obtener parámetros del mensaje
                    sysid = msg.get_srcSystem()
                    compid = msg.get_srcComponent()
                    seq = msg._header.seq
                    
                    # Obtener payload cifrado
                    encrypted_payload = msg.get_payload()
                    
                    # Intentar descifrar
                    plaintext = self.decrypt_message(encrypted_payload, sysid, compid, seq)
                    
                    if plaintext:
                        decrypted_count += 1
                        # Nota: PyMAVLink hace difícil reemplazar el payload descifrado
                        # Para simplificar, solo mostramos que se descifró correctamente
                        print(f"[{msg_count:04d}] {msg_type} (CIFRADO → DESCIFRADO)")
                    else:
                        print(f"[{msg_count:04d}] {msg_type} (ERROR DESCIFRADO)")
                    
                    continue
                
                # Mensajes no cifrados - procesamiento normal
                if msg_type == 'HEARTBEAT':
                    armed = 'ARMED' if (msg.base_mode & 128) else 'DISARMED'
                    print(f"[{msg_count:04d}] HEARTBEAT | {armed} | Mode:{msg.custom_mode}")
                
                elif msg_type == 'GLOBAL_POSITION_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0
                    print(f"[{msg_count:04d}] GPS | Lat:{lat:.6f} Lon:{lon:.6f} Alt:{alt:.1f}m")
                
                elif msg_type == 'ATTITUDE':
                    roll = msg.roll * 57.2958
                    pitch = msg.pitch * 57.2958
                    yaw = msg.yaw * 57.2958
                    print(f"[{msg_count:04d}] ATT | R:{roll:6.1f}° P:{pitch:6.1f}° Y:{yaw:6.1f}°")
                
                elif msg_type == 'SYS_STATUS':
                    bat = msg.battery_remaining
                    print(f"[{msg_count:04d}] STATUS | Battery:{bat}%")
                
                elif msg_type == 'VFR_HUD':
                    print(f"[{msg_count:04d}] VFR_HUD | AS:{msg.airspeed:.1f} GS:{msg.groundspeed:.1f}")
                
                else:
                    # Otros mensajes (comentar esta línea para menos spam)
                    # print(f"[{msg_count:04d}] {msg_type}")
                    pass
                
        except KeyboardInterrupt:
            print("\n" + "=" * 60)
            print(f"[GCS] Cerrando...")
            print(f"[GCS] Total mensajes: {msg_count}")
            print(f"[GCS] Cifrados recibidos: {encrypted_count}")
            print(f"[GCS] Descifrados exitosos: {decrypted_count}")
            if encrypted_count > 0:
                success_rate = (decrypted_count / encrypted_count) * 100
                print(f"[GCS] Tasa de éxito: {success_rate:.1f}%")


def main():
    print("=" * 60)
    print("  GCS con Cifrado ASCON-128")
    print("  Compatible con ArduPilot libraries/ascon/")
    print("=" * 60)
    print()
    
    gcs = EncryptedGCS('tcp:127.0.0.1:5760')
    
    if gcs.wait_heartbeat():
        gcs.receive_loop()
    else:
        print("[GCS] No se pudo conectar al dron")

if __name__ == "__main__":
    main()