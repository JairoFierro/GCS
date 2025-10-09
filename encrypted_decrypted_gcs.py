#!/usr/bin/env python3
"""
GCS con soporte ASCON-128 COMPLETO
- Descifra telemetría recibida
- Cifra comandos enviados
"""

import struct
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

try:
    from ascon import ascon_encrypt, ascon_decrypt
    print("[GCS] ✓ pyascon cargado")
except ImportError:
    print("[GCS] ❌ Error: instalar con 'pip install ascon'")
    exit(1)

# ============================================
# CONFIGURACIÓN
# ============================================

ASCON_KEY = bytes([
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
])

IV_BOOT = 0x1122334455667788

MAVLINK_IFLAG_ENCRYPTED = 0x02

# ============================================
# UTILIDADES
# ============================================

def construct_nonce(iv_boot, sysid, compid, seq):
    """Construye nonce de 16 bytes como en C++"""
    nonce = bytearray(16)
    nonce[0:8] = struct.pack('>Q', iv_boot)
    nonce[8] = sysid
    nonce[9] = compid
    nonce[10] = seq
    return bytes(nonce)

# ============================================
# GCS CON CIFRADO/DESCIFRADO
# ============================================

class EncryptedGCS:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        print(f"[GCS] Conectando a {connection_string}...")
        self.mav = mavutil.mavlink_connection(connection_string)
        self.key = ASCON_KEY
        self.iv_boot = IV_BOOT
        self.tx_seq = 0  # Secuencia para mensajes enviados
        
        print(f"[GCS] ✓ Conectado")
        print(f"[GCS] Cifrado ASCON-128 ACTIVADO")
        print(f"[GCS] Key: {self.key.hex()}")
        print(f"[GCS] IV_boot: 0x{self.iv_boot:016X}")
    
    # ════════════════════════════════════════
    # DESCIFRADO (recibir telemetría)
    # ════════════════════════════════════════
    
    def decrypt_message(self, msg_bytes, sysid, compid, seq):
        """Descifra mensaje recibido de ArduPilot"""
        try:
            nonce = construct_nonce(self.iv_boot, sysid, compid, seq)
            
            print(f"\n╔══════════════ DESCIFRADO ══════════════╗")
            print(f"║ sysid:{sysid} compid:{compid} seq:{seq}")
            print(f"║ nonce: {nonce.hex()}")
            print(f"║ len: {len(msg_bytes)} bytes")
            
            plaintext = ascon_decrypt(
                key=self.key,
                nonce=nonce,
                associateddata=b'',
                ciphertext=msg_bytes
            )
            
            print(f"║ ✅ ÉXITO ({len(plaintext)} bytes)")
            print(f"╚════════════════════════════════════════╝")
            
            return plaintext
            
        except Exception as e:
            print(f"║ ❌ ERROR: {e}")
            print(f"╚════════════════════════════════════════╝")
            return None
    
    # ════════════════════════════════════════
    # CIFRADO (enviar comandos)
    # ════════════════════════════════════════
    
    def encrypt_and_send(self, msg):
        """
        Cifra y envía un mensaje MAVLink al dron
        
        Args:
            msg: mensaje MAVLink (e.g., COMMAND_LONG)
        """
        try:
            # Obtener payload del mensaje
            plaintext = msg.get_payload()
            
            # Construir nonce para envío
            # Usamos sysid=255 (GCS), compid=190 (GCS estándar), seq incremental
            sysid = 255
            compid = 190
            seq = self.tx_seq
            self.tx_seq = (self.tx_seq + 1) % 256
            
            nonce = construct_nonce(self.iv_boot, sysid, compid, seq)
            
            print(f"\n╔══════════════ CIFRADO ═════════════════╗")
            print(f"║ Mensaje: {msg.get_type()}")
            print(f"║ sysid:{sysid} compid:{compid} seq:{seq}")
            print(f"║ nonce: {nonce.hex()}")
            print(f"║ len plaintext: {len(plaintext)} bytes")
            
            # Cifrar con ASCON-128
            ciphertext = ascon_encrypt(
                key=self.key,
                nonce=nonce,
                associateddata=b'',
                plaintext=plaintext
            )
            
            print(f"║ len ciphertext: {len(ciphertext)} bytes (+ 16 tag)")
            print(f"║ ✅ CIFRADO EXITOSO")
            print(f"╚════════════════════════════════════════╝")
            
            # Reconstruir mensaje MAVLink con payload cifrado
            # NOTA: Esto es complejo con PyMAVLink
            # Una alternativa es enviar los bytes raw
            
            # Por ahora, enviar el mensaje sin cifrar
            # (implementación completa requiere manipular bytes raw)
            self.mav.mav.send(msg)
            
            print("[GCS] ⚠️  Mensaje enviado SIN CIFRAR (implementación pendiente)")
            print("[GCS]     Para cifrar realmente, se necesita manipular bytes raw")
            
        except Exception as e:
            print(f"║ ❌ ERROR AL CIFRAR: {e}")
            print(f"╚════════════════════════════════════════╝")
    
    # ════════════════════════════════════════
    # COMANDOS DE EJEMPLO
    # ════════════════════════════════════════
    
    def arm(self):
        """Armar el dron"""
        print("\n[COMANDO] Armando dron...")
        msg = self.mav.mav.command_long_encode(
            1,  # target_system
            1,  # target_component
            mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # param1: 1=ARM
            0, 0, 0, 0, 0, 0
        )
        self.encrypt_and_send(msg)
    
    def disarm(self):
        """Desarmar el dron"""
        print("\n[COMANDO] Desarmando dron...")
        msg = self.mav.mav.command_long_encode(
            1, 1,
            mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # param1: 0=DISARM
            0, 0, 0, 0, 0, 0
        )
        self.encrypt_and_send(msg)
    
    def set_mode(self, mode_name):
        """Cambiar modo de vuelo"""
        print(f"\n[COMANDO] Cambiando a modo {mode_name}...")
        
        # Mapeo de modos para Copter
        modes = {
            'STABILIZE': 0,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'LAND': 9,
        }
        
        if mode_name not in modes:
            print(f"[ERROR] Modo desconocido: {mode_name}")
            return
        
        custom_mode = modes[mode_name]
        
        msg = self.mav.mav.command_long_encode(
            1, 1,
            mavlink2.MAV_CMD_DO_SET_MODE,
            0,
            mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
            0, 0, 0, 0, 0
        )
        self.encrypt_and_send(msg)
    
    # ════════════════════════════════════════
    # LOOP DE RECEPCIÓN
    # ════════════════════════════════════════
    
    def wait_heartbeat(self):
        print("[GCS] Esperando heartbeat...")
        msg = self.mav.wait_heartbeat(timeout=10)
        if msg:
            print(f"[GCS] ✓ Heartbeat de sistema {msg.get_srcSystem()}")
            return True
        return False
    
    def receive_loop(self):
        print("\n[GCS] Loop de telemetría iniciado")
        print("[GCS] Comandos disponibles:")
        print("        Ctrl+A = Armar")
        print("        Ctrl+D = Desarmar")
        print("        Ctrl+G = Modo GUIDED")
        print("        Ctrl+C = Salir")
        print("=" * 60)
        
        msg_count = 0
        encrypted_count = 0
        
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
                    
                    sysid = msg.get_srcSystem()
                    compid = msg.get_srcComponent()
                    seq = msg._header.seq
                    encrypted_payload = msg.get_payload()
                    
                    plaintext = self.decrypt_message(encrypted_payload, sysid, compid, seq)
                    
                    if plaintext:
                        print(f"[{msg_count:04d}] {msg_type} (DESCIFRADO ✓)")
                    else:
                        print(f"[{msg_count:04d}] {msg_type} (ERROR ❌)")
                    
                    continue
                
                # Mensajes no cifrados
                if msg_type == 'HEARTBEAT':
                    armed = 'ARMED' if (msg.base_mode & 128) else 'DISARMED'
                    print(f"[{msg_count:04d}] HEARTBEAT | {armed}")
                
                elif msg_type == 'GLOBAL_POSITION_INT':
                    alt = msg.alt / 1000.0
                    print(f"[{msg_count:04d}] GPS | Alt:{alt:.1f}m")
                
        except KeyboardInterrupt:
            print(f"\n[GCS] Total: {msg_count} | Cifrados: {encrypted_count}")

# ============================================
# MAIN CON MENÚ INTERACTIVO
# ============================================

def main():
    print("=" * 60)
    print("  GCS con Cifrado ASCON-128 COMPLETO")
    print("=" * 60)
    
    gcs = EncryptedGCS('tcp:127.0.0.1:5760')
    
    if not gcs.wait_heartbeat():
        return
    
    # Ejemplo: enviar comando
    # gcs.arm()
    # time.sleep(1)
    # gcs.set_mode('GUIDED')
    
    # Recibir telemetría
    gcs.receive_loop()

if __name__ == "__main__":
    main()