#!/usr/bin/env python3
"""
Ground Control Station - Versión Bidireccional
Recibe telemetría Y envía comandos al Bebop
"""

from pymavlink import mavutil
import time

class BidirectionalGCS:
    def __init__(self, connection_string='udp:192.168.42.255:14550'):
        """Inicializar GCS bidireccional"""
        print("=" * 60)
        print("GROUND CONTROL STATION - BIDIRECCIONAL")
        print("=" * 60)
        print(f"\n Conectando a: {connection_string}")
        
        self.master = mavutil.mavlink_connection(connection_string)
        
        print("Esperando heartbeat...")
        self.master.wait_heartbeat()
        
        print(f"¡Conectado exitosamente!")
        print(f"   Sistema: {self.master.target_system}")
        print(f"   Componente: {self.master.target_component}")
        print()
    
    def request_parameters(self):
        """Solicitar lista de parámetros del dron"""
        print("Solicitando lista de parámetros...")
        
        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )
        
        print("Comando enviado - Esperando respuesta...")
        
        # Recibir algunos parámetros
        params_received = 0
        timeout = time.time() + 10  # 10 segundos timeout
        
        while time.time() < timeout and params_received < 10:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg:
                params_received += 1
                param_id = msg.param_id
                param_value = msg.param_value
                print(f"   📋 {param_id}: {param_value}")
        
        print(f"Recibidos {params_received} parámetros\n")

    def request_data_stream(self, stream_id, rate_hz):
        """
        Solicitar stream de datos específico
        
        Args:
            stream_id: ID del stream (ver MAV_DATA_STREAM)
            rate_hz: Frecuencia en Hz
        """
        print(f"📤 Solicitando data stream {stream_id} a {rate_hz} Hz...")
        
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            stream_id,
            rate_hz,
            1  # start_stop (1 = start, 0 = stop)
        )
        
        print("Comando enviado\n")
    
    def set_mode(self, mode):
        """
        Cambiar modo de vuelo
        
        Args:
            mode: Nombre del modo (ej: 'STABILIZE', 'GUIDED', 'LOITER')
        """
        print(f"Cambiando modo a: {mode}")
        
        # Obtener número del modo
        mode_id = self.master.mode_mapping().get(mode)
        
        if mode_id is None:
            print(f"Modo '{mode}' no reconocido")
            print(f"   Modos disponibles: {list(self.master.mode_mapping().keys())}")
            return
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        print(f"Comando enviado - Esperando confirmación...")
        
        # Esperar confirmación
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg:
            if msg.result == 0:
                print(f"Modo cambiado exitosamente a {mode}\n")
            else:
                print(f"Comando aceptado pero resultado: {msg.result}\n")
        else:
            print("No se recibió confirmación\n")
    
    def arm_disarm(self, arm):
        """
        Armar o desarmar el dron
        
        Args:
            arm: True para armar, False para desarmar
        """
        action = "ARMANDO" if arm else "DESARMANDO"
        print(f"{action} el dron...")
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1 if arm else 0,  # param1: 1 = arm, 0 = disarm
            0, 0, 0, 0, 0, 0  # params 2-7 no usados
        )
        
        print(f"Comando enviado - Esperando confirmación...")
        
        # Esperar confirmación
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg:
            if msg.result == 0:
                status = "armado" if arm else "desarmado"
                print(f"Dron {status} exitosamente\n")
            else:
                print(f"Error al armar/desarmar: {msg.result}\n")
        else:
            print("No se recibió confirmación\n")
    
    def send_rc_override(self, roll, pitch, throttle, yaw):
        """
        Enviar comandos RC manual
        
        Args:
            roll, pitch, throttle, yaw: Valores PWM (1000-2000)
            Usar 0 para no enviar ese canal
        """
        print(f"Enviando comando RC: R={roll} P={pitch} T={throttle} Y={yaw}")
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            roll,      # channel 1
            pitch,     # channel 2
            throttle,  # channel 3
            yaw,       # channel 4
            0, 0, 0, 0  # channels 5-8
        )
        
        print("Comando enviado\n")
    
    def request_mission(self):
        """Solicitar lista de misión actual"""
        print("Solicitando lista de misión...")
        
        self.master.mav.mission_request_list_send(
            self.master.target_system,
            self.master.target_component
        )
        
        print("Comando enviado - Esperando respuesta...")
        
        msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=3)
        if msg:
            print(f"Misión tiene {msg.count} waypoints\n")
        else:
            print("No se recibió respuesta\n")
    
    def receive_telemetry(self, duration=30):
        """Recibir telemetría básica"""
        print(f"Recibiendo telemetría por {duration} segundos...")
        print("-" * 60)
        
        start_time = time.time()
        msg_count = 0
        
        while time.time() - start_time < duration:
            msg = self.master.recv_match(blocking=True, timeout=1)
            
            if msg:
                msg_count += 1
                msg_type = msg.get_type()
                
                if msg_type == 'HEARTBEAT':
                    mode = mavutil.mode_string_v10(msg)
                    armed = "ARMADO" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DESARMADO"
                    print(f"Heartbeat - Modo: {mode}, Estado: {armed}")
                
                elif msg_type == 'GPS_RAW_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0
                    sats = msg.satellites_visible
                    print(f"GPS: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m, sats={sats}")
                
                elif msg_type == 'ATTITUDE':
                    roll = msg.roll * 57.2958
                    pitch = msg.pitch * 57.2958
                    yaw = msg.yaw * 57.2958
                    print(f"Actitud: roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°")
                
                elif msg_type == 'SYS_STATUS':
                    battery = msg.battery_remaining
                    voltage = msg.voltage_battery / 1000.0
                    print(f"Batería: {battery}% ({voltage:.2f}V)")
        
        elapsed = time.time() - start_time
        print("\n" + "=" * 60)
        print(f"Telemetría completada")
        print(f"   Mensajes recibidos: {msg_count}")
        print(f"   Tasa: {msg_count/elapsed:.2f} msg/s")
        print("=" * 60)


def main():
    print("\n")
    print("╔════════════════════════════════════════════════════════╗")
    print("║       GCS BIDIRECCIONAL - ENVIAR Y RECIBIR            ║")
    print("╚════════════════════════════════════════════════════════╝")
    print()
    
    # Crear GCS
    gcs = BidirectionalGCS()
    
    # ========================================
    # DEMO: Enviar varios comandos
    # ========================================
    
    print("\nDEMO DE COMANDOS:\n")
    
    # 1. Solicitar parámetros
    gcs.request_parameters()
    
    time.sleep(2)
    
    # 2. Solicitar data stream (posición)
    # MAV_DATA_STREAM_POSITION = 2
    gcs.request_data_stream(stream_id=2, rate_hz=1)
    
    time.sleep(1)
    
    # 3. Ver modos disponibles
    print("Modos de vuelo disponibles:")
    for mode_name in gcs.master.mode_mapping().keys():
        print(f"   - {mode_name}")
    print()
    
    time.sleep(1)
    
    # # 4. Cambiar a modo STABILIZE (ejemplo - NO armará el dron)
    # # ADVERTENCIA: NO usar en dron real sin precaución
    # # gcs.set_mode('STABILIZE')
    # print("Comando set_mode comentado por seguridad")
    # print("   Descomentar en código si quieres probarlo\n")
    
    time.sleep(1)
    
    # 5. Solicitar misión
    gcs.request_mission()
    
    time.sleep(1)
    
    # 6. Recibir telemetría
    print("\n" + "=" * 60)
    gcs.receive_telemetry(duration=20)
    
    print("\nDemo completada\n")


if __name__ == "__main__":
    main()