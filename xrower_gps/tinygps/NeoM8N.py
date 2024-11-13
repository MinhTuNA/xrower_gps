from .tinygps_plus import TinyGPSPlus
import serial


uart_serial = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
gps = TinyGPSPlus()
def get_gps_data():
    try:
        line = uart_serial.readline().decode('ascii', errors='replace').strip()
        if line:
            gps.encode(line)
            if gps.gps_data.is_valid():
                return {
                    "latitude": gps.gps_data.latitude, 
                    "longitude": gps.gps_data.longitude,
                    "altitude": gps.gps_data.altitude,
                    "speed": gps.gps_data.speed,
                    "satellites": gps.gps_data.satellites
                }
            else:
                return None
        return None
    except Exception as e:
        return {"error": str(e)}