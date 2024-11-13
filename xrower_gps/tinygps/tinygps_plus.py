from .gps_data import GPSData
class TinyGPSPlus:
    
    def __init__(self):
        self.gps_data = GPSData()

    def encode(self, nmea_sentence):
        parts = nmea_sentence.split(',')
        if parts[0] == "$GNGGA":
            self.parse_gngga(parts)
        elif parts[0] == "$GNRMC":
            self.parse_gnrmc(parts)
        elif parts[0] == "$GNGSA":
            self.parse_gngsa(parts)
        elif parts[0] == "$GPGSV":
            self.parse_gpgsv(parts)

    def parse_gngga(self, parts):
        self.gps_data.latitude = self.convert_to_decimal(""+parts[2]+", "+parts[3]+"")
        self.gps_data.longitude = self.convert_to_decimal(""+parts[4]+", "+parts[5]+"")
        self.gps_data.altitude = float(parts[9])
        self.gps_data.satellites = int(parts[7])
        self.gps_data.fix_quality = int(parts[6])
    
    def parse_gnrmc(self, parts):
        if parts[2] == 'A':  # Status valid
            self.gps_data.latitude = self.convert_to_decimal(""+parts[3]+", "+parts[4]+"")
            self.gps_data.longitude = self.convert_to_decimal(""+parts[5]+", "+ parts[6]+"")
            self.gps_data.speed = float(parts[7]) * 1.15078  # knots to mph

    def parse_gngsa(self, parts):
        # Example parsing of GNGSA
        pass

    def parse_gpgsv(self, parts):
        # Example parsing of GPGSV
        pass

    def convert_to_decimal(self,coord_str):
        coord_parts = coord_str.split(', ')
        degrees = float(coord_parts[0])
        direction = coord_parts[1]
        decimal_degrees = (degrees // 100) + (degrees % 100) / 60.0
        if direction in ['S', 'W']:
            decimal_degrees *= -1

        return decimal_degrees

