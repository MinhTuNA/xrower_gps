class GPSData:
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed = None
        self.satellites = None
        self.hdop = None
        self.fix_quality = None
        self.date = None
        self.time = None

    def is_valid(self):
        return self.latitude is not None and self.longitude is not None
