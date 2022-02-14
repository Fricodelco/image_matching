from decimal import Decimal
from math import sqrt, cos, sin, pi, atan2, atan
import numpy as np
from numpy.core.fromnumeric import shape

class GeodeticConvert:
    def __init__(self) -> None:
        self.have_reference_ = False

        self.kSemimajorAxis = 6378137
        self.kSemiminorAxis = 6356752.3142
        self.kFirstEccentricitySquared = 6.69437999014 * 0.001
        self.kSecondEccentricitySquared = 6.73949674228 * 0.001
        self.kFlattening = 1 / 298.257223563
    
    def isInitialised(self) -> bool:
        return self.have_reference_
    
    def getReference(self) -> list:
        if self.have_reference_:
            return [self.initial_latitude_,\
                    self.initial_longitude_,\
                    self.initial_altitude_]
    
    def initialiseReference(self, latitude, longitude, altitude) -> None:
        self.initial_latitude_ = self.deg2Rad(latitude)
        self.initial_longitude_ = self.deg2Rad(longitude)
        self.initial_altitude_ = altitude

        self.initial_ecef_x_, self.initial_ecef_y_, self.initial_ecef_z_ = self.geodetic2Ecef(latitude, longitude, altitude)
        phiP = atan2(self.initial_ecef_z_, sqrt(self.initial_ecef_x_**2 + self.initial_ecef_y_**2))

        self.ecef_to_ned_matrix_ = self.nRe(phiP, self.initial_longitude_)
        self.ned_to_ecef_matrix_ = self.nRe(self.initial_latitude_, self.initial_longitude_).transpose()

        self.have_reference_ = True

    def geodetic2Ecef(self, latitude, longitude, altitude) -> list:
        lat_rad = self.deg2Rad(latitude)
        lon_rad = self.deg2Rad(longitude)
        xi = sqrt(1 - self.kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad))
        x = (self.kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad)
        y = (self.kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad)
        z = (self.kSemimajorAxis / xi * (1 - self.kFirstEccentricitySquared) + altitude) * sin(lat_rad)
        return [x,y,z]

    def geodetic2Ned(self, latitude, longitude, altitude) -> list:
        x, y, z = self.geodetic2Ecef(latitude, longitude, altitude)
        north, east, down = self.ecef2Ned(x, y, z)
        return [north, east, down]

    def ecef2Ned(self, x, y, z):
        vect = np.ones(3)
        ret = np.ones(3)

        vect[0] = x - self.initial_ecef_x_
        vect[1] = y - self.initial_ecef_y_
        vect[2] = z - self.initial_ecef_z_
        ret = self.ecef_to_ned_matrix_.dot(vect)
        north = ret[0]
        east = ret[1]
        down = ret[2]

        return [north, east, down]

    def deg2Rad(self, degrees) -> float:
        return degrees/180.0*pi

    def nRe(self, lat_radians, lon_radians):
        sLat = sin(lat_radians)
        sLon = sin(lon_radians)
        cLat = cos(lat_radians)
        cLon = cos(lon_radians)
    
        ret = np.zeros((3,3))
        ret[0, 0] = -sLat * cLon
        ret[0, 1] = -sLat * sLon
        ret[0, 2] = cLat
        ret[1, 0] = -sLon
        ret[1, 1] = cLon
        ret[1, 2] = 0.0
        ret[2, 0] = cLat * cLon
        ret[2, 1] = cLat * sLon
        ret[2, 2] = sLat
    
        return ret


# g_c = GeodeticConvert()

# g_c.initialiseReference(55.6751477, 37.884128, 99.8934033)
# print(g_c.geodetic2Ned(55.6335031, 37.9837743,  99.8934033))