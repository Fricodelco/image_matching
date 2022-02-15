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

    def ecef2Geodetic(self, x,  y, z): 
        r = sqrt(x * x + y * y)
        Esq = self.kSemimajorAxis * self.kSemimajorAxis - self.kSemiminorAxis * self.kSemiminorAxis
        F = 54 * self.kSemiminorAxis * self.kSemiminorAxis * z * z
        G = r * r + (1 - self.kFirstEccentricitySquared) * z * z - self.kFirstEccentricitySquared * Esq
        C = (self.kFirstEccentricitySquared * self.kFirstEccentricitySquared * F * r * r) /(G**3)
        S = (1 + C + sqrt(C * C + 2 * C))**(1./3.)
        P = F / (3 * ((S + 1 / S + 1)**2) * G * G)
        Q = sqrt(1 + 2 * self.kFirstEccentricitySquared * self.kFirstEccentricitySquared * P)
        r_0 = -(P * self.kFirstEccentricitySquared * r) / (1 + Q) + sqrt(
                                                                          0.5 * self.kSemimajorAxis * self.kSemimajorAxis * (1 + 1.0 / Q) - P * (1 - self.kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r)
        U = sqrt(((r - self.kFirstEccentricitySquared * r_0)**2) + z * z)
        V = sqrt(
            ((r - self.kFirstEccentricitySquared * r_0)**2) + (1 - self.kFirstEccentricitySquared) * z * z)
        Z_0 = self.kSemiminorAxis * self.kSemiminorAxis * z / (self.kSemimajorAxis * V)
        altitude = U * (1 - self.kSemiminorAxis * self.kSemiminorAxis / (self.kSemimajorAxis * V))
        latitude = self.rad2Deg(atan((z + self.kSecondEccentricitySquared * Z_0) / r))
        longitude = self.rad2Deg(atan2(y, x))
        return latitude, longitude, altitude

    def ned2Ecef(self, north, east, down):
        ned = np.ones(3)
        ret = np.ones(3)
        ned[0] = north
        ned[1] = east
        ned[2] = -down
        ret = self.ned_to_ecef_matrix_.dot(ned)
        x = ret[0] + self.initial_ecef_x_
        y = ret[1] + self.initial_ecef_y_
        z = ret[2] + self.initial_ecef_z_
        return x, y, z

    def ned2Geodetic(self, north, east, down):
        x,y,z = self.ned2Ecef(north, east, down)
        latitude, longitude, altitude = self.ecef2Geodetic(x, y, z)
        return latitude, longitude, altitude

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

    def rad2Deg(self, radians) -> float:
        return (radians/pi)*180

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