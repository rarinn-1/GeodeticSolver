#ifndef __GEODETIC_SOLVER_H
#define __GEODETIC_SOLVER_H

// Mean earth radius (great circle formulas use it).
#define RADIUS 6371008.8

// WGS-84 oblate spheroid parameters:
#define ELLIPSOID_SEMI_MAJOR_AXIS 6378137.0 // length of semi-major axis of the ellipsoid (equatorial radius)
#define ELLIPSOID_FLATTENING 1.0/298.257223563 // flattening of the ellipsoid
#define ELLIPSOID_SEMI_MINOR_AXIS (1-f)*a // length of semi-minor axis of the ellipsoid

// Following symbolic constants put a limit on how many times loops can be executed (Vincenty's formulae).
#define INVERSE_MAX_ITERATIONS 300000
#define INVERSE_MAX_ITERATIONS_ANTIPODAL 60000
#define DIRECT_MAX_ITERATIONS 100000

// Initial exponent (it will be used as negative number) for iteration tolerance and a minimum one (used when full accuracy does 
// not allow convergence) .
#define TOLERANCE_BASIC 12
#define TOLERANCE_MINIMUM 6

void great_circle_inverse(double first_latitude, double first_longitude, double second_latitude, double second_longitude, 
                  double *circle_azimuth_1, double *circle_azimuth_2, double *circle_distance);

void great_circle_direct(double first_latitude, double first_longitude, double circle_azimuth_1, double circle_distance_w, 
                         double *circle_azimuth_w, double *circle_latitude_w, double *circle_longitude_w);

int vincenty_inverse(double first_latitude, double first_longitude, double second_latitude, double second_longitude, 
              double *ellipsoid_azimuth_1, double *ellipsoid_azimuth_2, double *ellipsoid_distance);

int vincenty_direct(double first_latitude, double first_longitude, double ellipsoid_azimuth_1, double ellipsoid_distance_w, 
                         double *ellipsoid_azimuth_w, double *ellipsoid_latitude_w, double *ellipsoid_longitude_w);

#endif
