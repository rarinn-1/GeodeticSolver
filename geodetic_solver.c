#include <math.h>
#include "geodetic_solver.h"

// WGS-84 oblate spheroid parameters (use the values of symbolic constants from geodetic_solver.h, so that nice names [a, b and f] can 
// be used in the following equations without conflict with geodesic.h from GeographicLib).
#define a ELLIPSOID_SEMI_MAJOR_AXIS // length of semi-major axis of the ellipsoid (equatorial radius)
#define f ELLIPSOID_FLATTENING // flattening of the ellipsoid
#define b ELLIPSOID_SEMI_MINOR_AXIS // length of semi-minor axis of the ellipsoid


// This is a straightforward implementation of equations from https://en.wikipedia.org/wiki/Great-circle_navigation for inverse 
// geodesic problem.

void great_circle_inverse(double fi1, double lambda1, double fi2, double lambda2, 
                  double *circle_azimuth_1, double *circle_azimuth_2, double *circle_distance) {

	double lambda12, sigma12, s12, alpha1, alpha2;

	lambda12 = lambda2 - lambda1;

	sigma12 = 
		atan2( 
			hypot( 
				cos(fi1)*sin(fi2)-sin(fi1)*cos(fi2)*cos(lambda12) , 
				cos(fi2)*sin(lambda12)
			) 
			, sin(fi1)*sin(fi2)+cos(fi1)*cos(fi2)*cos(lambda12)
		)
	;

	s12 = RADIUS * sigma12;

	alpha1 = atan2( cos(fi2)*sin(lambda12), cos(fi1)*sin(fi2) - sin(fi1)*cos(fi2)*cos(lambda12) );
	alpha2 = atan2( cos(fi1)*sin(lambda12), -cos(fi2)*sin(fi1) + sin(fi2)*cos(fi1)*cos(lambda12) );

	// Return computed values via pointers.
	*circle_azimuth_1 = alpha1;
	*circle_azimuth_2 = alpha2;
	*circle_distance = s12;

}


// This is a practical implementation of equations from https://en.wikipedia.org/wiki/Great-circle_navigation for direct geodesic problem.

void great_circle_direct(double fi1, double lambda1, double alpha1, double d, 
                         double *circle_azimuth_w, double *circle_latitude_w, double *circle_longitude_w) {

	double alpha, alpha0, sigma01, sigma, lambda, lambda01, lambda0, fi, lambda_minus_lambda0;

	alpha0 = atan2( sin(alpha1)*cos(fi1) , hypot( cos(alpha1) , sin(alpha1)*sin(fi1) ) );

	if( (fi1==0.0) && ( (alpha1>(M_PI_2-1.0e-15)) && (alpha1<(M_PI_2+1.0e-15)) ) ) sigma01 = 0.0;
	else sigma01 = atan2( tan(fi1) , cos(alpha1) );

	lambda01 = atan2( sin(alpha0)*sin(sigma01) , cos(sigma01) );
	lambda0 = lambda1 - lambda01;

	sigma = sigma01 + d/RADIUS;

	fi = atan2( cos(alpha0)*sin(sigma) , hypot( cos(sigma), sin(alpha0)*sin(sigma) ) );
	if(fi > -5.0e-16) fi = fabs(fi); // make sure that 0.0 won't be interpreted as -0.0

	lambda_minus_lambda0 = atan2( sin(alpha0)*sin(sigma) , cos(sigma) );

	lambda = lambda_minus_lambda0 + lambda0;

	// To keep longitude of resulting point in the right range add or subtract 360° (2π) when L2 is outside the range. 
	if(lambda < (-M_PI)) lambda += 2.0*M_PI;
	if(lambda > M_PI) lambda -= 2.0*M_PI;

	alpha = atan2( tan(alpha0) , cos(sigma) );

	// Return computed values via pointers.
	*circle_azimuth_w = alpha;
	*circle_latitude_w = fi;
	*circle_longitude_w = lambda;

}


// The following function is practical implementation of Vincety's iterative solution of the inverse geodesic problem. Basic 
// solution for points that are not antipodal is almost the same as the one that can be found in "Direct and Inverse Solutions of 
// Geodesics on the Ellipsoid with application of nested equations" (Vinceny 1975a) or on 
// https://en.wikipedia.org/wiki/Vincenty%27s_formulae , only improvement is making sure that no division by 0 occurs. 
// Antipodal solution is combination of "Geodetic inverse solution between antipodal points" (Vinceny 1975b) and Geodesic Behavior 
// for Near Anti-Podal Points - General Case found in "Geometric Geodesy, Part II" (Rapp 1993).

int vincenty_inverse(double fi1, double L1, double fi2, double L2, 
              double *ellipsoid_azimuth_1, double *ellipsoid_azimuth_2, double *ellipsoid_distance) {

    double U1, U2, L, lambda, tol, diff, sin_sigma, cos_sigma, sigma, sin_alpha, cos_sq_alpha, cos_2_sigma_m;
    double A, B, C, lambda_pre, delta_sigma, s, alpha1, alpha2, U_sq;
    double D, sin_lambda, cos_lambda, sin_alpha_pre, diff_anti, tol_anti;
    double sin_alpha1, cos_alpha1;

	int i, j, antipodal_solution_required, lambda_pi_count, tolerance=TOLERANCE_BASIC;

	U1 = atan((1 - f) * tan(fi1));
	U2 = atan((1 - f) * tan(fi2));

	L = L2 - L1;

	// There is a chance that that a pair of nearly antipodal locations will have λ close to π and will trigger antipodal solution which 
	// will after some number of iterations produce NaN values of variables. To resolve this, most of the computations are performed in a
	// loop. Each successive execution of this loop has reduced degree of accuracy. This allows locations like 1°36′11″N 1°00′01″W and
	// 1°00′01″S 179°00′02″E to converge. Most pairs of locations will only require one execution.
	while(tolerance >= TOLERANCE_MINIMUM) {

		// At the start of every loop execution reset counter and flag variables.
		i=0;
		j=0;
		antipodal_solution_required=0;
		lambda_pi_count=0;

		lambda = L;

		tol = pow(10.0, -tolerance); // iteration tolerance
		diff = 1.0; // initial dummy value of variable responsible for keeping track of changes in lambda between iterations is set to 
			        // be large enough that it won't stop first execution of the while loop

		while ( diff > tol ) {

			sin_sigma = hypot( (cos(U2) * sin(lambda)) , cos(U1)*sin(U2) - sin(U1)*cos(U2)*cos(lambda) );
			cos_sigma = sin(U1) * sin(U2) + cos(U1) * cos(U2) * cos(lambda);
			sigma = atan2(sin_sigma , cos_sigma);
			sin_alpha = (cos(U1) * cos(U2) * sin(lambda)) / sin_sigma;
			cos_sq_alpha = 1.0 - pow(sin_alpha, 2.0);

			// When both points lie on the equator, set cos_2_sigma_m to the same value as the cos_sigma has, because cos_sq_alpha is then 
			// equal to 0.
			if( (fi1 == 0.0) && (fi2 == 0.0) ) cos_2_sigma_m = cos_sigma; 
			else cos_2_sigma_m = cos_sigma - ((2.0 * sin(U1) * sin(U2)) / cos_sq_alpha);

			C = (f / 16.0) * cos_sq_alpha * (4.0 + f * (4.0 - 3.0 * cos_sq_alpha));
			lambda_pre = lambda;
			lambda = L + (1.0 - C) * f * sin_alpha * 
				     (sigma + C * sin_sigma * (cos_2_sigma_m + C * cos_sigma * ( -1.0 + 2.0*pow(cos_2_sigma_m, 2.0) )));

			// Compute how much lambda has changed from its value in previous iteration. When diff is smaller than tol, iterations stop.
			diff = fabs(lambda_pre - lambda);

			// Count how many times absolute value of lambda has gotten close to the π (180°). If it happened enough times, stop the 
			// iterations and indicate that antipodal solution is required.
			if( fabs(lambda) > (M_PI-0.0027) ) {
				lambda_pi_count++;
				if(lambda_pi_count>120000) {
					antipodal_solution_required = 1;
					break;
				}
			}

			// Count how many iterations were performed in total and stop the iterations when maximum amount specified was reached.
			i++;
			if(i>=INVERSE_MAX_ITERATIONS) break;

		}

		// Calculate azimuths according to the following equations when points are not antipodal.
		if(antipodal_solution_required == 0) {

			alpha1 = atan2((cos(U2) * sin(lambda)), (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(lambda)));
			alpha2 = atan2( (cos(U1) * sin(lambda)), ( -sin(U1) * cos(U2) + cos(U1) * sin(U2) * cos(lambda) ) );

		}

		// Here is the antipodal solution. I didn't use primed L or λ from Vincenty (1975b). Those primed variables were supposed 
		// to increase calculations precision, but I couldn't make them work. Instead, regular L and λ variables with values close to 
		// π or -π are used, just like in Rapp (1993).
		else {

			// Set initial value of lambda to π if if L is positive, or to -π when L is negative.
			lambda = M_PI;
			if(L<0) lambda *= -1.0;

			cos_sq_alpha = 0.5;
			cos_2_sigma_m = 0;
			sigma = M_PI - fabs( U1 + U2 );

			sin_alpha = 1.0;

			tol_anti = pow(10.0, -tolerance); // iteration tolerance for an antipodal solution
			diff_anti = 1.0; // initial dummy value of variable responsible for keeping track of changes in sin_alpha between iterations

			j = 0;

			// Loop of "do while" type is used for the antipodal solution, witch guarantees that at 
			// least one loop execution will take place, no matter what the initial value of sin_alpha is.
			do {

				C = (f / 16.0) * cos_sq_alpha * (4.0 + f * (4.0 - 3.0 * cos_sq_alpha));
				cos_2_sigma_m = cos(sigma) - ((2.0 * sin(U1) * sin(U2)) / cos_sq_alpha);
				D = (1.0 - C) * f * ( sigma + C * sin(sigma) * ( cos_2_sigma_m + C * cos(sigma) * (-1.0+2.0*pow(cos_2_sigma_m, 2.0)) ) );
				
				sin_alpha_pre = sin_alpha;
				sin_alpha = (lambda - L) / D;
				cos_sq_alpha = 1.0 - pow(sin_alpha, 2.0);

				sin_lambda = ( sin_alpha * sin(sigma) ) / ( cos(U1) * cos(U2) );
				lambda = -asin(sin_lambda); // to compute lambda use arcsinus (value will be close to 0) 
				if(L<0) lambda -= M_PI; // and then depending on the sign of L, subtract or add π
				else lambda += M_PI;
				cos_lambda = cos(lambda);

				sin_sigma = hypot( cos(U2) * sin_lambda , cos(U1)*sin(U2) - sin(U1)*cos(U2)*cos_lambda );
				sigma = -asin(sin_sigma) + M_PI; // value of sigma should always be close to π in the antipodal case
				cos_sigma = cos(sigma);

				// Make sure that that during first execution of the loop diff_anti wont be smaller than tol_anti
				if(j>0) diff_anti = fabs(sin_alpha_pre - sin_alpha);
				else diff_anti = 1.0;

				// Count how many iterations were performed in total and stop the iterations when maximum amount specified was reached.
				// It is quite likely that maximum amount will be reached, but difference between the results here and in GeographicLib-2.1
				// shouldn't be greater than few μm.
				j++;
				if(j>=INVERSE_MAX_ITERATIONS_ANTIPODAL) break;

			} while ( (diff_anti > tol_anti) );

			sin_alpha1 = sin_alpha / cos(U1);
			cos_alpha1 = sqrt( 1.0 - pow(sin_alpha1, 2.0) );
			if( ( cos(U1)*sin(U2) + sin(U1)*cos(U2)*cos_lambda ) < 0 ) cos_alpha1 *= -1.0;
			alpha1 = atan2(sin_alpha1 , cos_alpha1);
			alpha2 = atan2(sin_alpha , -sin(U1)*sin_sigma + cos(U1)*cos_sigma*cos_alpha1);

		}

		// Check if during calculation of antipodal solution sin_alpha (and all of the other variables) did not become NaN (it happens because 
		// pair of locations should be treated by regular solution, but it couldn't handle it with regular precision). It sin_alpha was computed 
		// correctly, stop the loop.
		if( isfinite(sin_alpha) ) break;

		// If sin_alpha still did not converge, reduce desired accuracy.
		tolerance--;

	}

	// Distance is calculated with equations from "Direct and Inverse Solutions of "Direct and Inverse Solutions of Geodesics on the Ellipsoid 
	// with application of nested equations" (Vinceny 1975a) no matter what solution was used. Equations from "Geodetic inverse solution between 
	// antipodal points" (Vinceny 1975b) provide imprecise results.
	U_sq = cos_sq_alpha * ((pow(a, 2.0) - pow(b, 2.0)) / pow(b ,2.0));
	A = 1 + (U_sq / 16384.0) * (4096.0 + U_sq * (-768.0 + U_sq * (320.0 - 175.0 * U_sq)));
	B = (U_sq / 1024.0) * (256.0 + U_sq * (-128.0 + U_sq * (74.0 - 47.0 * U_sq)));
	delta_sigma = B * sin_sigma * (cos_2_sigma_m + 0.25 * B * (cos_sigma * (-1.0 + 2.0 * pow(cos_2_sigma_m, 2.0)) -
	              (1.0 / 6.0) * B * cos_2_sigma_m * (-3.0 + 4.0 * pow(sin_sigma, 2.0)) * (-3.0 + 4.0 * pow(cos_2_sigma_m, 2.0))));
	s = b * A * (sigma - delta_sigma);



	// Return computed values via pointers. Note that azimuths may be off for nearly antipodal points (basically sending you through other pole).
	// It happens mostly when function did not converge (negative return value), for example locations 1°36′08″N 1°00′03″W and 
	// 1°00′01″S  179°00′02″E.
	*ellipsoid_azimuth_1 = alpha1;
	*ellipsoid_azimuth_2 = alpha2;
	*ellipsoid_distance = s;

	// If everything converged nicely, return positive value, otherwise negative (distance still should be accurate to the tenths of millimeter).
	// Absolute value of return integer will be equal to the final tolerance exponent.
	if( (i<INVERSE_MAX_ITERATIONS) && (j<INVERSE_MAX_ITERATIONS_ANTIPODAL) ) return tolerance;
	else return -tolerance;

}

// The following function is practical implementation of Vincety's iterative solution of the direct geodesic problem. It uses 
// equations found in "Direct and Inverse Solutions of Geodesics on the Ellipsoid with application of nested equations" 
// (Vinceny 1975a) or on https://en.wikipedia.org/wiki/Vincenty%27s_formulae . Only minor adjustments are implemented.

int vincenty_direct(double fi1, double L1, double alpha1, double ellipsoid_distance_w, 
                         double *ellipsoid_azimuth_w, double *ellipsoid_latitude_w, double *ellipsoid_longitude_w) {

	double U1, sigma, sigma1, delta_sigma, sin_alpha, cos_sq_alpha, U_sq, A, B, two_sigma_m, cos_2_sigma_m;
	double diff, tol, sigma_pre;
	double fi2, lambda, C, L, L2, alpha2;

	int i=0;

	U1 = atan((1 - f) * tan(fi1));
	sigma1 = atan2( tan(U1), cos(alpha1) );
	sin_alpha = cos(U1) * sin(alpha1);
	cos_sq_alpha = 1.0 - pow(sin_alpha, 2.0);
	U_sq = cos_sq_alpha * ((pow(a, 2.0) - pow(b, 2.0)) / pow(b ,2.0));
	A = 1 + (U_sq / 16384.0) * (4096.0 + U_sq * (-768.0 + U_sq * (320.0 - 175.0 * U_sq)));
	B = (U_sq / 1024.0) * (256.0 + U_sq * (-128.0 + U_sq * (74.0 - 47.0 * U_sq)));

	sigma = ellipsoid_distance_w / (b*A);
	tol = pow(10.0, -12.0); // iteration tolerance
	diff = 1.0;

	while ( diff > tol ) {

		two_sigma_m = 2.0*sigma1 + sigma;
		cos_2_sigma_m = cos(two_sigma_m);
		
		delta_sigma = B * sin(sigma) * (cos_2_sigma_m + 0.25 * B * (cos(sigma) * (-1.0 + 2.0 * pow(cos_2_sigma_m, 2.0)) -
															(1.0 / 6.0) * B * cos_2_sigma_m * (-3.0 + 4.0 * pow(sin(sigma), 2.0)) *
															(-3.0 + 4.0 * pow(cos_2_sigma_m, 2.0))));

		sigma_pre = sigma;

		sigma = ellipsoid_distance_w / (b*A) + delta_sigma;

		diff = fabs(sigma_pre - sigma);

		i++;
		if(i>=DIRECT_MAX_ITERATIONS) break;

	}

	fi2 = atan2( sin(U1)*cos(sigma) + cos(U1)*sin(sigma)*cos(alpha1) , 
	             (1.0-f)*hypot(sin_alpha, sin(U1)*sin(sigma)-cos(U1)*cos(sigma)*cos(alpha1)) );

	lambda = atan2( sin(sigma)*sin(alpha1) , cos(U1)*cos(sigma)-sin(U1)*sin(sigma)*cos(alpha1) );
	C = (f / 16.0) * cos_sq_alpha * (4.0 + f * (4.0 - 3.0 * cos_sq_alpha));
	L = lambda - (1.0 - C) * f * sin_alpha * 
	    (sigma + C * sin(sigma) * (cos_2_sigma_m + C * cos(sigma) * ( - 1.0 + 2.0*pow(cos_2_sigma_m, 2.0) )));
	L2 = L + L1;

	// To keep longitude of resulting point in the right range add or subtract 360° (2π) when L2 is outside the range. 
	if(L2 < (-M_PI)) L2 += 2.0*M_PI;
	if(L2 > M_PI) L2 -= 2.0*M_PI;

	alpha2 = atan2(sin_alpha, -sin(U1)*sin(sigma) + cos(U1)*cos(sigma)*cos(alpha1) );

	// Return computed values via pointers.
	*ellipsoid_azimuth_w = alpha2;
	*ellipsoid_latitude_w = fi2;
	*ellipsoid_longitude_w = L2;

	// If everything converged nicely, return 1, otherwise -1.
	if( i<DIRECT_MAX_ITERATIONS ) return 1;
	else return -1;

}
