#include <stdlib.h>
#include <ncursesw/curses.h>
#include <math.h>
#include <locale.h>
#include "geodetic_solver.h"

// GeographicLib
#include "geodesic.h"

// Defines which angle format is used.
#define DMS 0
#define DECIMAL 1

// Defines type of field when regular numeric field is considered.
#define PADDED_FIELD 0
#define UNPADDED_FIELD 1

// Defines type of field when sign field is considered.
#define LATITUDE_FIELD 0
#define LONGITUDE_FIELD 1

// Maximum allowed fields in editable line.
#define ABSOLUTE_MAX_COLUMNS 8

// Defines type of field for use in print_angle() function.
#define LONG 1
#define SHORT 0
#define LONGITUDE 2
#define LATITUDE 1
#define AZIMUTH 0

// Starting values of editable fields (by default DMS angle format is used at start).
int value[2][ABSOLUTE_MAX_COLUMNS] = {
	{0,0,0,1, 0,0,0,1},
	{0,0,0,1, 0,0,0,1}
};

// Maximum allowable values of editable fields. Notice that those values are different for DMS and decimal degrees angle formats.
int max_value[2][ABSOLUTE_MAX_COLUMNS] = {
	{89,59,59,1, 179,59,59,1}, 
	{89,999999,1,0, 179,999999,1,0}
};

// Minimum allowable values of editable fields. Notice that those values are different for DMS and decimal degrees angle formats.
int min_value[2][ABSOLUTE_MAX_COLUMNS] = {
	{0,0,0,-1, 0,0,0,-1},
	{0,0,-1,0, 0,0,-1,0}
};

// Allowable characters that each field can occupy. Notice that those values are different for DMS and decimal degrees angle formats.
int max_decimal_characters[2][ABSOLUTE_MAX_COLUMNS] = {
	{2,2,2,1, 3,2,2,1},
	{2,6,1,0, 3,6,1,0}
};

// Horizontal position of each editable field. Notice that this position is different for DMS and decimal degrees angle formats.
int column_position[2][ABSOLUTE_MAX_COLUMNS] = {
	{0,3,6,9, 12,16,19,22},
	{0,3,10,0, 13,17,24,0}
};

// Chose vertical and initial horizontal position of both.
int line_position[2] = {2,5}, starting_column[2] = {2,2};

// Variables defining which field is currently editable, which save slot is currently selected, which angle format is selected globally,
// which variation of field it is and what the maximum number of columns is.
int c_line, c_field, c_save_slot=0, dms_or_decimal=0, field_type, max_columns;

// save_buffer hold a temporary copy of a save file along with save_dms_or_decimal. These arrays are used both when writing to and reading a file.
// Four save slots are used.
int save_buffer[4][2][ABSOLUTE_MAX_COLUMNS];
int save_dms_or_decimal[4] = {0,0,0,0};

// Horizontal position of each save field belonging to currently selected save sot. Notice that this position is different for DMS and 
// decimal degrees angle formats.
int column_position_s[2][ABSOLUTE_MAX_COLUMNS] = {
	{0,3,6,9, 12,16,19,22},
	{0,3,10,0, 13,17,24,0}
};

// Chose vertical and initial horizontal position of save display area being shown. 
int line_position_s[2] = {21,22}, starting_column_s[2] = {15,15};

// Those buffers allow angle formats to be changed (DMS and decimal degrees).
int input_conversion_buffer[ABSOLUTE_MAX_COLUMNS];
int output_conversion_buffer[ABSOLUTE_MAX_COLUMNS];
int save_conversion_buffer[ABSOLUTE_MAX_COLUMNS];

// This variable represents point on the "road" between first and second locations where the waypoint is located.
int waypoint_percentage;

// GeographicLib variables
double azi1, azi2, s12, lat2, lon2;
struct geod_geodesic g;

// Initialization of a window containing major portions of displayed text.
WINDOW *text;

// Filling of text window with words.
void print_text(void) {

	text = newwin(0, 0, 0, 0);

	mvwprintw(text, 1, 2, " First location:");

	mvwprintw(text, 4, 2, " Second location:");

	mvwprintw(text, 8, 2, " Decimal degrees:");

	mvwprintw(text, 12, 2, " Radians:");


	mvwprintw(text, 16, 2, "Percentage of way between");
	mvwprintw(text, 17, 2, "first and second locations");
	mvwprintw(text, 18, 2, "at which waypoint is");
	mvwprintw(text, 19, 2, "located:");

	mvwprintw(text, 21, 2, "Save slot:");

	mvwprintw(text, 1, 29, "Great-circle");
	mvwprintw(text, 1, 46, "Vincenty's");
	mvwprintw(text, 1, 63, "Newton's method");

	mvwprintw(text, 3, 28, "-- distance between two locations ---------------");
	mvwprintw(text, 5, 28, "-- azimuth at first location --------------------");
	mvwprintw(text, 7, 28, "-- azimuth at second location -------------------");

	mvwprintw(text, 10, 28, "-- distance between first location and waypoint -");
	mvwprintw(text, 12, 28, "-- azimuth at waypoint --------------------------");
	mvwprintw(text, 14, 28, "-- latitude of waypoint -------------------------");
	mvwprintw(text, 16, 28, "-- longitude of waypoint ------------------------");


}

// Check if given editable field is used by current angle format.
int determine_field_existence(int field_number) {

	int sign_existence_f;

	// DMS angle format takes 4 fields per coordinate (two of them in a row).
	if(dms_or_decimal==DMS) {

		sign_existence_f = 1;

	}

	// DECIMAL angle format is using only 3 fields per coordinate, so fields 3 and 7 are disabled. 
	else {

		if( (field_number==3) || (field_number==7) ) sign_existence_f = 0;
		else sign_existence_f = 1;
	
	}

	return sign_existence_f;

}

// Check if given field is a sign field. This varies depending on the angle format used.
int determine_sign_field_local(int field_number, int angle_format) {

	int sign_field_f;

	if(angle_format==DMS) {

		if( (field_number==3) || (field_number==7) ) sign_field_f = 1;
		else sign_field_f = 0;

	}

	else {

		if( (field_number==2) || (field_number==6) ) sign_field_f = 1;
		else sign_field_f = 0;
	
	}

	return sign_field_f;

}

// Check if given field is a sign field in currently selected angle format (dms_or_decimal).
int determine_sign_field(int field_number) {

	return determine_sign_field_local(field_number, dms_or_decimal);

}

// Check what type of field given field is. When it is a sign field only two types are possible: LATITUDE_FIELD or LONGITUDE_FIELD. 
// It means that a letter N, S, E or W is displayed. If it is not a sign field, a numerical value is being displayed of UNPADDED_FIELD or
// PADDED_FIELD kind.
int determine_field_type(int field_number) {

	int field_type_f;

	if(dms_or_decimal==DMS) {

		if( determine_sign_field(field_number) ) {

			if( field_number == 3 ) field_type_f = LATITUDE_FIELD;
			else field_type_f = LONGITUDE_FIELD;

		}

		else {

			if( (field_number==0) || (field_number==4)) field_type_f = UNPADDED_FIELD;
			else field_type_f = PADDED_FIELD;

		}

	}

	else {

		if( determine_sign_field(field_number) ) {

			if( field_number == 2 ) field_type_f = LATITUDE_FIELD;
			else field_type_f = LONGITUDE_FIELD;

		}

		else {

			if( (field_number==0) || (field_number==4)) field_type_f = UNPADDED_FIELD;
			else field_type_f = PADDED_FIELD;

		}
	
	}

	return field_type_f;

}

// Display given angle as AZIMUTH, LATITUDE or LONGITUDE in chosen location (row and column). angle is in decimal degrees and length of displayed value can vary.
// Considering the length, notice that only AZIMUTH and LONGITUDE require LONG length (because LATITUDE is at max 90 degrees [2 characters], while
// AZIMUTH and LONGITUDE max at 180 [3 characters]). In the LATITUDE case LONG length can be used to help with horizontal placement of displayed angle (so that 
// different angles placed one bellow the other are aligned).
void print_angle(int row, int column, double angle, int length, int format) {

	int D, M;
	double S;

	// If DMS angle format is currently being selected, convert angle to this format.
	if(dms_or_decimal==DMS) {

		D = angle;
		M = 60.0*fabs(angle-D);
		S = 3600.0*fabs(angle-D) - 60.0*M;

		if(S>59.999999) {
			S = 0.0;
			M++;

			if(M>=60) {
				M = 0;
				if(angle<0.0) D--;
				else D++;
			}

		}

		if(length == LONG) {

			if(format == AZIMUTH) {

				mvprintw(row, column, "%4d°%02d′%05.2f″", D, M, S);

			}

			else {

				mvprintw(row, column, "%3d°%02d′%05.2f″", abs(D), M, S);

				if(format == LATITUDE) {

					if( angle < 0.0 ) mvprintw(row, column+13, "S");
					else mvprintw(row, column+13, "N");

				}

				else { // LONGITUDE

					if( angle < 0.0 ) mvprintw(row, column+13, "W");
					else mvprintw(row, column+13, "E");

				}

			}

		}

		else {

			if(format == AZIMUTH) {

				mvprintw(row, column, "%3d°%02d′%05.2f″", D, M, S);

			}

			else {

				mvprintw(row, column, "%2d°%02d′%05.2f″", abs(D), M, S);

				if(format == LATITUDE) {

					if( angle < 0.0 ) mvprintw(row, column+12, "S");
					else mvprintw(row, column+12, "N");

				}

				else { // LONGITUDE

					if( angle < 0.0 ) mvprintw(row, column+12, "W");
					else mvprintw(row, column+12, "E");

				}

			}

		}

	}

	else {

		if(length == LONG) {

			if(format == AZIMUTH) {

				mvprintw(row, column, "%11.6f°", angle);

			}

			else {

				mvprintw(row, column, "%10.6f°", fabs(angle));

				if(format == LATITUDE) {

					if( angle < 0.0 ) mvprintw(row, column+11, "S");
					else mvprintw(row, column+11, "N");

				}

				else { // LONGITUDE

					if( angle < 0.0 ) mvprintw(row, column+11, "W");
					else mvprintw(row, column+11, "E");

				}

			}

		}

		else {

			if(format == AZIMUTH) {

				mvprintw(row, column, "%10.6f°", angle);

			}

			else {

				mvprintw(row, column, "%9.6f°", fabs(angle));

				if(format == LATITUDE) {

					if( angle < 0.0 ) mvprintw(row, column+10, "S");
					else mvprintw(row, column+10, "N");

				}

				else { // LONGITUDE

					if( angle < 0.0 ) mvprintw(row, column+10, "W");
					else mvprintw(row, column+10, "E");

				}

			}

		}

	}

}

// Display current values of editable fields. 
void display_values(void) {

	// Display associated °  ′  ″ . characters. Note that those characters and they placement depends on current global angle format (dms_or_decimal).
	if(dms_or_decimal==DMS) {
		mvprintw(line_position[0], starting_column[0], "  °  ′  ″      °  ′  ″");
		mvprintw(line_position[1], starting_column[1], "  °  ′  ″      °  ′  ″");
	}

	else {
		mvprintw(line_position[0], starting_column[0], "  .      °      .      ° ");
		mvprintw(line_position[1], starting_column[1], "  .      °      .      ° ");
	}

	// Variables representing currently processed field.
	int row, column;

	// All editable fields will be in bold.
	attron(A_BOLD);

	// Go through 2 rows.
	for(row=0; row<2; row++) {

		// Made up of 8 columns each (ABSOLUTE_MAX_COLUMNS).
		for(column=0; column<ABSOLUTE_MAX_COLUMNS; column++) {

			// When decimal degrees global angle format is selected, 2 of the 8 columns are inactive and those fields are not processed.
			if( determine_field_existence(column) ) {

				// If currently processed field is selected by the user, make it appear in reverse video.
				if( (row == c_line) && (column == c_field) ) attron(A_REVERSE);

				// Check for sign field. Those fields take value of -1 or 1 and represent South versus North, West versus East
				if( determine_sign_field(column) ) {

					if( determine_field_type(column) == LATITUDE_FIELD ) {

						if( value[row][column] == 1 ) mvprintw(line_position[row], column_position[dms_or_decimal][column]+starting_column[row], "N");
						else mvprintw(line_position[row], column_position[dms_or_decimal][column]+starting_column[row], "S");

					}

					else {

						if( value[row][column] == 1 ) mvprintw(line_position[row], column_position[dms_or_decimal][column]+starting_column[row], "E");
						else mvprintw(line_position[row], column_position[dms_or_decimal][column]+starting_column[row], "W");

					}

				}

				// When given field is not a sign field, it means that it is a numerical value that can be unpadded in case of leftmost field in a location 
				// (2 locations per row) or padded for each other numerical field.
				else {

					if( determine_field_type(column) == UNPADDED_FIELD ) {
						mvprintw(line_position[row], column_position[dms_or_decimal][column]+starting_column[row], 
							"%*d", max_decimal_characters[dms_or_decimal][column], value[row][column]);
					}

					else {
						mvprintw(line_position[row], column_position[dms_or_decimal][column]+starting_column[row], 
							"%0*d", max_decimal_characters[dms_or_decimal][column], value[row][column]);
					}

				}

				// Turn off reverse video (even if it weren't enabled).
				attroff(A_REVERSE);

			}

		}

	}

	// Disable bold font
	attroff(A_BOLD);

}

// Display everything in the terminal. 
void draw(void) {

	// Merge two windows that were filled with characters.
	overlay(text, stdscr);

	// Copy merged windows to the virtual screen.
	wnoutrefresh(stdscr);

	// Update physical screen with the changes to the virtual screen,
	doupdate();

}


// Here you can find all calls to geodesic functions located in different files (geodetic_solver.c and geodesic.c). Most of the data handling is done here. 
// Final result find their way to the screen here too. 
void compute(void) {

	// Variables responsible for initial data processing.
	double degrees, minutes, seconds, first_latitude, first_longitude, second_latitude, second_longitude;    
	double first_latitude_deg, first_longitude_deg, second_latitude_deg, second_longitude_deg;

	// Variables dealing with great-circle distance.
	double circle_azimuth_1, circle_azimuth_2, circle_distance, circle_azimuth_1_deg, circle_azimuth_2_deg;
	double circle_distance_w, circle_azimuth_w, circle_latitude_w, circle_longitude_w;

	// Variables dealing with geodesics on an ellipsoid (Vincenty's formulae).
	double ellipsoid_azimuth_1, ellipsoid_azimuth_2, ellipsoid_distance, ellipsoid_azimuth_1_deg, ellipsoid_azimuth_2_deg;
	double ellipsoid_distance_w, ellipsoid_azimuth_w, ellipsoid_latitude_w, ellipsoid_longitude_w;

	// Vincenty's formulae can fail some times (nearly antipodal points). That's why there are status variables.
	int vincenty_inverse_status, vincenty_direct_status;

	// Constants used to convert between angle formats.
	double rad = M_PI/180.0, deg = 180.0/M_PI;

	// Convert locations that for now have taken 3 or 4 editable fields into single decimal degrees variables. 
	if(dms_or_decimal==DMS) {

		// DMS

		degrees = value[0][0];
		minutes = value[0][1];
		seconds = value[0][2];
		first_latitude_deg = (degrees + minutes/60.0 + seconds/3600.0) * value[0][3];

		degrees = value[0][4];
		minutes = value[0][5];
		seconds = value[0][6];
		first_longitude_deg = (degrees + minutes/60.0 + seconds/3600.0) * value[0][7];

		degrees = value[1][0];
		minutes = value[1][1];
		seconds = value[1][2];
		second_latitude_deg = (degrees + minutes/60.0 + seconds/3600.0) * value[1][3];

		degrees = value[1][4];
		minutes = value[1][5];
		seconds = value[1][6];
		second_longitude_deg = (degrees + minutes/60.0 + seconds/3600.0) * value[1][7];

	}

	else {

		// Decimal degrees

		first_latitude_deg = ( value[0][0] + value[0][1]/1000000.0 ) * value[0][2];

		first_longitude_deg = ( value[0][4] + value[0][5]/1000000.0 ) * value[0][6];

		second_latitude_deg = ( value[1][0] + value[1][1]/1000000.0 ) * value[1][2];

		second_longitude_deg = ( value[1][4] + value[1][5]/1000000.0 ) * value[1][6];

	}

	// Show on the screen decimal degree representations of locations.
	mvprintw(9, 2, "%10.6f  %11.6f", first_latitude_deg, first_longitude_deg);
	mvprintw(10, 2, "%10.6f  %11.6f", second_latitude_deg, second_longitude_deg);

	// Conversion to radians
	first_latitude = first_latitude_deg * rad;
	first_longitude = first_longitude_deg * rad;
	second_latitude = second_latitude_deg * rad;
	second_longitude = second_longitude_deg * rad;

	// Show on the screen radian representations of locations.
	mvprintw(13, 2, "%10.7f  %11.8f", first_latitude, first_longitude);
	mvprintw(14, 2, "%10.7f  %11.8f", second_latitude, second_longitude);

	// Show waypoint percentage (percentage of the way between first and second location) in blinking bold.
	attron(A_BLINK|A_BOLD);
	mvprintw(19, 11, "%3d", waypoint_percentage);
	attroff(A_BLINK|A_BOLD);
	mvprintw(19, 15, "%%");

	// Spherical version of the inverse geodetic problem. This function takes coordinates of the two locations in radians and returns two azimuths 
	// (one for each of the two initial locations) and distance between locations.
	great_circle_inverse(first_latitude, first_longitude, second_latitude, second_longitude, &circle_azimuth_1, &circle_azimuth_2, &circle_distance);

	// Convert resulting radian azimuths into degrees.
	circle_azimuth_1_deg = circle_azimuth_1 * deg;
	circle_azimuth_2_deg = circle_azimuth_2 * deg;

	// Show great-circle distance between two locations in bold and follow it by km measurment unit.
	attron(A_BOLD);
	mvprintw(4, 28, "%12.6f", circle_distance/1000.0);
	attroff(A_BOLD);
	mvprintw(4, 41, "km");

	// Show azimuths.
	print_angle(6, 29, circle_azimuth_1_deg, LONG, AZIMUTH);
	print_angle(8, 29, circle_azimuth_2_deg, LONG, AZIMUTH);

	// Calculate distance from the first location to the waypoint.
	circle_distance_w = waypoint_percentage * circle_distance/100.0;

	// Spherical version of the direct geodetic problem. Calculate waypoint's azimuth and coordinates.
	great_circle_direct(first_latitude, first_longitude, circle_azimuth_1, circle_distance_w, 
	                         &circle_azimuth_w, &circle_latitude_w, &circle_longitude_w);

	// Display distance between first location and the waypoint in km.
	mvprintw(11, 28, "%12.6f km", circle_distance_w/1000.0);

	// Show great-circle waypoint's azimuth and coordinates. 
	print_angle(13, 29, circle_azimuth_w*deg, LONG, AZIMUTH);
	print_angle(15, 29, circle_latitude_w*deg, LONG, LATITUDE);
	print_angle(17, 29, circle_longitude_w*deg, LONG, LONGITUDE);

	// If both locations are the same, then set all of the variables that are supposed to be resulting from the Vincenty's formulae to 0. Otherwise
	// call for a function responsible for processing Vincenty's formulae.
	if( (first_latitude == second_latitude) && (first_longitude == second_longitude) ) {
		vincenty_inverse_status = 0;
		ellipsoid_azimuth_1 = 0;
		ellipsoid_azimuth_2 = 0;
		ellipsoid_distance = 0;
	}

	// Ellipsoid version of the inverse geodetic problem. This function takes coordinates of the two locations in radians and returns two azimuths 
	// (one for each of the two initial locations) and distance between locations. In the case of some nearly  antipodal locations returned azimuths 
	// will be wrong. Returned distance should always be right.
	else vincenty_inverse_status = vincenty_inverse(first_latitude, first_longitude, second_latitude, second_longitude, &ellipsoid_azimuth_1, 
	                                                &ellipsoid_azimuth_2, &ellipsoid_distance);

	// Convert resulting radian azimuths into degrees.
	ellipsoid_azimuth_1_deg = ellipsoid_azimuth_1 * deg;
	ellipsoid_azimuth_2_deg = ellipsoid_azimuth_2 * deg;

	// Show ellipsoid distance between two locations in bold and follow it by km measurement unit.
	attron(A_BOLD);
	mvprintw(4, 45, "%12.6f", ellipsoid_distance/1000.0);
	attroff(A_BOLD);
	mvprintw(4, 58, "km");

	// Show azimuths.
	print_angle(6, 46, ellipsoid_azimuth_1_deg, LONG, AZIMUTH);
	print_angle(8, 46, ellipsoid_azimuth_2_deg, LONG, AZIMUTH);

	// In the case of Vincenty's formulae failure, inform user about it.
	if(vincenty_inverse_status<0) mvprintw(19, 32, "Vincety's formulae did not converge properly!");

	// Calculate distance from the first location to the waypoint.
	ellipsoid_distance_w = waypoint_percentage * ellipsoid_distance/100.0;

	// Ellipsoid version of the direct geodetic problem. Calculate waypoint's azimuth and coordinates.
	vincenty_direct_status = vincenty_direct(first_latitude, first_longitude, ellipsoid_azimuth_1, ellipsoid_distance_w, 
	                                         &ellipsoid_azimuth_w, &ellipsoid_latitude_w, &ellipsoid_longitude_w);

	// Display distance between first location and the waypoint in km.
	mvprintw(11, 45, "%12.6f km", ellipsoid_distance_w/1000.0);

	// Show ellipsoid waypoint's azimuth and coordinates. 
	print_angle(13, 46, ellipsoid_azimuth_w*deg, LONG, AZIMUTH);
	print_angle(15, 46, ellipsoid_latitude_w*deg, LONG, LATITUDE);
	print_angle(17, 46, ellipsoid_longitude_w*deg, LONG, LONGITUDE);

	// If failure of Vincenty's formulae direct method occurs (it shouldn't ever happen), inform user about it.
	if(vincenty_direct_status<0) mvprintw(20, 32, "Vincety's direct solution failed!");

	// GeographicLib is used here to calculate inverse geodesic problem on ellipsoid. 
	geod_inverse(&g, first_latitude_deg, first_longitude_deg, second_latitude_deg, second_longitude_deg, &s12, &azi1, &azi2);

	// Show ellipsoid distance between two locations in bold and follow it by km measurement unit.
	attron(A_BOLD);
	mvprintw(4, 62, "%12.6f", s12/1000.0);
	attroff(A_BOLD);
	mvprintw(4, 75, "km");

	// Show azimuths.
	print_angle(6, 63, azi1, LONG, AZIMUTH);
	print_angle(8, 63, azi2, LONG, AZIMUTH);

	// Calculate distance from the first location to the waypoint.
	s12 = waypoint_percentage * s12/100.0;

	// Ellipsoid version of the direct geodetic problem (Newton's method). Calculate waypoint's azimuth and coordinates using GeographicLib.
	geod_direct(&g, first_latitude_deg, first_longitude_deg, azi1, s12, &lat2, &lon2, &azi2);

	// Display distance between first location and the waypoint in km.
	mvprintw(11, 62, "%12.6f km", s12/1000.0);

	// Show ellipsoid waypoint's azimuth and coordinates. 
	print_angle(13, 63, azi2, LONG, AZIMUTH);
	print_angle(15, 63, lat2, LONG, LATITUDE);
	print_angle(17, 63, lon2, LONG, LONGITUDE);

}

// Convert decimal degrees from input_conversion_buffer into DMS located in output_conversion_buffer.
void convert_to_dms(void) {

	int i;

	// There is a pair of latitude and longitude coordinates in one row, so this portion of code has to be executed twice.
	for(i=0; i<2; i++) {

		// Conversion of decimal numbers with 6 digits after a separator and taking 3 fields into 4-field DMS witch 2 digits for the minutes and seconds each.
		output_conversion_buffer[0+4*i] = input_conversion_buffer[0+4*i];
		output_conversion_buffer[1+4*i] = 60.0 * input_conversion_buffer[1+4*i]/1000000.0;
		output_conversion_buffer[2+4*i] = round( 3600.0 * input_conversion_buffer[1+4*i]/1000000.0 - 60.0 * output_conversion_buffer[1+4*i]);
		output_conversion_buffer[3+4*i] = input_conversion_buffer[2+4*i];

		// Make sure, that minutes and seconds stay within their bounds.
		if(output_conversion_buffer[2+4*i]>59) {
			output_conversion_buffer[2+4*i] = 0;
			output_conversion_buffer[1+4*i]++;

			if(output_conversion_buffer[1+4*i]>=60) {
				output_conversion_buffer[1+4*i] = 0;
				output_conversion_buffer[0+4*i]++;
			}

		}

	}

}

// Convert DMS from input_conversion_buffer into decimal degrees located in output_conversion_buffer.
void convert_to_decimal(void) {

	int i;

	// There is a pair of latitude and longitude coordinates in one row, so this portion of code has to be executed twice.
	for(i=0; i<2; i++) {

		// Conversion of 4-field DMS witch 2 digits for the minutes and seconds each, into decimal number with 6 digits after a separator and taking 3 fields.
		output_conversion_buffer[0+4*i] = input_conversion_buffer[0+4*i];
		output_conversion_buffer[1+4*i] = round( 1000000.0*( input_conversion_buffer[1+4*i]/60.0 + input_conversion_buffer[2+4*i]/3600.0 ) );
		output_conversion_buffer[2+4*i] = input_conversion_buffer[3+4*i];
		output_conversion_buffer[3+4*i] = 0;

	}

}

// Fill save_buffer with the content of saved_locations.txt
void read_file(void) {

	// i - save slot number; j - row number; k - column number;
	int i, j, k;

	FILE *fp; //file pointer 
	fp = fopen("saved_locations.txt", "r");

	// Check if file was opened properly
	if(fp != NULL) {

		for(i=0; i<4; i++) {

			// save_dms_or_decimal keeps track of angle format of all save slots.
			fscanf(fp, "%d ", &save_dms_or_decimal[i]);

			// Make sure that only important bit of angle format remains.
			save_dms_or_decimal[i] &= 0x01;

			for(j=0; j<2; j++) {

				for(k=0; k<ABSOLUTE_MAX_COLUMNS; k++) {

					fscanf(fp, "%d ", &save_buffer[i][j][k]);

					// Make sure that values read from the file stay within their bounds
					if( save_buffer[i][j][k] < min_value[save_dms_or_decimal[i]][k] )save_buffer[i][j][k] = min_value[save_dms_or_decimal[i]][k];
					if( save_buffer[i][j][k] > max_value[save_dms_or_decimal[i]][k] )save_buffer[i][j][k] = max_value[save_dms_or_decimal[i]][k];

					if( determine_sign_field_local(k, save_dms_or_decimal[i]) && (save_buffer[i][j][k]==0) ) save_buffer[i][j][k] = 1;

				}

			}

		}

		fclose(fp);

	}

	// If file was not opened properly, fill save_buffer with default values.
	else {


		for(i=0; i<4; i++) {

			save_dms_or_decimal[i] = 0;

			for(j=0; j<2; j++) {

				for(k=0; k<ABSOLUTE_MAX_COLUMNS; k++) {

					if( determine_sign_field_local(k, save_dms_or_decimal[i]) ) save_buffer[i][j][k] = 1;
					else save_buffer[i][j][k] = 0;

				}

			}

		}

	}

}

// Fill saved_locations.txt with the content of save_buffer.
void write_file(void) {

	// i - save slot number; j - row number; k - column number;
	int i, j, k;

	FILE *fp; //file pointer 
	fp = fopen("saved_locations.txt", "w");

	for(i=0; i<4; i++) {

		// save_dms_or_decimal keeps track of angle format of all save slots.
		fprintf(fp, "%d \n", save_dms_or_decimal[i]);

		for(j=0; j<2; j++) {

			for(k=0; k<ABSOLUTE_MAX_COLUMNS; k++) {
				fprintf(fp, "%d ", save_buffer[i][j][k]);
			}

			fprintf(fp, "\n");

		}

		fprintf(fp, "\n");

	}

	fclose(fp);


}

// Fill value array with contents of currently selected save slot.
void read_buffer(void) {

	// i - column number; j - row number;
	int i, j;

	// if save slot angle format and global dms_or_decimal are the same just perform straight coping of values
	if( save_dms_or_decimal[c_save_slot] == dms_or_decimal ) {

		for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
			value[0][i] = save_buffer[c_save_slot][0][i];
			value[1][i] = save_buffer[c_save_slot][1][i];
		}

	}

	// If angle formats are different, use input_conversion_buffer, conversion functions and output_conversion_buffer to change angle format of save slot.
	else {

		for(j=0; j<2; j++) {

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				input_conversion_buffer[i] = save_buffer[c_save_slot][j][i];
			}

			if(dms_or_decimal==DMS) convert_to_dms();
			else convert_to_decimal();

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				value[j][i] = output_conversion_buffer[i];
			}

		}

	}

}

// Fill currently selected save slot with contents of value array.
void write_buffer(void) {

	// i - column number;
	int i;

	save_dms_or_decimal[c_save_slot] = dms_or_decimal;

	for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
		save_buffer[c_save_slot][0][i] = value[0][i];
		save_buffer[c_save_slot][1][i] = value[1][i];
	}

}

// Show contents of current save slot.
void display_save_slot(void) {

	int i;

	// Move cursor to the position of a current save slot indicators.
	move(22, 2);

	// Four F1, F2, F3 and F4 keys are used to select save slot. Because of that, current save slot indicators consist of F1-F4 keys representations with the
	// current slot being shown in bold and underline.
	for(i=0; i<4; i++) {

		if( i == c_save_slot ) attron( A_BOLD | A_UNDERLINE );

		printw("F%d", i+1);

		attroff( A_BOLD | A_UNDERLINE );

		printw(" ");

	}

	// Display associated °  ′  ″ . characters. Note that those characters and they placement depends on current global angle format (dms_or_decimal).
	if(dms_or_decimal==DMS) {
		mvprintw(line_position_s[0], starting_column_s[0], "  °  ′  ″      °  ′  ″");
		mvprintw(line_position_s[1], starting_column_s[1], "  °  ′  ″      °  ′  ″");
	}

	else {
		mvprintw(line_position_s[0], starting_column_s[0], "  .      °      .      ° ");
		mvprintw(line_position_s[1], starting_column_s[1], "  .      °      .      ° ");
	}

	// Variables representing currently processed field.
	int row, column;

	// Go through 2 rows.
	for(row=0; row<2; row++) {

		// if save slot angle format and global dms_or_decimal are the same just perform straight coping of values
		if( save_dms_or_decimal[c_save_slot] == dms_or_decimal ) {

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				save_conversion_buffer[i] = save_buffer[c_save_slot][row][i];
			}

		}

		// If angle formats are different, use input_conversion_buffer, conversion functions and output_conversion_buffer to change angle format of save slot.
		else {

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				input_conversion_buffer[i] = save_buffer[c_save_slot][row][i];
			}

			if(dms_or_decimal==DMS) convert_to_dms();
			else convert_to_decimal();

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				save_conversion_buffer[i] = output_conversion_buffer[i];
			}

		}

		// Each row is made up of 8 columns (ABSOLUTE_MAX_COLUMNS).
		for(column=0; column<ABSOLUTE_MAX_COLUMNS; column++) {

			// When decimal degrees global angle format is selected, 2 of the 8 columns are inactive and those fields are not processed.
			if( determine_field_existence(column) ) {

				// Check for sign field. Those fields take value of -1 or 1 and represent South versus North, West versus East
				if( determine_sign_field(column) ) {

					if( determine_field_type(column) == LATITUDE_FIELD ) {

						if( save_conversion_buffer[column] == 1 ) mvprintw(line_position_s[row], column_position_s[dms_or_decimal][column]+starting_column_s[row], "N");
						else mvprintw(line_position_s[row], column_position_s[dms_or_decimal][column]+starting_column_s[row], "S");

					}

					else {

						if( save_conversion_buffer[column] == 1 ) mvprintw(line_position_s[row], column_position_s[dms_or_decimal][column]+starting_column_s[row], "E");
						else mvprintw(line_position_s[row], column_position_s[dms_or_decimal][column]+starting_column_s[row], "W");

					}

				}

				// When given field is not a sign field, it means that it is a numerical value that can be unpadded in case of leftmost field in a location 
				// (2 locations per row) or padded for each other numerical field.
				else {

					if( determine_field_type(column) == UNPADDED_FIELD ) {
						mvprintw(line_position_s[row], column_position_s[dms_or_decimal][column]+starting_column_s[row], 
							"%*d", max_decimal_characters[dms_or_decimal][column], save_conversion_buffer[column]);
					}

					else {
						mvprintw(line_position_s[row], column_position_s[dms_or_decimal][column]+starting_column_s[row], 
							"%0*d", max_decimal_characters[dms_or_decimal][column], save_conversion_buffer[column]);
					}

				}

			}

		}

	}

}


// main function consists mostly of calls to other functions and reactions to various input key presses.
int main() {	

	// ch variable - last key pressed; i - number of columns usually; j - number of rows usually
 	int ch, i, j;
	// buffer array is used to hold string of digits that is base 10 representation of current value. It is used when keys 0-9 are being pressed.
	char buffer[8];

	// set the current locale to that which supports characters ° ′ ″ 
	setlocale(LC_ALL, "C.UTF-8");

	initscr(); // Start curses mode
	raw(); // Line buffering disabled
	keypad(stdscr, TRUE); // We get F1, F2 etc..
	noecho(); // Don't echo() while we do getch
	curs_set(FALSE); // Don't show cursor

	// Initialize text window containing majority of descriptions.
	print_text();

	// Read file containing 4 saved locations pairs for the first time.
	read_file();

	// GeographicLib, geod_geodesic object initialization (ellipsoid parameters).
	double a = 6378137, f = 1/298.257223563; // WGS84
	geod_init(&g, a, f);

	// Perform majority of repeating computations inside this do...while loop.
	do {

		// Show contents of current save slot.
		display_save_slot();

		// Perform majority of calculations and display their results inside this function.
		compute();

		// Show contents of editable fields.
		display_values();

		// Update screen.
		draw();

		// Wait for new key to be pressed and store information concerning which key it was.
		ch = getch();

		if( (ch=='q') || (ch==27) ) break; //Press q or Esc to quit.

		// If KEY_RIGHT was pressed advance active column (field) to the right. Perform advancing and checking for the the advancement 
		// beyond final right column inside do...while loop so that non-existent fields can be omitted.
		if(ch==KEY_RIGHT) {
			do {
				c_field++;
				if(c_field >= ABSOLUTE_MAX_COLUMNS) c_field = 0;
			} while ( !determine_field_existence(c_field) );
		}

		// If KEY_LEFT was pressed advance active column (field) to the left. Perform advancing and checking for the the advancement 
		// beyond final left column inside do...while loop so that non-existent fields can be omitted.
		else if(ch==KEY_LEFT) {
			do {
				c_field--;
				if(c_field<0) c_field = ABSOLUTE_MAX_COLUMNS-1;
			} while ( !determine_field_existence(c_field) );
		}

		// If Tab, Space, PGUp or PgDn was pressed, change current line so that you edit the same part of coordinate, only for the second location.
		else if( (ch=='\t') || (ch==' ') || (ch==KEY_NPAGE) || (ch==KEY_PPAGE)) {
			c_line ^= 0x01;
		}

		// When Enter key is pressed, advance active field to the right. If advancement beyond final right column occurred, change the line and go to 
		// leftmost field. Perform advancing and checking for the the advancement beyond final right column inside do...while loop so that non-existent 
		// fields can be omitted.
		else if(ch=='\n') {
			do {
				c_field++;
				if(c_field>=ABSOLUTE_MAX_COLUMNS) {
					c_line ^= 0x01;
					c_field =0;
				}
			} while ( !determine_field_existence(c_field) );
		}

		// If KEY_UP is pressed flip the value of sign field, or increment regular numerical field.
		else if(ch==KEY_UP) {
			if( determine_sign_field(c_field) ) {
				value[c_line][c_field] *= -1;
			}
			else {
				value[c_line][c_field]++;
			}
		}

		// If KEY_DOWN is pressed flip the value of sign field, or decrement regular numerical field.
		else if(ch==KEY_DOWN) {
			if( determine_sign_field(c_field) ) {
				value[c_line][c_field] *= -1;
			}
			else {
				value[c_line][c_field]--;
			}
		}

		// Key 0-9 is being pressed. Pressing those keys makes corresponding digit to appear in the rightmost place of the current field. 
		// Rest of the characters are moved to the left.
		else if( (ch>='0') && (ch<='9') ) {
			// If currently selected field is a sign field, just flip its value (it can be -1 or 1) 
			if( determine_sign_field(c_field) ) {
				value[c_line][c_field] *= -1;
			}
			// If it is a numeric field, fill buffer with NULL characters (\0), then print into buffer string of digits being base 10 representation of current field 
			// value. Next, move contents of an array by 1 place. Later copy ch variable (last key pressed) into empty place in buffer array. Then convert 
			// string of digits into regular value of int type ( atoi() ).
			else {
				for(i=0; i<7; i++) buffer[i] = '\0';
				snprintf(buffer, max_decimal_characters[dms_or_decimal][c_field]+1, 
					"%*d", max_decimal_characters[dms_or_decimal][c_field], value[c_line][c_field]);
				for(i=0; i<7; i++) buffer[i] = buffer[i+1];
				buffer[max_decimal_characters[dms_or_decimal][c_field]-1] = ch;
				value[c_line][c_field] = atoi(buffer);
			}
		}

		// If key Backspace is being pressed, clear current field.
		else if(ch==KEY_BACKSPACE) {
			value[c_line][c_field] = 0;
		}

		// If key Delete is being pressed, clear all fields.
		else if(ch==KEY_DC) {
			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				if( determine_sign_field(i) ) {
					value[0][i] = 1;
					value[1][i] = 1;
				}
				else {
					value[0][i] = 0;
					value[1][i] = 0;
				}
			}
		}

		// If F1-F4 key is being pressed, change save slot and follow it by reading save file.
		else if( (ch>=KEY_F(1)) && (ch<=KEY_F(4)) ) {
			c_save_slot = ch - KEY_F(1);
			read_file();
		}

		// Press L key to read save file once again, and then copy current save slot to the working memory with editable fields.
		else if(ch=='l') {
			read_file();
			read_buffer();
		}

		// Press S key to copy working memory to the save buffer and write to the file at last.
		else if(ch=='s') {
			write_buffer();
			write_file();
		}

		// If D or Home key is pressed change global angle format. Conversion to the other angle format takes place here.
		else if( (ch=='d') || (ch==KEY_HOME) ) {

			dms_or_decimal ^= 0x01;

			for(j=0; j<2; j++) {

				for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
					input_conversion_buffer[i] = value[j][i];
				}

				if(dms_or_decimal==DMS) convert_to_dms();
				else convert_to_decimal();

				for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
					value[j][i] = output_conversion_buffer[i];
				}

			}

		}

		// Press E key to exchange locations, so that second location becomes the first one, and vice versa.
		else if(ch=='e') {

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				input_conversion_buffer[i] = value[1][i];
			}

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				value[1][i] = value[0][i];
			}

			for(i=0; i<ABSOLUTE_MAX_COLUMNS; i++) {
				value[0][i] = input_conversion_buffer[i];
			}

		}

		// Advance waypoint towards second location.
		else if(ch=='+') {
			waypoint_percentage++;
			if(waypoint_percentage>100) waypoint_percentage=0;
		}

		// Advance waypoint towards first location.
		else if(ch=='-') {
			waypoint_percentage--;
			if(waypoint_percentage<0) waypoint_percentage=100;
		}

		// Check if values of editable fields did not exceed their boundaries.
		if(value[c_line][c_field]>max_value[dms_or_decimal][c_field]) value[c_line][c_field]=min_value[dms_or_decimal][c_field];
		if(value[c_line][c_field]<min_value[dms_or_decimal][c_field]) value[c_line][c_field]=max_value[dms_or_decimal][c_field];

		// Clear the screen.
		erase();

	} while(1);

	endwin();			// End curses mode
	return 0;

}
