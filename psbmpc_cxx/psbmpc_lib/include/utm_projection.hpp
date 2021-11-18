/****************************************************************************************
*
*  File name : utm_projection.hpp
*
*  Function  : Header file for the geographic utm projection class, which represents
* 			   arbitrary ellipsoid parametrizations of the earth, and can perform
*  			   coordinate transformations between lla and UTM (northing easting).
*			   Basically a rewriting of 
* https://geographiclib.sourceforge.io/html/classGeographicLib_1_1TransverseMercator.html
*	           ---------------------
*
*  Version 1.0
*
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#pragma once

#include <string>
#include <GeographicLib/TransverseMercator.hpp>

namespace PSBMPC_LIB
{
	// Define a UTM projection for an arbitrary ellipsoid
	class UTM_Projection 
	{
	private:
		GeographicLib::TransverseMercator tm;         	// The projection
		double lon0;                                  	// Central longitude
		double false_easting, false_northing;			// Offsets to ensure positive easting, northing values
	public:
		/****************************************************************************************
		*  Name     : UTM_Projection
		*  Function : Constructor, initializes parameters and variables
		*  Author   : 
		*  Modified :
		*****************************************************************************************/
		UTM_Projection() : 
			tm(6378137.0, 0.003352810664747, GeographicLib::Constants::UTM_k0()), 
			lon0(3.0), 
			false_easting(5e5), 
			false_northing(100e5) {}

		UTM_Projection(
			double a,              	// In: Equatorial radius
			double f,              	// In: Flattening factor
			int zone,              	// In: The UTM zone
			bool northp				// In: Boolean determining if it is the northern or southern hemisphere
			)
			: tm(a, f, GeographicLib::Constants::UTM_k0())
			, lon0(6.0 * (double)zone - 183.0)
			, false_easting(5e5)
			, false_northing(northp ? 0 : 100e5) 
		{
			if (!(zone >= 1 && zone <= 60))
			throw GeographicLib::GeographicErr("zone not in [1,60]");
		}

		/****************************************************************************************
		*  Name     : forward
		*  Function : Calculates easting (x) and northing (y) from latitude and longitude
		*  Author   : 
		*  Modified :
		*****************************************************************************************/
		void forward(double &x, double &y, double lat, double lon) 
		{
			tm.Forward(lon0, lat, lon, x, y);
			x += false_easting;
			y += false_northing;
		}

		/****************************************************************************************
		*  Name     : reverse
		*  Function : Calculates latitude and longitude from easting (x) and northing (y)
		*  Author   : 
		*  Modified :
		*****************************************************************************************/
		void reverse(double &lat, double &lon, double x, double y) 
		{
			x -= false_easting;
			y -= false_northing;
			tm.Reverse(lon0, x, y, lat, lon);
		}
	};
}