#pragma once

#include <GeographicLib/TransverseMercator.hpp>
#include <string>

namespace PSBMPC_LIB {
// Define a UTM projection for an arbitrary ellipsoid
class UTM_Projection {
private:
  GeographicLib::TransverseMercator tm; // The projection
  double lon0;                          // Central longitude
  double false_easting,
      false_northing; // Offsets to ensure positive easting, northing values
public:
  /****************************************************************************************
   *  Name     : UTM_Projection
   *  Function : Constructor, initializes parameters and variables
   *  Author   :
   *  Modified :
   *****************************************************************************************/
  UTM_Projection()
      : tm(6378137.0, 0.003352810664747, GeographicLib::Constants::UTM_k0()),
        lon0(3.0), false_easting(5e5), false_northing(100e5) {}

  UTM_Projection(double a,   // In: Equatorial radius
                 double f,   // In: Flattening factor
                 int zone,   // In: The UTM zone
                 bool northp // In: Boolean determining if it is the northern or
                             // southern hemisphere
                 )
      : tm(a, f, GeographicLib::Constants::UTM_k0()),
        lon0(6.0 * (double)zone - 183.0), false_easting(5e5),
        false_northing(northp ? 0 : 100e5) {
    if (!(zone >= 1 && zone <= 60))
      throw GeographicLib::GeographicErr("zone not in [1,60]");
  }

  /****************************************************************************************
   *  Name     : forward
   *  Function : Calculates easting (x) and northing (y) from latitude and
   * longitude NOTE: ANGLES IN DEGREES. Author   : Modified :
   *****************************************************************************************/
  void forward(double &x, double &y, double lat, double lon) {
    tm.Forward(lon0, lat, lon, x, y);
    x += false_easting;
    y += false_northing;
  }

  /****************************************************************************************
   *  Name     : reverse
   *  Function : Calculates latitude and longitude from easting (x) and northing
   * (y) NOTE: ANGLES IN DEGREES. Author   : Modified :
   *****************************************************************************************/
  void reverse(double &lat, double &lon, double x, double y) {
    x -= false_easting;
    y -= false_northing;
    tm.Reverse(lon0, x, y, lat, lon);
  }
};
} // namespace PSBMPC_LIB