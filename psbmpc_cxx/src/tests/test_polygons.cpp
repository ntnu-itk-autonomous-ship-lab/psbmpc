#include <iostream>
#include <list>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/foreach.hpp>

int main()
{
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;
    typedef boost::geometry::model::linestring<point_type> linestring_type;
    typedef boost::geometry::model::multi_point<point_type> multi_point_type;
    typedef boost::geometry::model::multi_polygon<polygon_type> multi_polygon_type;
    point_type p(2,5);
    polygon_type poly;
    linestring_type line;
    multi_point_type mp;
    multi_polygon_type mpoly;

    boost::geometry::read_wkt(
        "POLYGON((2 1.3,2.4 1.7,2.8 1.8,3.4 1.2,3.7 1.6,3.4 2,4.1 3,5.3 2.6,5.4 1.2,4.9 0.8,2.9 0.7,2 1.3)"
            "(4.0 2.0, 4.2 1.4, 4.8 1.9, 4.4 2.2, 4.0 2.0))", poly);
    boost::geometry::read_wkt("MULTIPOLYGON(((1 2,1 4,3 4,3 2,1 2)),((4 2,4 4,6 4,6 2, 4 2)))", mpoly);
    line.push_back(point_type(0,0));
    line.push_back(point_type(0,3));
    mp.push_back(point_type(0,0));
    mp.push_back(point_type(3,3));
    
/*__________________________Calculate distance from point to polygon in multipolygon___________________________________________________________*/        
    BOOST_FOREACH(polygon_type const& my_poly, mpoly){
        std::cout << boost::geometry::wkt<polygon_type>(my_poly) << std::endl;
        for(auto it = boost::begin(boost::geometry::exterior_ring(my_poly)); it != boost::end(boost::geometry::exterior_ring(my_poly)); ++it)
            {
                std::cout << boost::geometry::get<0>(*it) << std::endl;
                std::cout << boost::geometry::get<1>(*it) << std::endl;
                //use the coordinates...
            }
    }
 
        
/*_____________________________________________________________________________________*/        
   /* std::cout
        << "Point-Poly: " << boost::geometry::distance(p, poly) << std::endl
        << "Point-Line: " << boost::geometry::distance(p, line) << std::endl
        << "Point-MultiPoly: " << boost::geometry::distance(p, mpoly) << std::endl
        << "Point-MultiPoint: " << boost::geometry::distance(p, mp) << std::endl;
    */
    return 0;
}