#include "shapefile_related/shapefil.h"
#include <Eigen/Dense>
#include <boost/foreach.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometry.hpp>
#include <filesystem>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>
#if ENABLE_TEST_FILE_PLOTTING
#include <engine.h>
#define BUFSIZE 1000000
#endif

using namespace boost::geometry;

template <typename T, typename F>
void read_shapefile(const std::string &filename, std::vector<T> &polygons,
                    F functor) {
  try {
    SHPHandle handle = SHPOpen(filename.c_str(), "rb");
    if (handle == nullptr) {
      throw std::string("File " + filename + " not found");
    }

    int nShapeType, nEntities;
    double adfMinBound[4], adfMaxBound[4];
    SHPGetInfo(handle, &nEntities, &nShapeType, adfMinBound, adfMaxBound);

    for (int i = 0; i < nEntities; i++) {
      SHPObject *psShape = SHPReadObject(handle, i);

      // Read only polygons, and only those without holes
      if (psShape->nSHPType == SHPT_POLYGON && psShape->nParts == 1) {
        T polygon;
        functor(psShape, polygon);
        polygons.push_back(polygon);
      }
      SHPDestroyObject(psShape);
    }
    SHPClose(handle);
  } catch (const std::string &s) {
    throw s;
  } catch (...) {
    throw std::string("Other exception");
  }
}

template <typename T> void convert(SHPObject *psShape, T &polygon) {
  double *x = psShape->padfX;
  double *y = psShape->padfY;
  for (int v = 0; v < psShape->nVertices; v++) {
    typename point_type<T>::type point;
    assign_values(point, x[v], y[v]);
    append(polygon, point);
  }
}

TEST(ReadShapefileTest, ReadAndPlot) {
  // Get absolute path to the test file
  std::filesystem::path test_file_path(__FILE__);
  std::filesystem::path test_dir = test_file_path.parent_path();
  std::filesystem::path shapefile_path =
      test_dir / "grounding_hazard_data" / "charts" / "land" / "land.shp";
  std::string filename = shapefile_path.string();

  typedef model::d2::point_xy<double> point_2d;
  typedef model::polygon<point_2d> polygon_2d;
  std::vector<polygon_2d> polygons;

  try {
    read_shapefile(filename, polygons, convert<polygon_2d>);
  } catch (const std::string &s) {
    std::cout << s << std::endl;
    GTEST_SKIP() << "Shapefile not available: " << s;
    return;
  }
  //*****************************************************************************************************************
  // Format polygon data for matlab plotting.
  //*****************************************************************************************************************

  Eigen::Matrix<double, -1, 2> mat_multi_poly;

  std::cout << "Simplified " << polygons.size() << std::endl;
  // iterating variable:
  int i = 0;

  // Very uunefficient counter, to count number of points in multi-polygon:

  BOOST_FOREACH (polygon_2d const &my_poly, polygons) {
    for (auto it = boost::begin(boost::geometry::exterior_ring(my_poly));
         it != boost::end(boost::geometry::exterior_ring(my_poly)); ++it) {
      i += 1;
    }
    i += 1;
  }
  mat_multi_poly.resize(i, 2);
  int j = 0;
  BOOST_FOREACH (polygon_2d const &my_poly, polygons) {

    /* print polygons in multipolygon type
    std::cout << boost::geometry::wkt<polygon_2d>(my_poly) << std::endl;
    */
    for (auto it = boost::begin(boost::geometry::exterior_ring(my_poly));
         it != boost::end(boost::geometry::exterior_ring(my_poly)); ++it) {
      /* print point in polygon type
      std::cout << boost::geometry::get<0>(*it) << std::endl;
      std::cout << boost::geometry::get<1>(*it) << std::endl;
      */
      mat_multi_poly(j, 0) = (boost::geometry::get<0>(*it));
      mat_multi_poly(j, 1) = (boost::geometry::get<1>(*it));

      j += 1;
    }
    mat_multi_poly(j, 0) = -1;
    mat_multi_poly(j, 1) = -1;
    j += 1;
  }

#if ENABLE_TEST_FILE_PLOTTING
  //*************************************************
  // Matlab engine setup
  //*************************************************
  Engine *ep = engOpen(NULL);
  if (ep == NULL) {
    std::cout << "engine start failed!" << std::endl;
  } else {

    std::cout << "engine start worked!" << std::endl;
  }

  char buffer[BUFSIZE + 1];

  /*Send Data to matlab*/
  mxArray *mat_multi_polygon_ptr = mxCreateDoubleMatrix(i, 2, mxREAL);
  double *pmmp = mxGetPr(mat_multi_polygon_ptr);
  Eigen::Map<Eigen::MatrixXd> map_mmp(pmmp, i, 2);
  map_mmp = mat_multi_poly;
  /*--------------------------------*/
  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);
  engPutVariable(ep, "P", mat_multi_polygon_ptr);
  engOutputBuffer(ep, buffer, BUFSIZE);
  engEvalString(ep, "test_polygon_plot");
  printf("%s", buffer);

  mxDestroyArray(mat_multi_polygon_ptr);
  engClose(ep);
#endif
}