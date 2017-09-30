#pragma once
//#define USE_OPENMESH

#ifdef USE_OPENMESH

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#ifndef _SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif

#ifdef _DEBUG
#pragma comment(lib, "OpenMeshCored.lib")
#pragma comment(lib, "OpenMeshToolsd.lib")
#else
#pragma comment(lib, "OpenMeshCore.lib")
#pragma comment(lib, "OpenMeshTools.lib")
#endif // _DEBUG

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<>  TriMesh;

#endif // USE_OPENMESH


#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

#ifdef SINGLE
#define REAL float
#else /* not SINGLE */
#define REAL double
#endif /* not SINGLE */
#include "./triangle/triangle.h"


using std::string;
using std::vector;
using std::cout;
using std::endl;

class Triangulator
{
	struct Polygon
	{
		int ptsCnt_ = 0;
		vector<Eigen::Vector3d> bdryPts_;
		void addPt(const Eigen::Vector3d &p) { bdryPts_.push_back(p); ptsCnt_++; }

		/*! 
		 * @brief Calculate the center of the polygon
		 *
		 * Used for `triangulatePolygonWithHoles` to compute the center to mark holes
		 * @return the center of polygon 
		 * @note  Since we just need to find a point in the polygon to mark it as a hole, 
		 * any point in the polygon is ok. But when the polygon is not convex, the center may be out of the polygon.
		 * The triangulation result will be wrong.
		 * @see
		*/
		Eigen::Vector3d getCenter() const 
		{
			Eigen::Vector3d O = Eigen::Vector3d(0.0, 0.0, 0.0);
			for (auto p : bdryPts_)
				O += p;
			O /= float(ptsCnt_);
			return O;
		}
	};
	typedef Polygon Hole;

public:
	Triangulator();
	~Triangulator();

	void testTriPoly();
	void testTriPolyWithHoles();

	/*! 
	 * @brief Triangulate a polygon.
	 *
	 * @param poly input polygon
	 * @param verts output mesh vertex
	 * @param faces output mesh faces(triangles)
	 * @return  
	 * @note  
	 * @see
	*/
	void triangulatePolygon(const Polygon& poly, vector<Eigen::Vector3d> &verts, vector<Eigen::Vector3i> &faces);
	/*! 
	 * @brief Triangulate a polygon with holes.
	 *
	 * Detailed explanation.
	 * @param poly input polygon
	 * @param holes input holes
	 * @param verts output mesh vertex
	 * @param faces output mesh faces(triangles)
	 * @return  
	 * @note  
	 * @see
	*/
	void triangulatePolygonWithHoles(const Polygon& poly, const vector<Hole> holes, vector<Eigen::Vector3d> &verts, vector<Eigen::Vector3i> &faces);
	void saveAsMesh(const string &file, const vector<Eigen::Vector3d> &verts, const vector<Eigen::Vector3i> &faces);


#ifdef USE_OPENMESH
	void triangulatePolygon(const Polygon& poly, TriMesh &mesh);
	void triangulatePolygonWithHoles(const Polygon& poly, const vector<Hole> holes, TriMesh &mesh);
	void saveAsMesh(const string &file, const TriMesh &mesh);
#endif // USE_OPENMESH



	string cmd_;

public:
	void initTriangulateio(triangulateio& io);
	void freeTriangualteio(triangulateio& io);

	void set_cmd(const string &cmd) { cmd_ = cmd; }
	void set_quiet(bool t) { quiet_ = t; }
	void set_maximun_area(double area) {maximun_area_ = area;}
	void set_minimun_angle(double angle) { minimun_angle_ = angle; }
	void set_maximum_steiner_points(int n) { maximum_steiner_points_ = n; }
	void set_convex_hull(bool t) { convex_hull_ = t; }
	void set_comformming_delaunay(bool t) { comformming_delaunay_ = t; }
	void set_suppress_boundary_splitting(bool t) { suppress_boundary_splitting_ = t; }

	string get_cmd() const { return cmd_; }
	double get_maximun_area() const {return maximun_area_;}
	double get_minimum_angle() const {return minimun_angle_;}
	int    get_maximum_steiner_points() const { return maximum_steiner_points_; }

	bool is_quiet() const { return quiet_; }
	bool is_convex_hull() const {return convex_hull_;}
	bool is_comformming_delaunay() const {return comformming_delaunay_;}
	bool is_suppress_boundary_splitting() const {return suppress_boundary_splitting_;}
	

private:
	string getTriangleCmd();
	bool init_plane(const vector<Eigen::Vector3d>& input_vertices_positions);
	Eigen::Vector3d   project_to_plane(const Eigen::Vector3d& v);
	Eigen::Vector3d unproject_to_plane(const Eigen::Vector3d& v);

private:
	double maximun_area_;						///* if -1, not use; if > 0, use it;
	double minimun_angle_;						///* if -1, not use; if > 0, use it;
	int    maximum_steiner_points_;             ///* if -1, not use; if > 0, use it;
	bool   convex_hull_;
	bool   comformming_delaunay_;
	bool   suppress_boundary_splitting_;
	bool   quiet_;

	Eigen::Vector3d plane_normal_;
	Eigen::Vector3d plane_center_;
	Eigen::Vector3d plane_x_axis_;
	Eigen::Vector3d plane_y_axis_;
};

