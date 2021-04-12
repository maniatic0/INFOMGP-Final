#ifndef SCENE_HEADER_FILE
#define SCENE_HEADER_FILE

#include <corecrt_math_defines.h>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <igl/bounding_box.h>
#include <igl/readMESH.h>
#include "ccd.h"
#include "volInt.h"
#include "auxfunctions.h"
#include "constraints.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/copyleft/cgal/delaunay_triangulation.h>
#include <igl/marching_tets.h>
#include <igl/vertex_components.h>
#include <igl/facet_components.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/resolve_duplicated_faces.h>
#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/copyleft/cgal/remesh_self_intersections.h>

#include <igl/copyleft/cgal/intersect_with_half_space.h>

#include <random>

using namespace Eigen;
using namespace std;


void support( const void* _obj, const ccd_vec3_t* _d, ccd_vec3_t* _p );
void stub_dir( const void* obj1, const void* obj2, ccd_vec3_t* dir );
void center( const void* _obj, ccd_vec3_t* dir );



//Impulse is defined as a pair <position, direction>
typedef std::pair<RowVector3d, RowVector3d> Impulse;

#define FIX_RNG 0
#if FIX_RNG
static std::mt19937 rng = std::mt19937( 1 );
#else
static std::mt19937 rng = std::mt19937(time(0));
#endif



//the class the contains each individual rigid objects and their functionality
class Mesh
{
public:
	size_t name;
	double density;
	size_t lifetime;

	MatrixXd origV; //original vertex positions, where COM=(0.0,0.0,0.0) - never change this!
	MatrixXd currV; //current vertex position
	MatrixXi F;     //faces of the tet mesh
	MatrixXi T;     //Tets in the tet mesh

	VectorXi boundTets; //indices (from T) of just the boundary tets, for collision

	//position of object in space. We must always have that currV = QRot(origV, orientation)+ COM
	RowVector4d orientation; //current orientation
	RowVector3d COM;         //current center of mass
	Matrix3d    invIT;       //Original *inverse* inertia tensor around the COM, defined in the rest state to the object (so to the canonical world system)

	VectorXd tetVolumes; //|T|x1 tetrahedra volumes
	VectorXd invMasses;  //|T|x1 tetrahedra *inverse* masses

	//kinematics
	bool        isFixed;   //is the object immobile
	double      totalMass; //sum(1/invMass)
	double      totalVolume;
	RowVector3d comVelocity; //the linear velocity of the center of mass
	RowVector3d angVelocity; //the angular velocity of the object.

	//checking collision between bounding boxes, and consequently the boundary tets if succeeds.
	//you do not need to update these functions (isBoxCollide and isCollide) unless you are doing a different collision

	bool isBoxCollide( const Mesh& m ) {
		if ( m.name == 0 || name == 0 ) {
			return false;
		}

		RowVector3d VMin1 = currV.colwise().minCoeff();
		RowVector3d VMax1 = currV.colwise().maxCoeff();
		RowVector3d VMin2 = m.currV.colwise().minCoeff();
		RowVector3d VMax2 = m.currV.colwise().maxCoeff();

		//checking all axes for non-intersection of the dimensional interval
		for ( int i = 0; i < 3; i++ )
			if ( ( VMax1( i ) < VMin2( i ) ) || ( VMax2( i ) < VMin1( i ) ) )
				return false;

		return true; //all dimensional intervals are overlapping = intersection

	}

	bool isCollide( const Mesh& m, double& depth, RowVector3d& intNormal, RowVector3d& intPosition ) {
		if ( ( isFixed && m.isFixed ) ) //collision does nothing
			return false;

		//collision between bounding boxes
		if ( !isBoxCollide( m ) )
			return false;

		//otherwise, full test
		ccd_t ccd;
		CCD_INIT( &ccd );
		ccd.support1 = support; // support function for first object
		ccd.support2 = support; // support function for second object
		ccd.center1  = center;
		ccd.center2  = center;

		ccd.first_dir      = stub_dir;
		ccd.max_iterations = 100; // maximal number of iterations


		void* obj1 = ( void* ) this;
		void* obj2 = ( void* ) &m;

		ccd_real_t _depth;
		ccd_vec3_t dir, pos;

		int nonintersect = ccdMPRPenetration( obj1, obj2, &ccd, &_depth, &dir, &pos );

		if ( nonintersect )
			return false;

		for ( int k = 0; k < 3; k++ ) {
			intNormal( k )   = dir.v[k];
			intPosition( k ) = pos.v[k];
		}

		depth = _depth;
		intPosition -= depth * intNormal / 2.0;

		//Vector3d p1=intPosition+depth*intNormal;
		//Vector3d p2=intPosition;
		//std::cout<<"intPosition: "<<intPosition<<std::endl;

		//std::cout<<"depth: "<<depth<<std::endl;
		//std::cout<<"After ccdGJKIntersect"<<std::endl;

		//return !nonintersect;

		return true;

	}


	//return the current inverted inertia tensor around the current COM. Update it by applying the orientation
	Matrix3d getCurrInvInertiaTensor() {
		/********
		TODO: complete from Practical 1
		*******/

		if ( this->isFixed ) return Matrix3d::Zero();
		Matrix3d R = Q2RotMatrix( orientation );
		return R * invIT * R.transpose(); //change this to your result
	}


	//Update the current position and orientation by integrating the linear and angular velocities, and update currV accordingly
	//You need to modify this according to its purpose
	void updatePosition( double timeStep ) {
		//just forward Euler now
		if ( isFixed )
			return; //a fixed object is immobile

		/********
		TODO: complete from Practical 1
		*******/

		COM += comVelocity * timeStep;
		{
			const RowVector3d tempAngVel = timeStep * angVelocity;
			orientation                  = QMult(
				QExp( RowVector4d( 0, tempAngVel.x(), tempAngVel.y(), tempAngVel.z() ) ),
				orientation
			);
		}

		for ( int i = 0; i < currV.rows(); i++ )
			currV.row( i ) << QRot( origV.row( i ), orientation ) + COM;
	}


	RowVector3d initStaticProperties( const double density ) {
		//TODO: compute tet volumes and allocate to vertices
		tetVolumes.conservativeResize( T.rows() );

		RowVector3d naturalCOM;
		naturalCOM.setZero();
		Matrix3d IT;
		IT.setZero();
		for ( int i = 0; i < T.rows(); i++ ) {
			Vector3d e01         = origV.row( T( i, 1 ) ) - origV.row( T( i, 0 ) );
			Vector3d e02         = origV.row( T( i, 2 ) ) - origV.row( T( i, 0 ) );
			Vector3d e03         = origV.row( T( i, 3 ) ) - origV.row( T( i, 0 ) );
			Vector3d tetCentroid = ( origV.row( T( i, 0 ) ) + origV.row( T( i, 1 ) ) + origV.row( T( i, 2 ) ) + origV.row( T( i, 3 ) ) ) / 4.0;
			tetVolumes( i )      = std::abs( e01.dot( e02.cross( e03 ) ) ) / 6.0;

			naturalCOM += tetVolumes( i ) * tetCentroid;

		}

		totalVolume = tetVolumes.sum();
		totalMass   = density * totalVolume;
		naturalCOM.array() /= totalVolume;

		//computing inertia tensor
		for ( int i = 0; i < T.rows(); i++ ) {
			RowVector4d xvec;
			xvec << origV( T( i, 0 ), 0 ) - naturalCOM( 0 ), origV( T( i, 1 ), 0 ) - naturalCOM( 0 ), origV( T( i, 2 ), 0 ) - naturalCOM( 0 ), origV( T( i, 3 ), 0 ) - naturalCOM( 0 );
			RowVector4d yvec;
			yvec << origV( T( i, 0 ), 1 ) - naturalCOM( 1 ), origV( T( i, 1 ), 1 ) - naturalCOM( 1 ), origV( T( i, 2 ), 1 ) - naturalCOM( 1 ), origV( T( i, 3 ), 1 ) - naturalCOM( 1 );
			RowVector4d zvec;
			zvec << origV( T( i, 0 ), 2 ) - naturalCOM( 2 ), origV( T( i, 1 ), 2 ) - naturalCOM( 2 ), origV( T( i, 2 ), 2 ) - naturalCOM( 2 ), origV( T( i, 3 ), 2 ) - naturalCOM( 2 );

			double   I00, I11, I22, I12, I21, I01, I10, I02, I20;
			Matrix4d sumMat = Matrix4d::Constant( 1.0 ) + Matrix4d::Identity();
			I00             = density * 6 * tetVolumes( i ) * ( yvec * sumMat * yvec.transpose() + zvec * sumMat * zvec.transpose() ).sum() / 120.0;
			I11             = density * 6 * tetVolumes( i ) * ( xvec * sumMat * xvec.transpose() + zvec * sumMat * zvec.transpose() ).sum() / 120.0;
			I22             = density * 6 * tetVolumes( i ) * ( xvec * sumMat * xvec.transpose() + yvec * sumMat * yvec.transpose() ).sum() / 120.0;
			I12             = I21 = -density * 6 * tetVolumes( i ) * ( yvec * sumMat * zvec.transpose() ).sum() / 120.0;
			I10             = I01 = -density * 6 * tetVolumes( i ) * ( xvec * sumMat * zvec.transpose() ).sum() / 120.0;
			I20             = I02 = -density * 6 * tetVolumes( i ) * ( xvec * sumMat * yvec.transpose() ).sum() / 120.0;

			Matrix3d currIT;
			currIT << I00, I01, I02,
				I10, I11, I12,
				I20, I21, I22;

			IT += currIT;

		}
		invIT = IT.inverse();
		if ( isFixed )
			invIT.setZero(); //infinite resistance to rotation

		return naturalCOM;

	}


	//Updating the linear and angular velocities of the object
	//You need to modify this to integrate from acceleration in the field (basically gravity)
	void updateVelocity( double timeStep ) {
		if ( isFixed )
			return;

		/********
		TODO: complete from Practical 1
		*******/
		RowVector3d gravityVector;
		gravityVector << 0, -9.81, 0.0;

		comVelocity += gravityVector * timeStep;
	}


	//the full integration for the time step (velocity + position)
	//You need to modify this if you are changing the integration
	void integrate( double timeStep ) {
		updateVelocity( timeStep );
		updatePosition( timeStep );
	}


	Mesh( const size_t _name, const MatrixXd& _V, const MatrixXi& _F, const MatrixXi& _T, const double _density, const bool _isFixed, const RowVector3d& _COM, const RowVector4d& _orientation ) {
		name        = _name;
		density     = _density;
		origV       = _V;
		F           = _F;
		T           = _T;
		isFixed     = _isFixed;
		COM         = _COM;
		orientation = _orientation;
		comVelocity.setZero();
		angVelocity.setZero();
		lifetime = 0;

		RowVector3d naturalCOM; //by the geometry of the object

		//initializes the original geometric properties (COM + IT) of the object
		naturalCOM = initStaticProperties( _density );

		origV.rowwise() -= naturalCOM; //removing the natural COM of the OFF file (natural COM is never used again)

		currV.resize( origV.rows(), origV.cols() );
		for ( int i = 0; i < currV.rows(); i++ )
			currV.row( i ) << QRot( origV.row( i ), orientation ) + COM;

		VectorXi boundVMask( origV.rows() );
		boundVMask.setZero();
		for ( int i = 0; i < F.rows(); i++ )
			for ( int j                 = 0; j < 3; j++ )
				boundVMask( F( i, j ) ) = 1;

		//cout<<"boundVMask.sum(): "<<boundVMask.sum()<<endl;

		vector<int> boundTList;
		for ( int i = 0; i < T.rows(); i++ ) {
			int incidence = 0;
			for ( int j = 0; j < 4; j++ )
				incidence += boundVMask( T( i, j ) );
			if ( incidence > 2 )
				boundTList.push_back( i );
		}

		boundTets.resize( boundTList.size() );
		for ( int i        = 0; i < boundTets.size(); i++ )
			boundTets( i ) = boundTList[i];
	}

	Mesh() {
		name    = 0;
		isFixed = true;
		comVelocity.setZero();
		angVelocity.setZero();
	}

	~Mesh() {}
};



//This class contains the entire scene operations, and the engine time loop.
class Scene
{
public:
	double                           breakImpulseMagnitude = 1e7;
	double                           currTime;
	int                              numFullV, numFullT;
	std::unordered_map<size_t, Mesh> meshes;
	size_t                           meshNum;

	//Practical 2
	vector<Constraint> constraints; //The (user) constraints of the scene

	//adding an objects. You do not need to update this generally
	void addMesh( const MatrixXd& V, const MatrixXi& F, const MatrixXi& T, const double density, const bool isFixed, const RowVector3d& COM, const RowVector4d& orientation ) {
		size_t name = meshNum++;
		std::cout << "Added mesh: " << name << std::endl;
		Mesh m( name, V, F, T, density, isFixed, COM, orientation );
		meshes.emplace( name, m );
	}

	void removeMesh( const size_t name ) {
		std::cout << "Removed mesh: " << name << std::endl;
		meshes.erase( name );
		if ( constraints.size() == 0 ) return;
		for ( size_t i = constraints.size() - 1; i >= 0; i-- ) {
			if ( constraints[i].m1 == name || constraints[i].m2 == name ) {
				constraints.erase( constraints.begin() + i );
			}
		}
	}

	static double Random() {
		static std::uniform_real_distribution<double> dis( 0, 1 );
		return dis( rng );
	}

	static double RandomUniform( double min, double max ) {
		static std::uniform_real_distribution<double> dis( 0, 1 );
		return min + ( max - min ) * dis( rng );
	}

	static double NormalizedRandom( double mean, double stddev ) {
		double u1 = Random();
		double u2 = Random();

		auto randStdNormal = std::sqrt( -2.0f * std::log( u1 ) ) *
			std::sin( 2.0f * M_PI * u2 );

		return mean + stddev * randStdNormal;
	}

	int l = 0;

	bool PrepareMesh( Mesh& mesh, const MatrixXd& V0, const MatrixXi& F0, const RowVector3d& offset, const RowVector4d& orientation, const double density ) {
		// New Mesh
		MatrixXi objT, objF;
		MatrixXd objV;

		MatrixXd SV, VV;
		MatrixXi SF, FF, IF, G, J, flip;
		VectorXi H,IM, UIM;
		igl::copyleft::cgal::RemeshSelfIntersectionsParam params;
		params.detect_only = false;
		igl::copyleft::cgal::remesh_self_intersections(V0,F0,params,VV,FF,IF,H,IM);
		std::for_each(FF.data(),FF.data()+FF.size(),[&IM](int & a){a=IM(a);});
		igl::remove_unreferenced(VV,FF,SV,SF,UIM);
		//igl::copyleft::cgal::outer_hull_legacy(SV,SF,G,J,flip);

		try {
			const int k = igl::copyleft::tetgen::tetrahedralize( SV, SF, "pYQ", objV, objT, objF );

			if ( k != 0 ) {
				std::cout << "\tTetgen failed with mesh: " << ( l - 1 ) << std::endl;
				return false;
			}
		} catch ( std::exception& e ) {
			std::cout << "\tCRRRAAASSSSHHHH with mesh: " << ( l - 1 ) << std::endl;
			return false;
		}

		RowVector3d naturalCOM;
		double      vol = 0;
		naturalCOM.setZero();
		for ( int i = 0; i < objT.rows(); i++ ) {
			Vector3d e01         = objV.row( objT( i, 1 ) ) - objV.row( objT( i, 0 ) );
			Vector3d e02         = objV.row( objT( i, 2 ) ) - objV.row( objT( i, 0 ) );
			Vector3d e03         = objV.row( objT( i, 3 ) ) - objV.row( objT( i, 0 ) );
			Vector3d tetCentroid = ( objV.row( objT( i, 0 ) ) + objV.row( objT( i, 1 ) ) + objV.row( objT( i, 2 ) ) + objV.row( objT( i, 3 ) ) ) / 4.0;

			const double tempVol = std::abs( e01.dot( e02.cross( e03 ) ) ) / 6.0;
			vol += tempVol;
			naturalCOM += tempVol * tetCentroid;
		}

		naturalCOM.array() /= vol;

		if (vol < 5)
		{
			std::cout << "\tToo small :( " << vol << std::endl;
			return false;
		}
		std::cout << "\tNice Volume :) " << vol << std::endl;

		//fixing weird orientation problem
		{
			MatrixXi tempF( objF.rows(), 3 );
			tempF << objF.col( 2 ), objF.col( 1 ), objF.col( 0 );
			objF = tempF;
		}

		mesh = Mesh( 0, objV, objF, objT, density, false, offset + QRot( naturalCOM, orientation ), orientation );

		return true;
	}

	static Vector2d Cross2d( Vector2d a, Vector2d b ) {
		return Vector2d( a.x() * b.y(), -b.x() * a.y() );
	}

	bool BreakMesh( const Mesh&     mesh, std::vector<Mesh>&        toAdd, const Vector3d& penPosition, const Vector3d& newCOMPosition,
					const Vector3d& newCOMVelocity, const Vector3d& newAngVelocity ) {

		size_t       meshCount      = toAdd.size();
		Vector3d     arm            = penPosition - mesh.COM.transpose();
		Vector3d     linearVelDelta = newCOMVelocity - mesh.comVelocity.transpose();
		Vector3d     angVelDelta    = newAngVelocity - mesh.angVelocity.transpose();
		Vector3d     impulse        = mesh.totalMass * ( linearVelDelta + angVelDelta.cross( arm ) );
		const double impulseNorm    = impulse.norm();
		assert( breakImpulseMagnitude > 0 );

		//if ( impulse.norm() < breakImpulseMagnitude ) return false;

		//TODO determine siteCount

		//std::cout << "Tanh: " << std::tanh( impulseNorm / breakImpulseMagnitude ) << std::endl;

		const double siteMaxNumber = 8.0;
		const size_t siteCount = std::floor(siteMaxNumber * std::tanh( impulseNorm / breakImpulseMagnitude ) );
		//std::cout << "Number of Sites: " << siteCount << std::endl;
		if ( siteCount < 4 ) return false;
		std::cout << "Number of Sites: " << siteCount << std::endl;

		MatrixXd sites = MatrixXd::Zero( siteCount, 2 );
		MatrixXi faces;

		// To local space
		Matrix3d rot  = Q2RotMatrix( mesh.orientation );
		Matrix3d roti = rot.transpose();

		// Impulse plane
		Vector3d lp = roti * ( penPosition - newCOMPosition );
		Vector3d n  = ( roti * impulse ) / impulseNorm;

		Vector3d VMin1 = mesh.origV.colwise().minCoeff().transpose() - lp;
		Vector3d VMax1 = mesh.origV.colwise().maxCoeff().transpose() - lp;

		//TODO what is this?
		VMin1 -= VMin1.dot( n ) * n;
		VMax1 -= VMax1.dot( n ) * n;

		Vector3d   newMax         = VMax1.cwiseMax( VMin1 );
		Vector3d   newMin         = VMax1.cwiseMin( VMin1 );
		Vector3d   diag           = newMax - newMin;
		Vector3d   diagNormalized = diag.normalized();
		AngleAxisd localRot       = AngleAxisd( 0.25 * M_PI, n );

		Vector3d nx = localRot * diagNormalized;
		Vector3d ny = localRot.inverse() * diagNormalized;

		//TODO is this correct???
		// Bounding box
		//
		Vector2d min2d = Vector2d(std::numeric_limits<double>().infinity(), std::numeric_limits<double>().infinity());
		Vector2d max2d = -min2d;
		for (size_t i = 0; i < 8; i++)
		{
			const double x = i & 1 ? newMin.x() : newMax.x();
			const double y = i & 2 ? newMin.y() : newMax.y();
			const double z = i & 4 ? newMin.z() : newMax.z();
			Vector3d corner(x, y, z);
			Vector2d corner2d(corner.dot(nx), corner.dot(ny));

			min2d = min2d.cwiseMin(corner2d);
			max2d = max2d.cwiseMax(corner2d);
		}
		
		sites.row( 0 ) = min2d * 1.5;                                              // min-min
		sites.row( 1 ) = max2d * 1.5;                                              // max-max
		sites.row( 2 ) = Vector2d( sites.row( 0 ).x(), sites.row( 1 ).y() ); // min-max
		sites.row( 3 ) = Vector2d( sites.row( 1 ).x(), sites.row( 0 ).y() ); // max-min

		// Random sites
		for ( int i = 4; i < siteCount; ++i ) {
			double   dist  = 0.5 * ( sites.row( 1 ) - sites.row( 0 ) ).norm() * std::abs( NormalizedRandom( 0.5f, 1.0f / 2.0f ) );
			double   angle = 2.0 * M_PI * Random();
			Vector2d pos   = Vector2d( dist * std::cos( angle ), dist * std::sin( angle ) );
			//pos            = pos.cwiseMin( max2d );
			//pos            = pos.cwiseMax( min2d );
			sites.row( i ) = pos;
			// TODO: Get distance to border, use cwiseMax and cwiseMin
		}

		igl::copyleft::cgal::delaunay_triangulation( sites, faces );

		for ( int i = 0; i < faces.rows(); ++i ) {
			Vector3i face = faces.row( i );

			Vector3d p0    = lp + sites.row( face[0] ).coeff( 0 ) * nx + sites.row( face[0] ).coeff( 1 ) * ny;
			Vector3d p1    = lp + sites.row( face[1] ).coeff( 0 ) * nx + sites.row( face[1] ).coeff( 1 ) * ny;
			Vector3d p2    = lp + sites.row( face[2] ).coeff( 0 ) * nx + sites.row( face[2] ).coeff( 1 ) * ny;
			Vector3d line0 = p1 - p0;
			Vector3d line1 = p2 - p1;
			Vector3d line2 = p0 - p2;

			RowVector3d planeNorm0 = impulse.cross( line0.normalized() ).transpose();
			RowVector3d planeNorm1 = impulse.cross( line1.normalized() ).transpose();
			RowVector3d planeNorm2 = impulse.cross( line2.normalized() ).transpose();

			//TODO when overriding V0, F0, some old data may not be replaced/removed
			MatrixXd   V0;
			MatrixXi   F0;
			MatrixXd   V1;
			MatrixXi   F1;
			MatrixXd   V2;
			MatrixXi   F2;
			MatrixXd   J;
			const bool i1 = igl::copyleft::cgal::intersect_with_half_space( mesh.origV, mesh.F, p0, planeNorm0, V0, F0, J );
			if ( !i1 ) {
				//std::cout << "\tDissapeard Piece! x1 wot" << std::endl;
				continue;
			}
			if ( F0.rows() == 0 ) {
				//std::cout << "\tDissapeard Piece! x1" << std::endl;
				continue;
			}

			const bool i2 = igl::copyleft::cgal::intersect_with_half_space( V0, F0, p1, planeNorm1, V1, F1, J );
			if ( !i2 ) {
				//std::cout << "\tDissapeard Piece! x2 wot" << std::endl;
				continue;
			}
			if ( F1.rows() == 0 ) {
				//std::cout << "\tDissapeard Piece! x2" << std::endl;
				continue;
			}

			const bool i3 = igl::copyleft::cgal::intersect_with_half_space( V1, F1, p2, planeNorm2, V2, F2, J );
			if ( !i3 ) {
				//std::cout << "\tDissapeard Piece! x3 wot" << std::endl;
				continue;
			}
			if ( F2.rows() == 0 ) {
				//std::cout << "\tDissapeard Piece! x3" << std::endl;
				continue;
			}

			VectorXi CF;
			igl::facet_components( F2, CF );

			for ( int j = 0; j <= CF.maxCoeff(); ++j ) {
				MatrixXd            V3 = V2;
				vector<RowVector3i> Fs = { };
				for ( int k = 0; k < CF.size(); ++k ) {
					if ( CF.coeff( k ) == j ) Fs.push_back( F2.row( k ) );
				}

				MatrixXi F3 = MatrixXi::Zero( Fs.size(), F2.cols() );
				for ( int k = 0; k < Fs.size(); ++k ) F3.row( k ) = Fs[k];

				MatrixXd V4;
				MatrixXi F4;
				VectorXi I;
				VectorXi J;
				igl::remove_unreferenced( V3, F3, V4, F4, I, J );

				Mesh rmesh;
				bool result = PrepareMesh( rmesh, V4, F4, newCOMPosition.transpose(), mesh.orientation, mesh.density );
				if ( result) {
					toAdd.push_back( std::move( rmesh ) );
					toAdd[toAdd.size() - 1].comVelocity = newCOMVelocity;
					toAdd[toAdd.size() - 1].angVelocity = newAngVelocity;
				}
			}
		}

		return toAdd.size() > meshCount;
	}

	/*********************************************************************
	This function handles collision constraints between objects m1 and m2 when found
	Input: meshes m1, m2
	depth: the depth of penetration
	contactNormal: the normal of the contact measured m1->m2
	penPosition: a point on m2 such that if m2 <= m2 + depth*contactNormal, then penPosition+depth*contactNormal is the common contact point
	CRCoeff: the coefficient of restitution
	
	You should create a "Constraint" class, and use its resolveVelocityConstraint() and resolvePositionConstraint() *alone* to resolve the constraint.
	You are not allowed to use practical 1 collision handling
	*********************************************************************/

	size_t k_frameProtection = 20;

	void handleCollision( Mesh& m1, Mesh& m2, vector<size_t>& remove, vector<Mesh>& toAdd, const double& depth, const RowVector3d& contactNormal, const RowVector3d& penPosition, const double CRCoeff, const double tolerance ) {
		//std::cout<<"contactNormal: "<<contactNormal<<std::endl;
		//std::cout<<"penPosition: "<<penPosition<<std::endl;

		double invMass1 = ( m1.isFixed ? 0.0 : 1.0 / m1.totalMass ); //fixed meshes have infinite mass
		double invMass2 = ( m2.isFixed ? 0.0 : 1.0 / m2.totalMass );

		auto constraint = Constraint( ConstraintType::COLLISION, ConstraintEqualityType::EQUALITY, 0, 0, 0, 0, invMass1, invMass2, contactNormal, 0.0, CRCoeff );

		{
			MatrixXd currCOMPositions( 2, 3 );
			currCOMPositions << m1.COM, m2.COM;
			MatrixXd currConstPositions( 2, 3 );
			currConstPositions << penPosition + depth * contactNormal, penPosition;
			MatrixXd currCOMVelocities( 2, 3 );
			currCOMVelocities << m1.comVelocity, m2.comVelocity;
			MatrixXd currAngVelocities( 2, 3 );
			currAngVelocities << m1.angVelocity, m2.angVelocity;

			Matrix3d invInertiaTensor1 = m1.getCurrInvInertiaTensor();
			Matrix3d invInertiaTensor2 = m2.getCurrInvInertiaTensor();
			MatrixXd correctedCOMVelocities, correctedAngVelocities, correctedCOMPositions;

			constraint.resolvePositionConstraint( currCOMPositions, currConstPositions, correctedCOMPositions, tolerance );
			constraint.resolveVelocityConstraint( currCOMPositions, currConstPositions, currCOMVelocities, currAngVelocities, invInertiaTensor1, invInertiaTensor2, correctedCOMVelocities, correctedAngVelocities, tolerance );

			if ( !m1.isFixed && m1.lifetime > k_frameProtection) {
				const bool result = BreakMesh( m1, toAdd, penPosition.transpose(), correctedCOMPositions.row( 0 ).transpose(), correctedCOMVelocities.row( 0 ).transpose(),
												correctedAngVelocities.row( 0 ).transpose() );

				if ( result ) {
					remove.push_back( m1.name );
				} else {
					m1.COM         = correctedCOMPositions.row( 0 );
					m1.comVelocity = correctedCOMVelocities.row( 0 );
					m1.angVelocity = correctedAngVelocities.row( 0 );
				}
			} else {
				m1.COM         = correctedCOMPositions.row( 0 );
				m1.comVelocity = correctedCOMVelocities.row( 0 );
				m1.angVelocity = correctedAngVelocities.row( 0 );
			}

			if ( !m2.isFixed && m2.lifetime > k_frameProtection) {
				const bool result = BreakMesh( m2, toAdd, penPosition.transpose(), correctedCOMPositions.row( 1 ).transpose(), correctedCOMVelocities.row( 1 ).transpose(),
												correctedAngVelocities.row( 1 ).transpose() );

				if ( result ) {
					remove.push_back( m2.name );
				} else {
					m2.COM         = correctedCOMPositions.row( 1 );
					m2.comVelocity = correctedCOMVelocities.row( 1 );
					m2.angVelocity = correctedAngVelocities.row( 1 );
				}
			} else {
				m2.COM         = correctedCOMPositions.row( 1 );
				m2.comVelocity = correctedCOMVelocities.row( 1 );
				m2.angVelocity = correctedAngVelocities.row( 1 );
			}
		}

		/***************
		TODO: practical 2
		update m(1,2) comVelocity, angVelocity and COM variables by using a Constraint class of type COLLISION
		***********************/

	}

	/*********************************************************************
	This function handles a single time step by:
	1. Integrating velocities, positions, and orientations by the timeStep
	2. (Practical 2) Detecting collisions and encoding them as constraints
	3. (Practical 2) Iteratively resolved positional and velocity constraints
	
	You do not need to update this function in Practical 2
	*********************************************************************/
	void updateScene( const double timeStep, const double CRCoeff, const double tolerance, const int maxIterations ) {

		//integrating velocity, position and orientation from forces and previous states
		for ( auto& mesh_pair : meshes ) {
			mesh_pair.second.integrate( timeStep );
		}


		//detecting and handling collisions when found
		//This is done exhaustively: checking every two objects in the scene.
		double         depth;
		RowVector3d    contactNormal, penPosition;
		vector<size_t> remove = { };
		vector<Mesh>   toAdd  = { };
		for ( auto it0 = meshes.begin(); it0 != std::prev( meshes.end() ); ++it0 ) {
			for ( auto it1 = std::next( it0 ); it1 != meshes.end(); ++it1 ) {
				if ( it0->second.isCollide( it1->second, depth, contactNormal, penPosition ) ) {
					handleCollision( it0->second, it1->second, remove, toAdd, depth, contactNormal, penPosition, CRCoeff, tolerance );
				}
			}
		}

		for ( size_t r : remove ) {
			removeMesh( r );
		}

		for ( auto& m : toAdd ) {
			m.name = meshNum++;
			std::cout << "Added Collision mesh: " << m.name << std::endl;
			meshes.emplace( m.name, m );
		}


		//Resolving user constraints iteratively until either:
		//1. Positions or velocities are valid up to tolerance (a full streak of validity in the iteration)
		//2. maxIterations has run out


		//Resolving velocity
		int currIteration  = 0;
		int zeroStreak     = 0; //how many consecutive constraints are already below tolerance without any change; the algorithm stops if all are.
		int currConstIndex = 0;
		while ( ( zeroStreak < constraints.size() ) && ( currIteration * constraints.size() < maxIterations ) ) {

			Constraint currConstraint = constraints[currConstIndex];

			RowVector3d origConstPos1 = meshes[currConstraint.m1].origV.row( currConstraint.v1 );
			RowVector3d origConstPos2 = meshes[currConstraint.m2].origV.row( currConstraint.v2 );

			RowVector3d currConstPos1 = QRot( origConstPos1, meshes[currConstraint.m1].orientation ) + meshes[currConstraint.m1].COM;
			RowVector3d currConstPos2 = QRot( origConstPos2, meshes[currConstraint.m2].orientation ) + meshes[currConstraint.m2].COM;

			//cout<<"(currConstPos1-currConstPos2).norm(): "<<(currConstPos1-currConstPos2).norm()<<endl;
			//cout<<"(meshes[currConstraint.m1].currV.row(currConstraint.v1)-meshes[currConstraint.m2].currV.row(currConstraint.v2)).norm(): "<<(meshes[currConstraint.m1].currV.row(currConstraint.v1)-meshes[currConstraint.m2].currV.row(currConstraint.v2)).norm()<<endl;
			MatrixXd currCOMPositions( 2, 3 );
			currCOMPositions << meshes[currConstraint.m1].COM, meshes[currConstraint.m2].COM;
			MatrixXd currConstPositions( 2, 3 );
			currConstPositions << currConstPos1, currConstPos2;
			MatrixXd currCOMVelocities( 2, 3 );
			currCOMVelocities << meshes[currConstraint.m1].comVelocity, meshes[currConstraint.m2].comVelocity;
			MatrixXd currAngVelocities( 2, 3 );
			currAngVelocities << meshes[currConstraint.m1].angVelocity, meshes[currConstraint.m2].angVelocity;

			Matrix3d invInertiaTensor1 = meshes[currConstraint.m1].getCurrInvInertiaTensor();
			Matrix3d invInertiaTensor2 = meshes[currConstraint.m2].getCurrInvInertiaTensor();
			MatrixXd correctedCOMVelocities, correctedAngVelocities, correctedCOMPositions;

			bool velocityWasValid = currConstraint.resolveVelocityConstraint( currCOMPositions, currConstPositions, currCOMVelocities, currAngVelocities, invInertiaTensor1, invInertiaTensor2, correctedCOMVelocities, correctedAngVelocities,
																			tolerance );

			if ( velocityWasValid ) {
				zeroStreak++;
			} else {
				//only update the COM and angular velocity, don't both updating all currV because it might change again during this loop!
				zeroStreak                            = 0;
				meshes[currConstraint.m1].comVelocity = correctedCOMVelocities.row( 0 );
				meshes[currConstraint.m2].comVelocity = correctedCOMVelocities.row( 1 );

				meshes[currConstraint.m1].angVelocity = correctedAngVelocities.row( 0 );
				meshes[currConstraint.m2].angVelocity = correctedAngVelocities.row( 1 );

			}

			currIteration++;
			currConstIndex = ( currConstIndex + 1 ) % ( constraints.size() );
		}

		if ( currIteration * constraints.size() >= maxIterations )
			cout << "Velocity Constraint resolution reached maxIterations without resolving!" << endl;


		//Resolving position
		currIteration  = 0;
		zeroStreak     = 0; //how many consecutive constraints are already below tolerance without any change; the algorithm stops if all are.
		currConstIndex = 0;
		while ( ( zeroStreak < constraints.size() ) && ( currIteration * constraints.size() < maxIterations ) ) {

			Constraint currConstraint = constraints[currConstIndex];

			RowVector3d origConstPos1 = meshes[currConstraint.m1].origV.row( currConstraint.v1 );
			RowVector3d origConstPos2 = meshes[currConstraint.m2].origV.row( currConstraint.v2 );

			RowVector3d currConstPos1 = QRot( origConstPos1, meshes[currConstraint.m1].orientation ) + meshes[currConstraint.m1].COM;
			RowVector3d currConstPos2 = QRot( origConstPos2, meshes[currConstraint.m2].orientation ) + meshes[currConstraint.m2].COM;

			MatrixXd currCOMPositions( 2, 3 );
			currCOMPositions << meshes[currConstraint.m1].COM, meshes[currConstraint.m2].COM;
			MatrixXd currConstPositions( 2, 3 );
			currConstPositions << currConstPos1, currConstPos2;

			MatrixXd correctedCOMPositions;

			bool positionWasValid = currConstraint.resolvePositionConstraint( currCOMPositions, currConstPositions, correctedCOMPositions, tolerance );

			if ( positionWasValid ) {
				zeroStreak++;
			} else {
				//only update the COM and angular velocity, don't both updating all currV because it might change again during this loop!
				zeroStreak = 0;

				meshes[currConstraint.m1].COM = correctedCOMPositions.row( 0 );
				meshes[currConstraint.m2].COM = correctedCOMPositions.row( 1 );

			}

			currIteration++;
			currConstIndex = ( currConstIndex + 1 ) % ( constraints.size() );
		}

		if ( currIteration * constraints.size() >= maxIterations )
			cout << "Position Constraint resolution reached maxIterations without resolving!" << endl;

		//updating currV according to corrected COM
		for ( auto& mesh : meshes ) {
			for ( int j = 0; j < mesh.second.currV.rows(); j++ )
				mesh.second.currV.row( j ) << QRot( mesh.second.origV.row( j ), mesh.second.orientation ) + mesh.second.COM;
			++mesh.second.lifetime;
		}
			
		currTime += timeStep;
	}

	//loading a scene from the scene .txt files
	//you do not need to update this function
	bool loadScene( const std::string dataFolder, const std::string sceneFileName, const std::string constraintFileName ) {

		ifstream sceneFileHandle, constraintFileHandle;
		sceneFileHandle.open( dataFolder + std::string( "/" ) + sceneFileName );
		if ( !sceneFileHandle.is_open() )
			return false;
		int numofObjects;

		currTime = 0;
		sceneFileHandle >> numofObjects;
		for ( int i = 0; i < numofObjects; i++ ) {
			MatrixXi    objT, objF;
			MatrixXd    objV;
			std::string MESHFileName;
			bool        isFixed;
			double      youngModulus, poissonRatio, density;
			RowVector3d userCOM;
			RowVector4d userOrientation;
			sceneFileHandle >> MESHFileName >> density >> youngModulus >> poissonRatio >> isFixed >> userCOM( 0 ) >> userCOM( 1 ) >> userCOM( 2 ) >> userOrientation( 0 ) >> userOrientation( 1 ) >> userOrientation( 2 ) >>
				userOrientation( 3 );
			userOrientation.normalize();


			MatrixXd VOFF;
			MatrixXi FOFF;
			if ( MESHFileName.find( ".off" ) != std::string::npos ) {

				igl::readOFF( dataFolder + std::string( "/" ) + MESHFileName, VOFF, FOFF );
				RowVectorXd mins = VOFF.colwise().minCoeff();
				RowVectorXd maxs = VOFF.colwise().maxCoeff();
				for ( int k = 0; k < VOFF.rows(); k++ )
					VOFF.row( k ) << 25.0 * ( VOFF.row( k ) - mins ).array() / ( maxs - mins ).array();

				if ( !isFixed )
					igl::copyleft::tetgen::tetrahedralize( VOFF, FOFF, "pq1.1", objV, objT, objF );
				else
					igl::copyleft::tetgen::tetrahedralize( VOFF, FOFF, "pq1.414Y", objV, objT, objF );
			} else {
				igl::readMESH( dataFolder + std::string( "/" ) + MESHFileName, objV, objT, objF );
				VOFF = objV;
				FOFF = objF;
			}

			//fixing weird orientation problem
			{
				MatrixXi tempF( objF.rows(), 3 );
				tempF << objF.col( 2 ), objF.col( 1 ), objF.col( 0 );
				objF = tempF;
			}

			addMesh( objV, objF, objT, density, isFixed, userCOM, userOrientation );
			cout << "COM: " << userCOM << endl;
			cout << "orientation: " << userOrientation << endl;
		}

		//Practical 2 change
		//reading intra-mesh attachment constraints
		int numofConstraints;
		constraintFileHandle.open( dataFolder + std::string( "/" ) + constraintFileName );
		if ( !constraintFileHandle.is_open() )
			return false;
		constraintFileHandle >> numofConstraints;
		for ( int i = 0; i < numofConstraints; i++ ) {
			int attachM1, attachM2, attachV1, attachV2;
			constraintFileHandle >> attachM1 >> attachV1 >> attachM2 >> attachV2;

			double initDist = ( meshes[attachM1].currV.row( attachV1 ) - meshes[attachM2].currV.row( attachV2 ) ).norm();
			//cout<<"initDist: "<<initDist<<endl;
			double invMass1 = ( meshes[attachM1].isFixed ? 0.0 : 1.0 / meshes[attachM1].totalMass ); //fixed meshes have infinite mass
			double invMass2 = ( meshes[attachM2].isFixed ? 0.0 : 1.0 / meshes[attachM2].totalMass );
#define FIXED_DISTANCE 0
#if FIXED_DISTANCE
      constraints.push_back(Constraint(DISTANCE, EQUALITY,attachM1, attachV1, attachM2, attachV2, invMass1,invMass2,RowVector3d::Zero(), initDist, 0.0));
#else
			constexpr double slack = 0.2;
			constraints.push_back( Constraint( DISTANCE, INEQUALITY_SMALLER, attachM1, attachV1, attachM2, attachV2, invMass1, invMass2, RowVector3d::Zero(), ( 1 + slack ) * initDist, 0.0 ) );
			constraints.push_back( Constraint( DISTANCE, INEQUALITY_GREATER, attachM1, attachV1, attachM2, attachV2, invMass1, invMass2, RowVector3d::Zero(), ( 1 - slack ) * initDist, 0.0 ) );
#endif
		}

		return true;
	}


	Scene() : meshNum( 1 ) {}
	~Scene() {}
};



/*****************************Auxiliary functions for collision detection. Do not need updating********************************/

/** Support function for libccd*/
void support( const void* _obj, const ccd_vec3_t* _d, ccd_vec3_t* _p ) {
	// assume that obj_t is user-defined structure that holds info about
	// object (in this case box: x, y, z, pos, quat - dimensions of box,
	// position and rotation)
	//std::cout<<"calling support"<<std::endl;
	Mesh*       obj = ( Mesh* ) _obj;
	RowVector3d p;
	RowVector3d d;
	for ( int i = 0; i < 3; i++ )
		d( i )  = _d->v[i]; //p(i)=_p->v[i];

	d.normalize();
	//std::cout<<"d: "<<d<<std::endl;

	int maxVertex  = -1;
	int maxDotProd = -32767.0;
	for ( int i = 0; i < obj->currV.rows(); i++ ) {
		double currDotProd = d.dot( obj->currV.row( i ) - obj->COM );
		if ( maxDotProd < currDotProd ) {
			maxDotProd = currDotProd;
			//std::cout<<"maxDotProd: "<<maxDotProd<<std::endl;
			maxVertex = i;
		}

	}
	//std::cout<<"maxVertex: "<<maxVertex<<std::endl;

	for ( int i  = 0; i < 3; i++ )
		_p->v[i] = obj->currV( maxVertex, i );

	//std::cout<<"end support"<<std::endl;
}

void stub_dir( const void* obj1, const void* obj2, ccd_vec3_t* dir ) {
	dir->v[0] = 1.0;
	dir->v[1] = 0.0;
	dir->v[2] = 0.0;
}

void center( const void* _obj, ccd_vec3_t* center ) {
	Mesh* obj = ( Mesh* ) _obj;
	for ( int i      = 0; i < 3; i++ )
		center->v[i] = obj->COM( i );
}



#endif
