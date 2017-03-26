#ifndef __VECTOR3D_H
#define __VECTOR3D_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <map>
#include <set>
#include <assert.h>
#define _USE_MATH_DEFINES

#define EPSILON 0.001f

// The vector class
class vector3d {
	public:
		// x coordinate
		double x;
		// y coordinate
		double y;
		// z coordinate
		double z;
		
		// Constructors
		vector3d();
		vector3d(double nX, double nY, double nZ);
		vector3d(double* rhs);
		vector3d(const vector3d& rhs);
		
		// Deconstructor
		~vector3d();
		
		// Function
		
		// Setters
		void set(double nX, double nY, double nZ);
		void set(vector3d v);
		void setX(double nX);
		void setY(double nY);
		void setZ(double nZ);
		
		// Getters
		double getX();
		double getY();
		double getZ();
		
		void loadZero();
		void loadOne();
		
		//rotations
		void rotateX(double angle);
		vector3d getRotatedX(double angle);
		void rotateY(double angle);
		vector3d getRotatedY(double angle);
		void rotateZ(double angle);
		vector3d getRotatedZ(double angle);
		void rotateAxis(double angle, vector3d & axis);
		vector3d getRotatedAxis(double angle, vector3d & axis);
		
		void packTo01();
		vector3d getPackedTo01();
		vector3d lerp(vector3d & v2, double factor);
		vector3d quadraticInterpolate(vector3d & v2, vector3d & v3, double factor);
		
		// equality check (value wise)
		int equals(vector3d m);
		
		// Rounding
		vector3d roundVector();
		bool operator < (const vector3d& other) const;
		double getAngle();
		double getAngleWith(vector3d a);
		// Addition
		vector3d operator + ( vector3d a);
		// Subtraction
		vector3d operator - ( vector3d a);
		// Dot Product
		double operator * ( vector3d a);
		// Division
		vector3d operator / (double a);
		// Cross Product - useful for normal calculation
		vector3d operator % ( vector3d a);
		// Length/Magnitude
		double mag();
		// Negation
		vector3d neg();
		// Unit (Normalized)
		vector3d normal();
		// Multiplication by a scalar
		vector3d operator * (double s);
		// Equals operator
		vector3d operator = (vector3d param);
		// Equality operator
		bool operator==(const vector3d & rhs);
		// Inequality operator
		bool operator!=(const vector3d & rhs);
		// self add, etc.
		void operator+=(const vector3d & rhs);
		void operator-=(const vector3d & rhs);
		void operator*=(const vector3d & rhs);
		void operator/=(const vector3d & rhs);
		//unary operators
		vector3d operator-();
		vector3d operator+();
		//cast to pointer to a (double *) for glVertex3fv etc
		operator double* ();
		
		// am I within the bounds stored in 'max' and 'min'?
		int withinBounds(vector3d maxB, vector3d minB);
		
		// perturb the vector to be within a distance 'r' from the original location
		vector3d perturb(double r, int w, int h);
		
		std::list<vector3d> getBoundingVectors(std::list<vector3d>& listOfUnitVectors);
};

#endif
