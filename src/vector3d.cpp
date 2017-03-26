#include "vector3d.h"

vector3d::vector3d()
{
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

vector3d::vector3d(double nX, double nY, double nZ)
{
	this->x = nX;
	this->y = nY;
	this->z = nZ;
}

vector3d::vector3d(double* rhs)
{
	this->x = *rhs;
	this->y = *(rhs+1);
	this->z = *(rhs+2);
}

vector3d::vector3d(const vector3d& rhs)
{
	this->x = rhs.x;
	this->y = rhs.y;
	this->z = rhs.z;
}

vector3d::~vector3d()
{

}

int vector3d::equals(vector3d m)
{
	if (fabsf(this->x - m.x) < EPSILON && fabsf(this->y - m.y) < EPSILON && fabsf(this->z - m.z) < EPSILON)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

vector3d vector3d::roundVector()
{
	vector3d temp;
	
	temp.x = floor(this->x);
	temp.y = floor(this->y);
	temp.z = floor(this->z);
	//temp.x = (int)(this->x + 0.5f);
	//temp.y = (int)(this->y + 0.5f);
	//temp.z = (int)(this->z + 0.5f);
	
	return temp;	
	
	/*double xR = roundf(this->x);
	double yR = roundf(this->y);
	double zR = roundf(this->z);
	
	return vector3d(xR, yR, zR);*/
}

// Set the current vector (3 floats)
void vector3d::set(double nX, double nY, double nZ)
{
	this->x = nX;
	this->y = nY;
	this->z = nZ;
}

// Set the current vector (1 vector3d)
void vector3d::set(vector3d v)
{
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
}

// Set the current vector's x-value
void vector3d::setX(double nX)
{
	this->x = nX;
}

// Set the current vector's y-value
void vector3d::setY(double nY)
{
	this->y = nY;
}

// Set the current vector's z-value
void vector3d::setZ(double nZ)
{
	this->z = nZ;
}

// Get the current vector's x-value
double vector3d::getX()
{
	return this->x;
}

// Get the current vector's y-value
double vector3d::getY()
{
	return this->y;
}

// Get the current vector's z-value
double vector3d::getZ()
{
	return this->z;
}

// Set all values to zero
void vector3d::loadZero()
{
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

// Set all values to one
void vector3d::loadOne()
{
	this->x = 1.0;
	this->y = 1.0;
	this->z = 1.0;
}

void vector3d::rotateX(double angle)
{
	(*this)=getRotatedX(angle);
}

vector3d vector3d::getRotatedX(double angle)
{
	vector3d temp;
	
	if(angle==0.0)
	{
		return (*this);
	}

	double sinAngle=(double)sin(M_PI*angle/180);
	double cosAngle=(double)cos(M_PI*angle/180);

	temp.x = this->x;
	temp.y = this->y*cosAngle - this->z*sinAngle;
	temp.z = this->y*sinAngle + this->z*cosAngle;
	
	//return vector3d(x, y*cosAngle - z*sinAngle, y*sinAngle + z*cosAngle);
	return temp;
}

void vector3d::rotateY(double angle)
{
	(*this)=getRotatedY(angle);
}

vector3d vector3d::getRotatedY(double angle)
{
	vector3d temp;
	
	if(angle==0.0)
	{
		return (*this);
	}

	double sinAngle=(double)sin(M_PI*angle/180);
	double cosAngle=(double)cos(M_PI*angle/180);

	temp.x = this->x*cosAngle + this->z*sinAngle;
	temp.y = this->y;
	temp.z = -this->x*sinAngle + this->z*cosAngle;
	
	//return vector3d(x*cosAngle + z*sinAngle, y, -x*sinAngle + z*cosAngle);
	return temp;
}

void vector3d::rotateZ(double angle)
{
	(*this)=getRotatedZ(angle);
}

vector3d vector3d::getRotatedZ(double angle)
{
	vector3d temp;
	
	if(angle==0.0)
	{
		return (*this);
	}

	double sinAngle=(double)sin(M_PI*angle/180);
	double cosAngle=(double)cos(M_PI*angle/180);
	
	temp.x = this->x*cosAngle - this->y*sinAngle;
	temp.y = this->x*sinAngle + this->y*cosAngle;
	temp.z = this->z;
	
	//return vector3d(x*cosAngle - y*sinAngle, x*sinAngle + y*cosAngle, z);
	return temp;
}

void vector3d::rotateAxis(double angle, vector3d & axis)
{
	(*this)=getRotatedAxis(angle, axis);
}

vector3d vector3d::getRotatedAxis(double angle, vector3d & axis)
{
	vector3d temp;
	
	if(angle==0.0)
	{
		return (*this);
	}

	vector3d u=axis.normal();

	vector3d rotMatrixRow0, rotMatrixRow1, rotMatrixRow2;

	double sinAngle=(double)sin(M_PI*angle/180);
	double cosAngle=(double)cos(M_PI*angle/180);
	double oneMinusCosAngle=1.0-cosAngle;

	rotMatrixRow0.x=(u.x)*(u.x) + cosAngle*(1-(u.x)*(u.x));
	rotMatrixRow0.y=(u.x)*(u.y)*(oneMinusCosAngle) - sinAngle*u.z;
	rotMatrixRow0.z=(u.x)*(u.z)*(oneMinusCosAngle) + sinAngle*u.y;

	rotMatrixRow1.x=(u.x)*(u.y)*(oneMinusCosAngle) + sinAngle*u.z;
	rotMatrixRow1.y=(u.y)*(u.y) + cosAngle*(1-(u.y)*(u.y));
	rotMatrixRow1.z=(u.y)*(u.z)*(oneMinusCosAngle) - sinAngle*u.x;
	
	rotMatrixRow2.x=(u.x)*(u.z)*(oneMinusCosAngle) - sinAngle*u.y;
	rotMatrixRow2.y=(u.y)*(u.z)*(oneMinusCosAngle) + sinAngle*u.x;
	rotMatrixRow2.z=(u.z)*(u.z) + cosAngle*(1-(u.z)*(u.z));

	// was alays commented out
	//return vector3d(this->DotProduct(rotMatrixRow0), this->DotProduct(rotMatrixRow1), this->DotProduct(rotMatrixRow2));
	
	// old
	//return vector3d((*this) * rotMatrixRow0, (*this) * rotMatrixRow1, (*this) * rotMatrixRow2);
	
	temp.x = (*this) * rotMatrixRow0;
	temp.y = (*this) * rotMatrixRow1;
	temp.z = (*this) * rotMatrixRow2;
	
	return temp;
}

void vector3d::packTo01()
{
	(*this)=getPackedTo01();
}

vector3d vector3d::getPackedTo01()
{
	vector3d temp(*this);
	vector3d h;

	temp = temp.normal();
	h.x = 0.5f;
	h.y = 0.5f;
	h.z = 0.5f;

	//temp=temp*0.5f+vector3d(0.5f, 0.5f, 0.5f);
	temp = temp*0.5f + h;
	
	return temp;
}

vector3d vector3d::lerp(vector3d & v2, double factor)
{
	vector3d temp;
	
	temp = (*this)*(1.0-factor) + v2*factor;
	
	//return (*this)*(1.0-factor) + v2*factor;
	return temp;
}

vector3d vector3d::quadraticInterpolate(vector3d & v2, vector3d & v3, double factor)
{
	vector3d temp;
	
	temp = (*this)*(1.0-factor)*(1.0-factor) + v2*2*factor*(1.0-factor) + v3*factor*factor;
	
	//return (*this)*(1.0-factor)*(1.0-factor) + v2*2*factor*(1.0-factor) + v3*factor*factor;
	return temp;
}

bool vector3d::operator < (const vector3d& other) const
{
	assert(false); //don't use this method anymore
	// is this < other?
	vector3d aThis = *this, anOther = other;
	assert(anOther.mag() == 1.0 && aThis.mag() == 1.0);
	return aThis.getAngle() < anOther.getAngle();
}

double vector3d::getAngle()
{
	vector3d toUnit = this->normal();
	if (toUnit.x == 0.0)
	{
		if (toUnit.y > 0.0)
		{
			return 0.5 * M_1_PI;
		}
		else
		{
			return 1.5 * M_1_PI;
		}
	}
	if (toUnit.y == 0.0)
	{
		if (toUnit.x > 0.0)
		{
			return 0.0;
		}
		else
		{
			return 2.0 * M_1_PI;
		}
	}
	int thisArea = toUnit.x * toUnit.y > 0.0 ? (toUnit.x > 0.0 ? 1 : 3) : (toUnit.x < 0.0 ? 2 : 4);
	double asinfThis = asinf(toUnit.x), angleThis;
	switch (thisArea)
	{
	case 1:
		angleThis = asinfThis;
		break;
	case 2:
		angleThis = M_1_PI - asinfThis;
		break;
	case 3:
		angleThis = M_1_PI + asinfThis;
		break;
	case 4:
		angleThis = 2.0 * M_1_PI - asinfThis;
		break;
	}
	return angleThis;
}

double vector3d::getAngleWith(vector3d a)
{
	return fabs(this->normal().getAngle() - a.normal().getAngle());
//	double angleDiff = fabs(this->normal().getAngle() - a.normal().getAngle());
//	return (angleDiff > 2 * M_1_PI - angleDiff) ? 2 * M_1_PI - angleDiff : angleDiff;
}

// Add two vectors
vector3d vector3d::operator + ( vector3d a)
{
	vector3d temp;
	temp.x = this->x + a.x;
	temp.y = this->y + a.y;
	temp.z = this->z + a.z;
	
	return temp;
}

// Subtract two vectors
vector3d vector3d::operator - ( vector3d a)
{
	vector3d temp;
	temp.x = this->x - a.x;
	temp.y = this->y - a.y;
	temp.z = this->z - a.z;
	
	return temp;
}

// Compute the dot product
double vector3d::operator * ( vector3d a)
{
	return ((this->x*a.x) + (this->y*a.y) + (this->z*a.z));
}

// Divide by a scalar
vector3d vector3d::operator / (double a)
{
	vector3d temp;
	temp.x = this->x/a;
	temp.y = this->y/a;
	temp.z = this->z/a;
	
	return temp;
}

// Compute the cross product
vector3d vector3d::operator % ( vector3d a)
{
	vector3d temp;

	temp.x = this->y * a.z - this->z * a.y;
	temp.y = this->z * a.x - this->x * a.z;
	temp.z = this->x * a.y - this->y * a.x;
	
	return temp;
}

// Compute the magnitude/length
double vector3d::mag()
{
	return sqrt(this->x*this->x + this->y*this->y + this->z*this->z);
}

// Negate the vector
vector3d vector3d::neg()
{
	vector3d temp;
	temp.x = -this->x;
	temp.y = -this->y;
	temp.z = -this->z;
	
	return temp;
}

// Compute the unit vector (normalize)
vector3d vector3d::normal()
{
	vector3d temp;
	double length = sqrt(this->x*this->x + this->y*this->y + this->z*this->z);
	
	if (length == 1.0)
	{
		return *this;
	}

	if (length < 0.00001f)
		length = 0.00001f;
	
	temp.x = this->x/length;
	temp.y = this->y/length;
	temp.z = this->z/length;
	
	return temp;
}

// Multiply by a scalar
vector3d vector3d::operator * (double s)
{
	vector3d temp;
	temp.x = this->x*s;
	temp.y = this->y*s;
	temp.z = this->z*s;
	
	return temp;
}

vector3d vector3d::operator= (vector3d param)
{
	this->x = param.x;
	this->y = param.y;
	this->z = param.z;
	
	return *this;
}

// Equality operator
bool vector3d::operator==(const vector3d & rhs)
{
	double diffX, diffY, diffZ;
	
	diffX = this->x - rhs.x;
	diffY = this->y - rhs.y;
	diffZ = this->z - rhs.z;
	
	if ((diffX < EPSILON && diffX > -EPSILON) && (diffY < EPSILON && diffY > -EPSILON) && (diffZ < EPSILON && diffZ > -EPSILON))
	{
		return true;
	}
	else
	{
		return false;
	}
	
	/*if(this->x==rhs.x && this->y==rhs.y && this->z==rhs.z)
	{
		return true;
	}
	else
	{
		return false;	
	}*/
}

// Inequality operator
bool vector3d::operator!=(const vector3d & rhs)
{
	return !((*this)==rhs);
}

// self add, etc.
void vector3d::operator+=(const vector3d & rhs)
{
	this->x += rhs.x;
	this->y += rhs.y;
	this->z += rhs.z;
}

void vector3d::operator-=(const vector3d & rhs)
{
	this->x -= rhs.x;
	this->y -= rhs.y;
	this->z -= rhs.z;	
}

void vector3d::operator*=(const vector3d & rhs)
{
	this->x *= rhs.x;
	this->y *= rhs.y;
	this->z *= rhs.z;	
}

void vector3d::operator/=(const vector3d & rhs)
{
	this->x /= rhs.x;
	this->y /= rhs.y;
	this->z /= rhs.z;	
}

//unary operators
vector3d vector3d::operator-()
{
	vector3d temp;
	
	temp.x = -this->x;
	temp.y = -this->y;
	temp.z = -this->z;
	
	//return vector3d(-this->x, -this->y, -this->z);
	return temp;
}

vector3d vector3d::operator+()
{
	return *this;
}

//cast to pointer to a (double *) for glVertex3fv etc
vector3d::operator double* ()
{
	return (double*) this;
}

int vector3d::withinBounds(vector3d maxB, vector3d minB)
{
	int result = 0;
	
	if (this->x >= minB.x && this->x <= maxB.x)
	{
		if (this->z >= minB.z && this->z <= maxB.z)
		{
			result = 1;
		}
	}
	
	return result;
}

vector3d vector3d::perturb(double r, int w, int h)
{
	vector3d v, locR1, locR2;
	vector3d temp;
	double d = 2.0*r;
	
	vector3d max, min;
	
	max.set(w, 0.0, h);
	min.loadZero();
	
	do
	{
		// compute a random offset vector
		v.set(-r + d*(rand()/((double)RAND_MAX)), 0.0, -r + d*(rand()/((double)RAND_MAX)));
		
		// normalize 'v'
		v = v.normal();
		
		// move [0,r] in the direction of 'v' from 'this'
		temp = (*this) + v*((rand()/((double)RAND_MAX))*r);
		
	} while (!temp.withinBounds(max, min));
	
	//locR1 = this->roundVector();
	//locR2 = temp.roundVector();
	//printf("v (%.2f, %.2f) ... Original (%d,%d) vs. New (%d,%d)\n", v.x, v.z, (int)locR1.x, (int)locR1.z, (int)locR2.x, (int)locR2.z);
	//exit(-1);
	
	// return the new position
	return temp;
}

std::list<vector3d> vector3d::getBoundingVectors(std::list<vector3d>& listOfVectors)
{
	if (listOfVectors.size() == 1 || listOfVectors.empty())
	{
		return listOfVectors;
	}
	std::map<vector3d, int> sorter;
	std::list<vector3d> res;
	if (listOfVectors.size() == 2)
	{
		return listOfVectors;
	}
	int i = 0;
	for (auto it = listOfVectors.begin(); it != listOfVectors.end(); it++, i++)
	{
		sorter.insert(std::make_pair(*it, i));
	}
	sorter.insert(std::make_pair(*this, listOfVectors.size()));

	int e1, e2;
	auto begining = sorter.begin();
	auto ending = sorter.end();
	ending--;

	if (begining->second == listOfVectors.size())
	{
		e1 = (ending)->second;
		e2 = (begining++)->second;
	}
	else if (ending->second == listOfVectors.size())
	{
		e1 = (ending--)->second;
		e2 = (begining)->second;
	}	
	else
	{
		for (auto it = sorter.begin(); it != sorter.end(); it++)
		{
			if (listOfVectors.size() == it->second)
			{
				it++;
				e2 = it->second;
				break;
			}
			e1 = it->second;
		}
	}

	i = 0;
	for (auto it = listOfVectors.begin(); it != listOfVectors.end(); it++, i++)
	{
		if (i == e1 || i == e2)
		{
			res.push_back(*it);
		}
	}
	return res;
}