#ifndef VERTEX_H
#define VERTEX_H
#include "Vector.h"
#include <iostream>
#include <math.h>

namespace ZLY{
template<class Real = float>
struct Point3D{
public:
	typedef Point3D<Real> Self;

	Point3D(){ pos[0] = 0; pos[1] = 0; pos[2] = 0; }
	Point3D(const Real &x, const Real &y, const Real &z){
		this->pos[0] = x;
		this->pos[1] = y;
		this->pos[2] = z;
	}
	Point3D(Real (&x)[3]){
		this->pos[0] = x[0];
		this->pos[1] = x[1];
		this->pos[2] = x[2];
	}
	Point3D(Vector<Real> &v){ pos = v; }
	Point3D(const Self &inP){
		pos = inP.pos;
	}

	Real &operator[](int idx){	return pos[idx]; }
	const Real &operator[](int idx) const { return pos[idx]; }
	
	Self operator+(const Self &toAdd){ return ((*this).pos + toAdd.pos); }
	Self operator-(const Self &toSub){ return ((*this).pos - toSub.pos); }
	template<class Xtype>
	Self operator*(const Xtype &scale){
		Self ret = pos * scale;
		return ret;
	}
	template<class Xtype>
	Self operator/(const Xtype &scale){
		Self ret = pos / scale;
		return ret;
	}

	Real length(){
		return pos.length();
	}
private:
	Vector<Real> pos;
};

template<class Real = float>
struct Normal{
public:
	typedef Normal<Real> Self;

	Normal(){ normal[0] = 0; normal[1] = 0; normal[2] = 0; }
	Normal(const Real &x, const Real &y, const Real &z){
		this->normal[0] = x;
		this->normal[1] = y;
		this->normal[2] = z;
	}
	Normal(Real (&normal)[3]){
		this->normal[0] = normal[0];
		this->normal[1] = normal[1];
		this->normal[2] = normal[2];
	}
	Normal(Vector<Real> &n){ normal = n; }
	Normal(const Self &inNormal){
		normal = inNormal.normal;
	}

	Real &operator[](int idx){	return normal[idx]; }
	const Real &operator[](int idx) const { return normal[idx]; }

	Self operator+(const Self &toAdd){ return ((*this).normal+ toAdd.normal); }
	Self operator-(const Self &toSub){ return ((*this).normal - toSub.normal); }
	template<class Xtype>
	Self operator*(const Xtype &scale){
		Self ret = normal * scale;
		return ret;
	}
	template<class Xtype>
	Self operator/(const Xtype &scale){
		Self ret = normal / scale;
		return ret;
	}
	void normalize(){
		Real len = (Real)sqrt(normal[0] * normal[0] + 
							  normal[1] * normal[1] +
							  normal[2] * normal[2]);
		if(len == 0){
			std::cout<<"the length of normal is zero!"<<std::endl;
		}else{
			normal[0] /= len;
			normal[1] /= len;
			normal[2] /= len;
		}
	}
private:
	Vector<Real> normal;
};

};


#endif