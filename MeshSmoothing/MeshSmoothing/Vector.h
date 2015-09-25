#ifndef VECTOR_H
#define VECTOR_H
#include <math.h>
namespace ZLY{
template<class Real = float>
struct Vector{
public:
	typedef Vector<Real> Self;

	Vector(){
		x[0] = Real();
		x[1] = Real();
		x[2] = Real();
	}
	Vector(Real (&x)[3]){
		this->x[0] = x[0];
		this->x[1] = x[1];
		this->x[2] = x[2];
	}
	Vector(Real x, Real y, Real z){
		this->x[0] = x;
		this->x[1] = y;
		this->x[2] = z;
	}
	Vector(const Self &inV){
		x[0] = inV[0]; x[1] = inV[1]; x[2] = inV[2];
	}

	Real &operator[](int idx){
		return x[idx];
	}
	const Real &operator[](int idx) const{
		return x[idx];
	}

	Self operator+(const Self &toAdd){
		Self sum(*this);
		sum[0] += toAdd[0];
		sum[1] += toAdd[1];
		sum[2] += toAdd[2];
		return sum;
	}

	Self operator-(const Self &toSub){
		Self sub(*this);
		sub[0] -= toSub[0];
		sub[1] -= toSub[1];
		sub[2] -= toSub[2];
		return sub;
	}
	template<class Xtype>
	Self operator*(const Xtype &scale){
		Self ret(*this);
		ret[0] *= scale; ret[1] *= scale; ret[2] *= scale;
		return ret;
	}
	template<class Xtype>
	Self operator/(const Xtype &scale){
		Self ret(*this);
		ret[0] /= scale; ret[1] /= scale; ret[2] /= scale;
		return ret;
	}

	Real length(){
		return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
	}
private:
	Real x[3];
};
}

#endif