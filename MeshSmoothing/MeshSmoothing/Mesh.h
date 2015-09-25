//http://blog.csdn.net/u013339596/article/details/19167359
///////////////////////////////////////////////////////////
#ifndef MESH_H
#define MESH_H
#define random(x) (rand()%x)
#include "Vertex.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
namespace ZLY{

struct Triangle{
public:
	typedef Triangle Self;

	Triangle(){	p0Index = -1; p1Index = -1; p2Index = -1; }
	Triangle(int p0Index, int p1Index, int p2Index){
		this->p0Index = p0Index;
		this->p1Index = p1Index;
		this->p2Index = p2Index;
	}

	bool hasVertex(int index){
		return p0Index == index || p1Index == index || p2Index == index;
	}

	int getOtherIndex(int i1, int i2){
		return p0Index + p1Index + p2Index - i1 - i2;
	}
public:
	int p0Index;
	int p1Index;
	int p2Index;
};

class Mesh{
public:
	int addVertex(Point3D<> &toAdd){
		int idx = Vertices.size();
		Vertices.push_back(toAdd);
		return idx;
	}
	int addFace(Triangle &tri){
		int idx = Faces.size();
		Faces.push_back(tri);
		return idx;
	}

	Normal<> calculateTriangleNormal(Point3D<> &p0, Point3D<> &p1, Point3D<> &p2);
	void calculateFaceNormals();
	void calculateVertexNormals();
	void calculateAdjacentFacesPerVertex();
	void calculateAdjacentVerteicesPerVertex();
	void disturb();
	//laplacian smooth
	void laplacianSmooth(int time, float lambda);
	/******************/

	//weighted laplacian smooth
public:
	void scaleDependentLaplacian(int time, float lambda = 1.0f);
private:
	Point3D<> getSmoothedVertex(int index, float lambda = 1.0f);
	float getWeight(int index, int adjIndex);
	/****************************/

	//cot weighted laplacian smooth
public:
	void curvatureFlowSmooth(int time);
private:
	float getCos(Point3D<> &ps, Point3D<> &pe1, Point3D<> &pe2);
	float getCot(Triangle &t, int index);
	float getWeight(int index, int adjIndex, std::vector<int> &adjVertices,
		std::vector<int> &adjFaces);
	Point3D<> getSmoothedVertex(int index, int);
	/******************************/
	
	//Taubin smooth
public:
	void taubinSmooth(int time, float lambda, float mu);
private:
	Point3D<> getSmoothedVertex_Taubin(int index, float lambda = 1.f);
	/**************/
	//HCLaplacian
public:
	void hcLaplacian(int time, float factor1, float factor2);
public:
	void proccess();

	void readObj(std::string &filename);
	void writeObj(std::string &filename);

public:
	std::vector<Point3D<>> Vertices;
	std::vector<Triangle> Faces;
	std::vector<Normal<>> VertexNormals;
	std::vector<Normal<>> FaceNormals;
	//存放点的邻接点集合
	std::vector<std::vector<int> *> adjacentVerticesPerVertex;
	//存放点的邻接面集合
	std::vector<std::vector<int> *> adjacentFacesPerVertex;
};

};

#endif