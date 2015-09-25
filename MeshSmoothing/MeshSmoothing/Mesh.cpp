#include "Mesh.h"

using namespace ZLY;

Normal<> Mesh::calculateTriangleNormal(Point3D<> &p0, Point3D<> &p1, Point3D<> &p2){
	Normal<> normal;
	Point3D<> v1 = p1 - p0;
	Point3D<> v2 = p2 - p1;
	normal[0] = v1[1] * v2[2] - v1[2] * v2[1];
	normal[1] = v1[2] * v2[0] - v1[0] * v2[2];
	normal[2] = v1[0] * v2[1] - v1[1] * v2[0];
	normal.normalize();
	return normal;
}

void Mesh::calculateFaceNormals(){
	FaceNormals.reserve(Faces.size());
	for(int i = 0; i < (int)Faces.size(); ++i){
		Point3D<>& p0 = Vertices[Faces[i].p0Index];
		Point3D<>& p1 = Vertices[Faces[i].p1Index];
		Point3D<>& p2 = Vertices[Faces[i].p2Index];
		Normal<> normal = calculateTriangleNormal(p0, p1, p2);
		FaceNormals.push_back(normal);
	}
}

void Mesh::calculateVertexNormals(){
	if(FaceNormals.size() == 0)//计算点法线前需要计算面法线
		calculateFaceNormals();
	if(adjacentFacesPerVertex.size() == 0)//计算点法线前需要计算完邻接面
		calculateAdjacentFacesPerVertex();
	VertexNormals.reserve(Vertices.size());
	for(int i = 0; i < (int)Vertices.size(); ++i){
		Normal<> meanNormal;
		std::vector<int>& tlist = *(adjacentFacesPerVertex[i]);
		int size = (int)tlist.size();
		for(int j = 0; j < size; ++j){
			meanNormal = meanNormal + FaceNormals[tlist[j]];
		}
		//求邻接面法向的均值
		VertexNormals.push_back(Normal<>(meanNormal[0] / size, 
			meanNormal[1] / size, meanNormal[2] / size));
	}
}

void Mesh::calculateAdjacentFacesPerVertex(){
	adjacentFacesPerVertex.reserve(Vertices.size());
	for(int i = 0; i < (int)Vertices.size(); ++i){
		std::vector<int>* list = new std::vector<int>();
		list->reserve(4); //预先假设每个点有4个邻接面
		adjacentFacesPerVertex.push_back(list); //预先分配好存储空间
	}
	for(int i = 0; i < (int)Faces.size(); ++i){
		Triangle& t = Faces[i];
		std::vector<int> *t0list = adjacentFacesPerVertex[t.p0Index];
		std::vector<int> *t1list = adjacentFacesPerVertex[t.p1Index];
		std::vector<int> *t2list = adjacentFacesPerVertex[t.p2Index];
		t0list->push_back(i);
		t1list->push_back(i);
		t2list->push_back(i);
	}//遍历三角形集合，使用三角形信息补充邻接面表
}

void Mesh::calculateAdjacentVerteicesPerVertex(){
	adjacentVerticesPerVertex.reserve(Vertices.size());
	for(int i = 0; i < (int)Vertices.size(); ++i){
		std::vector<int>* list = new std::vector<int>();
		list->reserve(4);
		adjacentVerticesPerVertex.push_back(list);
	}//预分配存储空间
	for(int i = 0; i < (int)Faces.size(); ++i){
		Triangle &t = Faces[i];
		std::vector<int> *p0list = adjacentVerticesPerVertex[t.p0Index];
		std::vector<int> *p1list = adjacentVerticesPerVertex[t.p1Index];
		std::vector<int> *p2list = adjacentVerticesPerVertex[t.p2Index];
		if(std::find(p0list->begin(), p0list->end(), t.p1Index) == p0list->end()){
			p0list->push_back(t.p1Index);
		}
		if(std::find(p0list->begin(), p0list->end(), t.p2Index) == p0list->end()){
			p0list->push_back(t.p2Index);
		}
		
		if(std::find(p1list->begin(), p1list->end(), t.p0Index) == p1list->end()){
			p1list->push_back(t.p0Index);
		}
		if(std::find(p1list->begin(), p1list->end(), t.p2Index) == p1list->end()){
			p1list->push_back(t.p2Index);
		}

		if(std::find(p2list->begin(), p2list->end(), t.p0Index) == p2list->end()){
			p2list->push_back(t.p0Index);
		}
		if(std::find(p2list->begin(), p2list->end(), t.p1Index) == p2list->end()){
			p2list->push_back(t.p1Index);
		}
	}//遍历三角形集合，使用三角形信息补充点邻接表
}

void Mesh::disturb(){
	srand((int)time(0));
	for(int i = 0; i < (int)Vertices.size(); ++i){
		float scale = (random(100) - 50) / 500.f;
		Vertices[i][0] += VertexNormals[i][0] * scale;
		Vertices[i][1] += VertexNormals[i][1] * scale;
		Vertices[i][2] += VertexNormals[i][2] * scale;
	}
}

void Mesh::laplacianSmooth(int time, float lambda){
	if(adjacentVerticesPerVertex.size() == 0) //平滑前需要计算邻接点
		calculateAdjacentVerteicesPerVertex();
	Point3D<>* tmpPos = new Point3D<>[Vertices.size()];
	for(int k = 0; k < time; ++k){
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Point3D<> meanPos;
			std::vector<int>& adjlist = *(adjacentVerticesPerVertex[i]);
			int adjcount = (int)adjlist.size();
			if(adjcount == 0)
				continue;
			for(int j = 0; j < adjcount; ++j){
				meanPos = meanPos + Vertices[adjlist[j]];
			}
			tmpPos[i] = meanPos / adjcount;
			tmpPos[i] = Vertices[i] + (tmpPos[i] - Vertices[i]) * lambda;
		}//对所有点计算邻接点的平均位置
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Vertices[i] = tmpPos[i];
		}
	}//每次循环意味着一次平滑
	delete[] tmpPos;
}

void Mesh::scaleDependentLaplacian(int time, float lambda){
	if(adjacentVerticesPerVertex.size() == 0) //平滑前需要计算邻接点
		calculateAdjacentVerteicesPerVertex();
	Point3D<>* tmpList = new Point3D<>[Vertices.size()];
	for(int k = 0; k < time; ++k){
		for(int i = 0; i < (int)Vertices.size(); ++i){
			tmpList[i] = getSmoothedVertex(i, lambda);
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Vertices[i] = tmpList[i];
		}
	}
	delete[] tmpList;
}

Point3D<> Mesh::getSmoothedVertex(int index, float lambda){
	std::vector<int> &adjVertices = *(adjacentVerticesPerVertex[index]);
	if(adjVertices.size() == 0)
		return Vertices[index];
	float sumWeight = 0.f;
	Point3D<> d;
	for(int i = 0; i < (int)adjVertices.size(); ++i){
		float weight = getWeight(index, adjVertices[i]); 
		d = d + (Vertices[adjVertices[i]] - Vertices[index]) * weight;
		sumWeight += weight;
	}
	if(sumWeight < 1e-4)
		return Vertices[index];
	d = d / sumWeight;
	Point3D<> newPos = d * lambda + Vertices[index];
	return newPos;
}

float Mesh::getWeight(int index, int adjIndex){
	Point3D<> &p = Vertices[index];
	Point3D<> &t = Vertices[adjIndex];
	Point3D<> diff = p - t;
	float sq = sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);
	if(sq < 1e-6)
		return 1000.f;
	return 1.f / sq;
}

float Mesh::getCos(Point3D<> &ps, Point3D<> &pe1, Point3D<> &pe2){
	Point3D<> pse1 = pe1 - ps;
	Point3D<> pse2 = pe2 - ps;
	float mo1 = pse1.length();
	float mo2 = pse2.length();
	if(mo1 < 1e-8 || mo2 < 1e-8)
		return 0.f;
    float mul = pse1[0] * pse2[0] + pse1[1] * pse2[1] + pse1[2] * pse2[2];
    return __min(1, __max(-1, mul / (mo1 * mo2)));
}

float Mesh::getCot(Triangle &t, int index){
	float cosine;
	if(t.p0Index == index){
		cosine = getCos(Vertices[t.p0Index], Vertices[t.p1Index], Vertices[t.p2Index]);
	}else if(t.p1Index == index){
		cosine = getCos(Vertices[t.p1Index], Vertices[t.p0Index], Vertices[t.p2Index]);
	}else if(t.p2Index == index){
		cosine = getCos(Vertices[t.p2Index], Vertices[t.p1Index], Vertices[t.p0Index]);
	}
	if((1 - cosine * cosine) < 1e-4)
		return 2000.f;
	return cosine / sqrt(1 - cosine * cosine);
}

float Mesh::getWeight(int index, int adjIndex, std::vector<int> &adjVertices, 
		std::vector<int> &adjFaces){
	float w = 0.f;
	int count = 0;
	for(int i = 0; i < (int)adjFaces.size(); ++i){
		Triangle &t = Faces[adjFaces[i]];
		if(t.hasVertex(adjIndex)){
			int otherIndex = t.getOtherIndex(index, adjIndex);
			float cot = getCot(t, otherIndex);
			w += cot;
			count++;
		}
	}
	if(count == 0)
		return 0;
	w = w / count;
	return w;	
}

Point3D<> Mesh::getSmoothedVertex(int index, int){
	std::vector<int>& adjVertices = *(adjacentVerticesPerVertex[index]);
	std::vector<int>& adjFaces = *(adjacentFacesPerVertex[index]);
	if(adjVertices.size() == 0 || adjVertices.size() != adjFaces.size()){
		return Vertices[index];
	}
	float sumWeight = 0.f;
	Point3D<> d;
	for(int i = 0; i < (int)adjVertices.size(); ++i){
		float cotWeight = getWeight(index, adjVertices[i], adjVertices, adjFaces);
		d = d + (Vertices[adjVertices[i]] - Vertices[index]) * cotWeight;
		sumWeight += cotWeight;
	}
	if(sumWeight < 1e-4)
		return Vertices[index];
	d = d / sumWeight;
	Point3D<> newPos = d + Vertices[index];
	return newPos;
}

void Mesh::curvatureFlowSmooth(int time){
	if(adjacentVerticesPerVertex.size() == 0) //平滑前需要计算邻接点
		calculateAdjacentVerteicesPerVertex();
	if(adjacentFacesPerVertex.size() == 0)
		calculateAdjacentFacesPerVertex();
	Point3D<> *tmpList = new Point3D<>[Vertices.size()];
	for(int k = 0; k < time; ++k){
		for(int i = 0; i < (int)Vertices.size(); ++i){
			tmpList[i] = getSmoothedVertex(i, 0);
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Vertices[i] = tmpList[i];
		}
	}
	delete[] tmpList;
}

void Mesh::taubinSmooth(int time, float lambda, float mu){
	if(adjacentVerticesPerVertex.size() == 0) //平滑前需要计算邻接点
		calculateAdjacentVerteicesPerVertex();
	Point3D<>* tmpList = new Point3D<>[Vertices.size()];
	for(int k = 0; k < time; ++k){
		for(int i = 0; i < (int)Vertices.size(); ++i){
			tmpList[i] = getSmoothedVertex_Taubin(i, lambda);
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Vertices[i] = tmpList[i];
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			tmpList[i] = getSmoothedVertex_Taubin(i, mu);
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Vertices[i] = tmpList[i];
		}
	}
}

Point3D<> Mesh::getSmoothedVertex_Taubin(int index, float lambda){
	std::vector<int> &adjVertices = *(adjacentVerticesPerVertex[index]);
	if(adjVertices.size() == 0)
		return Vertices[index];
	Point3D<> d;
	for(int i = 0; i < (int)adjVertices.size(); ++i){
		d = d + (Vertices[adjVertices[i]] - Vertices[index]);
	}
	d = d / adjVertices.size();
	Point3D<> newPos = d * lambda + Vertices[index];
	return newPos;
}

void Mesh::hcLaplacian(int time, float factor1, float factor2){
	if(adjacentVerticesPerVertex.size() == 0) //平滑前需要计算邻接点
		calculateAdjacentVerteicesPerVertex();
	std::vector<Point3D<>> point_vector;
	std::vector<Point3D<>> startPoint;
	point_vector.resize(Vertices.size());
	startPoint.resize(Vertices.size());
	for(int i = 0; i < (int)Vertices.size(); ++i){
		startPoint[i] = Vertices[i];
	}
	for(int k = 0; k < time; ++k){
		for(int i = 0; i < (int)Vertices.size(); ++i){
			std::vector<int>& adjVertices = *(adjacentVerticesPerVertex[i]);
			Point3D<> meanPoint;
			for(int j = 0; j < (int)adjVertices.size(); ++j){
				meanPoint = meanPoint + Vertices[adjVertices[j]];
			}
			meanPoint = meanPoint / adjVertices.size();
			point_vector[i] = meanPoint - (startPoint[i] * factor1 + Vertices[i] * (1 - factor1));
			startPoint[i] = meanPoint;
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Vertices[i] = point_vector[i];
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			std::vector<int>& adjVertices = *(adjacentVerticesPerVertex[i]);
			Point3D<> meanPoint;
			for(int j = 0; j < (int)adjVertices.size(); ++j){
				meanPoint = meanPoint + Vertices[adjVertices[j]];
			}
			meanPoint = meanPoint * (1 - factor2) / adjVertices.size();
			point_vector[i] = startPoint[i] - (Vertices[i] * factor2 + meanPoint);
		}
		for(int i = 0; i < (int)Vertices.size(); ++i){
			Vertices[i] = point_vector[i];
		}
	}

	//std::vector<Point3D<>> p, b;
	//p.resize(Vertices.size());
	//b.resize(Vertices.size());
	//for(int i = 0; i < (int)Vertices.size(); ++i){
	//	p[i] = Vertices[i];
	//}
	//for(int k = 0; k < time; ++k){
	//	std::vector<Point3D<>> q;
	//	q.resize(p.size());
	//	for(int i = 0; i < (int)p.size(); ++i){
	//		q[i] = p[i];
	//	}
	//	for(int i = 0; i < (int)Vertices.size(); ++i){
	//		std::vector<int>& adjVertices = *(adjacentVerticesPerVertex[i]);
	//		if(adjVertices.size() == 0){
	//			p[i] = q[i];
	//		}else{
	//			Point3D<> meanPos;
	//			for(int j = 0; j < (int)adjVertices.size(); ++j){
	//				meanPos = meanPos + q[adjVertices[j]];
	//			}
	//			p[i] = meanPos / adjVertices.size();
	//		}
	//		b[i] = p[i] - (Vertices[i] * factor1 + q[i] * (1 - factor1));
	//	}
	//	for(int i = 0; i < (int)Vertices.size(); ++i){
	//		std::vector<int>& adjVertices = *(adjacentVerticesPerVertex[i]);
	//		Point3D<> mb;
	//		for(int j = 0; j < (int)adjVertices.size(); ++j){
	//			mb = mb + b[adjVertices[j]];
	//		}
	//		mb = mb / adjVertices.size();
	//		p[i] = p[i] - (b[i] * factor2 + mb * (1 - factor2));
	//	}
	//}
	//for(int i = 0; i < (int)Vertices.size(); ++i){
	//	Vertices[i] = p[i];
	//}
}

void Mesh::proccess(){
	////add noise
	//readObj(std::string("./model/box.obj"));
	//calculateVertexNormals();
	//disturb();
	//writeObj(std::string("./model/box1.obj"));
	
	//smooth
	//readObj(std::string("./model/s24p0.obj"));
	//laplacianSmooth(50, 0.5f);
	//writeObj(std::string("./model/laplacian.obj"));

	//readObj(std::string("./model/s24p0.obj"));
	//scaleDependentLaplacian(50);
	//writeObj(std::string("./model/weightedLaplacian.obj"));

	//readObj(std::string("./model/s24p0.obj"));
	//curvatureFlowSmooth(10);
	//writeObj(std::string("./model/cotLaplacian.obj"));

	//readObj(std::string("./model/s24p0.obj"));
	//taubinSmooth(200, 0.5f, -0.5f);
	//writeObj(std::string("./model/taubin.obj"));

	readObj(std::string("./model/s24p0.obj"));
	hcLaplacian(200, 0.1f, 0.9f);
	writeObj(std::string("./model/hcLaplacian.obj"));
}

void Mesh::readObj(std::string &filename){
	std::ifstream ifs(filename.c_str(), std::ifstream::in);
	if(!ifs.is_open()){
		std::cout<<"open file failed!"<<std::endl;
		return;
	}
	std::string line;
	while(std::getline(ifs, line)){
		if(line.length() == 0)
			continue;
		std::istringstream istr;
		istr.str(line.c_str());
		//std::cout<<istr.str()<<std::endl;
		float x, y, z;
		int idx0, idx1, idx2;
		char c;
		if(line[0] == '#')
			continue;
		if(line[0] == 'v'){
			istr>>c>>x>>y>>z;
			Vertices.push_back(Point3D<>(x, y, z));
		}
		if(line[0] == 'f'){
			istr>>c>>idx0>>idx1>>idx2;
			Faces.push_back(Triangle(idx0 - 1, idx1 - 1, idx2 - 1));
		}
	}
	ifs.close();
}

void Mesh::writeObj(std::string &filename){
	std::ofstream ofs(filename.c_str(), std::ofstream::out);
	if(!ofs.is_open()){
		std::cout<<"open file failed!"<<std::endl;
		return;
	}
	for(int i = 0; i < (int)Vertices.size(); ++i){
		ofs<<"v "<<Vertices[i][0]<<" "<<Vertices[i][1]<<" "<<Vertices[i][2]<<std::endl;
	}
	for(int i = 0; i < (int)Faces.size(); ++i){
		ofs<<"f "<<Faces[i].p0Index + 1<<" "<<Faces[i].p1Index + 1<<" "<<Faces[i].p2Index + 1<<std::endl;
	} 
	ofs.close();
}