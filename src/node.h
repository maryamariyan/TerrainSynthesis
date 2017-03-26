#ifndef __NODE_H
#define __NODE_H

#include <iostream>

#include <vector>
#include <algorithm>
#include <memory>
#include <assert.h>
#include <functional>
#include <queue>

#include <map>
#include <list>
#include "edge.h"
#include <math.h>
#include "vector3d.h"

using namespace std;

typedef std::tuple<double, double, double> coord;

#define MAX_FACT ((double)3.40282346638528860e+38)

typedef struct {
	double cost;
	double distFromSource;
} dijkstra_elements;

struct influence {//from A to B
	//	size_t hop_distance_x;//pop_x
	//	size_t hop_distance_y;//pop_y
	size_t pop;
	size_t index_cur_dual_node;//
	vector3d slope_orientation;//(ABx, ABy, SlopeA/sizeA)	//direction from fixed_edge to edge_of_interest (x,z) and proportional to fixed_edge_slope(y)
	size_t index_fixed_dual_node;
};
struct ByCostNodeStruct {
	double cost;
	size_t index;
};
struct ByPositionOnPathNodeStruct {
	int positionOnPath;
	size_t index;
};
struct ByDistanceNodeStruct {
	coord cost2;
	size_t index;
};
class CompareByPositionOnPath {
public:
	inline bool operator()(ByPositionOnPathNodeStruct& t1, ByPositionOnPathNodeStruct& t2)
	{
		return t1.positionOnPath > t2.positionOnPath;
	}
};
class CompareByCost {
public:
	inline bool operator()(ByCostNodeStruct& t1, ByCostNodeStruct& t2)
	{
		return t1.cost > t2.cost;
	}
};
class CompareByDistance {
public:
	inline bool operator()(ByDistanceNodeStruct& t1, ByDistanceNodeStruct& t2)
	{
		auto a1 = t1.cost2;
		auto a2 = t2.cost2;
		auto b01 = pow(std::get<0>(a1), 2);
		auto b21 = pow(std::get<2>(a1), 2);
		auto b02 = pow(std::get<0>(a2), 2);
		auto b22 = pow(std::get<2>(a2), 2);
		return b01 + b21 > b02 + b22;
		//return pow(std::get<0>(t1.cost2), 2) + pow(std::get<2>(t1.cost2), 2) > pow(std::get<0>(t2.cost2), 2) + pow(std::get<2>(t2.cost2), 2);
	}
};

class edge;
typedef weak_ptr<edge> edge_wp;
typedef shared_ptr<edge> edge_sp;
typedef vector<edge_sp> edges_sv;
typedef vector<edge_wp> edges_wv;

class node;
typedef std::weak_ptr<node> node_wp;
typedef std::shared_ptr<node> node_sp;
typedef std::list<node_sp> nodes_t;

class node : public enable_shared_from_this<node>
{
protected:
	edges_sv edges;
	vector3d loc;
	double cost;
	vector3d cost2;
	vector3d orientation;
	vector3d peakLocation;
	int positionOnPath;
	bool isF, isV, isBlck, isBlu, isRd, izBlocker, isSpecialBlue;
	size_t index, pId, dijkstraId;
	vector<size_t> rId;
	node_wp src;
	node_wp prev;
	edge_sp originalEdge;
	vector<dijkstra_elements> djkElements;
public:
	list<influence> inf;
	node(vector3d position, bool isFixed, size_t idx);
	node(vector3d position, bool isFixed, size_t idx, bool isDual);
	void createEdges(std::vector<node_sp> neighbors);
	void createEdges(std::vector<node_sp> neighbors, bool isDual);
	virtual ~node();
	edges_sv getEdges();

	double getDistanceFrom(vector3d pos);
	bool hasEdgeTo(node_sp other);
	edge_sp getEdgeTo(node_sp other);
	bool isDiagonalNeighborWith(node_sp other);

	vector3d getLoc();
	void setLoc(vector3d pos);
	double getCost();
	void setCost(double cst);
	vector3d getCost2();
	void setCost2(vector3d cst2);
	vector3d getOrientation();
	void setOrientation(vector3d ornt);
	vector3d getPeakLocation();
	void setPeakLocation(vector3d pkLoc);
	int getPositionOnPath();
	void setPositionOnPath(int pop);
	bool isValid();
	void setValid(bool condition);
	bool isFixed();
	void setFixed(bool condition);
	bool isBlocker();
	void setBlocker(bool condition);
	bool isBlack();
	void setBlack(bool condition);
	bool isBlue();
	void setBlue(bool condition);
	bool isConnectingTwoBlueRidges();
	void setConnectingTwoBlueRidges(bool condition);
	bool isRed();
	void setRed(bool condition);
	node_wp getSource();
	bool hasSource();
	void setSource(node_sp sourceNode);
	node_wp getPrevious();
	bool hasPrevious();
	void setPrev(node_sp previousNode);
	size_t getIndex();
	void setIndex(size_t idx);
	size_t getPatchId();
	void setPatchId(size_t idx);
	size_t getDijkstraId();
	void setDijkstraId(size_t idx);	
	size_t getRidgeId(); //gives last written in container
	void setRidgeId(size_t rdx); //overwrites, empties container and adds rdx
	vector<size_t> getAllRidgeIds();
	void setOrAddRidgeId(size_t rdx);
	bool setsOfRidgesOverlapWith(vector<size_t> otherSetsOfRidges);
	edge_sp getOriginalEdge();
	void setOriginalEdge(edge_sp oe);
	vector<dijkstra_elements> getDijkstraElements();
	void setDijkstraElements(vector<dijkstra_elements> djk);
};

#endif
