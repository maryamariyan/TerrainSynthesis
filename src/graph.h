#ifndef __GRAPH_H
#define __GRAPH_H

#include "ofMain.h"
#include <map>
#include <list>

#include "edge.h"
#include "node.h"
#include "dualgraph.h"
#include "vector3d.h"
#include <math.h>

using namespace std;

class CompareByPositionOnPath;
class CompareByCost;
class CompareByDistance;

// if no incoming and no outgoing -> then flat points: point with edge weights eq zero to all neighbors
// if no incoming and with outgoing -> then peak and ordinary
// if with incoming and no outgoing -> discontinuity
// if with incoming and with outgoing -> ordinary
typedef enum ridge_point_status {
	UNIDENTIFIED_POINT = 0,
	DISCONTINUITY_POINT = 1,
	ORDINARY_POINT = 2,
	FLAT_POINT = 3
} ridge_point_status;
typedef enum path_status {
	START_OF_NEW_PATH = 0,
	CONTINUE_PATH_TO_NEXT = 1,
	CONTINUE_PATH_TO_PREV = 2

} path_status;
typedef enum weight_function {
	ONE_OVER_DISTANE = 0
} weight_function;

class vec2Key
{
public:
	double x, y;
	vec2Key()
	{
		x = 0.0;
		y = 0.0;
	}
	vec2Key(double xValue, double yValue)
	{
		x = xValue;
		y = yValue;
	}
	vec2Key(const ofPoint& p)
	{
		x = p.x;
		y = p.y;
	}
	bool operator < (const vec2Key& other) const
	{
		if (x == other.x) {
			return y < other.y;
		}
		return x < other.x;
	}
	double mag()
	{
		return sqrt(this->x*this->x + this->y*this->y);
	}
	vec2Key operator + (const vec2Key& other) const
	{
		return vec2Key(x + other.x, y + other.y);
	}
	vec2Key operator - (const vec2Key& other) const
	{
		return vec2Key(x - other.x, y - other.y);
	}
	vec2Key operator * (const double& other) const
	{
		return vec2Key(x * other, y * other);
	}
	vec2Key operator / (const double& other) const
	{
		return vec2Key(x / other, y / other);
	}
	double distanceFrom(const vec2Key& other) const
	{
		return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
	}
	bool withinBounds(const vec2Key& other) const
	{
		return (x >= 0 && y >= 0 && x<other.x && y<other.y);
	}
};

class node;
typedef weak_ptr<node> node_wp;
typedef shared_ptr<node> node_sp;
typedef list<node_sp> nodes_t;

class edge;
typedef weak_ptr<edge> edge_wp;
typedef shared_ptr<edge> edge_sp;
typedef vector<edge_sp> edges_sv;
typedef vector<edge_wp> edges_wv;

class graph;
typedef weak_ptr<graph> graph_wp;
typedef shared_ptr<graph> graph_sp;
typedef tuple<int, double, int> ridge_point; // x,h,z

class dual_graph;
typedef weak_ptr<dual_graph> dual_graph_wp;
typedef shared_ptr<dual_graph> dual_graph_sp;

class graph : public enable_shared_from_this<graph>
{
protected:
	vector<node_sp> nodes;
	dual_graph_sp dualGraph;
	int numDijkstra;
	//	size_t length;
public:
	graph(size_t n, map<int, ridge_point> pointsOnRidge);
	virtual ~graph(); 
	dual_graph_sp getDualGraph();
	void setDualGraph(dual_graph_sp dg);
	void createDual(size_t n);
//	void setInfinityToDiagonalEdges(size_t n);
	void DijkstraByCost(size_t n);
	void DijkstraByDistance(size_t n);
	void DijkstraByDistanceFindsRed(size_t n);
	void DijkstraByDistanceWithBlocker(size_t n);
	void Dijkstra3(size_t n);
	std::pair<vector<vector<size_t>>, vector<int>> DijkA(vector<int> indexesToGoInHeap, size_t n);
	void DijkB(int currentDijkElementsIndex, vector<int> indexesToGoInHeap, int currentPatchId, size_t n);
	vector<node_sp> getNodes();
//	node_sp getNodeFromLocation(vector3d crd, size_t n);
	node_sp getNodeFromIndex(size_t idx);
	size_t getIndex(vector3d crd, size_t n);
	bool withinBounds(vector3d v, int n);
	void calculateCostBasedOnHeight();
	void updateEdgeWeightsBasedOnCost();
	void setNumDijkstra(int num);
	void incrementNumDijkstra();
	int getNumDijkstra();

/*	void editEdgeWeights(map<int, ridge_point> pointsOnRidge, size_t n);
	void applyRidgeAnalysisCorrectedOnEdit(
		list<tuple<int, double, ridge_point_status>>& buffer,
		const map<int, ridge_point>& pointsOnRidge,
		map<pair<int, int>, list<tuple<int, int, vec2Key>>>& listOfGradients, size_t n);
	void distributeGradientsToNonFixedNodesDijkstra(
		const vector<ridge_point>& aPath, //x,h,z
		map<pair<int, int>, list<tuple<int, int, vec2Key>>>& listOfGradients, size_t n);
	void changeEdgeWeightTo(edge_sp fixedEdge, double newWeight);
	void changeEdgeWeights(const pair<int, int> position, list<tuple<int, int, vec2Key>>& listOfGradients, size_t n);
	double getWeightForSpecifiedDirectionBasedOnListOfGradients(list<vector3d>& gradients, vector3d directionUnit);
	double dotOfUnitsWithAngle(double angle);
*/
};

#endif