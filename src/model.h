#ifndef __MODEL_H
#define __MODEL_H

#include <map>
#include <list>

#include "edge.h"
#include "node.h"
#include "graph.h"
#include "dualgraph.h"
#include "vector3d.h"

using namespace std;

class CompareByPositionOnPath;
class CompareByCost;
class CompareByDistance;

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

class Model
{
#define MAX_EDGE_COST ((double)3.40282346638528860e+38)
#define MAX_VALUE ((double)3.40282346638528860e+38)
#define STEEPNESS_FACTOR 200
protected:
	graph_sp pGraph;
//	unique_ptr<graph> pGraph;
	vector<node_sp> gNodes;
	vector<size_t> solution;
	vector<size_t> frontier;
	map<int, ridge_point> pointsOnRidge;
	size_t length;
	graph_sp createGraph();
	vector<double> generateSinglePeakHeightmap();
	vector<double> setHeights();
	vector<double> setHeightsBackWithThreshold();
	map<int, ridge_point> getRidgeWithPeakOnBoundary();
	map<int, ridge_point> getRidgeWithOnePeakOnRightMid();
	map<int, ridge_point> getRidgeWithConstantHeight(); 
	map<int, ridge_point> getRidgeWithOneFixedEdge();
	map<int, ridge_point> getRidgeWithOnePeakAndTwoSlopesPerSide();
	map<int, ridge_point> getRidgeWithOnePeakAndSameSlopesOnEachSide();
	map<int, ridge_point> getRidgeMountFuji();

	void segmentRidgelinesToPaths();
	void doSegment(list<tuple<int, double, ridge_point_status>>& buffer, map<pair<int, int>, list<tuple<int, int, vec2Key>>>& listOfGradients);
	vector<double> tempSegmentRidgelinesToPaths();
	vector<double> tempDoSegment(list<tuple<int, double, ridge_point_status>>& buffer);
	vector<double> privateMethod1(vector<double> result, list<vector<ridge_point>> paths, bool isLastStage);
	vector<double> privateMethodCircular(vector<double> result, list<vector<ridge_point>> paths);
	vector<double> privateMethodDiamond(vector<double> result, list<vector<ridge_point>> paths);
	void makeyourownalgnow();
	vector<double> randomWalk(vector<int> indexes, double startingHeight);
	vector<double> testAverageRandomWalks(vector<int> indexes, double firstHeight, double lastHeight);
	double setupBlackRidgeHeights(vector<int> indexes, double maxDesired);
public:
	Model(size_t totalNodes);
	Model(size_t totalNodes, map<int, ridge_point> pointsOnRidge);
	virtual ~Model();
	vector<double> getResultingHeightfield();
	vector<double> getResultingHeightfield2();
	vector<double> getResultingHeightfield3();
};

#endif