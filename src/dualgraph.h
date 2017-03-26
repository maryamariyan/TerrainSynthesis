#ifndef __DUALGRAPH_H
#define __DUALGRAPH_H

#include <map>
#include <list>
#include <set>

#include "edge.h"
#include "node.h"
#include "graph.h"

using namespace std;

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

class dual_graph : public enable_shared_from_this<dual_graph>
{
#define HSIZE 5;
protected:
	int numDijkstra;
	vector<node_sp> nodes;
	graph_wp originalGraph;
	vector<node_sp> fixedDualNodes;
	vector<size_t> solution, frontier;////////////////////on nodes
	node_sp getNodeFromIndex(size_t idx);
	void setLookAhead1(size_t maxHopSize);
	void setLookAhead2(size_t maxHopSize);
	void setLookAhead(size_t maxHopSize);
	void setInfinityToDiagonalEdges();
	double getValueFromOrientationNormal(node_wp curDualNode, vector3d orientationNormal);
public:
	dual_graph(size_t n, graph_sp& g);
	virtual ~dual_graph();
	void estimateWeights();
	void setNumDijkstra(int num);
	void incrementNumDijkstra();
	int getNumDijkstra();
};

#endif