#ifndef __EDGE_H
#define __EDGE_H

#include <iostream>

#include <vector>
#include <algorithm>
#include <memory>
#include <assert.h>
#include <functional>
#include <queue>

#include <map>
#include <list>
#include "node.h"
#include "vector3d.h"

using namespace std;

#define MAX_FACT ((double)3.40282346638528860e+38)

class node;

typedef weak_ptr<node> node_wp;
typedef shared_ptr<node> node_sp;
typedef list<node_sp> nodes_t;

class edge;
typedef weak_ptr<edge> edge_wp;
typedef shared_ptr<edge> edge_sp;
typedef vector<edge_sp> edges_sv;
typedef vector<edge_wp> edges_wv;

class edge : public enable_shared_from_this<edge>
{
protected:
	node_wp n1;
	node_wp n2;
	node_wp dualNode;
	double wt;
	bool isF, isV;
	vector3d loc;
	double generateRandomEdgeWeight(double base, double range);
public:
	edge(node_sp src, node_sp tgt);
	edge(node_sp src, node_sp tgt, bool isDual);
	virtual ~edge();

	node_wp getSource();
	void setSource(node_sp src);
	node_wp getTarget();
	void setTarget(node_sp tgt);
	node_wp getDualNode();
	void setDualNode(node_sp dn);
	double getWeight();
	void setWeight(double wgt);
	bool isValid();
	void setValid(bool condition);
	bool isFixed();
	void setFixed(bool condition);
	vector3d getLoc();
	void setLoc(vector3d pos);
};

#endif
