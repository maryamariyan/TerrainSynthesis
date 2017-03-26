#include "edge.h"

edge::edge(node_sp src, node_sp tgt) {
	n1 = src;
	n2 = tgt;
	isV = true;
	isF = n1.lock()->isFixed() && n2.lock()->isFixed();
	double base = src.get()->getDistanceFrom(tgt.get()->getLoc());
	wt = generateRandomEdgeWeight(base, 10.0*base);
	//wt = 0.0;
	loc = vector3d(
		((src->getLoc().x + tgt->getLoc().x)) * 0.5,
		0.5,//midset
		((src->getLoc().z + tgt->getLoc().z)) * 0.5);
	if (src->isDiagonalNeighborWith(tgt))
	{
		// y: zero for blue, one for pink
		if ((src->getLoc().x - tgt->getLoc().x)*(src->getLoc().z - tgt->getLoc().z) > 0.0)
		{
			//blue: bottomset
			loc.y = 0.0;
		}
		else
		{
			//pink: topset
			loc.y = 1.0;
		}
	}
}
edge::edge(node_sp src, node_sp tgt, bool isDual) {
	n1 = src;
	n2 = tgt;
	isV = true;
	isF = n1.lock()->isFixed() && n2.lock()->isFixed();
	wt = src.get()->getDistanceFrom(tgt.get()->getLoc());
	//wt = 0.0;
	loc = vector3d(
		((src->getLoc().x + tgt->getLoc().x)) * 0.5,
		0.5,//midset
		((src->getLoc().z + tgt->getLoc().z)) * 0.5);
	if (src->isDiagonalNeighborWith(tgt))
	{
		// y: zero for blue, one for pink
		if ((src->getLoc().x - tgt->getLoc().x)*(src->getLoc().z - tgt->getLoc().z) > 0.0)
		{
			//blue: bottomset
			loc.y = 0.0;
		}
		else
		{
			//pink: topset
			loc.y = 1.0;
		}
	}
}
edge::~edge() {
	;
}
double edge::generateRandomEdgeWeight(double base, double range) {
	// New Weight Calculation Method
	double wt, rTmp = (2.0f*(rand() / ((float)RAND_MAX))) - 1.0f;
	wt = base + (min(range, base - 0.1f)*rTmp);
	return wt;
}
node_wp edge::getSource() {
	return n1;
}
void edge::setSource(node_sp src) {
	n1 = src;
}
node_wp edge::getTarget() {
	return n2;
}
void edge::setTarget(node_sp tgt) {
	n2 = tgt;
}
node_wp edge::getDualNode() {
	return dualNode;
}
void edge::setDualNode(node_sp dn) {
	dualNode = dn;
}
double edge::getWeight() {
	return wt;
}
void edge::setWeight(double wgt) {
	wt = wgt;
}
bool edge::isValid() {
	return isV;
}
void edge::setValid(bool condition) {
	isV = condition;
}
bool edge::isFixed() {
	return isF;
}
void edge::setFixed(bool condition) {
	isF = condition;
}
vector3d edge::getLoc() {
	return loc;
}
void edge::setLoc(vector3d pos) {
	loc = pos;
}