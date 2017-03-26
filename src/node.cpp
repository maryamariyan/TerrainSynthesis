#include "node.h"

node::node(vector3d position, bool isFixed, size_t idx) {
	index = idx;
	pId = 0;
	dijkstraId = 0;
	loc = position;
	cost = MAX_FACT;
	cost2 = vector3d(MAX_FACT, 0.0, MAX_FACT);
	positionOnPath = 0;
	isF = isFixed;
	isV = true;
	isBlck = false;
	isBlu = false;
	isSpecialBlue = false;
	isRd = false;
	izBlocker = false;
	assert(rId.empty());
	rId.push_back(0);
}
node::node(vector3d position, bool isFixed, size_t idx, bool isDual) {
	index = idx;
	pId = 0;
	dijkstraId = 0;
	loc = position;
	cost = MAX_FACT;
	cost2 = vector3d(MAX_FACT, 0.0, MAX_FACT);
	positionOnPath = 0;
	isF = isFixed;
	isV = true;
	isBlck = false;
	isBlu = false;
	isSpecialBlue = false;
	isRd = false;
	izBlocker = false;
	assert(rId.empty());
	rId.push_back(0);
}
void node::createEdges(std::vector<node_sp> neighbors) {
	bool isOnBoundary = false;
	if (neighbors.size() == 3 || neighbors.size() == 5)
	{
		isOnBoundary = true;
	}
	for (auto neighbor : neighbors)
	{
		edges.push_back(std::make_shared<edge>(shared_from_this(), neighbor));
		if (isOnBoundary && 
			((neighbor->getLoc().x - shared_from_this()->getLoc().x) == 0 || (neighbor->getLoc().z - shared_from_this()->getLoc().z) == 0))
		{
			edges.back()->setWeight(10 * edges.back()->getWeight());
		}
	}
}
void node::createEdges(std::vector<node_sp> neighbors, bool isDual) {
	for (auto neighbor : neighbors)
	{
		edges.push_back(std::make_shared<edge>(shared_from_this(), neighbor, isDual));
	}
}
node::~node() {
	;
}
edges_sv node::getEdges() {
	return edges;
}
double node::getDistanceFrom(vector3d pos) {	
	return std::fabs(
		sqrt(
		pow(loc.x -pos.x, 2)
		+
		pow(loc.z -pos.z, 2)
		));
}
bool node::hasEdgeTo(node_sp other) {
	bool foundEdge = false;
	for (auto anEdge : edges)
	{
		if (anEdge->getTarget().lock() == other)
		{
			foundEdge = true;
		}
	}
	return foundEdge;
}
edge_sp node::getEdgeTo(node_sp other) {
	assert(hasEdgeTo(other));
	for (auto anEdge : edges)
	{
		if (anEdge->getTarget().lock() == other)
		{
			return anEdge;
		}
	}
}
bool node::isDiagonalNeighborWith(node_sp other) {
	return (fabs(shared_from_this()->getLoc().x - other->getLoc().x) == 1
		&& fabs(shared_from_this()->getLoc().z - other->getLoc().z) == 1);
}
vector3d node::getLoc() {
	return loc;
}
void node::setLoc(vector3d pos) {
	loc = pos;
}
double node::getCost() {
	return cost;
}
void node::setCost(double cst) {
	cost = cst;
}
vector3d node::getCost2() {
	return cost2;
}
void node::setCost2(vector3d cst2) {
	cost2 = cst2;
}
vector3d node::getOrientation() {
	return orientation;
}
void node::setOrientation(vector3d ornt) {
	orientation = ornt;
}
vector3d node::getPeakLocation() {
	return peakLocation;
}
void node::setPeakLocation(vector3d pkLoc) {
	peakLocation = pkLoc;
}
int node::getPositionOnPath() {
	return positionOnPath;
}
void node::setPositionOnPath(int pop) {
	positionOnPath = pop;
}
bool node::isValid() {
	return isV;
}
void node::setValid(bool condition) {
	isV = condition;
}
bool node::isFixed() {
	return isF;
}
void node::setFixed(bool condition) {
	isF = condition;
}
bool node::isBlack() {
	return isBlck;
}
void node::setBlack(bool condition) {
	isBlck = condition;
}
bool node::isBlocker() {
	return izBlocker;
}
void node::setBlocker(bool condition) {
	izBlocker = condition;
}
bool node::isBlue() {
	return isBlu;
}
void node::setBlue(bool condition) {
	isBlu = condition;
}
bool node::isConnectingTwoBlueRidges() {
	return isSpecialBlue;
}
void node::setConnectingTwoBlueRidges(bool condition) {
	isSpecialBlue = condition;
}
bool node::isRed() {
	return isRd;
}
void node::setRed(bool condition) {
	isRd = condition;
}
node_wp node::getSource() {
	return src;
}
bool node::hasSource() {
	return !src.expired();
}
void node::setSource(node_sp sourceNode) {
	src = sourceNode;
}
node_wp node::getPrevious() {
	return prev;
}
bool node::hasPrevious() {
	return !prev.expired();
}
void node::setPrev(node_sp previousNode) {
	prev = previousNode;
}
size_t node::getIndex() {
	return index;
}
void node::setIndex(size_t idx) {
	index = idx;
}
size_t node::getPatchId() {
	return pId;
}
void node::setPatchId(size_t idx) {
	pId = idx;
}
size_t node::getDijkstraId() {
	return dijkstraId;
}
void node::setDijkstraId(size_t idx) {
	dijkstraId = idx;
}
size_t node::getRidgeId() {
	//gives last written in container
	return rId.back();
}
void node::setRidgeId(size_t rdx) {
	//overwrites, empties container and adds rdx
	rId.erase(rId.begin(), rId.end());
	rId.push_back(rdx);
}
vector<size_t> node::getAllRidgeIds() {
	return rId;
}
void node::setOrAddRidgeId(size_t rdx) {
	rId.push_back(rdx);
}
bool node::setsOfRidgesOverlapWith(vector<size_t> otherSetsOfRidges) {
	for (auto anRId : rId)
	{
		for (auto anotherRId : otherSetsOfRidges)
		{
			if (anRId == anotherRId)
			{
				return true;
			}
		}
	}
	return false;
}
edge_sp node::getOriginalEdge() {
	return originalEdge;
}
void node::setOriginalEdge(edge_sp oe) {
	originalEdge = oe;
}
vector<dijkstra_elements> node::getDijkstraElements() {
	return djkElements;
}
void node::setDijkstraElements(vector<dijkstra_elements> djk) {
	djkElements = djk;
}