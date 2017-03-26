#include "graph.h"

using namespace std;

graph::graph(size_t n, map<int, ridge_point> pointsOnRidge)
{
	//analyse input
	numDijkstra = 0;
	vector<vector3d> positions;
	for (size_t i = 0; i < n; i++)
	{
		for (size_t j = 0; j < n; j++)
		{
			positions.push_back(vector3d(j, -0.2, i));
		}
	}
	for (auto rp : pointsOnRidge)
	{
		positions.at(n*std::get<2>(rp.second) + std::get<0>(rp.second)).y = std::get<1>(rp.second);
	}
	//create nodes
	for (auto pos : positions)
	{
		nodes.push_back(std::make_shared<node>(pos, pos.y >= 0.0, nodes.size()));
	}
	//create edges for each node
	for (auto pos : positions)
	{
		std::vector<node_sp> nbp;
		for (int k = -1; k <= 1; k++)
		{
			for (int l = -1; l <= 1; l++)
			{
				vector3d tmp = vector3d((double)l + pos.x, 0.0, (double)k + pos.z);
				if (withinBounds(tmp, n) && !(k == 0 && l == 0))
				{
					nbp.push_back(nodes.at(n*tmp.z + tmp.x));//as if getNodeFromLocation
				}
			}
		}
		auto curNode = nodes.at(n*pos.z + pos.x);//as if getNodeFromLocation
		curNode->createEdges(nbp);
	}
}
graph::~graph()
{
	;
}

dual_graph_sp graph::getDualGraph()
{
	return dualGraph;
}

void graph::setDualGraph(dual_graph_sp dg)
{
	dualGraph = dg;
}

void graph::createDual(size_t n)
{
	dualGraph = std::make_shared<dual_graph>(n, shared_from_this());
}

/*
get a fixed edge
populate a cone edge distribution around it
set cost then height based on the configured slope
visualize
*/

/*
get a fixed edge
populate a cone edge distribution around it - not quite; have the opposite end (in 180 degrees) with slope zero
set cost then height based on the configured slope
visualize
*/

void graph::DijkstraByDistance(size_t n)
{
	numDijkstra++;
	assert(n*n == nodes.size());
	vector<size_t> solution, frontier;
	priority_queue<ByDistanceNodeStruct, vector<ByDistanceNodeStruct>, CompareByDistance> min_heap;
	for (size_t i = 0; i < nodes.size(); i++)
	{
		solution.push_back(0);
		frontier.push_back(0);
	}
	for (auto aNode : nodes)
	{
		if (aNode->isFixed())
		{
			frontier.at(getIndex(aNode->getLoc(), n)) = 1;
			aNode->setPositionOnPath(0);
			aNode->setCost2(vector3d(0.0, 0.0, 0.0));
			auto tmpMV = aNode->getCost2();
			ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), aNode->getIndex() };
			min_heap.push(bdns); 
			aNode->setDijkstraId(numDijkstra);
			// hn->prev = NULL;
			// hn->source = NULL;
		}
	}
	vector3d location;
	node_sp min, otherEnd;
	while (!min_heap.empty())
	{
		ByDistanceNodeStruct bdns = (min_heap.top());
		size_t mv = bdns.index;
		min_heap.pop();
		min = getNodeFromIndex(mv);
		if (solution.at(getIndex(min->getLoc(), n)) == 1)
			continue;

		frontier.at(getIndex(min->getLoc(), n)) = 0;
		solution.at(getIndex(min->getLoc(), n)) = 1;

		for (auto theEdge : min->getEdges())
		{
			if (theEdge && !theEdge->getTarget().expired())
			{
				node_sp otherEnd = theEdge->getTarget().lock();
//				if (otherEnd)
				if (otherEnd && !otherEnd->isDiagonalNeighborWith(min))
				{
					assert(theEdge->getWeight() >= 0.0);
					double dist = min->getCost() + theEdge->getWeight();
					auto tmp = vector3d(
						min->getCost2().x + theEdge->getWeight() * fabs(min->getLoc().x - otherEnd->getLoc().x),
						0.0,
						min->getCost2().z + theEdge->getWeight() * fabs(min->getLoc().z - otherEnd->getLoc().z));
					double
						aaa = pow(otherEnd->getCost2().x, 2) + pow(otherEnd->getCost2().z, 2),
						bbb = pow(tmp.x, 2) + pow(tmp.z, 2);

					//if (dist < otherEnd->getCost())
					if (bbb < aaa)
					{
						if (min->isFixed())
						{
							otherEnd->setSource(min);
							min->setPositionOnPath(0);
							// min->prev = NULL;
							// min->source = NULL;
						}
						else
						{
							assert(!min->getSource().expired());
							otherEnd->setSource(min->getSource().lock());
						}
						otherEnd->setPositionOnPath(min->getPositionOnPath() + 1);
						otherEnd->setPrev(min);
						otherEnd->setCost(sqrt(bbb));
						otherEnd->setCost2(tmp);
						otherEnd->setRidgeId(min->getRidgeId());


						auto tmpMV = otherEnd->getCost2();
						ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), otherEnd->getIndex() };
						min_heap.push(bdns);
						otherEnd->setDijkstraId(numDijkstra);
						frontier.at(getIndex(otherEnd->getLoc(), n)) = 1;
					}
				}
			}
		}
	}
}

void graph::DijkstraByDistanceFindsRed(size_t n)
{
	numDijkstra++;
	assert(n*n == nodes.size());
	vector<size_t> solution, frontier;
	priority_queue<ByDistanceNodeStruct, vector<ByDistanceNodeStruct>, CompareByDistance> min_heap;
	for (size_t i = 0; i < nodes.size(); i++)
	{
		solution.push_back(0);
		frontier.push_back(0);
	}
	assert(min_heap.empty());
	for (auto aNode : nodes)
	{
		if (aNode->isFixed())
		{
			assert(!aNode->isBlack());
			assert(!aNode->isRed());
			frontier.at(getIndex(aNode->getLoc(), n)) = 1;
			aNode->setPositionOnPath(0);
			aNode->setCost2(vector3d(0.0, 0.0, 0.0));
			auto tmpMV = aNode->getCost2();
			ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), aNode->getIndex() };
			min_heap.push(bdns);
			aNode->setDijkstraId(numDijkstra);
			// hn->prev = NULL;
			// hn->source = NULL;
		}
		if (aNode->isBlack())
		{
			assert(aNode->isBlocker());
		}
		if (aNode->isBlocker())
		{
			assert(aNode->isBlack());
		}
	}
	vector3d location;
	node_sp min, otherEnd;
	while (!min_heap.empty())
	{
		ByDistanceNodeStruct bdns = (min_heap.top());
		size_t mv = bdns.index;
		min_heap.pop();
		min = getNodeFromIndex(mv);
		if (solution.at(getIndex(min->getLoc(), n)) == 1)
			continue;

		frontier.at(getIndex(min->getLoc(), n)) = 0;
		solution.at(getIndex(min->getLoc(), n)) = 1;

		if (min->isBlocker() || min->isRed())//if (min->isBlack())
		{
			//min->setCost(0.0);
			//min->setCost2(vector3d(0.0, 0.0, 0.0));
			//min->setFixed(true);
			continue;//assert false
		}

		for (auto theEdge : min->getEdges())
		{
			if (theEdge && !theEdge->getTarget().expired())
			{
				node_sp otherEnd = theEdge->getTarget().lock();
				//if (otherEnd)
				if (otherEnd && !otherEnd->isDiagonalNeighborWith(min))
				{
					assert(theEdge->getWeight() >= 0.0);
					double dist = min->getCost() + theEdge->getWeight();
					auto tmp = vector3d(
						min->getCost2().x + theEdge->getWeight() * fabs(min->getLoc().x - otherEnd->getLoc().x),
						0.0,
						min->getCost2().z + theEdge->getWeight() * fabs(min->getLoc().z - otherEnd->getLoc().z));
					double
						aaa = pow(otherEnd->getCost2().x, 2) + pow(otherEnd->getCost2().z, 2),
						bbb = pow(tmp.x, 2) + pow(tmp.z, 2);

					bool preCondition = true;
					if (otherEnd->isRed() || otherEnd->isBlocker())
					{
						preCondition = false;
					}
					//if (dist < otherEnd->getCost())
					if (preCondition && bbb < aaa)
					{
						if (min->isFixed())
						{
							otherEnd->setSource(min);
							min->setPositionOnPath(0);
							// min->prev = NULL;
							// min->source = NULL;
						}
						else
						{
							assert(!min->getSource().expired());
							otherEnd->setSource(min->getSource().lock());
						}
						otherEnd->setPositionOnPath(min->getPositionOnPath() + 1);
						otherEnd->setPrev(min);
						otherEnd->setCost(sqrt(bbb));
						otherEnd->setCost2(tmp);
						otherEnd->setRidgeId(min->getRidgeId());

						auto tmpMV = otherEnd->getCost2();
						ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), otherEnd->getIndex() };
						min_heap.push(bdns);
						otherEnd->setDijkstraId(numDijkstra);
						frontier.at(getIndex(otherEnd->getLoc(), n)) = 1;
					}
				}
			}
		}
	}
}

void graph::DijkstraByDistanceWithBlocker(size_t n)
{
	numDijkstra++;
	assert(n*n == nodes.size());
	vector<size_t> solution, frontier;
	priority_queue<ByDistanceNodeStruct, vector<ByDistanceNodeStruct>, CompareByDistance> min_heap;
	for (size_t i = 0; i < nodes.size(); i++)
	{
		solution.push_back(0);
		frontier.push_back(0);
	}
	for (auto aNode : nodes)
	{
		if (aNode->isFixed())
		{
			frontier.at(getIndex(aNode->getLoc(), n)) = 1;
			aNode->setPositionOnPath(0);
			aNode->setCost2(vector3d(0.0, 0.0, 0.0));
			auto tmpMV = aNode->getCost2();
			ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), aNode->getIndex() };
			min_heap.push(bdns);
			aNode->setDijkstraId(numDijkstra);
			// hn->prev = NULL;
			// hn->source = NULL;
		}
	}
	vector3d location;
	node_sp min, otherEnd;
	while (!min_heap.empty())
	{
		ByDistanceNodeStruct bdns = (min_heap.top());
		size_t mv = bdns.index;
		min_heap.pop();
		min = getNodeFromIndex(mv);
		if (solution.at(getIndex(min->getLoc(), n)) == 1)
			continue;

		frontier.at(getIndex(min->getLoc(), n)) = 0;
		solution.at(getIndex(min->getLoc(), n)) = 1;

		if (min->isBlocker())//if (min->isBlack())
		{
			//min->setCost(0.0);
			//min->setCost2(vector3d(0.0, 0.0, 0.0));
			//min->setFixed(true);
			continue;
		}

		for (auto theEdge : min->getEdges())
		{
			if (theEdge && !theEdge->getTarget().expired())
			{
				node_sp otherEnd = theEdge->getTarget().lock();
				//				if (otherEnd)
				if (otherEnd && !otherEnd->isDiagonalNeighborWith(min))
				{
					assert(theEdge->getWeight() >= 0.0);
					double dist = min->getCost() + theEdge->getWeight();
					auto tmp = vector3d(
						min->getCost2().x + theEdge->getWeight() * fabs(min->getLoc().x - otherEnd->getLoc().x),
						0.0,
						min->getCost2().z + theEdge->getWeight() * fabs(min->getLoc().z - otherEnd->getLoc().z));
					double
						aaa = pow(otherEnd->getCost2().x, 2) + pow(otherEnd->getCost2().z, 2),
						bbb = pow(tmp.x, 2) + pow(tmp.z, 2);

					//if (dist < otherEnd->getCost())
					if (bbb < aaa)
					{
						if (min->isFixed())
						{
							otherEnd->setSource(min);
							min->setPositionOnPath(0);
							// min->prev = NULL;
							// min->source = NULL;
						}
						else
						{
							assert(!min->getSource().expired());
							otherEnd->setSource(min->getSource().lock());
						}
						otherEnd->setPositionOnPath(min->getPositionOnPath() + 1);
						otherEnd->setPrev(min);
						otherEnd->setCost(sqrt(bbb));
						otherEnd->setCost2(tmp);
						otherEnd->setRidgeId(min->getRidgeId());

						auto tmpMV = otherEnd->getCost2();
						ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), otherEnd->getIndex() };
						min_heap.push(bdns);
						otherEnd->setDijkstraId(numDijkstra);
						frontier.at(getIndex(otherEnd->getLoc(), n)) = 1;
					}
				}
			}
		}
	}
}

pair<vector<vector<size_t>>, vector<int>> graph::DijkA(vector<int> indexesToGoInHeap, size_t n) {
	numDijkstra++;
	vector<int> nodesInPatch;
	vector<vector<size_t>> ridgeIdsContainer;
	assert(n*n == nodes.size());
	priority_queue<ByCostNodeStruct, vector<ByCostNodeStruct>, CompareByCost> min_heap;
	for (auto anIndex: indexesToGoInHeap)
	{
		auto aNodeA = nodes.at(anIndex);
		aNodeA->setFixed(true);
		aNodeA->setPositionOnPath(0);
		aNodeA->setCost(0.0);
		aNodeA->setCost2(vector3d(0.0, 0.0, 0.0));
		aNodeA->setSource(NULL);
		aNodeA->setPrev(NULL);
		assert(aNodeA->getPrevious().expired());
		assert(aNodeA->getSource().expired());

		ByCostNodeStruct bcns = { aNodeA->getCost(), aNodeA->getIndex() };
		min_heap.push(bcns);

		aNodeA->setDijkstraId(numDijkstra);
	}
	assert(min_heap.size() == 1);
	vector3d location;
	node_sp min, otherEnd;
	while (!min_heap.empty())
	{
		ByCostNodeStruct bdns = (min_heap.top());
		size_t mv = bdns.index;
		min_heap.pop();
		min = getNodeFromIndex(mv);
		if (min->getCost() < bdns.cost)//zombie check
		{
			continue;
		}
		min->setValid(true);
		nodesInPatch.push_back(min->getIndex());
		for (auto theEdge : min->getEdges())
		{
			if (theEdge && !theEdge->getTarget().expired())
			{
				node_sp otherEnd = theEdge->getTarget().lock();
				if (otherEnd)
				{
					bool is4Connected = !otherEnd->isDiagonalNeighborWith(min);
					bool isColored = otherEnd->isBlack() || otherEnd->isBlue() || otherEnd->isRed();
					bool wasPushedInCurrentHeapBefore = otherEnd->getDijkstraId() == numDijkstra;
					if (is4Connected && !isColored)
					{
						assert(theEdge->getWeight() >= 0.0);
						double dist = min->getCost() + theEdge->getWeight();
						if (!wasPushedInCurrentHeapBefore)
						{
							otherEnd->setFixed(false);
						}
						if (!wasPushedInCurrentHeapBefore || dist < otherEnd->getCost())
						{
							if (min->isFixed())
							{
								otherEnd->setSource(min);
								min->setPositionOnPath(0);
								// min->prev = NULL;
								// min->source = NULL;
							}
							else
							{
								assert(!min->getSource().expired());
								otherEnd->setSource(min->getSource().lock());
							}
							otherEnd->setPositionOnPath(min->getPositionOnPath() + 1);
							otherEnd->setPrev(min);
							otherEnd->setCost(dist);
							otherEnd->setRidgeId(min->getRidgeId());
							otherEnd->setPatchId(min->getPatchId());

							ByCostNodeStruct bcns = { otherEnd->getCost(), otherEnd->getIndex() };
							min_heap.push(bcns);
							otherEnd->setDijkstraId(numDijkstra);
						}
					}
					if (is4Connected && isColored && !otherEnd->isConnectingTwoBlueRidges())//colored ridge
					{
						bool canPushBack = true;//process list of ridge ids and see if new
						int timesOfOverLapCounter = 0, loopCounter = 0;
						int aCandidIndex = -1;
						for (auto ridgeIds : ridgeIdsContainer)
						{
							if (otherEnd->setsOfRidgesOverlapWith(ridgeIds))
							{
								canPushBack = false;
								timesOfOverLapCounter++;
								aCandidIndex = loopCounter;
							}
							loopCounter++;
						}
						assert(timesOfOverLapCounter <= 2);
						if (canPushBack)
						{
							assert(timesOfOverLapCounter == 0);
							ridgeIdsContainer.push_back(otherEnd->getAllRidgeIds());
						}
						else if (timesOfOverLapCounter == 1)
						{
							//add rest of ridge ids to the overlapped one
							assert(aCandidIndex != -1);
							auto chosenRIds = ridgeIdsContainer.at(aCandidIndex);
							for (auto myRId : otherEnd->getAllRidgeIds())
							{
								bool alreadyInElement = false;
								for (auto aChosenRId : chosenRIds)
								{
									if (myRId == aChosenRId)
									{
										alreadyInElement = true;
										continue;
									}
								}
								if (!alreadyInElement)
								{
									chosenRIds.push_back(myRId);
								}
							}
						}
						else
						{
							//merge two older elements and add rest of ridge ids to one and erase the other
							assert(timesOfOverLapCounter == 2);
							assert(aCandidIndex != -1);
							assert(false); //never happens
						}
					}
				}
			}
		}
	}
	return std::make_pair(ridgeIdsContainer, nodesInPatch);
}

void graph::DijkB(int currentDijkElementsIndex, vector<int> indexesToGoInHeap, int currentPatchId, size_t n) {
	numDijkstra++;
	assert(n*n == nodes.size());
	priority_queue<ByCostNodeStruct, vector<ByCostNodeStruct>, CompareByCost> min_heap;
	for (auto anIndex : indexesToGoInHeap)
	{
		auto aNodeA = nodes.at(anIndex);
		aNodeA->setFixed(true);
		aNodeA->setPositionOnPath(0);
		//aNodeA->setCost(0.0); //cost is already set before this method call
		aNodeA->setCost2(vector3d(0.0, 0.0, 0.0));
		aNodeA->setSource(NULL);
		aNodeA->setPrev(NULL);
		assert(aNodeA->getPrevious().expired());
		assert(aNodeA->getSource().expired());

		ByCostNodeStruct bcns = { aNodeA->getCost(), aNodeA->getIndex() };
		min_heap.push(bcns);

		aNodeA->setDijkstraId(numDijkstra);
	}
	vector3d location;
	node_sp min, otherEnd;
	while (!min_heap.empty())
	{
		ByCostNodeStruct bdns = (min_heap.top());
		size_t mv = bdns.index;
		min_heap.pop();
		min = getNodeFromIndex(mv);
		if (min->getCost() < bdns.cost)//zombie check
		{
			continue;
		}
		if (min->getPatchId() == currentPatchId)
		{
			assert(!min->isBlack() && !min->isBlue() && !min->isRed());
			auto newElement = min->getDijkstraElements();
			dijkstra_elements newToStore;
			newToStore.cost = min->getCost();
			newToStore.distFromSource = min->getPositionOnPath();
			newElement.push_back(newToStore);
			min->setDijkstraElements(newElement);
		}

		for (auto theEdge : min->getEdges())
		{
			if (theEdge && !theEdge->getTarget().expired())
			{
				node_sp otherEnd = theEdge->getTarget().lock();
				if (otherEnd)
				{
					bool is4Connected = !otherEnd->isDiagonalNeighborWith(min);
					bool isColored = otherEnd->isBlack() || otherEnd->isBlue() || otherEnd->isRed();
					assert(!isColored || otherEnd->getPatchId() == 0);
					bool wasPushedInCurrentHeapBefore = otherEnd->getDijkstraId() == numDijkstra;
					if (is4Connected && otherEnd->getPatchId() == currentPatchId)
					{
						assert(theEdge->getWeight() >= 0.0);
						double dist = min->getCost() + theEdge->getWeight();
						if (!wasPushedInCurrentHeapBefore)
						{
							otherEnd->setFixed(false);
						}
						if (!wasPushedInCurrentHeapBefore || dist < otherEnd->getCost())
						{
							if (min->isFixed())
							{
								otherEnd->setSource(min);
								min->setPositionOnPath(0);
								// min->prev = NULL;
								// min->source = NULL;
							}
							else
							{
								assert(!min->getSource().expired());
								otherEnd->setSource(min->getSource().lock());
							}
							otherEnd->setPositionOnPath(min->getPositionOnPath() + 1);
							otherEnd->setPrev(min);
							otherEnd->setCost(dist);

							ByCostNodeStruct bcns = { otherEnd->getCost(), otherEnd->getIndex() };
							min_heap.push(bcns);
							otherEnd->setDijkstraId(numDijkstra);
						}
					}
				}
			}
		}
	}
}

//incorrect. wrong
void graph::Dijkstra3(size_t n) {
	;

	//priority = costM + sqrt(yI^2 + xI^2)
	//cost3 = tuple = (costM, yI, xI)
}

void graph::DijkstraByCost(size_t n)
{
	numDijkstra++;
	assert(n*n == nodes.size());
	vector<size_t> solution, frontier;
	priority_queue<ByCostNodeStruct, vector<ByCostNodeStruct>, CompareByCost> min_heap;
	for (auto aNode : nodes)
	{
		solution.push_back(0);
		frontier.push_back(0);
	}
	for (auto aNode : nodes)
	{
		if (aNode->isFixed())
		{
			frontier.at(getIndex(aNode->getLoc(), n)) = 1;
			ByCostNodeStruct bcns = { aNode->getCost(), aNode->getIndex() };
			min_heap.push(bcns);
			aNode->setDijkstraId(numDijkstra);
			aNode->setPositionOnPath(0);
			// hn->prev = NULL;
			// hn->source = NULL;
		}
	}
	vector3d location;
	node_sp min, otherEnd;
	while (!min_heap.empty())
	{
		ByCostNodeStruct bcns = (min_heap.top());
		size_t mv = bcns.index;
		min_heap.pop();
		min = getNodeFromIndex(mv);
		if (solution.at(getIndex(min->getLoc(), n)) == 1)
			continue;

		frontier.at(getIndex(min->getLoc(), n)) = 0;
		solution.at(getIndex(min->getLoc(), n)) = 1;

		for (auto theEdge : min->getEdges())
		{
			if (theEdge && !theEdge->getTarget().expired())
			{
				node_sp otherEnd = theEdge->getTarget().lock();
				if (otherEnd)
				{
					assert(theEdge->getWeight() >= 0.0);
					double dist = min->getCost() + theEdge->getWeight();
					if (dist < otherEnd->getCost())
					{
						if (min->isFixed())
						{
							otherEnd->setSource(min);
							min->setPositionOnPath(0);
							// min->prev = NULL;
							// min->source = NULL;
						}
						else
						{
							assert(!min->getSource().expired());
							otherEnd->setSource(min->getSource().lock());
						}
						otherEnd->setPositionOnPath(min->getPositionOnPath() + 1);
						otherEnd->setPrev(min);
						otherEnd->setCost(dist);
						otherEnd->setRidgeId(min->getRidgeId());

						ByCostNodeStruct bcns = { otherEnd->getCost(), otherEnd->getIndex() };
						min_heap.push(bcns);
						otherEnd->setDijkstraId(numDijkstra);
						frontier.at(getIndex(otherEnd->getLoc(), n)) = 1;
					}
				}
			}				
		}
	}
}

vector<node_sp> graph::getNodes() {
		return nodes;
	}
node_sp graph::getNodeFromIndex(size_t idx) {
	return nodes.at(idx);
}
size_t graph::getIndex(vector3d crd, size_t n) {
	return n*crd.z + crd.x;
	}
bool graph::withinBounds(vector3d v, int n)
	{
		if (v.x < 0.0 || v.x >= (double)n)
		{
			return false;
		}
		if (v.z < 0.0 || v.z >= (double)n)
		{
			return false;
		}
		return true;
	}
void graph::calculateCostBasedOnHeight() 
{
	double maxFixedHeightFound = -1.0;
	bool foundFixed;
	for (auto aNode : nodes)
	{
		if (aNode->isFixed())
		{
			auto nodeLocY = aNode->getLoc().y;
			if (nodeLocY > maxFixedHeightFound)
			{
				foundFixed = true;
				maxFixedHeightFound = nodeLocY;
			}
		}
	}
	assert(foundFixed);
	assert(maxFixedHeightFound != -1.0);
	for (auto aNode : nodes)
	{
		if (aNode->isFixed())
		{
			aNode->setCost(fabs(maxFixedHeightFound - aNode->getLoc().y));
		}
	}
}

void graph::updateEdgeWeightsBasedOnCost() {
	for (auto& aNode : nodes) {
		for (auto theEdge : aNode->getEdges()) {
			if (!theEdge->isFixed())
			{
				theEdge->setWeight(theEdge->getDualNode().lock()->getCost());
			}
		}
	}
}

void graph::setNumDijkstra(int num) {
	numDijkstra = num;
}
void graph::incrementNumDijkstra() {
	numDijkstra++;
}
int graph::getNumDijkstra() {
	return numDijkstra;
}

/*
void graph::editEdgeWeights(map<int, ridge_point> pointsOnRidge, size_t n) {
		map<pair<int, int>, list<tuple<int, int, vec2Key>>> distributedGradients;
		list<tuple<int, double, ridge_point_status>> buffer;
		for (auto rp : pointsOnRidge)
		{
			buffer.push_back(std::make_tuple(rp.first, std::get<1>(rp.second), ridge_point_status::UNIDENTIFIED_POINT));
		}
		applyRidgeAnalysisCorrectedOnEdit(buffer, pointsOnRidge, distributedGradients, n);
	}
void graph::applyRidgeAnalysisCorrectedOnEdit(
		list<tuple<int, double, ridge_point_status>>& buffer,
		const map<int, ridge_point>& pointsOnRidge,
		map<pair<int, int>, list<tuple<int, int, vec2Key>>>& listOfGradients, size_t n)
	{
		if (buffer.empty() || buffer.size() == 1)
		{
			return;
		}
		vector<vector<int>> simplePathContainer;
		list<int> currentPath;
		int currentPeakNumber;

		auto pNext = buffer.begin();
		pNext++;
		auto pCurrent = buffer.begin();
		int p_status = path_status::START_OF_NEW_PATH;
		for (; pNext != buffer.end(); pCurrent++, pNext++)
		{
			if (std::get<1>(*pCurrent) - std::get<1>(*pNext) < 0)
			{
				if (p_status != path_status::CONTINUE_PATH_TO_PREV)
				{
					if (p_status == path_status::CONTINUE_PATH_TO_NEXT)
					{
						//empty current path and set old path as final
						assert(*currentPath.begin() == currentPeakNumber);
						vector<int> vectorOfInt;
						for (auto it = currentPath.begin(); it != currentPath.end(); it++)
						{
							vectorOfInt.push_back(*it);
						}
						simplePathContainer.push_back(vectorOfInt);
						currentPath.erase(currentPath.begin(), currentPath.end());
						assert(currentPath.empty());

						//then:
						std::get<2>(*pCurrent) = ridge_point_status::DISCONTINUITY_POINT;
						assert(std::get<2>(*pCurrent) == ridge_point_status::DISCONTINUITY_POINT);
						currentPath.push_front(std::get<0>(*pCurrent));
					}
					else if (p_status == path_status::START_OF_NEW_PATH)//and current path not empty
					{
						//update current path and insert all inside it as if they were all inserted the other way
						currentPath.erase(currentPath.begin(), currentPath.end());
						assert(currentPath.empty());
						for (auto it = buffer.begin(); it != pCurrent; it++)
						{
							currentPath.push_front(std::get<0>(*it));
						}

						//then:
						currentPath.push_front(std::get<0>(*pCurrent));
					}
					p_status = path_status::CONTINUE_PATH_TO_PREV;
				}
				//insert in current path

				currentPath.push_front(std::get<0>(*pNext));
				currentPeakNumber = std::get<0>(*pNext);

				if (false)// (fabs((double)(std::get<0>(*pNext) - std::get<0>(*pCurrent))) == 1)
				{
					auto xhzInfoNext = pointsOnRidge.find(std::get<0>(*pNext))->second;
					auto xhzInfoCurrent = pointsOnRidge.find(std::get<0>(*pCurrent))->second;
					auto hnNext = getNodeFromLocation(vector3d(std::get<0>(xhzInfoNext), 0.0, std::get<2>(xhzInfoNext)), n);
					auto hnCurrent = getNodeFromLocation(vector3d(std::get<0>(xhzInfoCurrent), 0.0, std::get<2>(xhzInfoCurrent)), n);
					assert(hnNext->hasEdgeTo(hnCurrent));
					auto fixedEdge = hnNext->getEdgeTo(hnCurrent);
					changeEdgeWeightTo(fixedEdge, fabs((double)(hnNext->getCost() - hnCurrent->getCost())));
					fixedEdge->setValid(true);
					fixedEdge = hnCurrent->getEdgeTo(hnNext);
					fixedEdge->setValid(true);
				}
			}
			else if (std::get<1>(*pCurrent) - std::get<1>(*pNext) > 0)
			{
				if (p_status != path_status::CONTINUE_PATH_TO_NEXT)
				{
					if (p_status == path_status::CONTINUE_PATH_TO_PREV)
					{
						//empty current path and set old path as final
						assert(currentPeakNumber == std::get<0>(*pCurrent));
						vector<int> vectorOfInt;
						for (auto it = currentPath.begin(); it != currentPath.end(); it++)
						{
							vectorOfInt.push_back(*it);
						}
						simplePathContainer.push_back(vectorOfInt);
						currentPath.erase(currentPath.begin(), currentPath.end());
						assert(currentPath.empty());

						//then:
						currentPeakNumber = std::get<0>(*pCurrent);
						currentPath.push_back(std::get<0>(*pCurrent));
					}
					else if (p_status == path_status::START_OF_NEW_PATH)//and current path not empty
					{
						//update current path and insert all inside it as if they were all inserted the other way
						currentPath.erase(currentPath.begin(), currentPath.end());
						assert(currentPath.empty());
						for (auto it = buffer.begin(); it != pCurrent; it++)
						{
							currentPath.push_back(std::get<0>(*it));
						}
						currentPeakNumber = std::get<0>(*buffer.begin());
						//then:
						currentPath.push_back(std::get<0>(*pCurrent));
					}
					p_status = path_status::CONTINUE_PATH_TO_NEXT;
				}
				//insert in current path

				currentPath.push_back(std::get<0>(*pNext));

				if (false)// (fabs((double)(std::get<0>(*pNext) - std::get<0>(*pCurrent))) == 1)
				{
					auto xhzInfoNext = pointsOnRidge.find(std::get<0>(*pNext))->second;
					auto xhzInfoCurrent = pointsOnRidge.find(std::get<0>(*pCurrent))->second;
					auto hnNext = getNodeFromLocation(vector3d(std::get<0>(xhzInfoNext), 0.0, std::get<2>(xhzInfoNext)), n);
					auto hnCurrent = getNodeFromLocation(vector3d(std::get<0>(xhzInfoCurrent), 0.0, std::get<2>(xhzInfoCurrent)), n);
					assert(hnNext->hasEdgeTo(hnCurrent));
					auto fixedEdge = hnNext->getEdgeTo(hnCurrent);
					changeEdgeWeightTo(fixedEdge, fabs((double)(hnNext->getCost() - hnCurrent->getCost())));
					fixedEdge->setValid(true);
					fixedEdge = hnCurrent->getEdgeTo(hnNext);
					fixedEdge->setValid(true);
				}
			}
			else
			{
				assert(std::get<1>(*pCurrent) - std::get<1>(*pNext) == 0);
				if (p_status == path_status::START_OF_NEW_PATH)
				{
					//assume CONTINUE_PATH_TO_PREV, unless identified otherwise

					currentPath.push_front(std::get<0>(*pCurrent));
				}
				else if (p_status == path_status::CONTINUE_PATH_TO_NEXT)
				{
					currentPath.push_back(std::get<0>(*pNext));
				}
				else
				{
					assert(p_status == path_status::CONTINUE_PATH_TO_PREV);

					currentPath.push_front(std::get<0>(*pNext));
					currentPeakNumber = std::get<0>(*pNext);
				}
				if (false)// (fabs((double)(std::get<0>(*pNext) - std::get<0>(*pCurrent))) == 1)
				{
					auto xhzInfoNext = pointsOnRidge.find(std::get<0>(*pNext))->second;
					auto xhzInfoCurrent = pointsOnRidge.find(std::get<0>(*pCurrent))->second;
					auto hnNext = getNodeFromLocation(vector3d(std::get<0>(xhzInfoNext), 0.0, std::get<2>(xhzInfoNext)), n);
					auto hnCurrent = getNodeFromLocation(vector3d(std::get<0>(xhzInfoCurrent), 0.0, std::get<2>(xhzInfoCurrent)), n);
					assert(hnNext->hasEdgeTo(hnCurrent));
					auto fixedEdge = hnNext->getEdgeTo(hnCurrent);
					changeEdgeWeightTo(fixedEdge, fabs((double)(hnNext->getCost() - hnCurrent->getCost())));
					fixedEdge->setValid(true);
					fixedEdge = hnCurrent->getEdgeTo(hnNext);
					fixedEdge->setValid(true);
				}
			}
		}

		if (!currentPath.empty())
		{
			vector<int> vectorOfInt;
			switch (p_status)
			{

			case path_status::START_OF_NEW_PATH:
				currentPath.erase(currentPath.begin(), currentPath.end());
				assert(currentPath.empty());
				for (auto it = buffer.begin(); it != buffer.end(); it++)
				{
					currentPath.push_back(std::get<0>(*it));
				}
				currentPeakNumber = std::get<0>(*buffer.begin());
				// break;
			case path_status::CONTINUE_PATH_TO_NEXT:
				assert(*currentPath.begin() == currentPeakNumber);
				for (auto it = currentPath.begin(); it != currentPath.end(); it++)
				{
					vectorOfInt.push_back(*it);
				}
				simplePathContainer.push_back(vectorOfInt);
				currentPath.erase(currentPath.begin(), currentPath.end());
				assert(currentPath.empty());
				std::get<2>(buffer.back()) = ridge_point_status::DISCONTINUITY_POINT;
				assert(std::get<2>(buffer.back()) == ridge_point_status::DISCONTINUITY_POINT);
				break;

			case path_status::CONTINUE_PATH_TO_PREV:
				assert(currentPeakNumber == std::get<0>(*pCurrent));
				for (auto it = currentPath.begin(); it != currentPath.end(); it++)
				{
					vectorOfInt.push_back(*it);
				}
				simplePathContainer.push_back(vectorOfInt);
				currentPath.erase(currentPath.begin(), currentPath.end());
				assert(currentPath.empty());
				break;
			}
		}

		// remove flat and ordinary (includes peaks) points from buffer			
		auto new_end = std::remove_if(buffer.begin(), buffer.end(),
			[](const tuple<int, double, ridge_point_status>& value)
		{ return std::get<2>(value) != ridge_point_status::DISCONTINUITY_POINT; });
		buffer.erase(new_end, buffer.end());

		//change all disc to undef
		for (auto it = buffer.begin(); it != buffer.end(); it++)
		{
			std::get<2>(*it) = ridge_point_status::UNIDENTIFIED_POINT;
		}

		list<vector<ridge_point>> paths;
		for (auto aPath = simplePathContainer.begin(); aPath < simplePathContainer.end(); aPath++)	// paths
		{
			vector<ridge_point> vectorOfTuples;
			for (auto aPointInPath = aPath->begin(); aPointInPath < aPath->end(); aPointInPath++) // ridge points in path
			{
				vectorOfTuples.push_back(pointsOnRidge.find(*aPointInPath)->second);
			}
			paths.push_back(vectorOfTuples);
		}

		//			applyRidgeAnalysisCorrectedOnEdit(buffer, pointsOnRidge, listOfGradients, n);

		for (auto aPath = paths.begin(); aPath != paths.end(); aPath++)
		{
			// implement nodes within heap distance.
			distributeGradientsToNonFixedNodesDijkstra(*aPath, listOfGradients, n);
		}

		for (int yPos = 0; yPos < n; yPos++)
		{
			for (int xPos = 0; xPos < n; xPos++)
			{
				pair<int, int> vector3d = std::make_pair(xPos, yPos);
				list<tuple<int, int, vec2Key>> listOfGradientsForCurrentPosition;
				if (listOfGradients.find(vector3d) != listOfGradients.end())
				{
					listOfGradientsForCurrentPosition = listOfGradients.find(vector3d)->second;
				}
				//as if	//	changeEdgeWeights(vector3d, listOfGradientsForCurrentPosition, n);
			}
		}
	}
	void graph::distributeGradientsToNonFixedNodesDijkstra(
		const vector<ridge_point>& aPath, //x,h,z
		map<pair<int, int>, list<tuple<int, int, vec2Key>>>& listOfGradients, size_t n)
	{
		assert(aPath.size() >= 2);
		std::priority_queue<ByDistanceNodeStruct, vector<ByDistanceNodeStruct>, CompareByDistance> my_min_heap;

		map<pair<int, int>, ridge_point> referenceToPath;
		map <pair<int, int>, double> referenceToSlope;
		tuple<int, int, vec2Key> aGradientInfo;

		auto pNext = aPath.begin();
		pNext++;
		auto pCurrent = aPath.begin();
		auto pPeak = aPath.begin();
		for (; pNext < aPath.end(); pCurrent++, pNext++)
		{
			ridge_point current = *pCurrent;
			ridge_point next = *pNext;
			ridge_point peak = *pPeak;
			vec2Key currentMinusPeak = vec2Key(std::get<0>(current) -std::get<0>(peak), std::get<2>(current) -std::get<2>(peak));
			vec2Key nextMinusPeak = vec2Key(std::get<0>(next) -std::get<0>(peak), std::get<2>(next) -std::get<2>(peak));
			vec2Key nextMinusCurrent = nextMinusPeak - currentMinusPeak;

			double distance = sqrt(pow(nextMinusCurrent.x, 2) + pow(nextMinusCurrent.y, 2));

			vec2Key gradientUnitVector = vec2Key(nextMinusCurrent.x / distance, nextMinusCurrent.y / distance);
			referenceToSlope.insert(std::make_pair(std::make_pair(std::get<0>(current), std::get<2>(current)), (std::get<1>(current) -std::get<1>(next)) / distance));
		}

		ridge_point peak = *pPeak;
		double farthest = -1.0;
		for (auto pCurrent = aPath.begin(); pCurrent < aPath.end(); pCurrent++)
		{
			ridge_point current = *pCurrent;
			vec2Key currentMinusPeak = vec2Key(std::get<0>(current) -std::get<0>(peak), std::get<2>(current) -std::get<2>(peak));
			referenceToPath.insert(std::make_pair(std::make_pair(fabs(currentMinusPeak.x), fabs(currentMinusPeak.y)), current));
			farthest = max(sqrt(pow(currentMinusPeak.x, 2) + pow(currentMinusPeak.y, 2)), farthest);
		}
		assert(farthest != -1.0);
		int upperBound = farthest - (long)farthest;
		for (int i = -upperBound; i <= +upperBound; i++)
		{
			for (int j = -upperBound; j <= +upperBound; j++)
			{
				vec2Key gsize = vec2Key(n, n);
				double dist = sqrt(pow(j, 2) + pow(i, 2));
				pair<int, int> pointOfInterest = std::make_pair(std::get<0>(peak) +j, std::get<2>(peak) +i);
				auto posM = vector3d(std::get<0>(pointOfInterest), 0.0, std::get<1>(pointOfInterest));
				auto posMMVECTOR3 = vector3d(std::get<0>(pointOfInterest), 0.0, std::get<1>(pointOfInterest));

				if (dist < upperBound && withinBounds(posMMVECTOR3, n))
				{
					auto hn = getNodeFromLocation(posMMVECTOR3, n);
					auto iter = referenceToPath.find(std::make_pair(posM.x, posM.z));
					if (iter == referenceToPath.end()
						|| !(std::get<0>(iter->second) == std::get<0>(hn->getLoc()) && std::get<2>(iter->second) == std::get<2>(hn->getLoc())))
					{
						hn->setCost2(vector3d(j, 0.0, i));
						ByDistanceNodeStruct bdns = { hn->getCost2(), hn->getLoc() };
						my_min_heap.push(bdns);
					}
				}
			}
		}

		vector3d location;
		node_sp min, otherEnd;
		while (!my_min_heap.empty())
		{
			ByDistanceNodeStruct bdns = (my_min_heap.top());
			vector3d mv = bdns.location;
			my_min_heap.pop();
			min = getNodeFromLocation(mv, n);

			pair<int, int> pointOfInterest = std::make_pair(std::get<0>(mv), std::get<2>(mv));
			double slope;
			vector3d anotherUnitVector;
			if (referenceToPath.find(std::make_pair(std::get<0>(min->getCost2()), std::get<2>(min->getCost2()))) != referenceToPath.end())
			{
				auto current = referenceToPath.find(std::make_pair(std::get<0>(min->getCost2()), std::get<2>(min->getCost2())))->second;	//x,h,z
				vec2Key currentMinusPeak = vec2Key(std::get<0>(current) -std::get<0>(peak), std::get<2>(current) -std::get<2>(peak));
				vector3d minMinusPeak = vector3d(std::get<0>(min->getLoc()) - std::get<0>(peak), 0.0, std::get<2>(min->getLoc()) - std::get<1>(peak));

				slope = referenceToSlope.find(std::make_pair(std::get<0>(current), std::get<1>(current)))->second;	//x,z,slope
				double angle = vector3d(currentMinusPeak.x, 0.0, currentMinusPeak.y).getAngleWith(minMinusPeak);
				anotherUnitVector = vector3d(cos(angle), 0.0, sin(angle));
			}
			else
			{
				auto lb = referenceToPath.lower_bound(std::make_pair(std::get<0>(min->getCost2()), std::get<2>(min->getCost2())))->second;	//x,h,z bound1
				auto ub = referenceToPath.upper_bound(std::make_pair(std::get<0>(min->getCost2()), std::get<2>(min->getCost2())))->second;	//x,h,z bound2

				vector3d lbMinusPeak = vector3d(std::get<0>(lb) -std::get<0>(peak), 0.0, std::get<1>(lb) -std::get<1>(peak));
				vector3d ubMinusPeak = vector3d(std::get<0>(ub) -std::get<0>(peak), 0.0, std::get<1>(ub) -std::get<1>(peak));

				vector3d minMinusPeak = vector3d(std::get<0>(min->getLoc()) - std::get<0>(peak), 0.0, std::get<2>(min->getLoc()) - std::get<2>(peak));

				auto ubMinusMin = (ubMinusPeak - minMinusPeak);
				auto minMinusLb = (minMinusPeak - lbMinusPeak);
				auto ubMinusLb = ubMinusMin - minMinusLb;

				double deltaRFromLb = minMinusLb.mag() / ubMinusLb.mag();

				double slopeLb;
				if (referenceToSlope.find(std::make_pair(std::get<0>(lb), std::get<1>(lb))) != referenceToSlope.end())
				{
					slopeLb = referenceToSlope.find(std::make_pair(std::get<0>(lb), std::get<1>(lb)))->second;	//x,y,slope
				}
				else
				{
					slopeLb = 0.0;
				}

				auto currentMinusPeak = lbMinusPeak + ubMinusLb * (slopeLb * deltaRFromLb);
				double currentHeight = std::get<2>(lb) -slopeLb*deltaRFromLb;
				slope = slopeLb;

				double angle = vector3d(currentMinusPeak.x, 0.0, currentMinusPeak.y).getAngleWith(minMinusPeak);
				anotherUnitVector = vector3d(cos(angle), 0.0, sin(angle));
			}
			if (slope != 0)
			{
				pointOfInterest = std::make_pair(std::get<0>(min->getLoc()), std::get<2>(min->getLoc()));
				if (listOfGradients.find(pointOfInterest) != listOfGradients.end())
				{
					auto tempList = (listOfGradients.find(pointOfInterest))->second;
					aGradientInfo = std::make_tuple(std::get<0>(peak), std::get<2>(peak), vec2Key(slope*anotherUnitVector.x, slope*anotherUnitVector.y));
					tempList.push_back(aGradientInfo);
					listOfGradients.insert(std::make_pair(pointOfInterest, tempList));
				}
				else
				{
					// list of gradients for this point is empty
					list<tuple<int, int, vec2Key>> listOfGradientInfos;
					aGradientInfo = std::make_tuple(std::get<0>(peak), std::get<2>(peak), vec2Key(slope*anotherUnitVector.x, slope*anotherUnitVector.y));
					listOfGradientInfos.push_back(aGradientInfo);
					listOfGradients.insert(std::make_pair(pointOfInterest, listOfGradientInfos));
				}
			}
		}
	}
	void graph::changeEdgeWeightTo(edge_sp fixedEdge, double newWeight)
	{
		assert(newWeight >= 0);
		auto node1 = fixedEdge->getSource().lock();
		auto node2 = fixedEdge->getTarget().lock();

		fixedEdge->setWeight(newWeight);
		for (auto anEdge : node2->getEdges())
		{
			if (anEdge->getTarget().lock() == node1)
			{
				anEdge->setWeight(fixedEdge->getWeight());
				break;
			}
		}
	}
	void graph::changeEdgeWeights(const pair<int, int> position, list<tuple<int, int, vec2Key>>& listOfGradients, size_t n)
	{
		vector3d currentLocation = vector3d((double)(std::get<0>(position)), 0.0, (double)(std::get<1>(position)));
		auto node1 = getNodeFromLocation(currentLocation, n);
		list<vector3d> gradients;
		for (auto it = listOfGradients.begin(); it != listOfGradients.end(); it++)
		{
			gradients.push_back(vector3d(std::get<2>(*it).x, 0.0, std::get<2>(*it).y));
		}
		node_sp node2;
		bool isIncomingToNode1;
		double amountOfIncrease = 0.0;
		for (auto anEdge : node1->getEdges())
		{
			if (anEdge->getTarget().lock() == node1)
			{
				node2 = anEdge->getSource().lock();
				isIncomingToNode1 = true;
			}
			else
			{
				assert(anEdge->getSource().lock() == node1);
				node2 = anEdge->getTarget().lock();
				isIncomingToNode1 = false;
			}
			if (!anEdge->isValid())
			{
				vector3d directionToNeighbor = vector3d(std::get<0>(node2->getLoc()) - std::get<0>(node1->getLoc()),
					0.0, std::get<2>(node2->getLoc()) - std::get<2>(node1->getLoc())).normal();
				if (node1->isDiagonalNeighborWith(node2))
				{
					amountOfIncrease = sqrt(2.0)*getWeightForSpecifiedDirectionBasedOnListOfGradients(gradients, directionToNeighbor);
				}
				else
				{
					amountOfIncrease = getWeightForSpecifiedDirectionBasedOnListOfGradients(gradients, directionToNeighbor);
				}

		//		assert(amountOfIncrease >= 0.0);
		//		changeEdgeWeightTo(node1->edges[m], node1->edges[m]->wt + amountOfIncrease);
		//		assert(node1->edges[m]->wt >= 0.0);

				assert(amountOfIncrease >= 0.0);
				anEdge->setWeight(anEdge->getWeight() + amountOfIncrease);
				assert(anEdge->getWeight() >= 0.0);
				for (auto otherEdge : node2->getEdges())
				{
					if (isIncomingToNode1 == true && otherEdge->getTarget().lock() == node1
						|| isIncomingToNode1 == false && otherEdge->getSource().lock() == node1)
					{
						otherEdge->setWeight(anEdge->getWeight());
						break;
					}
				}
			}
		}
	}
	double graph::getWeightForSpecifiedDirectionBasedOnListOfGradients(list<vector3d>& gradients, vector3d directionUnit)
	{
		auto bounds = directionUnit.getBoundingVectors(gradients);
		// assert(!bounds.empty());
		if (bounds.empty())
		{
			return 0.0;
		}
		if (bounds.size() == 1)
		{
			double omega = 2 * M_1_PI;
			double factor = omega / M_1_PI;
			double theta = directionUnit.getAngleWith(*gradients.begin());
			double newWeight = (*gradients.begin()).mag(); //* dotOfUnitsWithAngle(theta / factor);
			assert(newWeight >= 0);
			return newWeight;
		}
		assert(bounds.size() == 2);
		vector3d first = *bounds.begin(), second = *(bounds.end()--);
		double omega = first.getAngleWith(second);
		double factor = omega / M_1_PI;
		double
			theta1 = directionUnit.getAngleWith(first),
			theta2 = directionUnit.getAngleWith(second);
		double newWeight = (first.mag() * dotOfUnitsWithAngle(theta1 / factor) + second.mag() * dotOfUnitsWithAngle(theta2 / factor))
			/ (dotOfUnitsWithAngle(theta1 / factor) + dotOfUnitsWithAngle(theta2 / factor));
		assert(newWeight >= 0);
		return newWeight;
	}
	double graph::dotOfUnitsWithAngle(double angle)
	{
		double newWeight = 1 + cos(angle);
		assert(newWeight >= 0);
		return newWeight;
	}
*/

