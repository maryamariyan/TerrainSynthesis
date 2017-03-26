#include "dualgraph.h"
using namespace std;

dual_graph::dual_graph(size_t n, graph_sp& g) {
	numDijkstra = 0;
	originalGraph = g;
	for (auto curNode : originalGraph.lock()->getNodes())
	{
		assert(curNode->isValid());
		for (auto& anEdge : curNode->getEdges())
		{
			auto otherNode = anEdge->getTarget().lock();
			//if node at the other end of this edge is not processed yet
			if (otherNode->isValid())
			{
				//create dual node
				nodes.push_back(std::make_shared<node>(anEdge->getLoc(), anEdge->isFixed(), nodes.size(), true));
				nodes.back()->setOriginalEdge(anEdge);
				solution.push_back(0);
				frontier.push_back(0);
				if (nodes.back()->isFixed())
				{
					fixedDualNodes.push_back(nodes.back());
					auto sc = anEdge->getSource().lock();
					auto tg = anEdge->getTarget().lock();

					///////////////
					double srcCost = sc->getCost(),
						tgtCost = tg->getCost();				

					bool srcIsTaller = srcCost < tgtCost;
					vector3d scLoc = sc->getLoc(), tgLoc = tg->getLoc();
					double
						STTSx = srcIsTaller ? (tgLoc.x - scLoc.x) : (scLoc.x - tgLoc.x),
						STTSz = srcIsTaller ? (tgLoc.z - scLoc.z) : (scLoc.z - tgLoc.z),
						STTSmag = sqrt(pow(STTSx, 2) + pow(STTSz, 2)),
						deltaH = fabs(tgtCost - srcCost),
						cosThetaMag = STTSmag / sqrt(pow(deltaH, 2) + pow(STTSmag, 2));
					assert(cosThetaMag >= 0);
					double sinThetaMag = sqrt(1 - pow(cosThetaMag, 2));
					vector3d slopeOr;
					vector3d slopeOrientation;

					slopeOr = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);
					slopeOrientation = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);

					/////////////
					auto oriantationMVECTOR3 = slopeOrientation;

					vector3d groundDifference = vector3d(STTSx, 0.0, STTSz),
						normal = oriantationMVECTOR3;
					
					assert(groundDifference.mag() != 0);

					double groundNormalProjection = fabs(normal*groundDifference) / groundDifference.mag(),
						heightNormalProjection = sqrt(pow(normal.mag(), 2) - pow(groundNormalProjection, 2));
					
					assert(heightNormalProjection != 0.0);
					
					double heightDifference = fabs(normal*groundDifference) / heightNormalProjection;

					assert(fabs(heightDifference - fabs(srcCost - tgtCost)) < 0.000000001);
					/////////////

					//set peak location, for now either s or t; whichever is taller
					nodes.back()->setPeakLocation(srcIsTaller ? sc->getLoc() : tg->getLoc());
					//set cost as deltaH btw s and t
					nodes.back()->setCost(fabs(srcCost - tgtCost));
					//set orientation normal perpendicular to slope btw s and t
					nodes.back()->setOrientation(vector3d(normal.x, normal.y, normal.z));
				}

				for (auto& neighborEdge : curNode->getEdges())
				{
					if (neighborEdge->getTarget().lock() == otherNode)
					{
						assert(neighborEdge == anEdge);
						neighborEdge->setDualNode(nodes.back());
					}
				}
				for (auto& neighborEdge : otherNode->getEdges())
				{
					if (neighborEdge->getTarget().lock() == curNode)
					{
						neighborEdge->setDualNode(nodes.back());
					}
				}
			}
		}
		curNode->setValid(false);
	}
	//assert dual node size == 2*(m-1)*(n-1) + (m-1)*(n) + (m)*(n-1)
	int dualSize = 2 * (n - 1)*(n - 1) + 2 * n*(n - 1);
	assert(nodes.size() == dualSize);

	for (auto curNode : originalGraph.lock()->getNodes())
	{
		assert(!curNode->isValid());
		for (auto& anEdge : curNode->getEdges())
		{
			auto otherNode = anEdge->getTarget().lock();
			//if node at the other end of this edge is not processed yet
			if (!otherNode->isValid())
			{
				//create dual edges for each dual node
				std::vector<node_sp> nbp;

				for (auto& neighborEdge : curNode->getEdges())
				{
					if (neighborEdge->getTarget().lock() != otherNode)
					{
						nbp.push_back(neighborEdge->getDualNode().lock());
					}
				}
				for (auto& neighborEdge : otherNode->getEdges())
				{
					if (neighborEdge->getTarget().lock() != curNode)
					{
						nbp.push_back(neighborEdge->getDualNode().lock());
					}
				}
				(anEdge->getDualNode().lock())->createEdges(nbp, true);
			}
		}
		curNode->setValid(true);
	}
}
/*----------------------------------------------------------------------*/
dual_graph::~dual_graph() {
	;
}
/*----------------------------------------------------------------------*/
void dual_graph::estimateWeights() {
	size_t MAX_HOP_SIZE = 20;
	;//()	setup vector3d(orientation) from value of edge weights in original graph.
	;//
	setLookAhead(MAX_HOP_SIZE);
	;//sum average the list of influences into vector3d(orientation)
	//set 
	;//
	;//diagonal dual nodes get cost infinity and others get cost from vector3d(orientation)
	setInfinityToDiagonalEdges();//incomplete
}
/*----------------------------------------------------------------------*/
void dual_graph::setLookAhead2(size_t maxHopSize) {
	//could call this right after constructor
	size_t counterForSolution = 0;
	for (auto& curFixedDualNode : fixedDualNodes)
	{
		counterForSolution++;
		priority_queue<ByCostNodeStruct, vector<ByCostNodeStruct>, CompareByCost> min_heap;
		curFixedDualNode->setCost(0.0);
		curFixedDualNode->setPositionOnPath(0.0);
		frontier.at(curFixedDualNode->getIndex()) = counterForSolution;
		ByCostNodeStruct bcns = { curFixedDualNode->getCost(), curFixedDualNode->getIndex() };
		min_heap.push(bcns);

		node_sp min, otherEnd;
		while (!min_heap.empty())
		{
			bcns = min_heap.top();
			size_t mv = bcns.index;
			min_heap.pop();
			min = getNodeFromIndex(mv);
			assert(min->getIndex() == mv);

			/*
			if (maxHopSize <= min->getPositionOnPath())
			{
				//don't insert in heap, dual nodes with POP >= HSIZE
				while (!min_heap.empty())
				{
					min_heap.pop();
				}
				continue;
			}*/


			if (solution.at(mv) == counterForSolution)
				continue;

			frontier.at(mv) = 0;
			solution.at(mv) = counterForSolution;

			/*
			double srcCost = curFixedDualNode->getOriginalEdge()->getSource().lock()->getCost(),
			tgtCost = curFixedDualNode->getOriginalEdge()->getTarget().lock()->getCost(),
			//min minus curFDN
			ABx = std::get<0>(min->getLoc()) - std::get<0>(curFixedDualNode->getLoc()),
			ABz = std::get<2>(min->getLoc()) - std::get<2>(curFixedDualNode->getLoc());
			bool srcIsTaller = srcCost < tgtCost;
			double
			srcX = std::get<0>(curFixedDualNode->getOriginalEdge()->getSource().lock()->getLoc()),
			srcZ = std::get<2>(curFixedDualNode->getOriginalEdge()->getSource().lock()->getLoc()),
			tgtX = std::get<0>(curFixedDualNode->getOriginalEdge()->getTarget().lock()->getLoc()),
			tgtZ = std::get<2>(curFixedDualNode->getOriginalEdge()->getTarget().lock()->getLoc()),
			STTSx = srcIsTaller ? (tgtX - srcX) : (srcX - tgtX),
			STTSz = srcIsTaller ? (tgtZ - srcZ) : (srcZ - tgtZ),
			STTSmag = sqrt(pow(STTSx, 2) + pow(STTSz, 2)),
			ABmag = sqrt(pow(ABx, 2) + pow(ABz, 2)),
			deltaH = fabs(tgtCost - srcCost),
			cosThetaMag = STTSmag / sqrt(pow(deltaH, 2) + pow(STTSmag, 2));
			assert(cosThetaMag >= 0);
			double sinThetaMag = sqrt(1 - pow(cosThetaMag, 2));
			vector3d slopeOr;
			vector3d slopeOrientation;
			if (ABmag == 0)
			{
			assert(curFixedDualNode == min);
			slopeOr = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);
			slopeOrientation = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);
			}
			else
			{
			slopeOr = vector3d(ABx*sinThetaMag / ABmag, cosThetaMag, ABz*sinThetaMag / ABmag);
			slopeOrientation = vector3d(ABx*sinThetaMag / ABmag, cosThetaMag, ABz*sinThetaMag / ABmag);
			}
			assert(slopeOr.mag() < 1.01 && slopeOr.mag() > 0.98);
			*/

			if (!min->isFixed())
			{
				vector3d peakLoc, minLoc, normalOrn, groundNewDistance, newNormal;
				peakLoc = curFixedDualNode->getPeakLocation();
				peakLoc.y = 0.0;
				minLoc = min->getLoc();
				minLoc.y = 0.0;
				normalOrn = curFixedDualNode->getOrientation();

				groundNewDistance = (minLoc - peakLoc)*2.0;//in case we cared about correct magnitude

				newNormal = groundNewDistance.normal() * (vector3d(normalOrn.x, 0.0, normalOrn.z).mag());
				newNormal.y = normalOrn.y;

				vector3d slopeOrientation = vector3d(newNormal.x, newNormal.y, newNormal.z);

				influence inflnc = { min->getPositionOnPath(), min->getIndex(), slopeOrientation, curFixedDualNode->getIndex() };
				min->inf.push_back(inflnc);
			}
			for (auto& neighborDualEdge : min->getEdges())
			{
				auto neighborDualNode = neighborDualEdge->getTarget().lock();
				auto neighborOriginalEdge = neighborDualNode->getOriginalEdge();
				auto scc = neighborOriginalEdge->getTarget().lock();
				auto tgg = neighborOriginalEdge->getSource().lock();

				//get (non-diagonal)nodes on other end its edges,
				if (neighborOriginalEdge->getLoc().y == 0.5)
				{
					//if othernode not fixed and not in solution yet,
					if (!neighborDualNode->isFixed() && solution.at(neighborDualNode->getIndex()) != counterForSolution)
					{
						assert(neighborDualEdge->getWeight() >= 0.0);
						double dist = min->getCost() + neighborDualEdge->getWeight();

						if (frontier.at(neighborDualNode->getIndex()) != counterForSolution || dist < neighborDualNode->getCost())
						{
							neighborDualNode->setPositionOnPath(min->getPositionOnPath() + 1);
							neighborDualNode->setCost(dist);

							frontier.at(neighborDualNode->getIndex()) = counterForSolution;
							ByCostNodeStruct bcns = { neighborDualNode->getCost(), neighborDualNode->getIndex() };
							min_heap.push(bcns);
						}
					}
				}
				else {
					//neighborOriginalEdge is diagonal
					double yLoc = neighborOriginalEdge->getLoc().y;
					assert(yLoc == 1.0 || yLoc == 0.0);
				}
			}
		}
	}
}
/*----------------------------------------------------------------------*/
void dual_graph::setLookAhead(size_t maxHopSize) {
	//could call this right after constructor
	size_t counterForSolution = 0;
	for (auto& curFixedDualNode : fixedDualNodes)
	{
		counterForSolution++;
		priority_queue<ByDistanceNodeStruct, vector<ByDistanceNodeStruct>, CompareByDistance> min_heap;
		curFixedDualNode->setPositionOnPath(0.0);
		curFixedDualNode->setCost2(vector3d(0.0, 0.0, 0.0));
		frontier.at(curFixedDualNode->getIndex()) = counterForSolution;
		auto tmpMV = curFixedDualNode->getCost2();
		ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), curFixedDualNode->getIndex() };
		min_heap.push(bdns);

		node_sp min, otherEnd;
		while (!min_heap.empty())
		{
			bdns = min_heap.top();
			size_t mv = bdns.index;
			min_heap.pop();
			min = getNodeFromIndex(mv);
			assert(min->getIndex() == mv);			
			/*
			if (maxHopSize <= min->getPositionOnPath())
			{
				//don't insert in heap, dual nodes with POP >= HSIZE
				while (!min_heap.empty())
				{
					min_heap.pop();
				}
				continue;
			}
			*/			
			if (solution.at(mv) == counterForSolution)
				continue;

			frontier.at(mv) = 0;
			solution.at(mv) = counterForSolution;

			/*
			double srcCost = curFixedDualNode->getOriginalEdge()->getSource().lock()->getCost(),
				tgtCost = curFixedDualNode->getOriginalEdge()->getTarget().lock()->getCost(),
				//min minus curFDN
				ABx = std::get<0>(min->getLoc()) - std::get<0>(curFixedDualNode->getLoc()),
				ABz = std::get<2>(min->getLoc()) - std::get<2>(curFixedDualNode->getLoc());
			bool srcIsTaller = srcCost < tgtCost;
			double
				srcX = std::get<0>(curFixedDualNode->getOriginalEdge()->getSource().lock()->getLoc()),
				srcZ = std::get<2>(curFixedDualNode->getOriginalEdge()->getSource().lock()->getLoc()),
				tgtX = std::get<0>(curFixedDualNode->getOriginalEdge()->getTarget().lock()->getLoc()),
				tgtZ = std::get<2>(curFixedDualNode->getOriginalEdge()->getTarget().lock()->getLoc()),
				STTSx = srcIsTaller ? (tgtX - srcX) : (srcX - tgtX),
				STTSz = srcIsTaller ? (tgtZ - srcZ) : (srcZ - tgtZ),
				STTSmag = sqrt(pow(STTSx, 2) + pow(STTSz, 2)),
				ABmag = sqrt(pow(ABx, 2) + pow(ABz, 2)),
				deltaH = fabs(tgtCost - srcCost),
				cosThetaMag = STTSmag / sqrt(pow(deltaH, 2) + pow(STTSmag, 2));
			assert(cosThetaMag >= 0);
			double sinThetaMag = sqrt(1 - pow(cosThetaMag, 2));			
			vector3d slopeOr;
			vector3d slopeOrientation;
			if (ABmag == 0)
			{
				assert(curFixedDualNode == min);
				slopeOr = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);
				slopeOrientation = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);
			}
			else
			{
				slopeOr = vector3d(ABx*sinThetaMag / ABmag, cosThetaMag, ABz*sinThetaMag / ABmag);
				slopeOrientation = vector3d(ABx*sinThetaMag / ABmag, cosThetaMag, ABz*sinThetaMag / ABmag);
			}
			assert(slopeOr.mag() < 1.01 && slopeOr.mag() > 0.98);
			*/
			vector3d slopeOrientation;
			if (!min->isFixed())
			{
				vector3d peakLoc, minLoc, normalOrn, groundNewDistance, newNormal;
				peakLoc = curFixedDualNode->getPeakLocation();
				peakLoc.y = 0.0;
				minLoc = min->getLoc();
				minLoc.y = 0.0;
				normalOrn = curFixedDualNode->getOrientation();

				groundNewDistance = (minLoc - peakLoc)*2.0;//in case we cared about correct magnitude

				newNormal = groundNewDistance.normal() * (vector3d(normalOrn.x, 0.0, normalOrn.z).mag());
				newNormal.y = normalOrn.y;
				
				slopeOrientation = vector3d(newNormal.x, newNormal.y, newNormal.z);

				influence inflnc = { min->getPositionOnPath(), min->getIndex(), slopeOrientation, curFixedDualNode->getIndex() };
				min->inf.push_back(inflnc);
			}
			else
			{
				slopeOrientation = curFixedDualNode->getOrientation();
			}
			
			for (auto& neighborDualEdge : min->getEdges())
			{
				auto neighborDualNode = neighborDualEdge->getTarget().lock();
				auto neighborOriginalEdge = neighborDualNode->getOriginalEdge();
				vector3d tggLoc = neighborOriginalEdge->getTarget().lock()->getLoc();
				vector3d sccLoc = neighborOriginalEdge->getSource().lock()->getLoc();

				//get (non-diagonal)nodes on other end its edges,
				if (neighborOriginalEdge->getLoc().y == 0.5)
				{
					//if othernode not fixed and not in solution yet,
					if (!neighborDualNode->isFixed() && solution.at(neighborDualNode->getIndex()) != counterForSolution)
					{
						assert(neighborDualEdge->getWeight() >= 0.0);
						double dist = min->getCost() + neighborDualEdge->getWeight();

						if (frontier.at(neighborDualNode->getIndex()) != counterForSolution || dist < neighborDualNode->getCost())
						{
							neighborDualNode->setPositionOnPath(min->getPositionOnPath() + 1);
							neighborDualNode->setCost(dist);
							neighborDualNode->setCost2(vector3d(
								min->getCost2().x + slopeOrientation.x * fabs(tggLoc.x - sccLoc.x),
								0.0,
								min->getCost2().z + slopeOrientation.z * fabs(tggLoc.z - sccLoc.z)));
							frontier.at(neighborDualNode->getIndex()) = counterForSolution;
							auto tmpMV = neighborDualNode->getCost2();
							ByDistanceNodeStruct bdns = { coord(tmpMV.x, tmpMV.y, tmpMV.z), neighborDualNode->getIndex() };
							min_heap.push(bdns);
						}
					}
				}
				else {
					//neighborOriginalEdge is diagonal
					double yLoc = neighborOriginalEdge->getLoc().y;
					assert(yLoc == 1.0 || yLoc == 0.0);
				}
			}
		}
	}
}
/*----------------------------------------------------------------------*/
void dual_graph::setLookAhead1(size_t maxHopSize) {
	//could call this right after constructor
	size_t counterForSolution = 0;
	for (auto& curFixedDualNode : fixedDualNodes)
	{
		counterForSolution++;
		priority_queue<ByPositionOnPathNodeStruct, vector<ByPositionOnPathNodeStruct>, CompareByPositionOnPath> min_heap;
		curFixedDualNode->setPositionOnPath(0);
		frontier.at(curFixedDualNode->getIndex()) = counterForSolution;
		ByPositionOnPathNodeStruct bpns = { curFixedDualNode->getPositionOnPath(), curFixedDualNode->getIndex() };
		min_heap.push(bpns);

		node_sp min, otherEnd;
		while (!min_heap.empty())
		{
			bpns = min_heap.top();
			size_t mv = bpns.index;
			min_heap.pop();
			min = getNodeFromIndex(mv);
			assert(min->getIndex() == mv);

			
			if (maxHopSize <= min->getPositionOnPath())
			{
			//don't insert in heap, dual nodes with POP >= HSIZE
			while (!min_heap.empty())
			{
			min_heap.pop();
			}
			continue;
			}
			

			if (solution.at(mv) == counterForSolution)
				continue;

			frontier.at(mv) = 0;
			solution.at(mv) = counterForSolution;

			/*
			double srcCost = curFixedDualNode->getOriginalEdge()->getSource().lock()->getCost(),
			tgtCost = curFixedDualNode->getOriginalEdge()->getTarget().lock()->getCost(),
			//min minus curFDN
			ABx = std::get<0>(min->getLoc()) - std::get<0>(curFixedDualNode->getLoc()),
			ABz = std::get<2>(min->getLoc()) - std::get<2>(curFixedDualNode->getLoc());
			bool srcIsTaller = srcCost < tgtCost;
			double
			srcX = std::get<0>(curFixedDualNode->getOriginalEdge()->getSource().lock()->getLoc()),
			srcZ = std::get<2>(curFixedDualNode->getOriginalEdge()->getSource().lock()->getLoc()),
			tgtX = std::get<0>(curFixedDualNode->getOriginalEdge()->getTarget().lock()->getLoc()),
			tgtZ = std::get<2>(curFixedDualNode->getOriginalEdge()->getTarget().lock()->getLoc()),
			STTSx = srcIsTaller ? (tgtX - srcX) : (srcX - tgtX),
			STTSz = srcIsTaller ? (tgtZ - srcZ) : (srcZ - tgtZ),
			STTSmag = sqrt(pow(STTSx, 2) + pow(STTSz, 2)),
			ABmag = sqrt(pow(ABx, 2) + pow(ABz, 2)),
			deltaH = fabs(tgtCost - srcCost),
			cosThetaMag = STTSmag / sqrt(pow(deltaH, 2) + pow(STTSmag, 2));
			assert(cosThetaMag >= 0);
			double sinThetaMag = sqrt(1 - pow(cosThetaMag, 2));
			vector3d slopeOr;
			vector3d slopeOrientation;
			if (ABmag == 0)
			{
			assert(curFixedDualNode == min);
			slopeOr = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);
			slopeOrientation = vector3d(STTSx*sinThetaMag / STTSmag, cosThetaMag, STTSz*sinThetaMag / STTSmag);
			}
			else
			{
			slopeOr = vector3d(ABx*sinThetaMag / ABmag, cosThetaMag, ABz*sinThetaMag / ABmag);
			slopeOrientation = vector3d(ABx*sinThetaMag / ABmag, cosThetaMag, ABz*sinThetaMag / ABmag);
			}
			assert(slopeOr.mag() < 1.01 && slopeOr.mag() > 0.98);
			*/

			if (!min->isFixed())
			{
				vector3d peakLoc, minLoc, normalOrn, groundNewDistance, newNormal;
				peakLoc = curFixedDualNode->getPeakLocation();
				peakLoc.y = 0.0;
				minLoc = min->getLoc();
				minLoc.y = 0.0;
				normalOrn = curFixedDualNode->getOrientation();

				groundNewDistance = (minLoc - peakLoc)*2.0;//in case we cared about correct magnitude

				newNormal = groundNewDistance.normal() * (vector3d(normalOrn.x, 0.0, normalOrn.z).mag());
				newNormal.y = normalOrn.y;

				vector3d slopeOrientation = vector3d(newNormal.x, newNormal.y, newNormal.z);

				influence inflnc = { min->getPositionOnPath(), min->getIndex(), slopeOrientation, curFixedDualNode->getIndex() };
				min->inf.push_back(inflnc);
			}

			for (auto& neighborDualEdge : min->getEdges())
			{
				auto neighborDualNode = neighborDualEdge->getTarget().lock();
				assert(min != neighborDualEdge->getTarget().lock());
				assert(min == neighborDualEdge->getSource().lock());
				auto neighborOriginalEdge = neighborDualNode->getOriginalEdge();

				//get (non-diagonal)nodes on other end its edges,
				if (neighborOriginalEdge->getLoc().y == 0.5)
				{
					//if othernode not fixed and not in solution yet,
					//lookup their pos on path
					if (!neighborDualNode->isFixed() && solution.at(neighborDualNode->getIndex()) != counterForSolution)
					{
						if (frontier.at(neighborDualNode->getIndex()) == counterForSolution)
						{
							assert(neighborDualNode->getPositionOnPath() >= min->getPositionOnPath());
						}
						else
						{
							neighborDualNode->setPositionOnPath(min->getPositionOnPath() + 1);
							frontier.at(neighborDualNode->getIndex()) = counterForSolution;
							ByPositionOnPathNodeStruct bpns = { neighborDualNode->getPositionOnPath(), neighborDualNode->getIndex() };
							min_heap.push(bpns);
						}
					}
				}
				else {
					//neighborOriginalEdge is diagonal
					double yLoc = neighborOriginalEdge->getLoc().y;
					assert(yLoc == 1.0 || yLoc == 0.0);
				}
			}
		}
	}
}
/*----------------------------------------------------------------------*/
void dual_graph::setInfinityToDiagonalEdges() {
	//TODO 1: set cost of dual nodes based on computed orientation
	//TODO 2: make priority sqrt(x^2 + z^2) instead of hop count in dual graph lookahead algorithm
	//TODO 3: male priority sqrt(x^2 + z^2) instead of hop count in original Dijkstra
	for (auto& curDualNode : nodes)
	{
		if (curDualNode->isFixed())
		{
			auto aa = curDualNode->getOriginalEdge()->getSource().lock()->getCost();
			auto bb = curDualNode->getOriginalEdge()->getTarget().lock()->getCost();
			curDualNode->setCost(fabs(aa - bb));
		}
		//else if diagonal
		else if (curDualNode->getOriginalEdge()->getLoc().y != 0.5)
		{
			assert(curDualNode->inf.size() == 0);
			//set cost == max_fact
			curDualNode->setCost(MAX_FACT);
		}
		//else is when is 4-connected
		else
		{
	//		assert(curDualNode->inf.size() != 0); //general case
	//		assert(curDualNode->inf.size() == fixedDualNodes.size()); //with no lookahead limit
			;//set cost based on sum average of list of influences
			vector3d tmp = vector3d(0.0, 0.0, 0.0), other = vector3d(0.0, 0.0, 0.0);
			double priority = 0.0, weightTotal = 0.0;
			auto infList = curDualNode->inf;
			for (auto& inf: infList)
			{
				priority = inf.pop;
				other = inf.slope_orientation * (1.0 / priority);
				weightTotal += 1.0/priority;
				tmp = tmp + other;
			}
			double dude;
			if (infList.size() != 0)
			{
				other = tmp / weightTotal;
				vector3d beep = other.normal();
				double ddd = beep.mag();
				assert(fabs(beep.mag() - 1) < 0.000000001);
				vector3d fic = vector3d(beep.x, beep.y, beep.z);
				dude = getValueFromOrientationNormal(curDualNode, fic);
			}
			else
			{
				dude = 0.1;
			}
			curDualNode->setCost(dude);
		}
	}
}
/*----------------------------------------------------------------------*/
double dual_graph::getValueFromOrientationNormal(node_wp curDualNode, vector3d orientationNormal) {

	auto cc = curDualNode.lock()->getOriginalEdge()->getSource().lock()->getLoc();
	auto dd = curDualNode.lock()->getOriginalEdge()->getTarget().lock()->getLoc();
	auto srcX = cc.x;
	auto srcZ = cc.z;
	auto tgtX = dd.x;
	auto tgtZ = dd.z;

	vector3d flatOrientation = vector3d(tgtX - srcX, 0.0, tgtZ - srcZ),
		normal = orientationNormal;

	assert(fabs(normal.mag() - 1) < 0.000000001);
	
	///////
	vector3d newNormal = flatOrientation.normal() * (vector3d(normal.x, 0.0, normal.z).mag());
	newNormal.y = normal.y;
	///////
	assert(vector3d(newNormal.x, 0.0, newNormal.z).mag() != 0.0);
	double heightDifference;
	heightDifference = flatOrientation.mag() * newNormal.y / vector3d(newNormal.x, 0.0, newNormal.z).mag();
	/////

//	heightDifference = flatOrientation.mag() * normal.y / (normal * flatOrientation.normal());
	
	return fabs(heightDifference);
}
/*----------------------------------------------------------------------*/
node_sp dual_graph::getNodeFromIndex(size_t idx) {
	return nodes.at(idx);
}
/*----------------------------------------------------------------------*/
void dual_graph::setNumDijkstra(int num) {
	numDijkstra = num;
}
void dual_graph::incrementNumDijkstra() {
	numDijkstra++;
}
int dual_graph::getNumDijkstra() {
	return numDijkstra;
}
