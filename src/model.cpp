#include "model.h"

Model::Model(size_t totalNodes, map<int, ridge_point> rp)
{
	assert(sqrt(totalNodes) - (long)sqrt(totalNodes) == 0.0);
	length = sqrt(totalNodes);
	if (rp.empty())
	{
//		pointsOnRidge = getRidgeWithOnePeakAndTwoSlopesPerSide();
//		pointsOnRidge = getRidgeWithOneFixedEdge();
//		pointsOnRidge = getRidgeMountFuji();
//		pointsOnRidge = getRidgeWithOnePeakAndSameSlopesOnEachSide();
		pointsOnRidge = getRidgeWithOnePeakOnRightMid();
	}
	else
	{
		pointsOnRidge = rp;
	}
	pGraph = createGraph();
	gNodes = pGraph->getNodes();
}
Model::Model(size_t totalNodes)
{
	assert(sqrt(totalNodes) - (long)sqrt(totalNodes) == 0.0);
	length = sqrt(totalNodes);
	pGraph = createGraph();
	gNodes = pGraph->getNodes();	
	
}
Model::~Model()
{
	;
}
graph_sp Model::createGraph() {
	graph_sp upg(std::make_shared<graph>(length, pointsOnRidge));
	return upg;
}
/*----------------------------------------------------------------------*/
vector<double> Model::generateSinglePeakHeightmap() {
	assert(pointsOnRidge.empty());
	auto aNode = gNodes.at(rand() % gNodes.size());
	vector3d randomPos = aNode->getLoc();
	randomPos.y = rand() % gNodes.size();
	aNode->setLoc(randomPos);
	aNode->setFixed(true);

	pGraph->calculateCostBasedOnHeight();

	pGraph->DijkstraByDistance(length);
	return setHeightsBackWithThreshold();
}
vector<double> Model::getResultingHeightfield2()
{
	if (pointsOnRidge.empty())
	{
		return generateSinglePeakHeightmap();
	}
	pGraph->calculateCostBasedOnHeight();

	pGraph->createDual(length);
	pGraph->getDualGraph()->estimateWeights();//estimation is saved as vector3d(orientation) in dual nodes
	pGraph->updateEdgeWeightsBasedOnCost();//update edge weights in original graph based on orientation on each dual node

	pGraph->DijkstraByDistance(length);
	return setHeightsBackWithThreshold();
	//[1]winter12 [2]summer12 [3]fall12 [4]winter13 [-]summer13 [5]fall13 [6]winter14 [7]summer14
}
vector<double> Model::getResultingHeightfield3()
{
	if (pointsOnRidge.empty())
	{
		return generateSinglePeakHeightmap();
	}
	return tempSegmentRidgelinesToPaths();
}
vector<double> Model::getResultingHeightfield()
{
	if (pointsOnRidge.empty())
	{
		return generateSinglePeakHeightmap();
	}
	vector<vector<int>> indexes; // HAVE CONTAINER PER RIDGE, STORE RIDGE ID ON NODE
	vector<int> tempIndex;
	vector<pair<int, int>> ridgeEndpoints;
	double maxNegativeHeight = 0;
	vector<size_t> nodeIndex;

	/*[1] void pGraph->setBlackRidgeEndPointsAt(DIAGONAL=true, 10_PERCENT_DSIZE, RANDOM=true) */
	//[1]{
	auto blackEndPoint = pGraph->getNodeFromIndex(length * (length / 2 - length / 7) + length / 2 - length / 7);
	//}[1]

	/*[2] void pGraph->Dijkstra(peak=ONE_END_POINT, RANDOM=true) // To get path from end point A to end point B.*/
	//[2]{
	pGraph->calculateCostBasedOnHeight();
	pGraph->DijkstraByDistance(length);//modify as 4-connected; record path by getting prev of prev of prev until you reach null
	//}[2]
	
	/*[3] vector<ridge_points> blackRidgePoints= pGraph->fillBlackRidgeBetweenPeakEndPointAndOtherEndPoint();*/
	//[3]{
	auto blkN = blackEndPoint;
	int counter = 0;
	assert(tempIndex.empty());

	tempIndex.push_back(blkN->getIndex());
	blkN->setBlack(true);
	while (blkN->hasPrevious())
	{
		blkN = blkN->getPrevious().lock();
		tempIndex.push_back(blkN->getIndex());
		blkN->setBlack(true);
	}
	//tempIndex.push_back(aNode->getIndex());//is duplicate
	assert(blkN->isBlack());

	indexes.push_back(tempIndex);
	assert(indexes.at(counter) == tempIndex);
	ridgeEndpoints.push_back(std::make_pair(tempIndex.at(0), tempIndex.back()));
	tempIndex.erase(tempIndex.begin(), tempIndex.end());
	//}[3]

	/*[4] void pGraph->setBlackRidgeHeight(blackRidgePoints, deltaHFirstToLast=0, range=DEFAULT, seaLevel = zero, numberOfPeaks = 2) "__/\/\__" */
	//[4]{
	//should prepare method to create random curve given: number of requested 1D x, yLast - yFirst, range
		//above is used for black ridge cost, red ridge cost,
	size_t maxDesired = 500;
	double maxH = setupBlackRidgeHeights(indexes.at(0), maxDesired);
	//}[4]

	size_t BLACK_STEP_SIZE = (int)(indexes.at(0).size())/6;
	for (auto idx : indexes.at(0))
	{
		auto aBlackNode = gNodes.at(idx);
		aBlackNode->setValid(false);
		aBlackNode->setBlocker(true);
	}
	size_t debugCounter = 0;
	for (auto idx : indexes.at(0))
	{
		auto aBlackNode = gNodes.at(idx);
		bool happened = false;
		if (aBlackNode->getIndex() == indexes.at(0).front())
		{
			assert(!aBlackNode->isValid());
		}
		if (!aBlackNode->isValid())
		{
			bool foundAtLeastOneSpecialNodeInNeighborhood = false;
			for (int i = aBlackNode->getLoc().z - BLACK_STEP_SIZE; i < aBlackNode->getLoc().z + BLACK_STEP_SIZE; i++)
			{
				for (int j = aBlackNode->getLoc().x - BLACK_STEP_SIZE; j < aBlackNode->getLoc().x + BLACK_STEP_SIZE; j++)
				{
					if (j >= 0 && i >= 0 && j < length && i < length)
					{
						size_t anIndex = pGraph->getIndex(vector3d(j, 0.0, i), length);
						auto anotherNode = gNodes.at(anIndex);
						if (anIndex == idx)
						{
							happened = true;
						}
						if (!anotherNode->isValid() &&
							(anotherNode->getIndex() == indexes.at(0).back() ||
							anotherNode->getIndex() == indexes.at(0).front() ||
							anotherNode->getLoc().y == maxH))
						{
							foundAtLeastOneSpecialNodeInNeighborhood = true;
							anotherNode->setBlocker(false);
						}
						anotherNode->setValid(true);
					}
				}
			}
			aBlackNode->setBlocker(false);
			debugCounter++;
		}
		if (aBlackNode->getIndex() == indexes.at(0).front())
		{
			assert(aBlackNode->isValid());
			assert(happened == true);
		}
		
	}
	//	lookup debugCounter;
	/*[5] vector<ridge_points> peaksAndValleys = pGraph->identifyPeaksAndValleys(blackRidgePoints) */
	//[5]{
		//vector<ridge_points> pointsOfInterest;
		//..
		//return pointsOfInterest;
	//}[5]

	/*[6] vector<ridge_points> blackRidgeSources = addMorePOIOtherThanPeaksAndValleys(peaksAndValleys, blackRidgePoints, STEP_SIZE_FOR_REJECTION_SAMPLING); */
	//[6]{
		// there may be other POIs, by adding more in empty spaces between peaks and valleys in distances larger than STEP_SIZE from peaks and valleys
	//}[6]

	/*[7] void pGraph->Dijkstra(peak=blackRidgeSources, RANDOM=true) // To get path of blue endpoints from boundary to black ridge sources. */
	//[7]{
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length; j++)
		{
			size_t index = i*length + j;
			auto aNode = gNodes.at(index);
			aNode->setSource(NULL);
			aNode->setPrev(NULL);
			assert(aNode->getPrevious().expired());
			assert(aNode->getSource().expired());
			assert(aNode->isValid());

			if (aNode->isBlack())
			{
				aNode->setCost(0.0);
				aNode->setCost2(vector3d(0.0, 0.0, 0.0));
				aNode->setFixed(true);
			}
			else
			{
				aNode->setCost(MAX_FACT);
				aNode->setCost2(vector3d(MAX_FACT, 0.0, MAX_FACT));
				aNode->setFixed(false);
			}
		}
	}
	//pGraph->calculateCostBasedOnHeight();
	pGraph->DijkstraByDistanceWithBlocker(length);
	//}[7]
	
	nodeIndex.erase(nodeIndex.begin(), nodeIndex.end());
	/*[8] vector<ridge_points> blueRidgePoints = void pGraph->bufferBlueEndPointNominees(criteria=ALL_ON_BOUNDARY) */
	//[8]{
	for (size_t j = 0; j < length; j++)
	{
		size_t index = 0 * length + j;
		auto aNode = gNodes.at(index);
		nodeIndex.push_back(aNode->getIndex());
		aNode->setValid(false);

		index = (length - 1) * length + j;
		aNode = gNodes.at(index);
		nodeIndex.push_back(aNode->getIndex());
		aNode->setValid(false);
	}
	for (size_t i = 1; i < length - 1; i++)
	{
		size_t index = i*length + 0;
		auto aNode = gNodes.at(index);
		nodeIndex.push_back(aNode->getIndex());
		aNode->setValid(false);

		index = i*length + (length - 1);
		aNode = gNodes.at(index);
		nodeIndex.push_back(aNode->getIndex());
		aNode->setValid(false);
	}
	//}[8]
	
	size_t BLUE_STEP_SIZE = (int)length / 10;
	double BLUE_STEEPNESS = 30;
	bool experimenting = true;
	/*[10] vector<ridge_points> blueRidgeSources = void pGraph->setAsBlueEndPaths(BLUE_STEP_SIZE, bufferedBlueEndpoints); */
	//[10]{
	// each blue ridge has unique ridge ID, also unique from black
	// do rejection sampling
	for (auto nIndex : nodeIndex)
	{
		auto aNode = gNodes.at(nIndex);
		if (!aNode->isValid())
		{
			counter++;
			for (int i = aNode->getLoc().z - BLUE_STEP_SIZE; i < aNode->getLoc().z + BLUE_STEP_SIZE; i++)
			{
				for (int j = aNode->getLoc().x - BLUE_STEP_SIZE; j < aNode->getLoc().x + BLUE_STEP_SIZE; j++)
				{
					if (j >= 0 && i >= 0 && j < length && i < length)
					{
						size_t anIndex = pGraph->getIndex(vector3d(j, 0.0, i), length);
						auto anotherNode = gNodes.at(anIndex);
						if (!anotherNode->isValid())
						{
							anotherNode->setValid(true);
						}
					}
				}
			}
			//
			tempIndex.push_back(aNode->getIndex());
			aNode->setBlue(true);
			if (experimenting)
			{
				auto tLoki = aNode->getLoc();
				assert(!aNode->getSource().expired());
				tLoki.y = aNode->getSource().lock()->getLoc().y - BLUE_STEEPNESS * aNode->getPositionOnPath();
				if (tLoki.y < maxNegativeHeight)
				{
					assert(maxNegativeHeight <= 0);
					tLoki.y = maxNegativeHeight;
				}
				aNode->setLoc(tLoki);
			}

			if (aNode->getRidgeId() == 0)
			{
				aNode->setRidgeId(counter);//overwrite old ridge id (0) with counter
			}
			else
			{
				aNode->setOrAddRidgeId(counter);//otherwise add more
			}
			aNode->setValid(true);
			bool foundIt = false;
			int otherEndPoint = -1;
			while (aNode->hasPrevious())
			{
				aNode = aNode->getPrevious().lock();
				if (!aNode->isBlack())
				{
					if (aNode->isBlue())
					{
						if (!foundIt)
						{
							otherEndPoint = aNode->getIndex();
							aNode->setConnectingTwoBlueRidges(true);
						}
						foundIt = true;
						assert(aNode->getRidgeId() != 0);
					}
					tempIndex.push_back(aNode->getIndex());
					aNode->setBlue(true);
					if (experimenting)
					{
						auto tLoki = aNode->getLoc();
						assert(!aNode->getSource().expired());
						tLoki.y = aNode->getSource().lock()->getLoc().y - BLUE_STEEPNESS * aNode->getPositionOnPath();
						if (tLoki.y < maxNegativeHeight)
						{
							assert(maxNegativeHeight <= 0);
							tLoki.y = maxNegativeHeight;
						}
						aNode->setLoc(tLoki);
					}

					if (aNode->getRidgeId() == 0)
					{
						aNode->setRidgeId(counter);//overwrite old ridge id (0) with counter
					}
					else
					{
						aNode->setOrAddRidgeId(counter);//otherwise add more
					}
				}
			}
			assert(aNode->isBlack());
			if (!foundIt)
			{
				otherEndPoint = aNode->getIndex();
			}

			indexes.push_back(tempIndex);
			assert(indexes.at(counter) == tempIndex);
			ridgeEndpoints.push_back(std::make_pair(tempIndex.at(0), otherEndPoint));	//this is invalid and wrong. should think how to segment blue forked ridges for ep
			//ridgeEndpoints.push_back(std::make_pair(tempIndex.at(0), tempIndex.back()));
			tempIndex.erase(tempIndex.begin(), tempIndex.end());
		}
	}
	for (auto nIndex : nodeIndex)
	{
		auto aNode = gNodes.at(nIndex);
		if (!aNode->isBlue())
		{
			aNode->setRed(true);

			int theX = aNode->getLoc().x, theZ = aNode->getLoc().z;
			bool idealCondition = (theX != 1.0 && theZ != 1.0 && theX != length - 2.0 && theZ != length - 2.0);
			assert(idealCondition || (theX == 0.0 || theZ == 0.0 || theX == length - 1 || theZ == length - 1));
		}
	}
	nodeIndex.erase(nodeIndex.begin(), nodeIndex.end());
	//}[10]

	/*[Z] lets find and set red ridges on boundary*/
	//[Z]{
	assert(nodeIndex.empty());
	int redsCountOnBoundary = 0;
	for (size_t j = 0; j < length; j++)
	{
		size_t index = 0 * length + j;
		auto aNode = gNodes.at(index);
		if (aNode->isRed())
		{
			redsCountOnBoundary++;
			nodeIndex.push_back(aNode->getIndex());
			aNode->setValid(false);
		}
		else
		{
			assert(aNode->isValid());
			assert(aNode->isBlue());
		}
		index = (length - 1) * length + j;
		aNode = gNodes.at(index);
		if (aNode->isRed())
		{
			redsCountOnBoundary++;
			nodeIndex.push_back(aNode->getIndex());
			aNode->setValid(false);
		}
		else
		{
			assert(aNode->isValid());
			assert(aNode->isBlue());
		}
	}
	for (size_t i = 1; i < length - 1; i++)
	{
		size_t index = i*length + 0;
		auto aNode = gNodes.at(index);
		if (aNode->isRed())
		{
			redsCountOnBoundary++;
			nodeIndex.push_back(aNode->getIndex());
			aNode->setValid(false);
		}
		else
		{
			assert(aNode->isValid());
			assert(aNode->isBlue());
		}

		index = i*length + (length - 1);
		aNode = gNodes.at(index);
		if (aNode->isRed())
		{
			redsCountOnBoundary++;
			nodeIndex.push_back(aNode->getIndex());
			aNode->setValid(false);
		}
		else
		{
			assert(aNode->isValid());
			assert(aNode->isBlue());
		}
	}
	for (auto nIndex : nodeIndex)
	{
		auto aNode = gNodes.at(nIndex);
		if (!aNode->isValid())
		{
			assert(aNode->isRed());
			assert(aNode->getEdges().size() == 3 || aNode->getEdges().size() == 5);
			vector<vector<int>> followUps;
			vector<int> result;

			counter++;			
			aNode->setValid(true);
			
			for (auto anEdge : aNode->getEdges())
			{
				auto otherEnd = anEdge->getTarget().lock();
				if (!otherEnd->isDiagonalNeighborWith(aNode))
				{
					if (!otherEnd->isValid())
					{
						assert(otherEnd->getEdges().size() == 3 || otherEnd->getEdges().size() == 5);//if it is a boundary too
						assert(otherEnd->isRed());
						assert(!otherEnd->isBlue());
						vector<int> tempFollowUp;

						otherEnd->setValid(true);
						tempFollowUp.push_back(otherEnd->getIndex());

						int newIndex = otherEnd->getIndex();
						do {
							otherEnd = gNodes.at(newIndex);
							newIndex = -1;
							for (auto anotherEdge : otherEnd->getEdges())
							{
								auto anotherEnd = anotherEdge->getTarget().lock();
								if (!anotherEnd->isDiagonalNeighborWith(otherEnd))
								{
									if (!anotherEnd->isValid())
									{
										assert(anotherEnd->isRed());
										assert(!anotherEnd->isBlue());
										anotherEnd->setValid(true);
										tempFollowUp.push_back(anotherEnd->getIndex());
										newIndex = anotherEnd->getIndex();
										continue;
									}
								}
							}
						} while (newIndex != -1);

						followUps.push_back(tempFollowUp);
					}
				}
			}
			assert(followUps.size() <= 2);

			if (followUps.size() == 2)
			{
				for (size_t i = 0; i < followUps.at(1).size(); i++)
				{
					result.push_back((followUps.at(1)).at(followUps.at(1).size() - 1 - i));
				}
			}
			result.push_back(aNode->getIndex());
			if (!followUps.empty())
			{
				for (auto anItem : followUps.at(0))
				{
					result.push_back(anItem);
				}
			}

			for (auto anItem : result)
			{
				auto aCandid = gNodes.at(anItem);
				tempIndex.push_back(aCandid->getIndex());

				if (aCandid->getRidgeId() == 0)
				{
					aCandid->setRidgeId(counter);//overwrite old ridge id (0) with counter
				}
				else
				{
					aCandid->setOrAddRidgeId(counter);//otherwise add more
				}
			}
			
			vector<int> eps;
			auto myNode1 = gNodes.at(tempIndex.at(0));
			//
			for (auto loopingEdge : myNode1->getEdges())
			{
				auto otherEnd = loopingEdge->getTarget().lock();
				if (!otherEnd->isDiagonalNeighborWith(myNode1) && (otherEnd->isBlue() &&
					(otherEnd->getEdges().size() == 3 || otherEnd->getEdges().size() == 5)))
				{
					eps.push_back(otherEnd->getIndex());
				}
			}
			assert(eps.size() == 1 || eps.size() == 2);
			if (eps.size() == 1)
			{
				auto myNode2 = gNodes.at(tempIndex.back());
				//
				for (auto loopingEdge : myNode2->getEdges())
				{
					auto otherEnd = loopingEdge->getTarget().lock();
					if (!otherEnd->isDiagonalNeighborWith(myNode2) && (otherEnd->isBlue() &&
						(otherEnd->getEdges().size() == 3 || otherEnd->getEdges().size() == 5)))
					{
						eps.push_back(otherEnd->getIndex());
					}
				}
				assert(eps.size() == 2);
			}
			else
			{
				bool bibiGandom = false;
			}

			indexes.push_back(tempIndex);
			assert(indexes.at(counter) == tempIndex);
			ridgeEndpoints.push_back(std::make_pair(eps.at(0), eps.back()));
			tempIndex.erase(tempIndex.begin(), tempIndex.end());
		}
	}
	nodeIndex.erase(nodeIndex.begin(), nodeIndex.end());
	//}[Z]
		
	/*[13] void pGraph->Dijkstra(peak=blueRidgeSources, RANDOM=true) // To get path of red endpoints from btw blue ridge sources. */
	//[13]{
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length; j++)
		{
			size_t index = i*length + j;
			auto aNode = gNodes.at(index);
			aNode->setSource(NULL);
			aNode->setPrev(NULL);
			assert(aNode->getPrevious().expired());
			assert(aNode->getSource().expired());
			assert(aNode->isValid());

			if (aNode->isBlue())// || aNode->isBlack())
			{
				//aNode->setCost(0.0);
				aNode->setCost2(vector3d(0.0, 0.0, 0.0));
				aNode->setFixed(true);
			}
			else
			{
				aNode->setCost(MAX_FACT);
				aNode->setCost2(vector3d(MAX_FACT, 0.0, MAX_FACT));
				aNode->setFixed(false);
				if (aNode->isBlack())
				{
					aNode->setBlocker(true);
				}
			}
		}
	}
	
	//	pGraph->calculateCostBasedOnHeight();
	// pGraph->DijkstraByDistance(length);
//	pGraph->DijkstraByDistanceWithBlocker(length);
	pGraph->DijkstraByDistanceFindsRed(length);
	//}[13]
	
	/*[14] bufferRedEndPointNominees(criteria = NODE_STRIPS_BETWEEN_TWO_RIDGES, bufferedRedEndpoints); */
	//[14]{
	assert(nodeIndex.empty());
	vector<int> palToNodeIndex;
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length; j++)
		{
			size_t index = i*length + j;
			auto aNode = gNodes.at(index);
			for (auto anEdge : aNode->getEdges())
			{
				auto otherEnd = anEdge->getTarget().lock();
				if (!otherEnd->isDiagonalNeighborWith(aNode))
				{
					if (!aNode->isBlue() && !aNode->isBlack() && !aNode->isRed() &&
						!otherEnd->isBlue() && !otherEnd->isBlack() && !otherEnd->isRed() &&
						otherEnd->getRidgeId() < aNode->getRidgeId())
					{
						if (aNode->isValid())
						{
							assert((gNodes.at(index)->isValid()));
							assert(aNode->isValid());

							//choose the one with larger ridge ID
							nodeIndex.push_back(aNode->getIndex());
							aNode->setValid(false);
							palToNodeIndex.push_back(otherEnd->getIndex());

							assert(!(gNodes.at(index)->isRed()));
							assert(!aNode->isRed());

							assert(!(gNodes.at(index)->isValid()));
							assert(!aNode->isValid());
						}
					}
				}
			}
		}
	}
	assert(palToNodeIndex.size() == nodeIndex.size());
	//}[14]

	size_t RED_STEP_SIZE = (int)length / 25;
	/*[15] vector<ridge_points> redRidgeSources = void pGraph->setAsRedPaths(RED_STEP_SIZE, bufferedBlueEndpoints); ();  */
	//[15]{
	//findRedRidges: each red ridge has unique ridge ID, also unique from black and blue
	int nodeIndexId = 0;
	for (auto nIndex : nodeIndex)
	{
		auto aNode = gNodes.at(nIndex);
		if (!aNode->isValid())
		{
			auto origNode = gNodes.at(aNode->getIndex());
			assert(!origNode->isBlue() && !origNode->isBlack() && !origNode->isRed());
			bool allowedIn = true;

			if ((length - origNode->getLoc().z) < BLUE_STEP_SIZE ||
				origNode->getLoc().z < BLUE_STEP_SIZE ||
				(length - origNode->getLoc().x) < BLUE_STEP_SIZE ||
				origNode->getLoc().x < BLUE_STEP_SIZE )
			{
				allowedIn = false;
			}

			int foundIndex = palToNodeIndex.at(nodeIndexId);
			bool palIsNeighbor = false, ignoreThis = false;
			for (auto anEdge : origNode->getEdges())
			{
				auto otherEnd = anEdge->getTarget().lock();
				if (!otherEnd->isDiagonalNeighborWith(origNode))
				{
					if (otherEnd->getIndex() == foundIndex)
					{
						palIsNeighbor = true;
						if (otherEnd->isRed())
						{
							ignoreThis = true;
							allowedIn = false;
						}
						else
						{
							assert(!otherEnd->isBlue() && !otherEnd->isBlack() && !otherEnd->isRed());
						}
					}
				}
			}
			assert(palIsNeighbor);
			if (!ignoreThis)
			{
				auto otherEnd = gNodes.at(foundIndex);
				auto origSource = origNode->getSource().lock();
				assert(!origNode->getSource().expired());// because it's not fixed

				if (otherEnd->isFixed())
				{
					if (!origSource->setsOfRidgesOverlapWith(otherEnd->getAllRidgeIds()))
					{
						allowedIn = allowedIn && true;
					}
					else
					{
						allowedIn = allowedIn && false;
					}
				}
				else {
					assert(!otherEnd->isFixed());
					if (!origSource->setsOfRidgesOverlapWith((otherEnd->getSource().lock())->getAllRidgeIds()))
					{
						allowedIn = allowedIn && true;
					}
					else
					{
						allowedIn = allowedIn && false;
					}
				}
			}
			
			if (!allowedIn)
			{
				origNode->setValid(true);
			}
			else
			{
				vector<vector<int>> followUps;
				vector<int> result;
				assert(aNode->getIndex() == origNode->getIndex());
				assert(!origNode->isValid());
				
				counter++;
				origNode->setValid(true);
				for (int i = origNode->getLoc().z - RED_STEP_SIZE; i < origNode->getLoc().z + RED_STEP_SIZE; i++)
				{
					for (int j = origNode->getLoc().x - RED_STEP_SIZE; j < origNode->getLoc().x + RED_STEP_SIZE; j++)
					{
						if (j >= 0 && i >= 0 && j < length && i < length)
						{
							size_t anIndex = pGraph->getIndex(vector3d(j, 0.0, i), length);
							auto anotherNode = gNodes.at(anIndex);
							if (!anotherNode->isValid())
							{
								anotherNode->setValid(true);
							}
						}
					}
				}
				int ep1 = -1, ep2 = -1;
				if (true)
				{
					vector<int> tempFollowUp;

					tempFollowUp.push_back(origNode->getIndex());
					int theX = origNode->getLoc().x, theZ = origNode->getLoc().z;
					bool idealCondition = (theX != 1.0 && theZ != 1.0 && theX != length - 2.0 && theZ != length - 2.0);
					assert(idealCondition || (theX == 0.0 || theZ == 0.0 || theX == length - 1 || theZ == length - 1));

					assert(origNode->getIndex() == nIndex);
					assert(origNode->getIndex() == aNode->getIndex());
					auto aLoopingNode = gNodes.at(origNode->getIndex());

					bool foundIt = false;
					int otherEndPoint = -1;
					while (aLoopingNode->hasPrevious())
					{
						aLoopingNode = aLoopingNode->getPrevious().lock();
						if (!(aLoopingNode->isBlack() || aLoopingNode->isBlue() || aLoopingNode->isRed()))// not colored
						{
							tempFollowUp.push_back(aLoopingNode->getIndex());
							int theX = aLoopingNode->getLoc().x, theZ = aLoopingNode->getLoc().z;
							bool idealCondition = (theX != 1.0 && theZ != 1.0 && theX != length - 2.0 && theZ != length - 2.0);
							assert(idealCondition || (theX == 0.0 || theZ == 0.0 || theX == length - 1 || theZ == length - 1));
						}
						else
						{
							if (!foundIt)
							{
								otherEndPoint = aLoopingNode->getIndex();
							}
							foundIt = true;
							continue;
						}
					}
					assert(aLoopingNode->isBlue() || aLoopingNode->isRed());// isColored
					if (!foundIt)
					{
						otherEndPoint = aLoopingNode->getIndex();
					}
					ep1 = otherEndPoint;
					followUps.push_back(tempFollowUp);
				}
				auto otherEnd = gNodes.at(foundIndex);
				if (!otherEnd->isFixed() && !(otherEnd->isBlack() || otherEnd->isBlue() || otherEnd->isRed()))//and not colored
				{
					vector<int> tempFollowUp;

					tempFollowUp.push_back(otherEnd->getIndex());
					assert(otherEnd->isValid());
					int theX = otherEnd->getLoc().x, theZ = otherEnd->getLoc().z;
					bool idealCondition = (theX != 1.0 && theZ != 1.0 && theX != length - 2.0 && theZ != length - 2.0);
					assert(idealCondition || (theX == 0.0 || theZ == 0.0 || theX == length - 1 || theZ == length - 1));

					bool foundIt = false;
					int otherEndPoint = -1;
					while (otherEnd->hasPrevious())
					{
						otherEnd = otherEnd->getPrevious().lock();
						if (!(otherEnd->isBlack() || otherEnd->isBlue() || otherEnd->isRed()))// not colored
						{
							tempFollowUp.push_back(otherEnd->getIndex());
							int theX = otherEnd->getLoc().x, theZ = otherEnd->getLoc().z;
							bool idealCondition = (theX != 1.0 && theZ != 1.0 && theX != length - 2.0 && theZ != length - 2.0);
							assert(idealCondition || (theX == 0.0 || theZ == 0.0 || theX == length - 1 || theZ == length - 1));
						}
						else
						{
							if (!foundIt)
							{
								otherEndPoint = otherEnd->getIndex();
							}
							foundIt = true;
							continue;
						}
					}
					assert(otherEnd->isBlue() || otherEnd->isRed());// isColored
					if (!foundIt)
					{
						otherEndPoint = otherEnd->getIndex();
					}
					ep2 = otherEndPoint;
					followUps.push_back(tempFollowUp);
				}

				///*start*///

				assert(followUps.size() == 1|| followUps.size() == 2);
				/*start*/
				if (followUps.size() == 2)
				{
					for (size_t i = 0; i < followUps.at(1).size(); i++)
					{
						result.push_back((followUps.at(1)).at(followUps.at(1).size() - 1 - i));
					}
				}
				for (auto anItem : followUps.at(0))
				{
					result.push_back(anItem);
				}
				/*end*/
				for (auto anItem : result)
				{
					auto aCandid = gNodes.at(anItem);
					tempIndex.push_back(aCandid->getIndex());
					aCandid->setRed(true);
					aCandid->setValid(true);

					int theX = aCandid->getLoc().x, theZ = aCandid->getLoc().z;
					bool idealCondition = (theX != 1.0 && theZ != 1.0 && theX != length - 2.0 && theZ != length - 2.0);
					if(idealCondition || (theX == 0.0 || theZ == 0.0 || theX == length - 1 || theZ == length - 1))
					{
						bool good = true;
					}
					else
					{
						bool isItBad = true;
					}

					aCandid->setRidgeId(counter);
				}
				///*end*///

				indexes.push_back(tempIndex);
				assert(indexes.at(counter) == tempIndex);
				ridgeEndpoints.push_back(std::make_pair(ep1, ep2));
				tempIndex.erase(tempIndex.begin(), tempIndex.end());				
			}
		}
		nodeIndexId++;
	}
	//}[15]
	
	/*[11] ***** hMblk .-.> hEPblu */
	//[11]{
	// To setup heights on boundary, you need to approximate heights proportional to [~hopCountOfBlueEndPoint,~profile]
	//}[11]

	/*[12] void pGraph->setBlueRidgeHeight(blueRidgePoints, deltaHFirstToLast=[known from *****], range=DEFAULT, seaLevel = zero) "\__" */
	//[12]{
	// setBlueProfile
	//blue ridge cost is monotonically decreasing

	if (!experimenting)
	{
		//////////////////////set blue ridge heights
		for (size_t i = 0; i < length; i++)
		{
			for (size_t j = 0; j < length; j++)
			{
				size_t index = i*length + j;
				auto aNode = gNodes.at(index);
				aNode->setSource(NULL);
				aNode->setPrev(NULL);
				assert(aNode->getPrevious().expired());
				assert(aNode->getSource().expired());

				if (aNode->isBlack())
				{
					auto amount = maxH - aNode->getLoc().y >= 0;
					assert(amount >= 0);
					aNode->setCost(amount);
					aNode->setCost2(vector3d(0.0, 0.0, 0.0));
					aNode->setFixed(true);
				}
				else
				{
					aNode->setCost(MAX_FACT);
					aNode->setCost2(vector3d(MAX_FACT, 0.0, MAX_FACT));
					aNode->setFixed(false);
				}
			}
		}
		//pGraph->calculateCostBasedOnHeight();
		pGraph->DijkstraByDistance(length);

		double maxCostFound0 = 0.0;
		vector3d temp0;
		for (auto mNode : gNodes)
		{
			if (mNode->getCost() > maxCostFound0 && mNode->isBlue())
			{
				maxCostFound0 = mNode->getCost();
			}
		}
		for (auto& mNode : gNodes)
		{
			if (mNode->isBlue())
			{
				assert(mNode->getCost() != MAX_FACT, "assert no node has cost of infinity");
				temp0 = mNode->getLoc();

				temp0.y = fabs(maxCostFound0 - mNode->getCost());

				mNode->setLoc(temp0);
			}
		}
	}
	//////////////////////
	//}[12]

	/*[16] setRedProfile(deltaFromStartToEnd, lengthOfRedStrip); */
	//[16]{
	int theIndex = 0;
	bool nowInRedRange = false;
	for (auto aCuteIndex : indexes)
	{
		if (!nowInRedRange)
		{
			assert(!aCuteIndex.empty());
			auto bibiNode = gNodes.at(aCuteIndex.at(0));
			if (bibiNode->isRed())
			{
				assert(!bibiNode->isBlack() && !bibiNode->isBlue());
				nowInRedRange = true;

				assert(indexes.at(theIndex) == aCuteIndex);
				auto firstIndex = ridgeEndpoints.at(theIndex).first;
				auto secondIndex = ridgeEndpoints.at(theIndex).second;
				auto suggestedHeights = testAverageRandomWalks(aCuteIndex, gNodes.at(firstIndex)->getLoc().y, gNodes.at(secondIndex)->getLoc().y);
				int miniCounter = 0;
				for (auto miniIndex : aCuteIndex)
				{
					auto tempNode = gNodes.at(miniIndex);
					auto tLoc = tempNode->getLoc();
					tLoc.y = suggestedHeights.at(miniCounter);
					if (tLoc.y < maxNegativeHeight)
					{
						assert(maxNegativeHeight <= 0);
						tLoc.y = maxNegativeHeight;
					}
					tempNode->setLoc(tLoc);
					assert(!tempNode->isBlack() && !tempNode->isBlue());
					if (tempNode->getLoc().y < -0.1999 && tempNode->getLoc().y > -0.2001)
					{
						bool letMeKnow = true;
					}
					tempNode->setBlocker(true);
					miniCounter++;
				}
			}
			else
			{
				for (auto miniIndex : aCuteIndex)
				{
					auto tempNode = gNodes.at(miniIndex);
					assert(tempNode->isBlack() || tempNode->isBlue());
					if (tempNode->getLoc().y < -0.1999 && tempNode->getLoc().y > -0.2001)
					{
						bool letMeKnow = true;
					}
					tempNode->setBlocker(true);
				}
			}
		}
		else
		{
			assert(indexes.at(theIndex) == aCuteIndex);
			auto firstIndex = ridgeEndpoints.at(theIndex).first;
			auto secondIndex = ridgeEndpoints.at(theIndex).second;
			auto suggestedHeights = testAverageRandomWalks(aCuteIndex, gNodes.at(firstIndex)->getLoc().y, gNodes.at(secondIndex)->getLoc().y);
			int miniCounter = 0;
			for (auto miniIndex : aCuteIndex)
			{
				auto tempNode = gNodes.at(miniIndex);
				auto tLoc = tempNode->getLoc();
				tLoc.y = suggestedHeights.at(miniCounter);
				if (tLoc.y < maxNegativeHeight)
				{
					assert(maxNegativeHeight <= 0);
					tLoc.y = maxNegativeHeight;
				}
				tempNode->setLoc(tLoc);
				assert(!tempNode->isBlack() && !tempNode->isBlue());
				if (tempNode->getLoc().y < -0.1999 && tempNode->getLoc().y > -0.2001)
				{
					bool letMeKnow = true;
				}
				tempNode->setBlocker(true);
				miniCounter++;
			}
		}
		theIndex++;
	}
	//}[16]

	/*[Y] setup now*/
	//[Y]{
	assert(maxNegativeHeight <= 0);
	double maxColoredHeight = 0, minColoredHeight = 0;
	for (auto yeNode : gNodes)
	{
		if (yeNode->isBlack() || yeNode->isBlue() || yeNode->isRed())
		{
			assert(yeNode->isBlocker());
		}
		if (yeNode->isBlocker())
		{
			assert(yeNode->isBlack() || yeNode->isBlue() || yeNode->isRed());
			auto tLoc = yeNode->getLoc();
			tLoc.y = tLoc.y + fabs(maxNegativeHeight);
			yeNode->setLoc(tLoc);
			if (tLoc.y < minColoredHeight)
			{
				minColoredHeight = tLoc.y;
			}
			if (maxColoredHeight < tLoc.y)
			{
				maxColoredHeight = tLoc.y;
			}
		}
	}
	bool letsSeeHeights = true;
	vector<int> greenIndexes;
	for (auto loopedNode : gNodes)
	{
		if (!loopedNode->isBlocker())//is non-colored
		{
			loopedNode->setValid(false);
			greenIndexes.push_back(loopedNode->getIndex());
		}
		else	// is colored
		{
			loopedNode->setValid(true);
		}
	}
	//}[Y]

	/*[17] fillInGreenArea(); */
	//[17]{
	int patchCounter = 0;
	for (auto aGreenIndex : greenIndexes)
	{
		auto aGreenNode = gNodes.at(aGreenIndex);
		if (!aGreenNode->isValid())
		{
			patchCounter++;
			vector<int> indexesToGoInHeap;
			aGreenNode->setPatchId(patchCounter);
			aGreenNode->setValid(true);
			indexesToGoInHeap.push_back(aGreenNode->getIndex());
			auto pairOfSurroundingRidgeListAndNodesInPatch = pGraph->DijkA(indexesToGoInHeap, length);
			auto setOfRidgeidsContainer = pairOfSurroundingRidgeListAndNodesInPatch.first;
			auto nodesInCurrentPatch = pairOfSurroundingRidgeListAndNodesInPatch.second;
			/*for (auto nodeInCurrentPatch : nodesInCurrentPatch)
			{
				auto aNodeA = gNodes.at(nodeInCurrentPatch);
				vector<dijkstra_elements> aDE;
				for (size_t i = 0; i < setOfRidgeidsContainer.size(); i++)
				{
					dijkstra_elements tmpDE;
					tmpDE.cost = MAX_FACT;
					tmpDE.distFromSource = 0.0;
					aDE.push_back(tmpDE);
				}
				aNodeA->setDijkstraElements(aDE);
				assert(aNodeA->isValid() && aNodeA->getPatchId() == patchCounter);
			}
			*/
			int currentDijkElementsIndex = 0;
			for (auto setOfRidgeIds : setOfRidgeidsContainer) // [1,2] : [1,2 - 4,5,8 - 6 - 9]
			{
				vector<int> OtherIndexesToGoInHeap;
				for (auto aRidgeId : setOfRidgeIds)
				{
					for (auto gd : indexes.at(aRidgeId))
					{
						auto aNodeWithCurRidgeId = gNodes.at(gd);
						if (!aNodeWithCurRidgeId->isConnectingTwoBlueRidges())
						{
							//cost = inverted heights on ridges;
							double kcost = maxColoredHeight - aNodeWithCurRidgeId->getLoc().y;
							assert(kcost >= 0);
							aNodeWithCurRidgeId->setCost(kcost);
							OtherIndexesToGoInHeap.push_back(aNodeWithCurRidgeId->getIndex());
						}
					}
				}				
				pGraph->DijkB(currentDijkElementsIndex, OtherIndexesToGoInHeap, patchCounter, length);
				currentDijkElementsIndex++;
			}
			aGreenNode->setValid(true);
		}
	}

	//lets add patch index for all nodes, 0 means none, no 1 and higher are valid patch ids.
	//weight ~ 1/hopCount
	bool wannaShowRidgeColors = false;
	if (!wannaShowRidgeColors)
	{
		for (auto aGreenIndex : greenIndexes)
		{
			auto aGreenNode = gNodes.at(aGreenIndex);
			double sumOfWts = 0.0, sumOfWtDotCost = 0.0;
			assert(!aGreenNode->getDijkstraElements().empty());
			for (auto aveDijk : aGreenNode->getDijkstraElements())
			{
				sumOfWtDotCost += aveDijk.cost / aveDijk.distFromSource;
				sumOfWts += 1.0 / aveDijk.distFromSource;
			}
			assert(sumOfWts != 0.0);
			double resCost = sumOfWtDotCost / sumOfWts;
			aGreenNode->setCost(resCost);
			auto ttLoc = aGreenNode->getLoc();
			if (resCost >= maxColoredHeight)
			{
				ttLoc.y = 0.0;
			}
			else
			{
				ttLoc.y = maxColoredHeight - resCost;
			}
			aGreenNode->setLoc(ttLoc);
		}
	}
	//how to average
	//for each green(non-colored) node, at least two costs exist to do weight averaging.
	//Sigma(cost*wt)/Sigma(wt)

	//}[17]
	vector<double> heightfield;
	if (wannaShowRidgeColors)
	{
		for (auto& aNode : gNodes)
		{
			if (aNode->isBlack())
			{
				aNode->setCost(80);
			}
			else if (aNode->isBlue())
			{
				aNode->setCost(20);
			}
			else if (aNode->isRed())
			{
				aNode->setCost(50);

				int theX = aNode->getLoc().x, theZ = aNode->getLoc().z;
				bool idealCondition = (theX != 1.0 && theZ != 1.0 && theX != length - 2.0 && theZ != length - 2.0);
				assert(idealCondition || (theX == 0.0 || theZ == 0.0 || theX == length - 1 || theZ == length - 1));
			}
			else
			{
				aNode->setCost(0);
			}
		}
	}
	double maxCostFound = 0.0;
	vector3d temp;
	for (auto aNode : gNodes)
	{
		if (aNode->getCost() > maxCostFound)
		{
			maxCostFound = aNode->getCost();
		}
	}
	for (auto& aNode : gNodes)
	{
		assert(aNode->getCost() != MAX_FACT, "assert no node has cost of infinity");
		temp = aNode->getLoc();

		temp.y = fabs(maxCostFound - aNode->getCost());

		aNode->setLoc(temp);
	}
	
	for (auto aNode : gNodes)
	{
		heightfield.push_back(aNode->getLoc().y);
	}
	return heightfield;

	//return setHeightsBackWithThreshold();

}
/*----------------------------------------------------------------------*/
map<int, ridge_point> Model::getRidgeWithOnePeakOnRightMid() {
	map<int, ridge_point> heightOfPointsOnRidge;
	double m = STEEPNESS_FACTOR/5;
	size_t i = 0;
	heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(length / 2 + length / 7, m, length / 2 + length / 7)));
	return heightOfPointsOnRidge;
}
map<int, ridge_point> Model::getRidgeWithPeakOnBoundary() {
	map<int, ridge_point> heightOfPointsOnRidge;
	double m = STEEPNESS_FACTOR;
	for (size_t i = 0; i < length; i++)
	{
		heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, m * (1 - (double)i / length), (int)length / 2)));
	}
	return heightOfPointsOnRidge;
}
map<int, ridge_point> Model::getRidgeWithConstantHeight() {
	map<int, ridge_point> heightOfPointsOnRidge;
	double m = STEEPNESS_FACTOR;
	for (size_t i = 0; i < length; i++)
	{
		heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, m, (int)length / 2)));
	}
	return heightOfPointsOnRidge;
}
map<int, ridge_point> Model::getRidgeWithOneFixedEdge() {
	map<int, ridge_point> heightOfPointsOnRidge;
	double m = STEEPNESS_FACTOR;

	heightOfPointsOnRidge.insert(std::make_pair(0, std::make_tuple((int)length / 2, m - 5, (int)length / 2)));
	heightOfPointsOnRidge.insert(std::make_pair(1, std::make_tuple((int)length / 2 + 1, m, (int)length / 2)));

	return heightOfPointsOnRidge;
}
map<int, ridge_point> Model::getRidgeWithOnePeakAndTwoSlopesPerSide() {
	map<int, ridge_point> heightOfPointsOnRidge;
	double m = STEEPNESS_FACTOR;
	for (size_t i = 0; i < length; i++)
	{
		if (i < length / 4.0)
		{
			heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, (m / 0.25) * (double)i / length, (int)length / 2)));
		}
		else
		{
			heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, (m / 0.75) * (0.875 - (double)0.5 * i / length), (int)length / 2)));
		}
	}
	return heightOfPointsOnRidge;
}
map<int, ridge_point> Model::getRidgeWithOnePeakAndSameSlopesOnEachSide() {
	map<int, ridge_point> heightOfPointsOnRidge;
	double m = STEEPNESS_FACTOR;
	for (size_t i = 0; i < length; i++)
	{
		if (i< length / 2.0)
		{
			heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, (m / 0.5) * (double)i / length, (int)length / 2)));
		}
		else
		{
			heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, (m / 0.5) * (1.0 - (double)i / length), (int)length / 2)));
		}
	}
	return heightOfPointsOnRidge;
}
map<int, ridge_point> Model::getRidgeMountFuji() {
	map<int, ridge_point> heightOfPointsOnRidge;
	double m = STEEPNESS_FACTOR;
	for (size_t i = 0; i < length; i++)
	{
		if (i< length / 2.0)
		{
			heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, (8.0 / m) * (double)pow(i, 2), (int)length / 2)));
		}
		else
		{
			heightOfPointsOnRidge.insert(std::make_pair(i, std::make_tuple(i, (8.0 / m) * (double)pow((length - i), 2), (int)length / 2)));
		}
	}
	return heightOfPointsOnRidge;
}
/*----------------------------------------------------------------------*/
vector<double> Model::setHeights()
{
	vector<double> heightfield;
	//		pGraph->calculateHeightBasedOnCost();
	double maxCostFound = -1;
	vector3d temp;
	for (auto aNode : gNodes)
	{
		if (aNode->getCost() > maxCostFound && aNode->getCost() != MAX_FACT)
			maxCostFound = aNode->getCost();
	}
	for (auto& aNode : gNodes)
	{
		assert(aNode->getCost() != MAX_FACT, "assert no node has cost of infinity");
		temp = aNode->getLoc();
		temp.y = fabs(maxCostFound - aNode->getCost());
		aNode->setLoc(temp);
	}

	for (auto aNode : gNodes)
	{
		heightfield.push_back(aNode->getLoc().y);
	}
	return heightfield;
}
vector<double> Model::setHeightsBackWithThreshold()
{
	vector<double> heightfield;
	double maxFixedCostFound = -1.0, minFixedHeightFound = 0.0;
	vector3d temp;
	for (auto aNode : gNodes)
	{
		if (aNode->getCost() > maxFixedCostFound && aNode->getCost() != MAX_FACT && aNode->isFixed())
		{
			maxFixedCostFound = aNode->getCost();
			minFixedHeightFound = aNode->getLoc().y;
		}
	}
	for (auto& aNode : gNodes)
	{
		assert(aNode->getCost() != MAX_FACT, "assert no node has cost of infinity");
		temp = aNode->getLoc();
		if (aNode->getCost() >= maxFixedCostFound + minFixedHeightFound)
		{
			temp.y = 0.0;
		}
		else
		{
			temp.y = fabs(maxFixedCostFound + minFixedHeightFound - aNode->getCost());
		}
		aNode->setLoc(temp);
	}

	for (auto aNode : gNodes)
	{
		heightfield.push_back(aNode->getLoc().y);
	}
	return heightfield;
}
/*----------------------------------------------------------------------*/
void Model::segmentRidgelinesToPaths() {
	map<pair<int, int>, list<tuple<int, int, vec2Key>>> distributedGradients;
	list<tuple<int, double, ridge_point_status>> buffer;
	for (auto rp : pointsOnRidge)
	{
		buffer.push_back(std::make_tuple(rp.first, std::get<1>(rp.second), ridge_point_status::UNIDENTIFIED_POINT));
	}
	doSegment(buffer, distributedGradients);
}
/*----------------------------------------------------------------------*/
void Model::doSegment(list<tuple<int, double, ridge_point_status>>& buffer,	map<pair<int, int>, list<tuple<int, int, vec2Key>>>& listOfGradients)
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

	//	applyRidgeAnalysisCorrectedOnEdit(buffer, pointsOnRidge, listOfGradients, n);

	for (auto& aPath : paths)
	{
		vector<ridge_point> vectorOfTuples;
		vectorOfTuples = aPath;
		auto peak = *vectorOfTuples.begin();//x,h,z

		auto crd = vector3d(std::get<0>(peak), 0.0, std::get<2>(peak));
		auto aNode = gNodes.at(pGraph->getIndex(crd, length));
		auto pos = aNode->getLoc();
		pos.y = gNodes.size();
		aNode->setLoc(pos);
		aNode->setFixed(true);

		// implement nodes within heap distance.
//		distributeGradientsToNonFixedNodesDijkstra(*aPath, listOfGradients);
		//ADD NODES TO GRAPH PER NEW POINT FROM PATH?
	}
//	pGraph->DijkstraRegisterEndOfPaths();//if a node is the end point of any shortest path flag it true

//	pGraph->forEachEndOfPathEstimateCost();//There is only one sea level based on input ridge: Hs; 
}
/*----------------------------------------------------------------------*/
//temp starts below
/*----------------------------------------------------------------------*/
vector<double> Model::tempSegmentRidgelinesToPaths() {
	list<tuple<int, double, ridge_point_status>> buffer;
	for (auto rp : pointsOnRidge)
	{
		buffer.push_back(std::make_tuple(rp.first, std::get<1>(rp.second), ridge_point_status::UNIDENTIFIED_POINT));
	}
	assert(pointsOnRidge.size() == buffer.size());
	return tempDoSegment(buffer);
}
/*----------------------------------------------------------------------*/
vector<double> Model::tempDoSegment(list<tuple<int, double, ridge_point_status>>& buffer)
{
	if (buffer.empty() || buffer.size() == 1)
	{
		vector<double> result;
		//auto tempGraph = createGraph();
		//auto tempNodes = pGraph->getNodes();
		for (size_t i = 0; i < length; i++)
		{
			for (size_t j = 0; j < length; j++)
			{
				size_t index = i*length + j;
				auto aNode = gNodes.at(index);
				aNode->setFixed(false);
				result.push_back(0.0);
			}
		}
		return result;
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

	bool isLastStage = buffer.size() == pointsOnRidge.size();

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

	auto result = tempDoSegment(buffer);
	//return privateMethod1(result, paths, isLastStage);
	return privateMethodCircular(result, paths);
}
/*----------------------------------------------------------------------*/
vector<double> Model::privateMethodDiamond(vector<double> result, list<vector<ridge_point>> paths) {
	if (false)
	{
		return result;
	}
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length; j++)
		{
			size_t index = i*length + j;
			auto aNode = gNodes.at(index);
			aNode->setFixed(false);
			aNode->setPositionOnPath(0);
			aNode->setCost(MAX_FACT);
			aNode->setCost2(vector3d(MAX_FACT, 0.0, MAX_FACT));
		}
	}
	for (auto& aPath : paths)
	{
		vector<ridge_point> vectorOfTuples;
		vectorOfTuples = aPath;
		auto peak = *vectorOfTuples.begin();//x,h,z

		auto crd = vector3d(std::get<0>(peak), 0.0, std::get<2>(peak));
		auto aNode = gNodes.at(pGraph->getIndex(crd, length));
		if (!aNode->isFixed())
		{
			auto pos = aNode->getLoc();
			//	pos.y = length;
			aNode->setLoc(pos);
			aNode->setFixed(true);
		}
	}
	pGraph->DijkstraByDistance(length);
	for (auto& aPath : paths)
	{
		for (size_t i = 0; i < length; i++)
		{
			for (size_t j = 0; j < length; j++)
			{
				size_t index = i*length + j;
				auto aNode = gNodes.at(index);
				int M = aNode->getPositionOnPath();
				result.at(index) = M >= (*paths.begin()).size() ? 0.0 : std::get<1>((*paths.begin()).at(M));
			}
		}
	}
	return result;
}
/*----------------------------------------------------------------------*/
vector<double> Model::privateMethodCircular(vector<double> result, list<vector<ridge_point>> paths) {
	if (false)
	{
		return result;
	}
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length; j++)
		{
			size_t index = i*length + j;
			auto aNode = gNodes.at(index);
			aNode->setFixed(false);
			aNode->setPositionOnPath(0);
			aNode->setCost(MAX_FACT);
			aNode->setCost2(vector3d(MAX_FACT, 0.0, MAX_FACT));
		}
	}
	for (auto& aPath : paths)
	{
		vector<ridge_point> vectorOfTuples;
		vectorOfTuples = aPath;
		auto peak = *vectorOfTuples.begin();//x,h,z

		auto crd = vector3d(std::get<0>(peak), 0.0, std::get<2>(peak));
		auto aNode = gNodes.at(pGraph->getIndex(crd, length));
		if (!aNode->isFixed())
		{
			auto pos = aNode->getLoc();
			//	pos.y = length;
			aNode->setLoc(pos);
			aNode->setFixed(true);
		}
	}
	pGraph->DijkstraByDistance(length);
	for (auto& aPath : paths)
	{
		for (size_t i = 0; i < length; i++)
		{
			for (size_t j = 0; j < length; j++)
			{
				size_t index = i*length + j;
				auto aNode = gNodes.at(index);
				auto C = aNode->getCost2().mag();
				result.at(index) = C >= (*paths.begin()).size() ? 0.0 : std::get<1>((*paths.begin()).at((int)C));
			}
		}
	}
	return result;
}
/*----------------------------------------------------------------------*/
vector<double> Model::privateMethod1(vector<double> result, list<vector<ridge_point>> paths, bool isLastStage) {
	if (false)
	{
		return result;
	}
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length; j++)
		{
			size_t index = i*length + j;
			auto aNode = gNodes.at(index);
			aNode->setFixed(false);
			aNode->setPositionOnPath(0);
			aNode->setCost(MAX_FACT);
			aNode->setCost2(vector3d(MAX_FACT, 0.0, MAX_FACT));
		}
	}	
	double maxValue = 0;

	// deltaXFromPeak = (x - xPeak);
	// distanceFromPeak = getCost2().mag()
	// sum up. for x < xPeak, multiply 
	// cos(phi/2) -> 0.5 + (pointWithDistanceFromPeakProjectedOnDirection(pointOnPathAt(distanceFromPeak) - peakPoint) / distanceFromPeak)/2
		
	for (auto& aPath : paths)
	{
		vector<ridge_point> vectorOfTuples;
		vectorOfTuples = aPath;
		auto peak = *vectorOfTuples.begin();//x,h,z

		auto crd = vector3d(std::get<0>(peak), 0.0, std::get<2>(peak));
		auto aNode = gNodes.at(pGraph->getIndex(crd, length));
		if (!aNode->isFixed())
		{
			auto pos = aNode->getLoc();
			//	pos.y = length;
			aNode->setLoc(pos);
			aNode->setFixed(true);
		}
	}
	pGraph->DijkstraByDistance(length);
	for (auto& aPath : paths)
	{
		for (size_t i = 0; i < length; i++)
		{
			for (size_t j = 0; j < length; j++)
			{
				size_t index = i*length + j;
				auto aNode = gNodes.at(index);
				int M = aNode->getPositionOnPath();
				auto C = aNode->getCost2().mag();
				vector3d startPathLoc = vector3d(std::get<0>(*aPath.begin()), 0.0, std::get<2>(*aPath.begin()));
				vector3d endPathLoc = vector3d(std::get<0>(aPath.back()), 0.0, std::get<2>(aPath.back()));
				if (isLastStage)
				{
					// result.at(index) += C >= (*paths.begin()).size() ? 0.0 : std::get<1>((*paths.begin()).at((int)C)) - result.at(index);
				}
				else
				{
					result.at(index) += C >= (startPathLoc - endPathLoc).mag() ? 0.0 : 
						std::get<1>(aPath.at(0)) * fabs(C - (startPathLoc - endPathLoc).mag()) / (startPathLoc - endPathLoc).mag();
				}
			}
		}
	}
	//	for each returned node in past method
	//	assume hop is M; set height = M>pathSize? 0 : height at Mth element in path.
	return result;
}
/*----------------------------------------------------------------------
//void addRidges(10,0.5,rp) {
	size_t numberOfEndPoints = 10;
	double radiusDepartureForEndPoints = 0.5*(endOfRidge - startOfRidge).mag();
	setFixedNodes(allCost=0, allPointsOnRidge= where isFixed==true);
	vector vPrime = Dijkstra(when positionOnPath or cost2 == radiusDepartureForEndPoints -> insert into vector V; return V).
	assert(numberOfEndPoints < vPrime.size());
	vector endPoints = selectEndPointsAsHomogeneus(mod = vPrime.size()/numberOfEndPoints); get vPrime.at(mod*i + constantC where i = from 0 to numberOfEndPoints);
	vector<list> paths = recordPathsFromMainRidgeToEachEndPoint();
	//endPointPairs are for example endPoints.at(n) and endPoint.at(n-1)...
	forEachEndPointPairs(e.g. ;n;n-1) {
		
		// ? HOW-TO-DESIDE-ON-AREA-SIZE-FOR-SUBREGIONS ?

		randomised = rand()%(int)min(n.pathSize , (n-1).pathSize);
		ref=positionOnPath or cost2;
		pointA = lookUpForPointInPathWith(n, ref, at = randomised);
		pointB = lookUpForPointInPathWith(n-1, ref, at = randomised);

		// ? HOWTO-GET-PATH-FROM-POINT-A-TO-POINT-B ?
	}

}
----------------------------------------------------------------------*/
/*----------------------------------------------------------------------*/
void Model::makeyourownalgnow() {
	//assume getRidgeWithDiffSlopesPerSide;

	auto tempGraph = createGraph();
	auto tempNodes = pGraph->getNodes();
	tempGraph->DijkstraByDistance(length);
	/*
	//use Dijkstra cost2 priority
		for pointMin
			set vectorMP=(min-peak).unit()
			in dijkstra figure out bounding ridges ????????????????????
			foreach bounding ridge i
				set weight[i] = vectorMP.project(endOfRidge-peak)
				boundaryRef.push_back(pointAtSameCost2AsMin - peak)
			maxAllowedDistanceFromPeak = Sum(weight[i]*(endOfRidge-peak).mag())/Sum(weight[i])
			getDistancePercentage and average height at each bounding ridge for same distance percentage
	*/
}
/*----------------------------------------------------------------------*/
/*
STROKE
view: modify view to show top view on stokeUtil
model: enable random top view ridge placement
DIJKSTRA
set ridge points isBlack to true
set all isBlack into Dijkstra
	have random edge weights; set base range 
*/
double Model::setupBlackRidgeHeights(vector<int> indexes, double maxDesired) {
	double oldValue = 0.0, nextValue = oldValue;
	double coeff;
	int count = 0;
	vector<double> someHeight1, someHeight3;

	double RANDOM_WALK_STEP_SIZE = (maxDesired - oldValue) / (double)(indexes.size() / 2.0);

	for (size_t idx = 0; idx < indexes.size(); idx++)
	{
		count++;
		coeff = ((rand() % 20000) / 10000.0 - 1.0) * (RANDOM_WALK_STEP_SIZE);

		if (oldValue + coeff < 0.2 * maxDesired && count < 0.50 * indexes.size())
		{
			coeff = fabs(coeff);
		}
		if (oldValue + coeff > 0.8 * maxDesired & count > 0.50 * indexes.size())
		{
			coeff = -1 * fabs(coeff);
		}
		
		nextValue = oldValue + coeff;
		someHeight1.push_back(oldValue);
		oldValue = nextValue;
	}
	assert(indexes.size() == count);
 
	count = 0;
	double maxHeightFound = 0;
	for(auto idx : indexes)//for each element from first to last
	{
		count++;
		auto aNode = gNodes.at(idx);
		auto itsLoc = aNode->getLoc();
		
		if (count < 0.5* indexes.size())
		{
			itsLoc.y = someHeight1.at(count - 1);
		}
		else
		{
			itsLoc.y = someHeight1.at(count - 2 *(count - (int)(indexes.size()/2.0 + 1)));
		}
		//TODO: h = a*(1-x) + b(x)
		if (itsLoc.y > maxHeightFound)
		{
			maxHeightFound = itsLoc.y;
		}
		//someHeight3.push_back(itsLoc.y);//
		aNode->setLoc(itsLoc);
	}
	return maxHeightFound;
}


vector<double> Model::randomWalk(vector<int> indexes, double startingHeight) {
	double oldValue = startingHeight, nextValue = oldValue;
	double delta;
	int count = 0;
	double RANDOM_WALK_STEP_SIZE = 3.0;
	vector<double> someHeight1, someHeight2, someHeight3;

	for (size_t idx = 0; idx < indexes.size(); idx++)
	{
		count++;
		delta = ((rand() % 20000) / 10000.0 - 1.0) * (RANDOM_WALK_STEP_SIZE);

		nextValue = oldValue + delta;
		someHeight1.push_back(oldValue);
		oldValue = nextValue;
	}
	return someHeight1;
}

vector<double> Model::testAverageRandomWalks(vector<int> indexes, double firstHeight, double lastHeight) {
	vector<double> rw1, rw2, randomWalk3;
	rw1 = randomWalk(indexes, firstHeight);
	rw2 = randomWalk(indexes, lastHeight);
	size_t size = indexes.size();

	for (size_t i = 0; i < size; i++)
	{
		double x = (double)i / (size);
		double h;
		h = (rw1.at(i) * (1 - x) + rw2.at(i) * (x));
		randomWalk3.push_back(h);
	}

	//TODO: h = (rw1)*(1-x) + (rw2)(x)
	return randomWalk3;
}