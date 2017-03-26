#include "heapNode.h"
#include <memory>
#include <vector>
#include <algorithm>
#include <assert.h>

heapNode::heapNode()
{
	vector3d tmp(0.0f, 0.0f, 0.0f);
	
	set(tmp);
}

heapNode::heapNode(vector3d nLoc)
{
	set(nLoc);
}

void heapNode::set(vector3d nLoc)
{
	this->eCount = 0;
	this->inECount = 0;
	this->beenVisited = 0;
	this->cost = MAX_FACT;
	this->cost2 = vector3d(MAX_FACT, 0.0, MAX_FACT);
	this->positionOnPath = 0;
	this->fixedNode = false;
	this->isValid = true;
	
	this->loc = nLoc;
	
	this->source = NULL;
	this->prev = NULL;
	
	this->edges = (heapEdge**) malloc(sizeof(heapEdge*)*HN_CONN);
	this->inEdges = (heapEdge**) malloc(sizeof(heapEdge*)*HN_CONN);
	
	assert(this->edges && this->inEdges);
}

heapNode::~heapNode()
{
	int i;
	this->eCount = 0;
	this->inECount = 0;	
	this->beenVisited = 0;
	this->cost = MAX_FACT;
	this->cost2 = vector3d(MAX_FACT, 0.0, MAX_FACT);
	this->positionOnPath = 0;
	this->fixedNode = false;
	
	this->loc.loadZero();
	
	this->source = NULL;
	this->prev = NULL;
	
	if (this->edges)
	{
		for (i = 0; i < this->eCount; i++)
		{
			delete this->edges[i];
		}
	}

	if (this->inEdges)
	{
		for (i = 0; i < this->inECount; i++)
		{
			delete this->inEdges[i];
		}
//		free(this->inEdges);
	}
}

void heapNode::setLoc(vector3d nLoc)
{
	this->loc = nLoc;
}

int heapNode::addEdge(heapEdge* e)
{
	if (this->eCount == HN_CONN)
	{
		return 0;
	}
	
	this->edges[this->eCount] = e;
	this->eCount++;
	
	return 1;
}

int heapNode::addInEdge(heapEdge* e)
{	
	if (this->inECount == HN_CONN)
	{
		return 0;
	}
	
	this->inEdges[this->inECount] = e;
	this->inECount++;
	
	return 1;
}

/*// an inline function: the body will be inserted everywhere the function is used
// http://www.java2s.com/Tutorial/Cpp/0180__Class/inlinemethod.htm
bool heapNode::operator <= (const heapNode & rhs)
{
	return (this->cost <= rhs.d);
}*/

heapNode heapNode::operator = (heapNode param)
{
	heapNode temp;
	
	temp.loc = param.loc;
	temp.eCount = param.eCount;
	temp.inECount = param.inECount;
	temp.edges = param.edges;
	temp.inEdges = param.inEdges;
	temp.beenVisited = param.beenVisited;
	temp.cost = param.cost;
	temp.cost2 = param.cost2;
	temp.source = param.source;
	temp.prev = param.prev;
	temp.fixedNode = param.fixedNode;

	return temp;
}

void heapNode::reset()
{	
	this->beenVisited = 0;
	this->cost = MAX_FACT;
	this->cost2 = vector3d(MAX_FACT, 0.0, MAX_FACT);
	this->positionOnPath = 0;
	this->source = NULL;
	this->prev = NULL;
	this->isValid = true;
	this->fixedNode = false;
	
	for(int i=0; i < this->eCount; i++)
	{
		this->edges[i]->wt = this->edges[i]->oWt;
	}
}

float heapNode::getCost()
{
	return this->cost;
}

void heapNode::setCost(float desiredCost)
{
	this->cost = desiredCost;
}
//////////////////////////////////////////////////MARYAM'S ADDED METHODS:

vector3d heapNode::getCost2()
{
	return this->cost2;
}

void heapNode::setCost2(vector3d desiredCost)
{
	this->cost2 = desiredCost;
}

//for hnode class:
vector3d heapNode::getLocation()
{
	return this->loc;
}

//for hnode class:
bool heapNode::isFixedNode()
{
	return this->fixedNode;
}

//for hnode class:
float heapNode::distanceFrom(heapNode* otherNode)
{
	return sqrt(
				  pow(
				  this->getLocation().getX() - otherNode->getLocation().getX()
					,2) 
				+ pow(
					this->getLocation().getZ() - otherNode->getLocation().getZ()
					,2)
				);
}

//in hnode class
bool heapNode::hasEdgeTo(heapNode* hn)
{
	bool foundEdge = false;
	for(int i=0; i < this->eCount; i++)
		if(this->edges[i]->n2 == hn)
			foundEdge = true;
	return foundEdge;
}

//in hnode class
void heapNode::addUpToItsExistingEdgeWeights(float amount)
{
	bool foundAtLeastOne = false;
	for(int i=0; i < this->eCount; i++)
	{
		this->edges[i]->wt += amount;
		if (foundAtLeastOne == false)
			foundAtLeastOne = true;
	}
	assert(foundAtLeastOne, "assert you found At-Least-One-Existing edge to this node to add weight amount.");
}

//in hnode class
bool heapNode::isDiagonalNeighborWith(heapNode* hn)
{
	if (	fabs(this->getLocation().getX() - hn->getLocation().getX()) == 1
		&&	fabs(this->getLocation().getZ() - hn->getLocation().getZ()) == 1)
		return true;
	else
		return false;
}

//in hnode class
void heapNode::createBidirectionalEdgeWith(heapNode* neighbor, float theChosenWeight)
{
	bool foundDuplicate = false;
	vector3d neighborLocation;
	neighborLocation.set((float)((int)neighbor->getLocation().getX()), 0.0f, (float)((int)neighbor->getLocation().getZ()));
	neighborLocation = neighborLocation.roundVector();
	heapEdge* edgeFromCurrentNodeToNeighbor;
	heapEdge* edgeFromNeighborToCurrentNode;

	edgeFromCurrentNodeToNeighbor = new heapEdge();
	assert(edgeFromCurrentNodeToNeighbor, "ensure edge From CurrentNode To Neighbor Ptr is created");
	
	// assign end points of the edge
	// source node		(edgeFromCurrentNodeToNeighbor is outgoing from n1)
	edgeFromCurrentNodeToNeighbor->n1 = (this);
	// destination node (edgeFromCurrentNodeToNeighbor is incoming to n2)
	edgeFromCurrentNodeToNeighbor->n2 = (neighbor);
	
	// make sure n1 doesn't already have an edge that leads to n2
	for(int m=0; m < edgeFromCurrentNodeToNeighbor->n1->eCount; m++)
	{
		if (edgeFromCurrentNodeToNeighbor->n1->edges[m]->n2 == edgeFromCurrentNodeToNeighbor->n2)
		{
			// found a duplicate
			foundDuplicate = true;
		}
	}
	// found a duplicate - delete 'edgeFromCurrentNodeToNeighbor'
	if (foundDuplicate)
	{
		delete edgeFromCurrentNodeToNeighbor;
		printf("Program is about to exit: cannot create new edge\n");
		printf("Why? because a duplicate edge is found\n");
		std::cin.get();exit(-1);
	}
	// Create 'edgeFromNeighborToCurrentNode' - the opposite edge of 'edgeFromCurrentNodeToNeighbor'
	edgeFromNeighborToCurrentNode = new heapEdge();
	assert(edgeFromNeighborToCurrentNode, "ensure edge from Neighbor to Current node is added.");

	edgeFromNeighborToCurrentNode->n1 = edgeFromCurrentNodeToNeighbor->n2;
	edgeFromNeighborToCurrentNode->n2 = edgeFromCurrentNodeToNeighbor->n1;
	
	edgeFromCurrentNodeToNeighbor->wt = theChosenWeight;
	// Store the original wt
	// Allows for quick re-initializing
	edgeFromCurrentNodeToNeighbor->oWt	= edgeFromCurrentNodeToNeighbor->wt;
	// store the same weight in 'edgeFromNeighborToCurrentNode'
	edgeFromNeighborToCurrentNode->wt	= edgeFromCurrentNodeToNeighbor->wt;
	edgeFromNeighborToCurrentNode->oWt	= edgeFromNeighborToCurrentNode->wt;

	assert(edgeFromCurrentNodeToNeighbor->n1->addEdge(edgeFromCurrentNodeToNeighbor), "Ensure edge is added");
	assert(edgeFromNeighborToCurrentNode->n1->addEdge(edgeFromNeighborToCurrentNode), "Ensure edge is added");

	assert(edgeFromCurrentNodeToNeighbor->n2->addInEdge(edgeFromCurrentNodeToNeighbor), "Ensure Incoming Edge is added");
	assert(edgeFromNeighborToCurrentNode->n2->addInEdge(edgeFromNeighborToCurrentNode), "Ensure Incoming Edge is added");
}

//in hnode class
bool heapNode::hasInfinityCost()
{
	if (this->cost == MAX_FACT)
		return true;
	else
		return false;
}

//in hnode class
heapEdge* heapNode::getEdgeTo(heapNode* hn)
{
	bool foundEdge = false;
	heapEdge* theEdge;
	for(int i=0; i < this->eCount; i++)
		if(this->edges[i]->n2 == hn)
		{
			foundEdge = true;
			theEdge = this->edges[i];
		}
	assert(foundEdge, "hint: use 'hasEdgeTo()' before using 'getEdgeTo()'");
	return theEdge;
}

void ProcessLargeObject(const heapNode& hn) {
	return;
}
void usingSmartPointersForHNode() {
	vector3d mvector;
	mvector.set(1, 1, 1);

	// Using a raw pointer -- not recommended.
	heapNode* pNode = new heapNode(mvector);
	// Don't forget to delete! 
	delete pNode;

	// Declare a smart pointer on stack and pass it the raw pointer.
	unique_ptr<heapNode> smartPointer(new heapNode(mvector));
	float c = smartPointer->cost;

	// Create the object and pass it to a smart pointer
	std::unique_ptr<heapNode> aUniquePointer(new heapNode(mvector));
	//Call a method on the object
	aUniquePointer->hasInfinityCost();
	// Pass a reference to a method.
	ProcessLargeObject(*aUniquePointer);

	// Create the object and pass it to a smart pointer
	std::unique_ptr<heapNode> anotherUniquePointer(new heapNode(mvector));
	//Call a method on the object
	anotherUniquePointer->hasInfinityCost();
	// Free the memory before we exit function block.
	anotherUniquePointer.reset();
	// Do some other work...

	// Create the object and pass it to a smart pointer
	std::unique_ptr<heapNode> justAnotherUniquePointer(new heapNode(mvector));
	//Call a method on the object
	justAnotherUniquePointer->hasInfinityCost();
	// Pass raw pointer to an edge
	std::unique_ptr<heapEdge> aUniqueEdgePointer(new heapEdge(justAnotherUniquePointer.get(), anotherUniquePointer.get(), c));

	// Use make_shared function when possible.
	auto sp1 = make_shared<heapNode>(mvector);
	// Ok, but slightly less efficient.  
	// Note: Using new expression as constructor argument 
	// creates no named variable for other code to access.
	shared_ptr<heapNode> sp2(new heapNode(mvector));
	// When initialization must be separate from declaration, e.g. class members,  
	// initialize with nullptr to make your programming intent explicit.
	shared_ptr<heapNode> sp5(nullptr);
	//Equivalent to: shared_ptr<heapNode> sp5; 
	//...
	sp5 = make_shared<heapNode>();

	//Initialize with copy constructor. Increments ref count.
	auto sp3(sp2);
	//Initialize via assignment. Increments ref count.
	auto sp4 = sp2;
	//Initialize with nullptr. sp7 is empty.
	shared_ptr<heapNode> sp7(nullptr);
	// Initialize with another shared_ptr. sp1 and sp2 
	// swap pointers as well as ref counts.
	sp1.swap(sp2);

	vector<shared_ptr<heapNode>> v;
	mvector.set(1, 0, 0);
	v.push_back(make_shared<heapNode>(mvector));
	mvector.set(0, 1, 0);
	v.push_back(make_shared<heapNode>(mvector));
	mvector.set(0, 0, 1);
	v.push_back(make_shared<heapNode>(mvector));
	vector<shared_ptr<heapNode>> v2;
	//	remove_copy_if(v.begin(), v.end(), back_inserter(v2), [](shared_ptr<heapNode> hn)
	//	{
	//		return hn->loc.x == 1;
	//	});
	for (const auto& hn : v2)
	{
		cout << hn->cost << ":" << hn->beenVisited << endl;
	}

	mvector.set(1, 0, 0);
	v.push_back(make_shared<heapNode>(mvector));
	mvector.set(0, 1, 0);
	v.push_back(make_shared<heapNode>(mvector));
	mvector.set(0, 0, 1);
	v.push_back(make_shared<heapNode>(mvector));

	// Initialize two separate raw pointers. 
	// Note that they contain the same values.
	mvector.set(1, 0, 0);
	auto hn1 = new heapNode(mvector);
	mvector.set(0, 1, 0);
	auto hn2 = new heapNode(mvector);
	// Create two unrelated shared_ptrs.
	shared_ptr<heapNode> p1(hn1);
	shared_ptr<heapNode> p2(hn2);
	// Unrelated shared_ptrs are never equal.
	wcout << "p1 < p2 = " << std::boolalpha << (p1 < p2) << endl;
	wcout << "p1 == p2 = " << std::boolalpha << (p1 == p2) << endl;
	// Related shared_ptr instances are always equal.
	shared_ptr<heapNode> p3(p2);
	wcout << "p3 == p2 = " << std::boolalpha << (p3 == p2) << endl;
}