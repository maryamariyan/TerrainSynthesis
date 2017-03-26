#include <assert.h>
#include "heapEdge.h"

heapEdge::heapEdge()
{
	this->n1 = NULL;
	this->n2 = NULL;
	this->wt = 0.0f;
	this->oWt = this->wt;
	this->isValid = false;
	this->beenVisited = false;
}

heapEdge::heapEdge(heapNode* nN1, heapNode* nN2, float nWt)
{
	this->n1 = nN1;
	this->n2 = nN2;
	this->wt = nWt;
	this->oWt = this->wt;
	this->isValid = false;
	this->beenVisited = false;
}

heapEdge::~heapEdge()
{
	this->n1 = NULL;
	this->n2 = NULL;
	this->wt = 0.0f;
	this->oWt = this->wt;
	this->isValid = false;
	this->beenVisited = false;
}

heapEdge heapEdge::operator = (heapEdge param)
{
	heapEdge temp;
	
	temp.n1 = param.n1;
	temp.n2 = param.n2;
	temp.wt = param.wt;
	temp.oWt = param.oWt;
	temp.isValid = param.isValid;
	temp.beenVisited = param.beenVisited;
	
	return temp;
}

float heapEdge::getWeight()
{
	return this->wt;
}

void heapEdge::modifyWeight(float newWeight)
{
	assert(this->oWt == this->wt);
	this->wt = newWeight;
	this->oWt = this->wt;
}