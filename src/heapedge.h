#ifndef __HEDGE_H
#define __HEDGE_H
#include <stdlib.h>
#include <stdio.h>
#include "heapNode.h"

class heapNode;
class heapEdge {
	
	private:

	public:
		
		heapNode* n1;
		heapNode* n2;
		
		float wt;
		float oWt;
		bool isValid;//if an edge connects two fixed nodes then it is valid
		bool beenVisited;//all edge need to be visited in edge rivision methods

//MARYAM ADDED
		bool beenVisitedInDebugMode; //is false by default, use for test purposes, when done debugging set back to false
//
		
		heapEdge();
		heapEdge(heapNode* nN1, heapNode* nN2, float nWt);
		~heapEdge();
		
		heapEdge operator = (heapEdge param);
		
//MARYAM ADDED
		float getWeight();
		void modifyWeight(float newWeight);
//
};

#endif
