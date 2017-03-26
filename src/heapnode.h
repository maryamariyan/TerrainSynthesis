#ifndef __HNODE_H
#define __HNODE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;
#include "vector3d.h"
#include "heapEdge.h"

#define HN_CONN 8

#define HEDGE_DEBUG 0

#define MAX_FACT ((float)3.40282346638528860e+38)

class heapEdge;

class heapNode {
	
	private:
		
	public:
//MARYAM'S ADDED ATTRIBUTES		
//{
		//for hnode class:
		bool fixedNode; //by default false
		bool isValid; //by default true
		bool beenVisitedInDebugMode; //is false by default, use for test purposes, when done debugging set back to false
//}
		vector3d loc;
		heapEdge** edges;
		heapEdge** inEdges;
		int eCount;
		int inECount;
		
		// visited status
		int beenVisited;
		
		// cost
		float cost;
		vector3d cost2;
						
		// what position on the path am I?
		int positionOnPath;
		
		// Pointer to the source node this node originated from
		heapNode* source;
		
		// Previous Node on the path
		heapNode* prev;
								
		heapNode();
		heapNode(vector3d nLoc);
		~heapNode();
		
		void set(vector3d nLoc);
		void setLoc(vector3d nLoc);
		
		int addEdge(heapEdge* e);
		int addInEdge(heapEdge* e);
		
		// an inline function: the body will be inserted everywhere the function is used
		// http://www.java2s.com/Tutorial/Cpp/0180__Class/inlinemethod.htm
		inline bool operator <= (const heapNode & rhs) { return (this->cost <= rhs.cost); }
		
		heapNode operator = (heapNode param);
				
		// reset everything
		void reset();
				
		float getCost();
		vector3d getLocation();
		void setCost(float desiredCost);

//MARYAM METHODS: PUBLIC
//{
		bool isFixedNode();
		void setChecked(bool trueFalse);
		bool hasAlreadyBeenChecked();
		float distanceFrom(heapNode* otherNode);
		bool hasEdgeTo(heapNode* hn);
		void addUpToItsExistingEdgeWeights(float amount);
		bool isDiagonalNeighborWith(heapNode* hn);
		void createBidirectionalEdgeWith(heapNode* neighbor, float theChosenWeight);
		heapEdge* getEdgeTo(heapNode* hn);
		bool hasInfinityCost();

		vector3d getCost2();
		void setCost2(vector3d desiredCost);
//}
};

#endif
