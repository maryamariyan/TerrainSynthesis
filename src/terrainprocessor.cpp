#include "terrainprocessor.h"
#include <assert.h>

terrainprocessor::terrainprocessor() {
	;
}
/*----------------------------------------------------------------------*/
terrainprocessor::~terrainprocessor() {
	;
}
/*----------------------------------------------------------------------*/
void terrainprocessor::fillbackup(PPMImage *img)
{
	int i, s, t;
	if (img)
	{
		for (i = 0; i<img->x*img->y; i++)
		{
			s = i%img->x;
			t = i / img->x;
			backup[img->y - 1 - t][s] = img->data[i];
		}
	}
//	512 = img->x;
//	512 = img->y;
}
/*----------------------------------------------------------------------*/
void terrainprocessor::emptyheap(heaptype *heap)
{
	// throw out contents of heap, if any

	heap->numentries = 0;
}
/*----------------------------------------------------------------------*/
strippednode terrainprocessor::maxfromheap(heaptype *heap)
{
	// remove the max from the heap and fix the heap
	strippednode retval;
	int i;
	strippednode top;
	int curindex;
	int rightchild, leftchild, largerchild;

	retval = heap->entry[0];

	// put last entry at top of heap:
	heap->numentries--;
	if (heap->numentries < 0)
	{
		fprintf(stderr, "Tried to pop from empty heap, crash\n");
		assert(false);
	}
	heap->entry[0] = heap->entry[heap->numentries];
	top = heap->entry[0];

	// trickle down:
	curindex = 0;
	while (curindex < heap->numentries / 2)
	{
		leftchild = 2 * curindex + 1;
		rightchild = leftchild + 1;

		if ((rightchild < heap->numentries) &&
			(heap->entry[leftchild].distance > // hi pri to lower cost
			heap->entry[rightchild].distance))
		{
			largerchild = rightchild;
		}
		else
		{
			largerchild = leftchild;
		}

		if (top.distance <= heap->entry[largerchild].distance)
			break; // done
		else
		{
			heap->entry[curindex] = heap->entry[largerchild];
			curindex = largerchild;
		}

	} // end while

	heap->entry[curindex] = top;

	return(retval);

}
/*----------------------------------------------------------------------*/
void terrainprocessor::insertintoheap(strippednode innode, heaptype *heap)
{

	// insert innode into heap-> First place at end, then trickle up.

	int curindex;
	int parent;
	strippednode bottom;
	int found;

	if (heap->numentries == 512 * 512 * HEAP_FACTOR)
	{
		// heap filled!
		fprintf(stderr, "Heap full!\n");
		assert(false);
	}

	heap->entry[heap->numentries] = innode;

	bottom = innode;

	curindex = heap->numentries;
	heap->numentries++;
	parent = (curindex - 1) / 2;

	while ((curindex > 0) &&
		(heap->entry[parent].distance >
		bottom.distance))
	{
		heap->entry[curindex] = heap->entry[parent];
		curindex = parent;
		parent = (parent - 1) / 2;
	}

	heap->entry[curindex] = bottom;

}
/*---------------------------------------------------------------------------*/
void terrainprocessor::ridges()
{
	// create initial ridge data
	numridgepts = 10;

	for (int i = 0; i < numridgepts; i++)
	{
		ridgedata[i].loc = rand() % (512*512);
		ridgedata[i].val = 0;
	}
	ridgedata[0].loc = (512*512) / 2 + 80;
}
/*---------------------------------------------------------------------------*/
void terrainprocessor::terrain()
{
	int bigdist = 0;
	int regsize;

	int mycost;
	strippednode sn, newnode;

	int s, t, d;
	int i, j, p, q, x, y;


	// initialize all distance values to +infinity
	for (int i = 0; i < 512 * 512; i++)
	{
		pix[i].partcost = 512*512 * 100; // approximation of infinity
		
//		pix[i].partcost = (int)(0.5*1255*sqrt((pix[i].loc.x-512/2)*
//		(pix[i].loc.x-512/2)+
//		(pix[i].loc.y-512/2)*
//		(pix[i].loc.y-512/2)));
	}

	emptyheap(&heap);

	for (int i = 0; i < numridgepts; i++)
	{
		sn.id = ridgedata[i].loc;
		sn.distance = ridgedata[i].val;
		pix[sn.id].partcost = ridgedata[i].val;

		insertintoheap(sn, &heap);
	}

	fprintf(stderr, "there are %i entries! (size %i %i)\n", numridgepts, 512, 512);

	while (heap.numentries > 0) // continue until nothing left
	{
		sn = maxfromheap(&heap);
		s = sn.id;

		if (sn.distance == pix[s].partcost)
			// verify that it's not a zombie
		{
			// this node is good, pix[s] can become part of current region
			if (bigdist < sn.distance)
				bigdist = sn.distance;


			// run through all neighbours and add to heap:
			for (j = 0; j != pix[s].numedges; j++)
			{
				t = pix[s].edges[j].farend;

				mycost = pix[s].partcost +
					// plus edge cost:
					pix[s].edges[j].cost + 1;
				if (pix[s].edges[j].cost < 1)
					fprintf(stderr, "Falase edge! %i %i\n", pix[s].edges[j].cost, j);
				if (
					(mycost < pix[t].partcost) // better path? 
					)
				{
					// it is better, so update, insert into heap
					pix[t].partcost = mycost;

					newnode.id = t;
					newnode.distance = mycost;
					insertintoheap(newnode, &heap);
				}
				else
				{
					// node was rejected, do nothing

				}
			}
		}


	} // processed whole terrain

	// next need to visualize distance values, put into screen
	int temp;

	for (int i = 0; i < 512*512; i++)
	{
		s = pix[i].loc.y;
		t = pix[i].loc.x;
		temp = 255 - (256 * pix[i].partcost / bigdist);

		screen[s][t].red = temp;
		screen[s][t].green = temp;
		screen[s][t].blue = temp;
	}
}
/*----------------------------------------------------------------------*/
void terrainprocessor::makegraph(void)
{
	int numnodes = 0;
	// set up 4-connected graph
	int i, j, k, s, t, p;
	edgetype newedge;
	intvector dir[4];
	numnodes = 0;

	dir[0].x = 0;
	dir[0].y = -1;

	dir[1].x = -1;
	dir[1].y = 0;

	dir[2].x = 0;
	dir[2].y = 1;

	dir[3].x = 1;
	dir[3].y = 0;


	for (i = 0; i != 512; i++)
	for (j = 0; j != 512; j++)
	{

		pix[numnodes].loc.x = i;
		pix[numnodes].loc.y = j;
		pix[numnodes].id = numnodes;
		pix[numnodes].partcost = 0;
		pix[numnodes].numedges = 0;

		for (k = 0; k != 4; k++)
		{
			// mod for tor topo:
			s = (i + dir[k].x + 512) % 512;
			t = (j + dir[k].y + 512) % 512;

			// no special topo
			s = (i + dir[k].x);
			t = (j + dir[k].y);

			if ((s >= 0) && (s < 512) && (t >= 0) && (t < 512))
			{
				p = pix[numnodes].numedges;
				newedge.nearend = j + i*512;
				newedge.farend = t + s*512;
				//newedge.cost = backup[j][i].red + 1 + (rand() % 50);
				newedge.cost = 1 + (rand() % 50);

				// + fabs(dir[k].y)*(rand()%200); -- for preferred crack direx
				pix[numnodes].edges[p] = newedge;
				pix[numnodes].numedges++;
			}
		}
		numnodes++;
	}
}
/*----------------------------------------------------------------------*/
rgbvector* terrainprocessor::getresult() {
	return *screen;
}