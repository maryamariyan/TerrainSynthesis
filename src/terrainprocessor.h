#ifndef __TERRAINPROCESSOR_H
#define __TERRAINPROCESSOR_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define LENGTH 200
#define TRUE 1
#define FALSE 0
#define HEAP_FACTOR 7
#define BIGNUM 320001
#define RGB_COMPONENT_COLOR 255
#define MAX(X,Y) ((X) < (Y) ? (Y) : (X))
#define bXD LENGTH
#define bYD LENGTH
#define NOTEXIST -99
#define MAXRIDGES 1000

typedef struct {
	float x;
	float y;
	float z;
} vectorf;

typedef struct {
	double red;
	double green;
	double blue;
} rgbtotal;

typedef struct {
	unsigned char red;
	unsigned char green;
	unsigned char blue;
} rgbvector;

typedef struct {
	int x;
	int y;
	int z;
} intvector;

typedef struct {
	long int distance;
	long int temp;
	int id;
} strippednode;

typedef struct {
	strippednode entry[bXD*bYD*HEAP_FACTOR];
	int numentries;
	int cutoff;
} heaptype;

typedef struct {
	int x, y;
	rgbvector *data;
} PPMImage;

typedef struct {
	int loc;
	int val;
} ridgepoint;

typedef struct {
	int nearend;
	int farend;
	int cost;
} edgetype;

typedef struct {
	intvector loc;
	int id;
	int partcost;
	int dist;
	int regionid;
	edgetype edges[9];
	int whichpass;
	int numedges;
} nodetype;

class terrainprocessor {
public:
	terrainprocessor();
	~terrainprocessor();
	void fillbackup(PPMImage *img);
	void emptyheap(heaptype *heap);
	strippednode maxfromheap(heaptype *heap);
	void insertintoheap(strippednode innode, heaptype *heap);
	void ridges();
	void terrain();
	void makegraph(void);
	rgbvector* getresult();
private:
	int actx, acty;
	heaptype heap;
	int noisemap[bXD][bYD];
	int weight[bXD][bYD];
	int gradient[bXD][bYD];
	nodetype pix[bXD*bYD];
	rgbvector screen[bXD][bYD];
	int BASE[5][5]; //[bXD][bYD];
	rgbvector postfilter_c[bXD][bYD];
	rgbvector backup[bXD][bYD];
	int numridgepts;
	ridgepoint ridgedata[MAXRIDGES];
};

#endif