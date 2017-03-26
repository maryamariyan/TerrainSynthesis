#pragma once
#include "ofMain.h"

#include <vector>
#include <algorithm>
#include <memory>
#include <functional>
#include <queue>

//#include <Eigen/Eigen>
//#include <Eigen/Sparse>

#include <iostream>

#include <algorithm>
#include <memory>
#include <assert.h>
#include <functional>
#include <queue>

#include <vector>
#include <map>
#include <list>

#include "vector3d.h"

#include "node.h"
#include "edge.h"
#include "graph.h"
#include "dualgraph.h"
#include "model.h"

#include "terrainprocessor.h"

using namespace std;

class CompareByPositionOnPath;
class CompareByCost;
class CompareByDistance;

class node;
typedef weak_ptr<node> node_wp;
typedef shared_ptr<node> node_sp;
typedef list<node_sp> nodes_t;

class edge;
typedef weak_ptr<edge> edge_wp;
typedef shared_ptr<edge> edge_sp;
typedef vector<edge_sp> edges_sv;
typedef vector<edge_wp> edges_wv;

class graph;
typedef weak_ptr<graph> graph_wp;
typedef shared_ptr<graph> graph_sp;
typedef tuple<int, double, int> ridge_point; // x,h,z

class dual_graph;
typedef weak_ptr<dual_graph> dual_graph_wp;
typedef shared_ptr<dual_graph> dual_graph_sp;

class vec2Key;

typedef enum solve_status {
	DONT_SOLVE = 0,
	SOLVE_DIJKSTRA = -1,
	SOLVE_POISSON = -2,
	SOLVE_DIJKSTRA_VALUES = -3,
	SOLVE_POISSON_VALUES = -4,
	SOLVE_DIJKSTRA_GRADIENTS = -5,
	SOLVE_POISSON_GRADIENTS = -6
} solve_status;

typedef struct status {
	bool mouse_over_content;
	bool sketch_mode;
} status;

typedef struct canvas {
	ofColor bg_color;
	std::string label;
	ofPoint start_position;
	vec2Key content_size;
	vec2Key padding_top;
	vec2Key padding_bottom;
	vec2Key margin_top;
	vec2Key margin_bottom;
	status canvas_status;
} canvas;

//typedef Eigen::Triplet<double> Tr;
typedef struct box {
	vec2Key grid_size;
	//std::vector<Tr> constraint_triplets;
	vector<double> constraints;
	vector<vec2Key> cross_section;
	map<vec2Key, vec2Key> gradient_map;
	map<pair<int, int>, double> value_map;
} box;

namespace ProgressDetail {
	typedef enum month {
		NOVEMBER_2013 = 201311,
		DECEMBER_2013 = 201312,
		JANUARY_2014 = 201401,
		PROGRESS = -1,
		TODO = -2,
		VIEW = -3,
		DIJKSTRA = -4
	} month;
	class aClassInProgressUtil : ofBaseApp {
	public:
		void setup()
		{
			infoCanvas.start_position.set(420, 420);
			infoCanvas.margin_top = vec2Key(0, 0);
			infoCanvas.padding_top = vec2Key(0, 0);
			buffer = month::TODO;
			stringBuffer = getCommentsOn(buffer);
		}
		void update()
		{
			;
		}
		void draw()
		{
			drawCommentsFor(buffer);
		}
		void keyPressed(int key)
		{
			if (key == 'n')
			{
				buffer = month::NOVEMBER_2013;
			}
			else if (key == 'd')
			{
				buffer = month::DECEMBER_2013;
			}
			else if (key == 'j')
			{
				buffer = month::JANUARY_2014;
			}
			else if (key == 't')
			{
				buffer = month::TODO;
			}
			else if (key == 'p')
			{
				buffer = month::PROGRESS;
			}
			else if (key == 'v')
			{
				buffer = month::VIEW;
			}
			else if (key == 'u')
			{
				buffer = month::DIJKSTRA;
			}
			stringBuffer.clear();
			stringBuffer = getCommentsOn(buffer);
		}
	private:
		vector<string> getCommentsOn(int n) {
			vector<string> str;
			switch (n)
			{
			case NOVEMBER_2013:
				str.push_back("NOVEMBER 2013");
				str.push_back("in setup create terrain");
				str.push_back("in update apply heat equation");
				str.push_back("in draw show");
				break;
			case DECEMBER_2013:
				str.push_back("DECEMBER 2013");
				str.push_back("STROKE::accept sketch");
				str.push_back("\tphase one: get ridge from top view\t=> check, PARTIAL");
				str.push_back("\tphase two: get its height profile\t=> check, PARTIAL");
				str.push_back("SOLVER::compute low resolution poisson solver for input");
				str.push_back("\tpoisson constraints\t=> check, DONE");
				str.push_back("\tvalue constraints\t=> check, DONE");
				str.push_back("\tgradient constraints\t=> check, DONE");
				str.push_back("DIJKSTRA::apply dijkstra by setting edge as computed slopes from solver");
				str.push_back("\tset edge weights\t->slope\t=> check, FINAL TOUCH");
				str.push_back("\tset node cost\t->height\t=> check, DONE");
				break;
			case JANUARY_2014:
				str.push_back("JANUARY 2014");
				str.push_back("1-\tuse shape detected from x-z path instead of using circular shape.");
				str.push_back("2-\timprove node detection: undefined, ordinary, peak or flat; "); 
				str.push_back("may include flats as with a path and keep either ordinary or undefined.");
				str.push_back("3-\tdetect edge weights of smaller scales first and set them as valid.");
				str.push_back("4-\twhen at larger scale do the same steps only skip valid edges.");
				str.push_back("WON'T FIX:");
				str.push_back("1-\timprove setInput() and input resize");
				str.push_back("1-\ttop view is used for sending off info to solvers");
				str.push_back("2-\tstore differences not values");
				str.push_back("3-\timplement algorithm to propagate cones");
				str.push_back("5-\tslope zero should be swapped with ghost peaks");
				break;
			case VIEW:
				str.push_back("5-\tif value of any point in side view is known visualize it");
				str.push_back("6-\twhen mouse hovered on point from known ridge change its color to highlight");
				str.push_back("7-\tif mouse clicked on highlighted ridge then select that ridge cross section");
				str.push_back("8-\tside view is translated back in top view");
				str.push_back("9-\ttop view is used for sending off info to solvers");
				break;
			case PROGRESS:
				str.push_back("PROGRESS");
				str.push_back("-\tSet boundaryleft/boundaryRight = 1/(difAt cross section ends).");
				str.push_back("-\tMake more flexible input ridge validation for cross section.");
				str.push_back("-\tInclude Dijkstra default and per input calculations in project.");
				str.push_back("-\tPut solve update in a loop.");
				str.push_back("-\tLoad image instead of point draw, may increase rendering speed.");
				break;
			case TODO:
				str.push_back("TODO");
				str.push_back("todo 9: for any user cross section input get mid-H cross section and set 4 boundaries to that.");
				str.push_back("todo 3: use calculated minimum in setInput to normalize boundary.");
				str.push_back("todo 6: make input ridge validation more flexible.");
				str.push_back("todo 8: improve gradient randomization process (currently commented out).");
				break;
			case DIJKSTRA:
				str.push_back("Dijkstra");
				str.push_back("Press 4 to show Dijkstra from solver.");
				str.push_back("-\tEdge weights = solver + epsilon.");
				str.push_back("-\tImplement method to propagate edge weights from image.");
				str.push_back("-\tImplement method.");
				break;
			}
			return str;
		}
		void drawCommentsFor(int month) {
			int count = 1;
			for (auto tmp = stringBuffer.begin(); tmp < stringBuffer.end(); tmp++, count++)
			{
				const char * c = tmp->c_str();
				ofDrawBitmapString(c, infoCanvas.start_position.x + infoCanvas.margin_top.x + infoCanvas.padding_top.x,
					infoCanvas.start_position.y + infoCanvas.margin_top.y + infoCanvas.padding_top.y + count * 20);
			}
		}
		canvas infoCanvas;
		int buffer;
		vector<string> stringBuffer;
	};
}

namespace StrokeUtil {
#include <string>;
	bool _compare_min_x(ofPoint const &p1, ofPoint const &p2) { return p1.x < p2.x; }
	bool _compare_min_y(ofPoint const &p1, ofPoint const &p2) { return p1.y < p2.y; }
	class bresenham {
	public:
		static vector<ofPoint> getAllPoints(vector<ofPoint> s) {
			vector<ofPoint> v, t;
			vector<ofPoint>::iterator it = s.begin();
			v.push_back(*it);
			for (vector<ofPoint>::iterator itt = it + 1; itt < s.end(); itt++)
			{
				t = getElementsBetween(*it, *itt);
				for (vector<ofPoint>::iterator ittt = t.begin(); ittt < t.end(); ittt++)
				{
					v.push_back(*ittt);
				}
				t.clear();
				v.push_back(*itt);
				it = itt;
			}
			return v;
		}
	private:
		static vector<ofPoint> getElementsBetween(ofPoint a, ofPoint b)
		{
			vector<ofPoint> elements;
			int p1x = a.x, p1z = a.y, p2x = b.x, p2z = b.y;
			int F, x, z;
			int tmp; //for swap
			int count = 0;
			if (p1x > p2x)  // Swap points if p1 is on the right of p2
			{
				//swap(p1x, p2x);
				tmp = p1x;	p1x = p2x;	p2x = tmp;
				//swap(p1z, p2z);
				tmp = p1z;	p1z = p2z;	p2z = tmp;
			}

			// Handle trivial cases separately for algorithm speed up.
			// Trivial case 1: m = +/-INF (Vertical line)
			if (p1x == p2x)
			{
				if (p1z > p2z)  // Swap z-MVECTOR3inates if p1 is above p2
				{
					//swap(&p1z, &p2z);
					tmp = p1z;	p1z = p2z;	p2z = tmp;
				}

				x = p1x;
				z = p1z;
				while (z <= p2z)
				{
					elements.push_back(ofPoint(x, z));
					z++;
				}
				return elements;
			}
			// Trivial case 2: m = 0 (Horizontal line)
			else if (p1z == p2z)
			{
				x = p1x;
				z = p1z;

				while (x <= p2x)
				{
					elements.push_back(ofPoint(x, z));
					x++;
				}
				return elements;
			}


			int dz = p2z - p1z;  // z-increment from p1 to p2
			int dx = p2x - p1x;  // x-increment from p1 to p2
			int dz2 = (dz << 1);  // dz << 1 == 2*dz
			int dx2 = (dx << 1);
			int dz2_minus_dx2 = dz2 - dx2;  // precompute constant for speed up
			int dz2_plus_dx2 = dz2 + dx2;


			if (dz >= 0)    // m >= 0
			{
				// Case 1: 0 <= m <= 1 (Original case)
				if (dz <= dx)
				{
					F = dz2 - dx;    // initial F

					x = p1x;
					z = p1z;
					while (x <= p2x)
					{
						elements.push_back(ofPoint(x, z));
						if (F <= 0)
						{
							F += dz2;
						}
						else
						{
							z++;
							F += dz2_minus_dx2;
						}
						x++;
					}
				}
				// Case 2: 1 < m < INF (Mirror about z=x line
				// replace all dz by dx and dx by dz)
				else
				{
					F = dx2 - dz;    // initial F

					z = p1z;
					x = p1x;
					while (z <= p2z)
					{
						elements.push_back(ofPoint(x, z));
						if (F <= 0)
						{
							F += dx2;
						}
						else
						{
							x++;
							F -= dz2_minus_dx2;
						}
						z++;
					}
				}
			}
			else    // m < 0
			{
				// Case 3: -1 <= m < 0 (Mirror about x-axis, replace all dz by -dz)
				if (dx >= -dz)
				{
					F = -dz2 - dx;    // initial F

					x = p1x;
					z = p1z;
					while (x <= p2x)
					{
						elements.push_back(ofPoint(x, z));
						if (F <= 0)
						{
							F -= dz2;
						}
						else
						{
							z--;
							F -= dz2_plus_dx2;
						}
						x++;
					}
				}
				// Case 4: -INF < m < -1 (Mirror about x-axis and mirror 
				// about z=x line, replace all dx by -dz and dz by dx)
				else
				{
					F = dx2 + dz;    // initial F

					z = p1z;
					x = p1x;
					while (z >= p2z)
					{
						elements.push_back(ofPoint(x, z));
						if (F <= 0)
						{
							F += dx2;
						}
						else
						{
							x++;
							F += dz2_plus_dx2;
						}
						z--;
					}
				}
			}
			return elements;
		}
	};
	class aClassInStrokeUtil : ofBaseApp {
#define SSIZE LENGTH;
	public:
		void setup()
		{
			ofBackground(225, 255, 255);
			ofNoFill();
			//ofEnableAlphaBlending ();
			int n = SSIZE;
			topViewCanvas.content_size = vec2Key(n, n);
			topViewCanvas.label = "top view";
			topViewCanvas.margin_top = vec2Key(10, 10);
			topViewCanvas.margin_bottom = vec2Key(10, 10);
			topViewCanvas.padding_top = vec2Key(0, 0);
			topViewCanvas.padding_bottom = vec2Key(0, 0);
			topViewCanvas.start_position.set(0, 0);
			topViewCanvas.canvas_status.mouse_over_content = false;
			topViewCanvas.canvas_status.sketch_mode = false;

			sideViewCanvas.content_size = vec2Key(n, n);
			sideViewCanvas.label = "side view";
			sideViewCanvas.margin_top = vec2Key(10, 10);
			sideViewCanvas.margin_bottom = vec2Key(10, 10);
			sideViewCanvas.padding_top = vec2Key(0, 0);
			sideViewCanvas.padding_bottom = vec2Key(0, 0);
			sideViewCanvas.start_position.set(
				topViewCanvas.start_position.x + topViewCanvas.margin_top.x + topViewCanvas.padding_top.x + topViewCanvas.content_size.x + topViewCanvas.margin_bottom.x + topViewCanvas.padding_bottom.x,
				topViewCanvas.start_position.y);
			sideViewCanvas.canvas_status.mouse_over_content = false;
			sideViewCanvas.canvas_status.sketch_mode = false;

			lastVisitedCanvas = topViewCanvas;
			isValidRidgeInput = true;
			isSetCrossSection = false;
			setRandomSideView = true;

			theCurve.clear();
			ridgeCurves.clear();
		}
		void update()
		{
			if (topViewCanvas.canvas_status.mouse_over_content && topViewCanvas.canvas_status.sketch_mode)
			{
				;	//if the side view has defined points for previously selected ridgeCurve, then save the ridge line
			}
			else if (sideViewCanvas.canvas_status.mouse_over_content && sideViewCanvas.canvas_status.sketch_mode)
			{
				if (setRandomSideView && points.size() == 0)
				{
					setRandomCurve();
					setRandomSideView = false;
					evaluateAsRidge();
					sideViewCanvas.canvas_status.sketch_mode = false;
				}
				else
				{
					evaluateAsRidge();
				}
			}
			else if (points.size() != 0)
			{
				if (lastVisitedCanvas.label == topViewCanvas.label)
				{
					saveCrossSection();
//					sideViewCanvas.content_size.y = points.size();	//may be constant
					sideViewCanvas.content_size.x = crossSectionSet.size();
				}
				else if (lastVisitedCanvas.label == sideViewCanvas.label)
				{
					saveRidgeIfValid();
				}
				this->points.clear();
				isValidRidgeInput = true;
			}
		}
		void draw()
		{
			drawCanvas(topViewCanvas);
			drawCanvas(sideViewCanvas);
			if (!sideViewCanvas.canvas_status.sketch_mode)
			{
				drawRidgeOnCanvas(sideViewCanvas);
			}
			if (!topViewCanvas.canvas_status.sketch_mode)
			{
				drawTopViewOnCanvas(topViewCanvas);
			}
			if (points.size() != 0)
			{
				ofSetColor(255, 0, 0);
				ofBeginShape();
				for (vector<ofPoint>::iterator it = points.begin(); it < points.end(); it++) {
					ofPoint p = *it;
					ofVertex(p.x, p.y);
				}
				ofEndShape();
				ofSetColor(0, 0, 0);
				if (topViewCanvas.canvas_status.mouse_over_content)
				{
					;
				}
				else if (sideViewCanvas.canvas_status.mouse_over_content)
				{
					drawStrokeInfo();
				}
			}
		}
		void keyPressed(int key)
		{
			if (key == '1' && topViewCanvas.canvas_status.mouse_over_content)
			{
				topViewCanvas.canvas_status.sketch_mode = !topViewCanvas.canvas_status.sketch_mode;
			}
			if (key == '2' && sideViewCanvas.canvas_status.mouse_over_content)
			{
				sideViewCanvas.canvas_status.sketch_mode = !sideViewCanvas.canvas_status.sketch_mode;
			}
			if (key == 'x')
			{
				setRandomSideView = true;
			}
		}
		void mouseMoved(int x, int y) {
			if (onCanvasContent(topViewCanvas, x, y))
			{
				topViewCanvas.canvas_status.mouse_over_content = true;
				setLastVisitedCanvas(topViewCanvas);
			} else {
				topViewCanvas.canvas_status.mouse_over_content = false;
				topViewCanvas.canvas_status.sketch_mode = false;
			}
			if (onCanvasContent(sideViewCanvas, x, y))
			{
				sideViewCanvas.canvas_status.mouse_over_content = true;
				setLastVisitedCanvas(sideViewCanvas);
			} else {
				sideViewCanvas.canvas_status.mouse_over_content = false;
				sideViewCanvas.canvas_status.sketch_mode = false;
			}
			if ((sideViewCanvas.canvas_status.sketch_mode && sideViewCanvas.canvas_status.mouse_over_content) || (topViewCanvas.canvas_status.sketch_mode && topViewCanvas.canvas_status.mouse_over_content))
			{
				this->points.push_back(ofPoint(x, y));
			}
		}
		void mouseDragged(int x, int y, int button) {
			;
		}
		void mousePressed(int x, int y, int button) {
			;
		}
		void mouseReleased(int x, int y, int button) {
			;
		}
		map<vec2Key, double> resizeInputTo(int desiredSize)
		{
			int n = SSIZE;
			if (n != desiredSize)
			{
				assert(n % desiredSize == 0);
				size_t scale = n / desiredSize;
				map<vec2Key, double> temp;
				//TODO: later try to replace this with STL remove_if
				for (auto it = locationToValue.begin(); it != locationToValue.end(); it++)
				{
					if ((int)it->first.x % scale == 0 && (int)it->first.y % scale == 0) {
						temp.insert(std::make_pair(vec2Key(it->first.x / scale, it->first.y / scale), it->second));
					}
				}
				return temp;
			}
			return locationToValue;
		}
		box empty(box m) {
			//m.constraint_triplets.clear();
			m.constraints.clear();
			m.cross_section.clear();
			m.gradient_map.erase(m.gradient_map.begin(), m.gradient_map.end());
			m.gradient_map.clear();
			return m;
		}
		box WrappedModelForPoissonSolver() {
			box model;
			model.grid_size = topViewCanvas.content_size;
			if (locationToValue.size() == 0)
			{
				return empty(model);
			}
			// calculate values and set them in constraint containers for pSolver
			for (map<vec2Key, double>::iterator it = locationToValue.begin(); it != locationToValue.end(); it++)
			{
				if (((int)it->first.x) != 0 || ((int)it->first.x) != model.grid_size.x - 1)
				{
					model.value_map.insert(std::make_pair(std::make_pair(it->first.x, it->first.y), it->second));
					model.cross_section.push_back(it->first);
					int index = it->first.x + model.grid_size.x*(it->first.y);
					//model.constraint_triplets.push_back(Tr(model.constraints.size(), index, 1));
					model.constraints.push_back((*it).second);
				}
			}
			// calculate gradients and set them in constraint containers for pSolver
			int lookAheadDistance = 15;
			map<vec2Key, vec2Key> bim;
			vec2Key curBim(0.0, 0.0);
			double curBam, defaultSlope = 10;
			map<vec2Key, double> slope, bam;
			for (vector<vec2Key>::iterator it = model.cross_section.begin(); it < model.cross_section.end(); it++)
			{
				slope.insert(std::make_pair(*it, defaultSlope));
			}
			for (vector<vec2Key>::iterator it = model.cross_section.begin(); it < model.cross_section.end(); it++)
			{
				for (size_t i = -lookAheadDistance; i < lookAheadDistance; i++)
				{
					for (size_t j = -lookAheadDistance; j < lookAheadDistance; j++)
					{
						vec2Key currentNeighbor = vec2Key(it->x + i, it->y + j);
						if (i != 0 && j != 0 && currentNeighbor.withinBounds(model.grid_size))
						{
							if (slope.find(currentNeighbor) != slope.end())
							{
								auto tallerOne = locationToValue.find(currentNeighbor)->second > locationToValue.find(*it)->second ? currentNeighbor : *it;
								slope.find(tallerOne)->second = std::max(
									(locationToValue.find(currentNeighbor)->second - locationToValue.find(*it)->second) /
									(locationToValue.find(*it)->first).distanceFrom(currentNeighbor), slope.find(tallerOne)->second);
							}
						}
					}
				}
			}
			for (vector<vec2Key>::iterator it = model.cross_section.begin(); it < model.cross_section.end(); it++)
			{
				for (size_t i = -lookAheadDistance; i < lookAheadDistance; i++)
				{
					for (size_t j = -lookAheadDistance; j < lookAheadDistance; j++)
					{
						vec2Key currentNeighbor = vec2Key(it->x + i, it->y + j);
						if (i != 0 && j != 0 && currentNeighbor.withinBounds(topViewCanvas.content_size))
						{
							if (bim.find(currentNeighbor) == bim.end() && slope.find(currentNeighbor) == slope.end())
							{
								bim.insert(std::make_pair(currentNeighbor, vec2Key(0.0, 0.0)));
								bam.insert(std::make_pair(currentNeighbor, 0.0));
							}
						}
					}
				}
			}
			for (vector<vec2Key>::iterator it = model.cross_section.begin(); it < model.cross_section.end(); it++)
			{
				for (size_t i = -lookAheadDistance; i < lookAheadDistance; i++)
				{
					for (size_t j = -lookAheadDistance; j < lookAheadDistance; j++)
					{
						vec2Key currentNeighbor = vec2Key(it->x + i, it->y + j);
						if (i != 0 && j != 0 && currentNeighbor.withinBounds(topViewCanvas.content_size))
						{
							curBim = bim.find(*it)->second + (currentNeighbor - *it) * slope.find(*it)->second / pow(it->distanceFrom(currentNeighbor), 2);
							curBam = bam.find(*it)->second + (double)1.0 / it->distanceFrom(currentNeighbor);
							bim.find(*it)->second = curBim;
							bam.find(*it)->second = curBam;
						}
					}
				}
			}
			for (map<vec2Key, double>::iterator it = bam.begin(); it != bam.end(); it++)
			{
				curBim = bim.find(it->first)->second / it->second;
				model.gradient_map.insert(std::make_pair(it->first, curBim));
			}
			int k = 1;
			for (map<vec2Key, vec2Key>::iterator it = model.gradient_map.begin(); it != model.gradient_map.end(); it++)
			{

				vec2Key currentNeighbor = vec2Key(it->first.x + k, it->first.y);
				if (currentNeighbor.withinBounds(topViewCanvas.content_size))
				{
					int index = it->first.x + model.grid_size.x*(it->first.y);
					int otherIndex = currentNeighbor.x + model.grid_size.x*(currentNeighbor.y);
					/*model.constraint_triplets.push_back(Tr(model.constraints.size(), index, -1));
					model.constraint_triplets.push_back(Tr(model.constraints.size(), otherIndex, 1));*/
					model.constraints.push_back(it->second.x);
				}
				currentNeighbor = vec2Key(it->first.x, it->first.y + k);
				if (currentNeighbor.withinBounds(topViewCanvas.content_size))
				{
					int index = it->first.x + model.grid_size.x*(it->first.y);
					int otherIndex = currentNeighbor.x + model.grid_size.x*(currentNeighbor.y);
				/*	model.constraint_triplets.push_back(Tr(model.constraints.size(), index, -1));
					model.constraint_triplets.push_back(Tr(model.constraints.size(), otherIndex, 1));*/
					model.constraints.push_back(it->second.y);
				}
			}
			return model;
		}
	private:
		bool onCanvasContent(canvas c, int mx, int my) {
			if ((vec2Key(c.start_position.x, c.start_position.y) + c.margin_top + c.padding_top).x </*=*/ vec2Key(mx, my).x
				&& vec2Key(mx, my).x </*=*/ (vec2Key(c.start_position.x, c.start_position.y) + c.margin_top + c.padding_top + vec2Key(c.content_size.x, c.content_size.y)).x
				&& (vec2Key(c.start_position.x, c.start_position.y) + c.margin_top + c.padding_top).y </*=*/ vec2Key(mx, my).y
				&& vec2Key(mx, my).y </*=*/ (vec2Key(c.start_position.x, c.start_position.y) + c.margin_top + c.padding_top + vec2Key(c.content_size.x, c.content_size.y)).y
				)
			{
					return true;
			}
			return false;
		}
		void setLastVisitedCanvas(canvas c) {
			lastVisitedCanvas.bg_color = c.bg_color;
			lastVisitedCanvas.content_size = vec2Key(c.content_size.x, c.content_size.y);
			lastVisitedCanvas.label = c.label;
			lastVisitedCanvas.margin_top = vec2Key(c.margin_top.x, c.margin_top.y);
			lastVisitedCanvas.margin_bottom = vec2Key(c.margin_bottom.x, c.margin_bottom.y);
			lastVisitedCanvas.padding_top = vec2Key(c.padding_top.x, c.padding_top.y);
			lastVisitedCanvas.padding_bottom = vec2Key(c.padding_bottom.x, c.padding_bottom.y);
			lastVisitedCanvas.start_position.set(c.start_position.x, c.start_position.y);
			lastVisitedCanvas.canvas_status.mouse_over_content = c.canvas_status.mouse_over_content;
			lastVisitedCanvas.canvas_status.sketch_mode = c.canvas_status.sketch_mode;
		}
		void drawCanvas(canvas c) {
			if (c.canvas_status.mouse_over_content)
			{
				ofSetColor(255, 0, 0);
			}
			else 
			{
				ofSetColor(0, 0, 0);
			}
			ofRect(c.start_position.x + c.margin_top.x + c.padding_top.x,
				c.start_position.y + c.margin_top.y + c.padding_top.y,
				c.content_size.x, c.content_size.y);
			ofSetColor(0, 0, 0);
			if (c.canvas_status.sketch_mode)
			{
				ofDrawBitmapString("Sketch mode", c.start_position.x + c.margin_top.x + c.padding_top.x + 10,
					c.start_position.y + c.margin_top.y + c.padding_top.y + 20);
			}
		}
		void drawRidgeOnCanvas(canvas c) {
			if (theCurve.size() == 0)
			{
				ofDrawBitmapString("Front",
					c.start_position.x + c.margin_top.x + c.padding_top.x + 10,
					c.start_position.y + c.margin_top.y + c.padding_top.y + 20);
			}
			else
			{
				ofSetColor(0, 0, 0);
				theCurve.draw();
				ofSetColor(0, 0, 0);
			}
		}
		void drawTopViewOnCanvas(canvas c) {
			if (ridgeCurves.size() == 0)
			{
				ofDrawBitmapString("Top",
					c.start_position.x + c.margin_top.x + c.padding_top.x + 10,
					c.start_position.y + c.margin_top.y + c.padding_top.y + 20);
			}
			else
			{
				for (auto it = ridgeCurves.begin(); it < ridgeCurves.end(); it++)
				{
					ofSetColor(0, 0, 0);
					for (auto itt = it->begin(); itt < it->end(); itt++)
					{
						ofEllipse(itt->x, itt->y, 1, 1);
					}
					ofSetColor(0, 0, 0);
				}
			}
		}
		void drawStrokeInfo() {
			double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
			if (points.size() > 1) {
				min_x = (*std::min_element(points.begin(), points.end(), &_compare_min_x)).x;
				min_y = (*std::min_element(points.begin(), points.end(), &_compare_min_y)).y;
				max_x = (*std::max_element(points.begin(), points.end(), &_compare_min_x)).x;
				max_y = (*std::max_element(points.begin(), points.end(), &_compare_min_y)).y;

				ofEllipse(min_x, min_y, 5, 5);
				ofEllipse(max_x, max_y, 5, 5);
				ofDrawBitmapString("min_x, min_y", min_x - 45, min_y - 8);
				ofDrawBitmapString("max_x, max_y", max_x - 45, max_y + 8);
				if (isValidRidgeInput)
				{
					//use shared ptr later to accept wider input range
					ofDrawBitmapString("is valid ridge input", min_x - 45, min_y - 18);
				}
			}
		}
		void setCrossSection() {
			if (crossSectionPoints.size() == 0)
			{
				setDefaultCrossSection();
			}
			else
			{
				crossSectionSet.clear();
				for (vector<ofPoint>::iterator it = crossSectionPoints.begin(); it < crossSectionPoints.end(); it++)
				{
					crossSectionSet.insert(std::make_pair(it->x, it->y));
				}
				isSetCrossSection = true;
			}
		}
		void setDefaultCrossSection() {
			crossSectionSet.clear();
			for (size_t i = 0; i < topViewCanvas.content_size.x; i++)
			{
				crossSectionSet.insert(std::make_pair(i, topViewCanvas.content_size.y / 2));
			}
			isSetCrossSection = true;
		}
		void evaluateAsRidge() {
			ofPoint leftMostPoint, rightMostPoint;
			if (points.size() > 1) {
				leftMostPoint = (*std::min_element(points.begin(), points.end(), &_compare_min_x));
				rightMostPoint = (*std::max_element(points.begin(), points.end(), &_compare_min_x));
				if (!(leftMostPoint == points.at(0) && rightMostPoint == points.at(points.size() - 1)))
				{
					isValidRidgeInput = false;
				}
			}
		}
		void saveRidgeIfValid() {
			if (isValidRidgeInput && points.size() != 0)
			{
				double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
				min_x = (*std::min_element(points.begin(), points.end(), &_compare_min_x)).x;
				min_y = (*std::min_element(points.begin(), points.end(), &_compare_min_y)).y;
				max_x = (*std::max_element(points.begin(), points.end(), &_compare_min_x)).x;
				max_y = (*std::max_element(points.begin(), points.end(), &_compare_min_y)).y;

				lastSavedValidRidgePoints.clear();
				double lastX = 0;
				for (vector<ofPoint>::iterator it = points.begin(); it < points.end(); it++)
				{
					lastSavedValidRidgePoints.push_back(ofPoint(
						it->x - min_x + sideViewCanvas.start_position.x + sideViewCanvas.margin_top.x + sideViewCanvas.padding_top.x,
						it->y
					));
					lastX = it->x;
				}
				if (lastX < sideViewCanvas.start_position.x + sideViewCanvas.margin_top.x + sideViewCanvas.padding_top.x + sideViewCanvas.content_size.x)
				{
					lastSavedValidRidgePoints.push_back(ofPoint(sideViewCanvas.start_position.x + sideViewCanvas.margin_top.x + sideViewCanvas.padding_top.x + sideViewCanvas.content_size.x,
						(points.begin()+points.size()-1)->y));
				}
				assert(lastSavedValidRidgePoints.size() != 0);
				theCurve.clear();
				theCurve.addVertices(lastSavedValidRidgePoints);
				vector<ofPoint> allPoints = bresenham::getAllPoints(theCurve.getVertices());
				locationToValue.erase(locationToValue.begin(), locationToValue.end());
				locationToValue.clear();
				if (!isSetCrossSection)
				{
					setCrossSection();
				}
				vector<vec2Key> cs;
				map<vec2Key, double> simplifiedStroke;
				for (auto i = sideViewCanvas.start_position.x + sideViewCanvas.margin_top.x + sideViewCanvas.padding_top.x;
					i < sideViewCanvas.start_position.x + sideViewCanvas.margin_top.x + sideViewCanvas.padding_top.x + sideViewCanvas.content_size.x; i++)
				{
					cs.push_back(vec2Key(i, 0));
				}
				for (vector<vec2Key>::iterator it = cs.begin(); it < cs.end(); it++)
				{
					simplifiedStroke.insert(std::make_pair(*it, sideViewCanvas.start_position.y + sideViewCanvas.margin_top.y + sideViewCanvas.padding_top.y + sideViewCanvas.content_size.y));
				}
				for (vector<ofPoint>::iterator it = allPoints.begin(); it < allPoints.end(); it++) {
					std::map<vec2Key, double>::iterator itt = simplifiedStroke.find(vec2Key(it->x, 0));
					if (itt != simplifiedStroke.end() && itt->second > it->y)
					{
						itt->second = it->y;
					}
				}
				assert(crossSectionSet.size() == sideViewCanvas.content_size.x);
				assert(simplifiedStroke.size() == sideViewCanvas.content_size.x);
				assert(simplifiedStroke.size() == crossSectionSet.size(), "they should be equal so they can be mapped now");
				double maxY = maxValueIn(simplifiedStroke);
				auto crossSecIter = crossSectionSet.begin();
				double baseY = sideViewCanvas.start_position.y + sideViewCanvas.margin_top.y + sideViewCanvas.padding_top.y + sideViewCanvas.content_size.y;
				for (std::map<vec2Key, double>::iterator yIter = simplifiedStroke.begin(); yIter != simplifiedStroke.end(); yIter++, crossSecIter++) {
					// locationToValue.insert(std::make_pair(vec2Key(crossSecIter->first, crossSecIter->second), maxY - yIter->second));
					locationToValue.insert(std::make_pair(vec2Key(crossSecIter->first, crossSecIter->second), baseY - yIter->second));
				}
				assert(locationToValue.size() == crossSectionSet.size(), "output should be equal to canvas size");
			}
		}
		double maxValueIn(map<vec2Key, double> s) {
			return max_element(s.begin(), s.end(),
				bind(less<double>(),
				bind(&map<vec2Key, double>::value_type::second, placeholders::_1),
				bind(&map<vec2Key, double>::value_type::second, placeholders::_2)))->second;
			//	cout << "point with min value in stroke of " << this->second << " is:<" << this->first.x << "," << this->first.y << ">" << endl;
		}
		void saveCrossSection() {
			if (points.size() != 0)
			{
				crossSectionPoints.clear();
				vector<ofPoint> v = bresenham::getAllPoints(points);
				for (vector<ofPoint>::iterator it = v.begin(); it < v.end(); it++)
				{
					crossSectionPoints.push_back(ofPoint(it->x - topViewCanvas.start_position.x - topViewCanvas.margin_top.x - topViewCanvas.padding_top.x,
						it->y - topViewCanvas.start_position.y - topViewCanvas.margin_top.y - topViewCanvas.padding_top.y));
				}
			}
			setCrossSection();
			vector<ofPoint> temp;
			for (auto it = crossSectionSet.begin(); it != crossSectionSet.end(); it++)
			{
				temp.push_back(
					ofPoint(
						it->first + topViewCanvas.start_position.x + topViewCanvas.margin_top.x + topViewCanvas.padding_top.x,
						it->second + topViewCanvas.start_position.y + topViewCanvas.margin_top.y + topViewCanvas.padding_top.y));
			}
			ridgeCurves.push_back(temp);
		}
		void setRandomCurve(){
			double base = sideViewCanvas.start_position.y + sideViewCanvas.margin_top.y + sideViewCanvas.padding_top.y + sideViewCanvas.content_size.y
				, firstElement = sideViewCanvas.start_position.x + sideViewCanvas.margin_top.x + sideViewCanvas.padding_top.x
				, maxAllowed = sideViewCanvas.content_size.y;
			int mod = 4;
			base -= maxAllowed / 2.0;
			double oldValue = base, sign = 1.0, nextValue;
			bool abandonned = false, modChanged = false;
			assert(mod >= 4, "mod cannot be less than 4");
			for (int i = firstElement; i < firstElement + sideViewCanvas.content_size.x - 1; i++)
			{
				if (fabs(oldValue - base) > maxAllowed / 3.0 && modChanged)
				{
					modChanged = true;
					mod /= 2;
				}
				if (fabs(oldValue - base) < maxAllowed / 2.0 && !abandonned)
				{
					sign = (rand() % 4) < 2 ? 1.0 : -1.0;
					nextValue = oldValue - rand() % mod*sign;
				}
				else
				{
					abandonned = true;
					nextValue = oldValue;
				}
				points.push_back(ofPoint(i, oldValue));
				oldValue = nextValue;
			}
			points.push_back(ofPoint(firstElement + sideViewCanvas.content_size.x, nextValue));
		}
		void setRandomCurveX() {
			
			double base = sideViewCanvas.start_position.y + sideViewCanvas.margin_top.y + sideViewCanvas.padding_top.y + sideViewCanvas.content_size.y
				, firstElement = sideViewCanvas.start_position.x + sideViewCanvas.margin_top.x + sideViewCanvas.padding_top.x;
			/*	, maxAllowed = sideViewCanvas.content_size.y;
			int mod = 4;
			base -= maxAllowed / 2.0;
			double oldValue = base, sign = 1.0, nextValue;*/

			double oldValue = base, nextValue = oldValue;
			double coeff;
			int mod, count = 0;
			size_t theSize = sideViewCanvas.content_size.x;
			double maxDesired = 100;
			for (size_t idx = firstElement; idx < theSize; idx++)
			{
				count++;
				//auto aNode = gNodes.at(idx);
				//auto itsLoc = aNode->getLoc();
				coeff = (rand() % 4) < 2 ? 3.0 * (rand() % 2) : -2.0 * (rand() % 2);
				//h = a*(1-x) + b(x)

				if (oldValue + coeff < 0.2 * maxDesired && count < 0.50 * theSize)
				{
					coeff = fabs(coeff);
				}
				if (oldValue + coeff > 0.8 * maxDesired & count > 0.50 * theSize)
				{
					coeff = -1 * fabs(coeff);
				}

				nextValue = oldValue - coeff;

				points.push_back(ofPoint(idx, oldValue)); //itsLoc.y = oldValue;
				oldValue = nextValue;
			}
		}
		canvas topViewCanvas, sideViewCanvas, lastVisitedCanvas;
		bool isValidRidgeInput, isSetCrossSection, setRandomSideView;
		ofPolyline theCurve;
		vector<vector<ofPoint>> ridgeCurves;
		vector<ofPoint> points, lastSavedValidRidgePoints, crossSectionPoints;
		set<pair<int, int>> crossSectionSet;
		map<vec2Key, double> locationToValue;
	};
}

//namespace Solver {
//#include <algorithm>;
//	using namespace Eigen;
//	typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
//	typedef Eigen::Triplet<double> Tr;
//	class aClassInSolver : ofBaseApp {
//	public:
//		void setup()
//		{
//			width = LENGTH;
//			height = width;
//			clearContainers();
//			setDefaultBoundaryConditions();
//
//			resultCanvas.bg_color = ofColor(255, 255, 255);
//			resultCanvas.label = "Solver result";
//			resultCanvas.start_position.set(0, 420);
//			resultCanvas.content_size = vec2Key(width, height);
//			resultCanvas.margin_top = vec2Key(10, 10);
//			resultCanvas.margin_bottom = vec2Key(10, 10);
//			resultCanvas.padding_top = vec2Key(0, 0);
//			resultCanvas.padding_bottom = vec2Key(0, 0);
//
//			crossSectionCanvas.bg_color = ofColor(255, 255, 255);
//			crossSectionCanvas.label = "cross section view";
//			crossSectionCanvas.start_position.set(
//				resultCanvas.start_position.x + resultCanvas.margin_top.x + resultCanvas.padding_top.x + resultCanvas.content_size.x + resultCanvas.margin_bottom.x + resultCanvas.padding_bottom.x,
//				resultCanvas.start_position.y);
//			crossSectionCanvas.content_size = vec2Key(width, height);
//			crossSectionCanvas.margin_top = vec2Key(10, 10);
//			crossSectionCanvas.margin_bottom = vec2Key(10, 10);
//			crossSectionCanvas.padding_top = vec2Key(0, 0);
//			crossSectionCanvas.padding_bottom = vec2Key(0, 0);
//		}
//		void update()
//		{
//			updateView();
//		}
//		void draw()
//		{
//			if (gridVector.size() == width*height)
//			{
//				drawHeightmap();
//				drawCrossSectionTopView();
//				drawCrossSectionFrontView();
//				ofRect(crossSectionCanvas.start_position.x + crossSectionCanvas.margin_top.x + crossSectionCanvas.padding_top.x,
//					crossSectionCanvas.start_position.y + crossSectionCanvas.margin_top.y + crossSectionCanvas.padding_top.y,
//					crossSectionCanvas.content_size.x, crossSectionCanvas.content_size.y);
//			}
//		}
//		void keyPressed(int key)
//		{
//			;
//		}
//		void solveAndUpdateModel(box receivedModel) {
//			constraints = receivedModel.constraints;
//			constraintTriplets = receivedModel.constraint_triplets;
//			crossSectionTopViewPositions = receivedModel.cross_section;
//			width = receivedModel.grid_size.x;
//			height = receivedModel.grid_size.y;
//			// gradientMap = receivedModel.gradient_map;	// used in dijkstra not in poisson
//
//			// Solve and update model
//			solve();
//		}
//	private:
//		void solve()
//		{
//			if (constraints.size() == 0 || constraintTriplets.size() == 0)
//			{
//				constraints.clear();
//				constraintTriplets.clear();
//				solveWithDefaultConstraints();
//				return;
//			}
//			int n = width;
//			int m = n*n;  // number of pixels
//			// Assembly:
//			Eigen::VectorXd b(m + constraints.size());
//			std::vector<Tr> coefficients;
//			assert(width == height, "Solver assumes width == height");
//			buildProblem(coefficients, b, width);
//			SpMat A(m + constraints.size(), m);
//			A.setFromTriplets(coefficients.begin(), coefficients.end());
//			Eigen::VectorXd result = solveSparse(A, b);
//			vector<double> temp;
//			for (int k = 0; k < result.outerSize(); ++k)
//			{
//				for (VectorXd::InnerIterator it(result, k); it; ++it)
//				{
//					temp.push_back(it.value());
//				}
//			}
//			updateModel(temp);
//		}
//		void setBoundaryFromCurve(int choice) {
//			Eigen::ArrayXd boundaryOfChoice;
//			switch (choice)
//			{
//			case 0:
//				boundaryOfChoice = boundaryUp;
//				break;
//			case 1:
//				boundaryOfChoice = boundaryDown;
//				break;
//			case 2:
//				boundaryOfChoice = boundaryLeft;
//				break;
//			case 3:
//				boundaryOfChoice = boundaryRight;
//				break;
//			}
//			boundaryOfChoice.setOnes();	//instead of this just set from input, if no input is saved then just probably do this
//		}
//		void setDefaultBoundaryConditions() {
//			boundaryUp = Eigen::ArrayXd::LinSpaced(width, M_PI, 3 * M_PI).cos();
//			//		boundaryDown = Eigen::ArrayXd::LinSpaced(width, 0, M_PI).sin().pow(2);
//			boundaryDown = Eigen::ArrayXd::LinSpaced(width, M_PI, 3 * M_PI).cos();
//			boundaryLeft = Eigen::ArrayXd::LinSpaced(width, M_PI, 3 * M_PI).cos();
//			boundaryRight = Eigen::ArrayXd::LinSpaced(width, M_PI, 3 * M_PI).cos();
//			//		boundaryUp.setZero();
//			//		boundaryDown.setZero();
//			//		boundaryLeft.setZero();
//			//		boundaryRight.setZero();
//		}
//		void insertCoefficient(int id, int i, int j, double w, std::vector<Tr>& coeffs, Eigen::VectorXd& b)
//		{
//			int n = boundaryUp.size();
//			int id1 = i + j*n;
//			if (i == -1) b(id) -= w * boundaryLeft(j); // constrained coefficient
//			else if (i == n) b(id) -= w * boundaryRight(j); // constrained coefficient
//			else if (j == -1) b(id) -= w * boundaryUp(i); // constrained coefficient
//			else if (j == n) b(id) -= w * boundaryDown(i); // constrained coefficient
//			else  coeffs.push_back(Tr(id, id1, w));              // unknown coefficient
//		}
//		void buildProblem(std::vector<Tr>& coefficients, Eigen::VectorXd& b, int n)
//		{
//			assert(constraints.size() != 0, "should set constraints before buildProgram");
//			assert(constraintTriplets.size() != 0, "should set constraintTriplets before buildProgram");
//			b.setZero();
//			int k = 0;
//			for (vector<Tr>::iterator it = constraintTriplets.begin(); it < constraintTriplets.end(); it++)
//			{
//				coefficients.push_back(*it);
//			}
//			k = 0;
//			for (vector<double>::iterator it = constraints.begin(); it < constraints.end(); it++, k++)
//			{
//				b(k) = *it;
//			}
//			//			Eigen::ArrayXd boundary1 = Eigen::ArrayXd::LinSpaced(n, M_PI, 3 * M_PI).cos();
//			//			Eigen::ArrayXd boundary = Eigen::ArrayXd::LinSpaced(n, 0, M_PI).sin().pow(2);
//			for (int j = 0; j<n; ++j)
//			{
//				for (int i = 0; i<n; ++i)
//				{
//					int id = i + j*n + constraints.size();
//					insertCoefficient(id, i, j - 1, -1, coefficients, b);
//					insertCoefficient(id, i - 1, j, -1, coefficients, b);
//					insertCoefficient(id, i, j, 4, coefficients, b);
//					insertCoefficient(id, i + 1, j, -1, coefficients, b);
//					insertCoefficient(id, i, j + 1, -1, coefficients, b);
//				}
//			}
//		}
//		void solveWithDefaultConstraints()
//		{
//			int n = width;
//			int m = n*n;  // number of pixels
//			assert(constraints.size() == 0, "constraints should be empty");
//			assert(constraintTriplets.size() == 0, "constraintTriplets should be empty");
//			// Assembly:
//			for (int i = 1; i < 99; i++)
//			{
//				for (int p = 1; p < 99; p++)
//				{
//					int index = p + width*i;
//					constraintTriplets.push_back(Tr(constraints.size(), index, 1));
//					constraints.push_back(2);
//				}
//			}
//			Eigen::VectorXd b(m + constraints.size());
//			std::vector<Tr> coefficients;
//			assert(width == height, "Solver assumes width == height");
//			buildProblem(coefficients, b, width);
//			SpMat A(m + constraints.size(), m);
//			A.setFromTriplets(coefficients.begin(), coefficients.end());
//			Eigen::VectorXd result = solveSparse(A, b);
//			vector<double> temp;
//			for (int k = 0; k < result.outerSize(); ++k)
//			{
//				for (VectorXd::InnerIterator it(result, k); it; ++it)
//				{
//					temp.push_back(it.value());
//				}
//			}
//			updateModel(temp);
//		}
//		Eigen::VectorXd solveSparse(SpMat A, Eigen::VectorXd b)
//		{
//			SpMat D = A.transpose() * A;
//			Eigen::VectorXd d = A.transpose() * b;
//			Eigen::SimplicialCholesky<SpMat> chol(D);  // performs a Cholesky factorization of A
//			Eigen::VectorXd x = chol.solve(d);         // use the factorization to solve for the given right hand side
//			return x;
//		}
//		vector<double> setupArrayFor(SpMat *X)
//		{
//			vector<double> v;
//			cout << "X->outerSize() = " << X->outerSize() << endl;
//			for (int k = 0; k < X->outerSize(); ++k) // k = row index
//			{
//				for (SparseMatrix<double, RowMajor>::InnerIterator it(X[0], k); it; ++it) // col index
//				{
//					v.push_back(it.value());
//					//it.row();   // row index
//					//it.col();   // col index
//					//it.index(); // inner index
//					//it.valueRef() *= 1;
//				}
//			}
//			return v;
//		}
//		
//		void drawHeightmap()
//		{
//			assert(width*height == gridVector.size());
//			ofSetColor(0, 0, 0);
//			ofPoint p;
//			int count = 0; //=i * width + j
//			for (vector<double>::iterator it = gridVector.begin(); it < gridVector.end(); it++, count++)
//			{
//				double val = *it;
//				p.set(resultCanvas.start_position.x + resultCanvas.margin_top.x + resultCanvas.padding_top.x + count % (int)resultCanvas.content_size.x,
//					resultCanvas.start_position.y + resultCanvas.margin_top.y + resultCanvas.padding_top.y + count / (int)resultCanvas.content_size.y);
//				ofSetColor(val, val, val);
//				ofEllipse(p, 2, 2);
//			}
//			ofSetColor(0, 0, 0);
//		}
//		void findMinMax(vector<double> values, double& min, double& max)
//		{
//			min = *values.begin();
//			max = *values.begin();
//			for (vector<double>::iterator it = values.begin(); it < values.end(); it++)
//			{
//				*it > max ? max = *it : max;
//				*it < min ? min = *it : min;
//			}
//		}
//		void rescale(vector<double>& values, const double base, const double range)
//		{
//			vector<double> other;
//			double min, max, tmp;
//			findMinMax(values, min, max);
//			for (vector<double>::iterator it = values.begin(); it < values.end(); it++)
//			{
//				tmp = (*it - min);
//				assert(tmp >= 0);
//				*it = base + tmp * range / (max - min);
//			}
//			findMinMax(values, min, max);
//			assert(min == base && max < range + base + 1, "min should be (base) and max less than (range) more.");
//		}
//		void drawCrossSectionTopView() {
//			ofSetColor(255, 0, 0);
//			for (vector<vec2Key>::iterator it = crossSectionTopViewPositions.begin(); it < crossSectionTopViewPositions.end(); it++)
//			{
//				ofEllipse(it->x + resultCanvas.start_position.x + resultCanvas.margin_top.x + resultCanvas.padding_top.x,
//					it->y + resultCanvas.start_position.y + resultCanvas.margin_top.y + resultCanvas.padding_top.y, 1, 1);
//			}
//			ofSetColor(0, 0, 0);
//		}
//		void drawCrossSectionFrontView() {
//			ofSetColor(255, 0, 0);
//			crossSectionFrontView.draw();
//			ofSetColor(0, 0, 0);
//		}
//		void updateModel(vector<double> result)
//			{
//				gridVector.clear();
//				for (auto it = result.begin(); it < result.end(); it++)
//				{
//					gridVector.push_back(*it);
//				}
//			assert(gridVector.size() == height*width, "solved curve size == height*width");
//			rescale(gridVector, 0.0, 255.0);
//			gridMap.erase(gridMap.begin(), gridMap.end());
//			gridMap.clear();
//			int k = 0;
//			for (vector<double>::iterator it = gridVector.begin(); it < gridVector.end(); it++, k++)
//			{
//				gridMap.insert(std::make_pair(std::make_pair(k % width, (int)(k / width)), *it));
//			}
//		}
//		void updateView() {
//			// refreshCrossSection Top View
//			if (crossSectionTopViewPositions.size() == 0)
//			{
//				setDefaultCrossSection();
//			}
//			crossSectionVector.clear();
//			for (vector<vec2Key>::iterator it = crossSectionTopViewPositions.begin(); it < crossSectionTopViewPositions.end(); it++)
//			{
//				assert(gridMap.find(std::make_pair(it->x, it->y)) != gridMap.end(), "points in cross section should be inside grid");
//				crossSectionVector.push_back(gridMap.find(std::make_pair(it->x, it->y))->second);
//			}
//			// refreshCrossSection Canvas Size
//			crossSectionCanvas.content_size.x = crossSectionVector.size();
//			// refreshCrossSection Front View
//			double base = crossSectionCanvas.start_position.y + crossSectionCanvas.margin_top.y + crossSectionCanvas.padding_top.y + crossSectionCanvas.content_size.y,
//				range = crossSectionCanvas.content_size.y;
//			crossSectionFrontView.clear();
//			ofPoint a, b;
//			a.set(crossSectionCanvas.start_position.x + crossSectionCanvas.margin_top.x + crossSectionCanvas.padding_top.x, base - range*(crossSectionVector[0] - 0.0) / 255.0);
//			crossSectionFrontView.addVertex(a);
//			for (int i = 0; i < crossSectionVector.size() - 1; i++)
//			{
//				b.set(a.x + 1, base - range*(crossSectionVector[i + 1] - 0.0) / 255.0);
//				crossSectionFrontView.addVertex(a);
//				a.set(b);
//			}
//		}
//		void setDefaultCrossSection() {
//			crossSectionTopViewPositions.clear();
//			for (size_t i = 0; i < width; i++)
//			{
//				crossSectionTopViewPositions.push_back(vec2Key(i, height / 2));
//			}
//		}
//		void clearContainers() {
//			constraintTriplets.clear();
//			constraints.clear();
//			crossSectionTopViewPositions.clear();
//			crossSectionVector.clear();
//			gridMap.erase(gridMap.begin(), gridMap.end());
//			gridMap.clear();
//			gridVector.clear();
//		}
//
//		int width, height;
//		std::vector<Tr> constraintTriplets;
//		vector<double> constraints;
//		map<pair<int, int>, double> gridMap;
//		Eigen::VectorXd boundaryUp, boundaryDown, boundaryLeft, boundaryRight;
//
//		std::vector<double> gridVector, crossSectionVector;
//		canvas resultCanvas, crossSectionCanvas;
//		ofPolyline crossSectionFrontView;
//		vector<vec2Key> crossSectionTopViewPositions;
//
//	};
//}

namespace DijkstraUtil {

	class aClassInDijkstraUtil : ofBaseApp {
		#define DSIZE LENGTH
		
	public:
		void setup() {
			width = DSIZE;
			height = width;

			resultCanvas.bg_color = ofColor(255, 255, 255);
			resultCanvas.label = "Dijkstra result";
			resultCanvas.start_position.set(0, 100 + DSIZE);
			resultCanvas.content_size = vec2Key(width, height);
			resultCanvas.margin_top = vec2Key(10, 10);
			resultCanvas.margin_bottom = vec2Key(10, 10);
			resultCanvas.padding_top = vec2Key(0, 0);
			resultCanvas.padding_bottom = vec2Key(0, 0);

			crossSectionCanvas.bg_color = ofColor(255, 255, 255);
			crossSectionCanvas.label = "cross section view";
			crossSectionCanvas.start_position.set(
				resultCanvas.start_position.x + resultCanvas.margin_top.x + resultCanvas.padding_top.x + resultCanvas.content_size.x + resultCanvas.margin_bottom.x + resultCanvas.padding_bottom.x,
				resultCanvas.start_position.y);
			crossSectionCanvas.content_size = vec2Key(resultCanvas.content_size.x, resultCanvas.content_size.y);
			crossSectionCanvas.margin_top = vec2Key(10, 10);
			crossSectionCanvas.margin_bottom = vec2Key(10, 10);
			crossSectionCanvas.padding_top = vec2Key(0, 0);
			crossSectionCanvas.padding_bottom = vec2Key(0, 0);
		}
		void update()
		{
			updateView();
		}
		void draw()
		{
			if (gridVector.size() == width*height)
			{
				drawHeightmap();
	//			drawCrossSectionTopView();
				drawCrossSectionFrontView();
				ofRect(crossSectionCanvas.start_position.x + crossSectionCanvas.margin_top.x + crossSectionCanvas.padding_top.x,
					crossSectionCanvas.start_position.y + crossSectionCanvas.margin_top.y + crossSectionCanvas.padding_top.y,
					crossSectionCanvas.content_size.x, crossSectionCanvas.content_size.y);
			}
		}
		void keyPressed(int key) {
			;
		}
		void mousePressed(int x, int y, int button) {
			;
		}
		static PPMImage* readPPM(const char *filename)
		{
			/*
			vector<int> bbb;
			FILE * pFile;
			int ccc;
			int n = 0;
			char filenames[3][20] = { "ttr", "feralcal", "loki" };
			char curfile[80];
			int k = 0;
			sprintf(curfile, "images/%s.ppm", filenames[k]);
			pFile = fopen(curfile, "rb");
			if (pFile == NULL)
			{
				assert(false); //error opening file
			}
			else
			{
				do {
					ccc = getc(pFile);
					bbb.push_back(ccc);
					n++;
				} while (ccc != EOF);
				fclose(pFile);
				printf("File contains %d.\n", n-1);
			}*/
			//first 38 ints are corrupt and las is EOF

			char buff[16];
			PPMImage *img;
			FILE *fp;
			int c, rgb_comp_color;
			//open PPM file for reading
			fp = fopen(filename, "rb");
			if (!fp) {
				fprintf(stderr, "Unable to open file '%s'\n", filename);
				assert(false);
			}

			//read image format
			if (!fgets(buff, sizeof(buff), fp)) {
				perror(filename);
				assert(false);
			}

			//check the image format
			if (buff[0] != 'P' || buff[1] != '6') {
				fprintf(stderr, "Invalid image format (must be 'P6')\n");
				assert(false);
			}

			//alloc memory form image
			img = (PPMImage *)malloc(sizeof(PPMImage));
			if (!img) {
				fprintf(stderr, "Unable to allocate memory\n");
				assert(false);
			}

			//check for comments
			bool hadComments = false;
			int commentsCount = 0;
			c = getc(fp);
			while (c == '#') {
				while (getc(fp) != '\n') {
					hadComments = true;
					commentsCount++;
				}
				c = getc(fp);
			}
			if (hadComments)
			{
				printf("commentsCount %d", commentsCount);
			}

			ungetc(c, fp);
			//read image size information
			if (fscanf(fp, "%d %d", &img->x, &img->y) != 2) {
				fprintf(stderr, "Invalid image size (error loading '%s')\n", filename);
				assert(false);
			}

			//read rgb component
			if (fscanf(fp, "%d", &rgb_comp_color) != 1) {
				fprintf(stderr, "Invalid rgb component (error loading '%s')\n", filename);
				assert(false);
			}

			//check rgb component depth
			if (rgb_comp_color != RGB_COMPONENT_COLOR) {
				fprintf(stderr, "'%s' does not have 8-bits components\n", filename);
				assert(false);
			}

			while (fgetc(fp) != '\n');
			//memory allocation for pixel data
			img->data = (rgbvector*)malloc(img->x * img->y * sizeof(rgbvector));

			if (!img) {
				fprintf(stderr, "Unable to allocate memory\n");
				assert(false);
			}

			//read pixel data from file
			if (fread(img->data, 3 * img->x, img->y, fp) != img->y) {
				fprintf(stderr, "Error loading image '%s'\n", filename);
				assert(false);
			}

			fclose(fp);
			return img;
		}
		std::vector<double> fillInHF(rgbvector* result) {
			std::vector<double> r;
			for (size_t i = 0; i < LENGTH * LENGTH; i++)
			{
				r.push_back((double)result[i].red);
			}
			return r;
		}
		void solveAndUpdateModel(box receivedModel) {
			map<int, ridge_point> pointsOnRidge;
			if (receivedModel.value_map.size() != 0)
			{
				int i = 0;
				for (auto it = receivedModel.value_map.begin(); it != receivedModel.value_map.end(); it++, i++)
				{
					//<order sketched, xyz>
					pointsOnRidge.insert(std::make_pair(i, std::make_tuple(std::get<0>(it->first), it->second, std::get<1>(it->first))));
				}
			}
			unique_ptr<Model> pModel(new Model(DSIZE * DSIZE, pointsOnRidge));
//			unique_ptr<Model> pModel(new Model(DSIZE * DSIZE));
			updateModel(pModel->getResultingHeightfield());
		}
		void solveAndUpdateUsingterrainprocessor(box receivedModel) {
			unique_ptr<terrainprocessor> terrainprocessor(new terrainprocessor());
			char filenames[3][20] = { "tertem",	"ttr",	"loki" };
			char curfile[80];
			int k = 0;
			sprintf(curfile, "images/%s.ppm", filenames[k]);
			//auto img = readPPM(curfile);
			//terrainprocessor->fillbackup(img);

			terrainprocessor->makegraph();
			fprintf(stderr, "made graph\n");
			terrainprocessor->ridges();
			fprintf(stderr, "made ridges\n");
			terrainprocessor->terrain();
			fprintf(stderr, "made mountain\n");
			
			//free(img->data);
			//free(img);

//			terrainprocessor->testStripe();
			std::vector<double> resultingHeightfield = fillInHF(terrainprocessor->getresult());
			
			updateModel(resultingHeightfield);
		}
	private:
		void drawHeightmap()
		{
			assert(width*height == gridVector.size());
			ofSetColor(0, 0, 0);
			ofPoint p;
			int count = 0; //=i * width + j
			for (vector<double>::iterator it = gridVector.begin(); it < gridVector.end(); it++, count++)
			{
				double val = *it;
				p.set(resultCanvas.start_position.x + resultCanvas.margin_top.x + resultCanvas.padding_top.x + count % (int)resultCanvas.content_size.x,
					resultCanvas.start_position.y + resultCanvas.margin_top.y + resultCanvas.padding_top.y + count / (int)resultCanvas.content_size.y);
				ofSetColor(val, val, val);
				ofEllipse(p, 2, 2);
			}
			ofSetColor(0, 0, 0);
		}
		void drawHeightmapColored()
		{
			//WHEN USING THIS METHOD, COMMENT OUT: "rescale(gridVector, 0.0, 255.0);"
			assert(width*height == gridVector.size());
			ofSetColor(0, 0, 0);
			ofPoint p;
			int count = 0; //=i * width + j
			for (std::vector<double>::iterator it = gridVector.begin(); it < gridVector.end(); it++, count++)
			{
				double val = *it;
				p.set(resultCanvas.start_position.x + resultCanvas.margin_top.x + resultCanvas.padding_top.x + count % (int)resultCanvas.content_size.x,
					resultCanvas.start_position.y + resultCanvas.margin_top.y + resultCanvas.padding_top.y + count / (int)resultCanvas.content_size.y);
				switch ((int)val)
				{
				case 80:
					//white
					ofSetColor(255, 255, 255);
					break;
				case 0:
					//black
					ofSetColor(0, 0, 0);
					break;
				case 60:
					//blue
					ofSetColor(0, 0, 255);
					break;
				case 30:
					//red
					ofSetColor(255, 0, 0);
					break;
				default:
					assert(false);
					break;
				}
//				ofSetColor(val, val, val);
				ofEllipse(p, 2, 2);
			}
			ofSetColor(0, 0, 0);
		}
		void findMinMax(std::vector<double> values, double& min, double& max)
		{
			min = *values.begin();
			max = *values.begin();
			for (std::vector<double>::iterator it = values.begin(); it < values.end(); it++)
			{
				*it > max ? max = *it : max;
				*it < min ? min = *it : min;
			}
		}
		void rescale(std::vector<double>& values, const double base, const double range)
		{
			std::vector<double> other;
			double min, max, tmp;
			findMinMax(values, min, max);
			for (std::vector<double>::iterator it = values.begin(); it < values.end(); it++)
			{
				tmp = (*it - min);
				assert(tmp >= 0);
				*it = base + tmp * range / (max - min);
			}
			findMinMax(values, min, max);
			assert(min == base && max < range + base + 1, "min should be (base) and max less than (range) more.");
		}
		void drawCrossSectionTopView() {
			ofSetColor(255, 0, 0);
			for (std::vector<vec2Key>::iterator it = crossSectionTopViewPositions.begin(); it < crossSectionTopViewPositions.end(); it++)
			{
				ofEllipse(it->x + resultCanvas.start_position.x + resultCanvas.margin_top.x + resultCanvas.padding_top.x,
					it->y + resultCanvas.start_position.y + resultCanvas.margin_top.y + resultCanvas.padding_top.y, 1, 1);
			}
			ofSetColor(0, 0, 0);
		}
		void drawCrossSectionFrontView() {
			ofSetColor(255, 0, 0);
			crossSectionFrontView.draw();
			ofSetColor(0, 0, 0);
		}
		void updateModel(std::vector<double> result)
		{
			gridVector.clear();
			for (auto it = result.begin(); it < result.end(); it++)
			{
				gridVector.push_back(*it);
			}
			assert(gridVector.size() == height*width, "solved curve size == height*width");
			rescale(gridVector, 0.0, 255.0);
			gridMap.erase(gridMap.begin(), gridMap.end());
			gridMap.clear();
			int k = 0;
			for (std::vector<double>::iterator it = gridVector.begin(); it < gridVector.end(); it++, k++)
			{
				gridMap.insert(std::make_pair(std::make_pair(k % width, (int)(k / width)), *it));
			}
		}
		void updateView() {
			// refreshCrossSection Top View
			if (crossSectionTopViewPositions.size() == 0)
			{
				setDefaultCrossSection();
			}
			crossSectionVector.clear();
			for (std::vector<vec2Key>::iterator it = crossSectionTopViewPositions.begin(); it < crossSectionTopViewPositions.end(); it++)
			{
				assert(gridMap.find(std::make_pair(it->x, it->y)) != gridMap.end(), "points in cross section should be inside grid");
				crossSectionVector.push_back(gridMap.find(std::make_pair(it->x, it->y))->second);
			}
			// refreshCrossSection Canvas Size
			crossSectionCanvas.content_size.x = crossSectionVector.size();
			// refreshCrossSection Front View
			double base = crossSectionCanvas.start_position.y + crossSectionCanvas.margin_top.y + crossSectionCanvas.padding_top.y + crossSectionCanvas.content_size.y,
				range = crossSectionCanvas.content_size.y;
			crossSectionFrontView.clear();
			ofPoint a, b;
			a.set(crossSectionCanvas.start_position.x + crossSectionCanvas.margin_top.x + crossSectionCanvas.padding_top.x, base - range*(crossSectionVector[0] - 0.0) / 255.0);
			crossSectionFrontView.addVertex(a);
			for (int i = 0; i < crossSectionVector.size() - 1; i++)
			{
				b.set(a.x + 1, base - range*(crossSectionVector[i + 1] - 0.0) / 255.0);
				crossSectionFrontView.addVertex(a);
				a.set(b);
			}
		}
//		other anOther;
		void setDefaultCrossSection() {
			crossSectionTopViewPositions.clear();
			for (size_t i = 0; i < width; i++)
			{
				crossSectionTopViewPositions.push_back(vec2Key(i, height / 2));
			}
		}
		int width, height;
		canvas resultCanvas, crossSectionCanvas;
		std::vector<double> gridVector, crossSectionVector;
		ofPolyline crossSectionFrontView;
		std::vector<vec2Key> crossSectionTopViewPositions;
	//suggested model
		//std::vector<Tr> constraintTriplets;
		//vector<double> constraints;
		map<pair<int,int>, double> gridMap;
		//map<vec2Key, vec2Key> gradientMap;
	};
}

class terrainapplication : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

private:
	//ofTrueTypeFont myFont;
	StrokeUtil::aClassInStrokeUtil stroke;
	//Solver::aClassInSolver solver;
	DijkstraUtil::aClassInDijkstraUtil dijkstra;
	ProgressDetail::aClassInProgressUtil progress;
	int status;
};

void terrainapplication::setup()
{
	stroke.setup();
	//solver.setup();
	dijkstra.setup();
	progress.setup();
	status = solve_status::SOLVE_POISSON;
}

void terrainapplication::update()
{
	stroke.update();
	if (status == solve_status::SOLVE_POISSON_VALUES)
	{
		// Solve and update model
		//solver.solveAndUpdateModel(stroke.WrappedModelForPoissonSolver());
		// Update view
		//solver.update();
		status = solve_status::DONT_SOLVE;
	}
	if (status == solve_status::SOLVE_DIJKSTRA_VALUES)
	{
		// Solve and update model
		dijkstra.solveAndUpdateUsingterrainprocessor(stroke.WrappedModelForPoissonSolver());
		// Update view
		dijkstra.update();
		status = solve_status::DONT_SOLVE;
	}
	progress.update();
}

void terrainapplication::draw()
{
	ofBackground(255.0);
	ofSetColor(0.0, 0.0, 0.0);
	stroke.draw();
//	solver.draw();
	dijkstra.draw();
	//myFont.loadFont("verdana.ttf", 12.0);
//	progress.draw();
	//myFont.drawString("Maryam Ariyan", 10, 770);
}

void terrainapplication::keyPressed(int key)
{
	stroke.keyPressed(key);
	//solver.keyPressed(key);
	dijkstra.keyPressed(key);
	progress.keyPressed(key);
	if (key == '3')
	{
		status = solve_status::SOLVE_POISSON_VALUES;
	}
	if (key == '4')
	{
		status = solve_status::SOLVE_DIJKSTRA_VALUES;
	}
	if (key == 's')
	{
		ofImage savedBg;
		//	savedBg.grabScreen(0, 0, ofGetScreenWidth(), ofGetScreenHeight());
		savedBg.grabScreen(10, 210, LENGTH, LENGTH);
		savedBg.saveImage("screenshot-" + ofGetTimestampString() + ".ppm");
		savedBg.saveImage("screenshot-" + ofGetTimestampString() + ".png");
	}
}

void terrainapplication::keyReleased(int key)
{
	;
}

void terrainapplication::mouseMoved(int x, int y)
{
	stroke.mouseMoved(x, y);
}

void terrainapplication::mouseDragged(int x, int y, int button)
{
	stroke.mouseDragged(x, y, button);
}

void terrainapplication::mousePressed(int x, int y, int button)
{
	stroke.mousePressed(x, y, button);
//	dijkstra.mousePressed(x, y, button);
}

void terrainapplication::mouseReleased(int x, int y, int button)
{
	stroke.mouseReleased(x, y, button);
}

void terrainapplication::windowResized(int w, int h)
{
	;
}

void terrainapplication::dragEvent(ofDragInfo dragInfo)
{
	;
}

void terrainapplication::gotMessage(ofMessage msg)
{
	;
}

//========================================================================
int main(){

	ofSetupOpenGL(1200, 750, OF_WINDOW);			// <-------- setup the GL context
	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new terrainapplication());
}
//========================================================================