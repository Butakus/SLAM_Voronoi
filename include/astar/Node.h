#ifndef NODE_H
#define NODE_H

#include "Edge.h"
#include <list>
#include <set>

#include <opencv2/opencv.hpp>

using namespace cv;

class Node
{
public:
	Node();
	Node(Point2f,Node*,float);

	void setParentNode(Node *node);
	void setGValue(float g);
	void setFValue(float f);
	void setAdjacents(std::list<Edge>);
	void setObstaculo(bool);

	void insertAdjacent(Edge edge);

	Node* getParentNode() const;
	Point2f getCoordinates() const;
	float getGValue() const;
	float getFValue() const;
	const std::list<Edge>& getAdjacents() const;
	bool getObstaculo() const;

	//bool operator<(Node* left) const;

	virtual ~Node();
private:
	std::list<Edge> adjacents;
	Point2f coordinates;
	Node* parentNode;
	float f,g;
	bool obstaculo;
};

#endif
