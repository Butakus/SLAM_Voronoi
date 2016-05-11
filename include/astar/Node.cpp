#include "Node.h"

using namespace std;
using namespace cv;

Node::Node()
{
}

Node::Node(Point2f coor,Node *parent,float gvalue)
{
	coordinates=coor;
	adjacents=list<Edge>();
	parentNode=parent;
	g=gvalue;
	obstaculo=false;
}

void Node::setParentNode(Node *node)
{
	parentNode=node;
}
void Node::setGValue(float gvalue)
{
	g=gvalue;
}
void Node::setFValue(float fvalue)
{
	f=fvalue;
}
void Node::setAdjacents(list<Edge> listAdj)
{
	adjacents=listAdj;
}
void Node::setObstaculo(bool obs)
{
	obstaculo=obs;
}

void Node::insertAdjacent(Edge edge)
{
	adjacents.push_back(edge);
}

Node *Node::getParentNode() const
{
	return parentNode;
}

float Node::getGValue() const
{
	return g;
}

float Node::getFValue() const
{
	return f;
}

Point2f Node::getCoordinates() const
{
	return coordinates;
}

const list<Edge> &Node::getAdjacents() const
{
	return adjacents;
}

bool Node::getObstaculo() const
{
	return obstaculo;
}

/*bool Node::operator<(Node* left) const
{
	return this->getFValue()<left->getFValue();
}*/

Node::~Node(void)
{
}