#include "Edge.h"

using namespace std;

Edge::Edge()
{
	adjacent=nullptr;
	adjG=0;
}

Edge::Edge(Node* node, float val)
{
	adjacent=node;
	adjG=val;
}

Node* Edge::getAdjacentNode() const
{
	return adjacent;
}

float Edge::getAdjacentG() const
{
	return adjG;
}

Edge::~Edge(void)
{
}