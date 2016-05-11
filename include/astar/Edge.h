#ifndef EDGE_H
#define EDGE_H

class Node;

class Edge
{
public:
	Edge();
	Edge(Node*,float);

	Node* getAdjacentNode() const;
	float getAdjacentG() const;

	virtual ~Edge();
private:
	Node* adjacent;
	float adjG;
};

#endif
