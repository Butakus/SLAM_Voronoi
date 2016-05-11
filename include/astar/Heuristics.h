#ifndef HEURISTICS_H
#define HEURISTICS_H

#include "Node.h"
#include <math.h>

inline float manhattan(Node* actual, Node* finalNode)
{
	float hValue=abs(actual->getCoordinates().x-finalNode->getCoordinates().x) + 
				abs(actual->getCoordinates().y-finalNode->getCoordinates().y);
	return hValue;
};

inline float euclidean(Node* actual, Node* finalNode)
{
	float hValue=sqrt((actual->getCoordinates().x-finalNode->getCoordinates().x)*(actual->getCoordinates().x-finalNode->getCoordinates().x)+
		(actual->getCoordinates().y-finalNode->getCoordinates().y)*(actual->getCoordinates().y-finalNode->getCoordinates().y));
	return hValue;
};

inline float diagonal(Node* actual, Node* finalNode)
{
	float dx=abs(actual->getCoordinates().x-finalNode->getCoordinates().x);
	float dy=abs(actual->getCoordinates().y-finalNode->getCoordinates().y);
	float diagonal=std::min(dx,dy);
	float derecho=dx+dy;
	float hValue=diagonal+derecho-2*diagonal;
	return hValue;
};

inline float none(Node* actual, Node* finalNode)
{
	return 0;
};

#endif