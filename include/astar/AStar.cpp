#include "AStar.h"
#include <algorithm>

using namespace std;

bool sortNodes(Node* node1, Node* node2)
{
	return node1->getFValue()<node2->getFValue();
};


std::tuple<std::list<Node*>, std::vector<Node*>, std::vector<Node*>> calculateAStar(Node* initialNode, Node* finalNode, std::function<float(Node*, Node*)> typeHeu)
{
	vector<Node*> openList;
	vector<Node*> closedList;
	
	//std::sort(openList.begin(),openList.end(),sortNodes);
	bool finish=false;
	
	if(initialNode!=nullptr && finalNode!=nullptr)
	{
		//Insertar nodo inicial en lista abierta
		openList.push_back(initialNode);

		while(!openList.empty() && !finish)
		{
			//Cogemos el nodo con menor F como nodo actual, lo metemos en la lista cerrada lo sacamos de la lista abierta.
			Node* actualNode=*openList.begin();
			closedList.push_back(actualNode);
			openList.erase(openList.begin());


			//Recorremos los nodos adyacentes al nodo actual
			for(auto it=actualNode->getAdjacents().begin();it!=actualNode->getAdjacents().end();it++)
			{
				if(std::find(closedList.begin(),closedList.end(),it->getAdjacentNode())==closedList.end() && !it->getAdjacentNode()->getObstaculo())
				{
					//Si no está en la lista abierta lo metemos
					if(std::find(openList.begin(),openList.end(),it->getAdjacentNode())==openList.end())
					{
						float h=typeHeu(it->getAdjacentNode(),finalNode);
						it->getAdjacentNode()->setGValue(actualNode->getGValue()+it->getAdjacentG());
						it->getAdjacentNode()->setFValue(it->getAdjacentNode()->getGValue()+h);
						it->getAdjacentNode()->setParentNode(actualNode);
						openList.push_back(it->getAdjacentNode());
						std::sort(openList.begin(),openList.end(),sortNodes);
					}
					//Si está en la lista abierta, comparamos el valor de G que tiene el nodo metido en la lista abierta y
					// y el valor de G del nodo adyacente que estamos comprobando.
					else
					{
						//Si el valor que tiene el nodo adyacente es menor al que tenía el de la lista abierta, reemplazamos
						// este nuevo nodo adyacente por el que había en la lista abierta
						float newG=actualNode->getGValue()+it->getAdjacentG();
						if(newG<it->getAdjacentNode()->getGValue())
						{
							float h=typeHeu(it->getAdjacentNode(),finalNode);
							it->getAdjacentNode()->setGValue(newG);
							it->getAdjacentNode()->setFValue(newG+h);
							it->getAdjacentNode()->setParentNode(actualNode);
						}
					}			
				}
			}

			//Si se mete en la lista cerrada el nodo final, quiere decir que ya hemos encontrado el camino
			if(std::find(closedList.begin(),closedList.end(),finalNode)!=closedList.end())
				finish=true;
		}
	}
	//Una vez encontrado el camino, cogemos el nodo padre de cada nodo padre del anterior nodo, empezando por el nodo padre del nodo final
	if(finish)
	{
		list<Node*> road;
		bool findIni=false;

		road.push_front(finalNode);
		while(!findIni)
		{
			/*Node *parent=road.front()->getParentNode();
			if(parent==nullptr)*/
			if(road.front()==initialNode)
				findIni=true;
			else
			{
				Node *parent=road.front()->getParentNode();
				road.push_front(parent);
			}
		}
		return std::make_tuple(road, closedList, openList);
		//return road;
	}
	else
		return std::make_tuple(list<Node*>(), closedList, openList);
		//return list<Node*>();
}