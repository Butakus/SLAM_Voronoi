/** INFO ABOUT THE OLD JAVA ALGORITHM: 
 * This algorithm computes the bidirectional ring which minimizes the total ring length, using an ACO (Ant Colony Optimization) heuristic, described as
 * Ant System in the literature: M. Dorigo, V. Maniezzo, A. Colorni, "Ant system: optimization by a colony of cooperating agents", IEEE T on Cybernetics, 1996.
 * The cost of a link equals the euclidean distance between link end nodes. The algorithm executes ACO iterations until the maxExecTime is reached.
 * In each ACO iteration, a loop for each ant is executed. Each ant, creates a greedy-randomized solution using the pheromones information of each potential link, 
 * and each link length, as follows. Each ant starts in a node chosen randomly. At each iteration, an ant in node n decides the next node to visit 
 * randomly among the non-visited nodes, being the probability of choosing node n' proportional to ph_nn'^alpha b_nn'^beta. ph_nn' is the amount of pheromones 
 * associated to link nn', b_nn' is the inverse of the distance between both nodes. alpha and beta are parameters tuning the importance of pheromones 
 * and link distances respectively. After all ants have finished, an evaporation strategy is executed, where each link nn' looses pheromones multiplicatively 
 * (pheromones are multiplied by 1-r, where r is a 0...1 evaporation factor). After evaporation phase, a reinforcement step is completed, where each ant a adds 
 * a quantity 1/La to the pheromones of all links traversed, being La the total distance of its ring. 
 * @author Pablo Pavon-Marino
 * @version 1.0, April 2014
 * @since 1.0 */


#include <iostream>
#include <set>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

using namespace std;
using namespace cv;


/* Initialize some variables */

// Maximum running time. The time the algorithm will be running
double maxExecTimeSecs = 10.0;

// Number of ants
const int numAnts = 10;

// Pheromones factor
double alpha = 1.0;
// Heuristic benefit factor
double beta = 1.0;

// Percentage [0-1] of pheromones evaporation in each iteration
double evaporationFactor = 0.5;

// Random seed
long randomSeed = 5;



/* Receives a vector with values proportional to the probabilities p[s] of each option s. Returns the index of the sample chosen. 
 * Each sample s has a probability to be chosen proportional to p[s] */
int sampleUniformDistribution (vector<pair<int, double>>& next_nodes, double totalSumProbabilities)
{
	double comp = ((double) rand() / (RAND_MAX)) * totalSumProbabilities;
	double accumValue = 0;
	
	for (int index = 0; index < next_nodes.size(); ++index)
	{
		accumValue += next_nodes[index].second;
		if (accumValue >= comp) return index;
	}
	return (next_nodes.size() - 1);
}

double euclideanDist(Point p, Point q)
{
    Point diff = p - q;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}

/* This function implements the greedy-randomized computation of a ring by an ant, as described in the class documentation */
double computeAntSolution (vector<Point>& nodes, int start_node, int end_node, Mat pheromones, Mat graph_distances, vector<int>& solution_path, double alpha, double beta)
{
	// Number of nodes
	int N = nodes.size();

	// vector to store the computed solution
	solution_path.clear();
	solution_path.push_back(start_node);

	// Keep a set of non-visited nodes. Initially, the set contains all nodes but the first
	set<int> notVisitedNode;
	for (int i = 0; i < N; ++i) if (i != start_node) notVisitedNode.insert(i);
	
	// In each iteration, a node is added to the path
	double path_cost = 0;
	int currentNode = start_node;
	while (currentNode != end_node)
	{
		//cout << "Current node: " << currentNode << endl;
		/* Create a list with the probabilities of possible next nodes  */
		// Store the index and probability of being chosen of each node
		vector<pair<int, double>> next_nodes;
		next_nodes.resize(notVisitedNode.size());

		double totalSumProbabilities = 0;
		bool empty = true;
		int n = 0;
		for (auto it = notVisitedNode.begin(); it != notVisitedNode.end(); ++it, ++n)
		{
			// Check if we can navigate to this node
			double distance = graph_distances.at<double>(currentNode, *it);
			if (distance > 0)
			{
				empty = false;
				//cout << "distance " << currentNode << "," << *it << ": " << distance << endl;
				// Store the node index
				next_nodes[n].first = *it;
				// Compute and store this node's probability of being chosen
				double distance_heuristic = 1.0 / euclideanDist(nodes[*it], nodes[end_node]);
				next_nodes[n].second = pow(pheromones.at<double>(currentNode,*it), alpha) * pow(distance_heuristic, beta);
				//cout << "probability: " << next_nodes[n].second << endl;
				totalSumProbabilities += next_nodes[n].second;
			}
		}
		if (empty)
		{
			cout << "No more nodes to jump" << endl;
			solution_path.clear();
			return 999999.9;
		}
		/* Choose next node randomly according to the probabilities computed */
		int nextNodeIndex = sampleUniformDistribution(next_nodes, totalSumProbabilities);
		//cout << "next node index: " << nextNodeIndex << endl;
		int nextNode = next_nodes[nextNodeIndex].first;
		
		/* Add the node to the rings */
		solution_path.push_back(nextNode);
		notVisitedNode.erase(nextNode);
		path_cost += graph_distances.at<double>(currentNode,nextNode);
		currentNode = nextNode;
	}

	return path_cost;
}


/* ACO Planner: Compute the best path between 2 nodes for a given graph.
*	Inputs params:
*		int nodes: number of nodes in the graph
*		int start_node: index of the starting point
*		int end_node: index of the ending point
*		Mat graph_distances: Matrix with the distance between each node. -1 if there is no connection
*	Output params:
*		vector<int> best_path: oOutput vector with the indexes of the nodes in the best path found
*	Returns:
		double best_cost: Path ditance
*/
double ACOPlanner(vector<Point>& nodes, int start_node, int end_node, Mat graph_distances, vector<int>& best_path)
{
	// Initialize the random number generator with the seed
	srand(randomSeed);

	// Number of nodes
	int N = nodes.size();
	
	/* The best solution found so far (incumbent solution) is stored in these variables */
	best_path.clear();
	double best_cost = 99999999.9;

	/* Initialize some ACO control variables: the pheromones */
	Mat pheromones = Mat_<double>(N, N);
	// Initialize all paths pheromones to 1.0
	pheromones.setTo(1.0);

	/* Main loop. Stop when maximum execution time is reached */
	clock_t algorithmStartTime = clock();
	while (((clock() - algorithmStartTime) / (double)CLOCKS_PER_SEC) < maxExecTimeSecs)
	{
		//cout << "Iter" << endl;
		// Array with the solution paths and costs given by each ant
		vector<int> solutions_paths[numAnts];
		double solutions_costs[numAnts];
		//cout << "1" << endl;
		for (int a = 0; a < numAnts; ++a)
		{
			/* Build a greedy-random solution using pheromones info */
			//cout << "1.1" << endl;
			vector<int> solution_path;
			solutions_costs[a] = computeAntSolution(nodes, start_node, end_node, pheromones, graph_distances, solution_path, alpha, beta);
			//cout << "1.2" << endl;
			solutions_paths[a] = solution_path;
			/* Update incumbent solution */
			if (solutions_costs[a] < best_cost)
			{
				best_cost = solutions_costs[a];
				best_path = solution_path;
			}
		}
		/* Apply evaporation strategy */
		for (int i = 0; i < N; ++i)
		{
			for (int j = 0; j < N ; ++j)
			{
				if (i != j) pheromones.at<double>(i,j) *= (1.0 - evaporationFactor);
			}
		}
		/* Apply reinforcement strategy */
		for (int a = 0; a < numAnts; ++a)
		{
			int solution_size = solutions_paths[a].size();
			double benefit = 1.0 / solutions_costs[a];
			
			if (solution_size > 0)
			{
				for (int n = 0; n < solution_size - 1; ++n)
				{
					pheromones.at<double>(solutions_paths[a][n], solutions_paths[a][n+1]) += benefit;
				}
				pheromones.at<double>(solutions_paths[a][solution_size - 1], solutions_paths[a][0]) += benefit;
			}
		}
	}
	cout << "Finished" << endl;
	return best_cost;
}
