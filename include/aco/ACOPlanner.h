/** Info about the modified ACO algorithm:
 * This is a variation of an algorithm thats computes the bidirectional ring which minimizes the total ring length, using an ACO (Ant Colony Optimization) heuristic, described as
 * Ant System in the literature: M. Dorigo, V. Maniezzo, A. Colorni, "Ant system: optimization by a colony of cooperating agents", IEEE T on Cybernetics, 1996.
 * The cost of a link equals the euclidean distance between link end nodes. The algorithm executes ACO iterations until the maxExecTime is reached.
 * In each ACO iteration, a loop for each ant is executed. Each ant, creates a greedy-randomized solution using the pheromones information of each potential link, 
 * and each link length, as follows. Each ant starts in a node chosen randomly. At each iteration, an ant in node n decides the next node to visit 
 * randomly among the non-visited nodes, being the probability of choosing node n' proportional to ph_nn'^alpha b_nn'^beta. ph_nn' is the amount of pheromones 
 * associated to link nn', b_nn' is the inverse of the distance between both nodes. alpha and beta are parameters tuning the importance of pheromones 
 * and link distances respectively. After all ants have finished, an evaporation strategy is executed, where each link nn' looses pheromones multiplicatively 
 * (pheromones are multiplied by 1-r, where r is a 0...1 evaporation factor). After evaporation phase, a reinforcement step is completed, where each ant a adds 
 * a quantity 1/La to the pheromones of all links traversed, being La the total distance of its ring. 
 * 
 * Original Java algorithm to solve TSP problem:
 * 	@author Pablo Pavon-Marino
 * 	@version 1.0, April 2014
 * 	@since 1.0 
 * C++ variation to solve shortest path between 2 nodes:
 * 	@author Francisco Moreno
 * 	@version 1.0, May 2016
 * 	@since 1.0
 */

#ifndef ACOPLANNER_H
#define ACOPLANNER_H

#include <iostream>
#include <set>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

using namespace std;
using namespace cv;

#define RANDOM_SEED 5
#define PARAM_NUM_ANTS 10
#define PARAM_ALPHA 1.0
#define PARAM_BETA 1.0
#define PARAM_EVAPORATION 0.5
#define PARAM_EXEC_TIME 5.0

class ACOPlanner
{
public:
	ACOPlanner(int numAnts = PARAM_NUM_ANTS, double alpha = PARAM_ALPHA, double beta = PARAM_BETA, double evaporationFactor = PARAM_EVAPORATION);
	~ACOPlanner();

	/* Compute the best path between 2 nodes for a given graph.
	*	Inputs params:
	*		int start_node: index of the starting point
	*		int end_node: index of the ending point
	*	Output params:
	*		vector<int> best_path: oOutput vector with the indexes of the nodes in the best path found
	*	Returns:
	*		double best_cost: Path ditance
	*/
	double computePath(int start_node, int end_node, Mat& graph_distances, vector<int>& best_path, double maxExecTimeSecs = PARAM_EXEC_TIME);

	/* This function implements the greedy-randomized computation of a path by an ant */
	double computeAntSolution (int start_node, int end_node, Mat& graph_distances, vector<int>& solution_path);

private:
	/* Receives a vector with values proportional to the probabilities p[s] of each option s. Returns the index of the sample chosen. 
	 * Each sample s has a probability to be chosen proportional to p[s] */
	int sampleUniformDistribution(vector<pair<int, double>>& next_nodes, double totalSumProbabilities);

	// Number of ants
	int numAnts;

	// Pheromones level of each link
	Mat pheromones;

	// Pheromones factor
	double alpha;
	// Heuristic benefit factor
	double beta;

	// Percentage [0-1] of pheromones evaporation in each iteration
	double evaporationFactor;

};
#endif