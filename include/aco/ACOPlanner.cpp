#include "ACOPlanner.h"

using namespace std;
using namespace cv;


ACOPlanner::ACOPlanner(int numAnts, double alpha, double beta, double evaporationFactor)
{
	srand(RANDOM_SEED);
	this->numAnts = numAnts;
	this->alpha = alpha;
	this->beta = beta;
	this->evaporationFactor = evaporationFactor;	
}

ACOPlanner::~ACOPlanner()
{
	pheromones.release();
}



int ACOPlanner::sampleUniformDistribution (vector<pair<int, double>>& next_nodes, double totalSumProbabilities)
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


double ACOPlanner::computeAntSolution (int start_node, int end_node, Mat& graph_distances, vector<int>& solution_path)
{
	// Number of nodes
	int N = graph_distances.rows;

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
				double distance_heuristic = 1.0 / abs(graph_distances.at<double>(*it, end_node));
				next_nodes[n].second = pow(pheromones.at<double>(currentNode,*it), alpha) * pow(distance_heuristic, beta);
				//cout << "probability: " << next_nodes[n].second << endl;
				totalSumProbabilities += next_nodes[n].second;
			}
		}
		if (empty)
		{
			// Could not find a solution
			//cout << "No more nodes to jump" << endl;
			solution_path.clear();
			return 999999.9;
		}
		/* Choose next node randomly according to the probabilities computed */
		int nextNodeIndex = sampleUniformDistribution(next_nodes, totalSumProbabilities);
		int nextNode = next_nodes[nextNodeIndex].first;
		
		/* Add the node to the rings */
		solution_path.push_back(nextNode);
		notVisitedNode.erase(nextNode);
		path_cost += graph_distances.at<double>(currentNode,nextNode);
		currentNode = nextNode;
	}

	return path_cost;
}


double ACOPlanner::computePath(int start_node, int end_node, Mat& graph_distances, vector<int>& best_path, double maxExecTimeSecs)
{
	// Number of nodes
	int N = graph_distances.rows;
	
	/* The best solution found so far (incumbent solution) is stored in these variables */
	best_path.clear();
	double best_cost = 99999999.9;

	/* Initialize some ACO control variables: the pheromones */
	pheromones = Mat_<double>(N, N);
	// Initialize all paths pheromones to 1.0
	pheromones.setTo(1.0);

	/* Main loop. Stop when maximum execution time is reached */
	clock_t algorithmStartTime = clock();
	while (((clock() - algorithmStartTime) / (double)CLOCKS_PER_SEC) < maxExecTimeSecs)
	{
		//cout << "iter" << endl;
		// Array with the solution paths and costs given by each ant
		vector<int> solutions_paths[numAnts];
		double solutions_costs[numAnts];

		for (int a = 0; a < numAnts; ++a)
		{
			/* Build a greedy-random solution using pheromones info */
			vector<int> solution_path;
			solutions_costs[a] = computeAntSolution(start_node, end_node, graph_distances, solution_path);
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
	return best_cost;
}
