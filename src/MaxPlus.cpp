#ifndef MPLUS

#define MPLUS

#include "carAction.hpp";
#include "utils.hpp"

class MaxPlus {
private:
	/*0 for Exploration and 1 for not exploration*/
	int edgeExplorationFlag;

	/*0 for Exploration and 1 for not exploration*/
	int nodeExplorationFlag;

	/*0 for Normalization and 1 for not normalization*/
	int normalizationFlag;

	/*The exploration term for FV-MCTS*/
	float C;

	/*The gamma term for the reward at each timestep.It is always between 0 and 1*/
	float gamma;

	/*The number of agents participating in the state*/
	int numberOfAgents;

public:
	MaxPlus(int numberOfAgents) {
		this->C = EXPLORATION_TERM;
		this->numberOfAgents = numberOfAgents;
		this->edgeExplorationFlag = 1;
		this->nodeExplorationFlag = 0;
		this->normalizationFlag = 0;
		this->gamma = FVMCTS_GAMMA;
	}

	/*Utility function for printing the graph statistics at the moment called
	*
	*/
	void printGraph() {
		for (int j = 0; j < maxPlusGraph.size(); j++) {
			//std::cout << "Node " << maxPlusGraph[j].getCar().getCarNumber() << " best temp action: "<<maxPlusGraph[j].getTemporaryBestActionIndex() << std::endl;
		}
	}

	void setC(float c) {
		this->C = c;
	}

	/*Initiates the maxplusgraph,with zeros in both node and edge statistics
	*
	*/
	void createGraph(laneFreeGlobalState s) {

		/* Creating the nodes of the graph */
		for (int i = 0; i < s.getNumberOfCarsInRoads(); i++) {
			Car nodecar = s.getCar(i);
			node n = node(nodecar);
			maxPlusGraph.push_back(n);
		}

		/* Creating the edges of the graph*/
		for (int i = 0; i < maxPlusGraph.size(); i++) {
			for (int j = i + 1; j < maxPlusGraph.size(); j++) {
				if (abs(maxPlusGraph[i].getCar().getPositionX() - maxPlusGraph[j].getCar().getPositionX()) < DISTANCE_FOR_CREATING_EDGE) {
					edge* e = new edge(maxPlusGraph[i].getCar().getCarNumber(), maxPlusGraph[j].getCar().getCarNumber());
					maxPlusGraph[i].addEdge(e);
					maxPlusGraph[j].addEdge(e);
				}
			}
		}

	}

	/*Updatess flag for node exploration
	*
	*/
	void updateNodeExplorationFlag(int explorationFlag) {
		this->nodeExplorationFlag = explorationFlag;
	}


	/*Updates the flag for Normalization
	*
	*/
	void updateNormalizationFlag(int normalizationFlag) {
		this->normalizationFlag = normalizationFlag;
	}

	/*Getter for the constant gamma
	*
	*/
	float getGamma() {
		return gamma;
	}

	/*Initial part of a MaxPlus simulation. The agents pick a random action
	*
	*/
	void generateRandomActions() {
		int initActionIndex = 2;
		float initActionValue = 0;
		for (int i = 0; i < maxPlusGraph.size(); i++) {
			node nod = maxPlusGraph[i];
			nod.setTemporaryBestActionIndex(initActionIndex); // zero acceleration in both directions
			nod.setTemporaryBestActionValue(initActionValue);

			for (edge* e : nod.getAdjacencyList()) {
				if (e->getCarNumberI() == nod.getCar().getCarNumber()) {
					e->setBestActionIndexI(initActionIndex);
					e->setBestActionValueI(initActionValue);
				}
				else {
					e->setBestActionIndexJ(initActionIndex);
					e->setBestActionValueJ(initActionValue);
				}
			}
			maxPlusGraph[i] = nod;
		}
	}


	/*One maxplus iteration. It makes simulations for a time given , with an
	* exploration constant c, and outputs the best coordination action the
	* agents could produce
	*
	* @Params
	* INT duration: The number of iteration for 1 execution of maxplus
	*
	* @Return -> coordinated action List<carActionPair>
	*/
	void maxplus() {

		// Initially,pick random actions for the simulation.
		generateRandomActions();

		/*Apply the MaxPlus to get one coordinated action at one leaf of the Monte Carlo Tree */
		for (int i = 0; i < maxPlusMCSimulationRounds; i++) {

			setInitialMessageValues();
			messagePassing();
			calculateNodeQAndGetBestAction();

		}
		std::cout << "MAXPLUS ENDED WITH: " << std::endl;
		for (int i = 0; i < maxPlusGraph.size(); i++) {
			std::cout << " agent " << maxPlusGraph[i].getCar().getCarNumber() << ", action = " << maxPlusGraph[i].getTemporaryBestActionIndex() << ", q=" << maxPlusGraph[i].getTemporaryBestActionValue() << std::endl;
		}
	}


	/* Between two iterations, the message values stored in the graph, must be set to zero.
	*
	*/
	void setInitialMessageValues() {
		for (int i = 0; i < maxPlusGraph.size(); i++) {
			for (edge* e : maxPlusGraph[i].getAdjacencyList()) {
				e->initialiseMij();
				e->initialiseMji();
			}
		}
	}

	/* The message passing phase of the maxplus algorithm
	*
	* @Param
	* List<carActionPair> : the actions of the agents from the previous iteration.
	*/
	void messagePassing() {
		int rounds = 0;

		while (rounds < maxPlusMessagePassingPerRound) {
			rounds++;

			// Add exploration terms in the last iteration
			if (rounds == maxPlusMessagePassingPerRound - 1) {
				edgeExplorationFlag = 0;
				normalizationFlag = 0;
			}

			// For all agents i
			for (int i = 0; i < maxPlusGraph.size(); i++) {
				node agent = maxPlusGraph[i];

				// For all edges that the node has
				// 
				for (int j = 0; j < agent.getAdjacencyList().size(); j++) {
					edge* e = agent.getAdjacencyList()[j];
					bool is_i = agent.getCar().getCarNumber() == e->getCarNumberI();

					for (int action = 0; action < availableActions; action++) {


						//UPDATE
						for (int actionj = 0; actionj < availableActions; actionj++) {
							//std::cout << "AJ="<<actionj << std::endl;
							float newMij = agent.getQ()[action] + e->getQij()[is_i ? action : actionj][is_i ? actionj : action];// +sumOfMki(agent, e, action);
							float qij = e->getQij()[is_i ? action : actionj][is_i ? actionj : action];
							if (edgeExplorationFlag == 0) {

								//std::cout << "ACTION "<<action<<" , aj="<<actionj<<", edge exploration components : Q(ai) = " << agent.getQ()[action] << ", Qij(aj)="<<qij<<",N="<<agent.getN()<<", log(N + 1) = " << log(agent.getN() + 1) << ", Ni(ai) = " << e->getNij()[is_i ? action : e->getBestActionIndexI()][is_i ? e->getBestActionIndexJ() : action] << std::endl;
								newMij = newMij + C * sqrt(log(agent.getN() + 1) / (e->getNij()[is_i ? action : actionj][is_i ? actionj : action] + 1));
							}

							if (is_i) {
								if (normalizationFlag == 0) {
									newMij = newMij - (e->getMijRowSum(action) / availableActions);
								}
								e->setMij(action, actionj, newMij);
							}
							else {
								if (normalizationFlag == 0) {
									newMij = newMij - (e->getMjiRowSum(action) / availableActions);
								}
								e->setMji(action, actionj, newMij);
							}
						}
					}
					agent.setEdge(j, e);
				}

				maxPlusGraph[i] = agent;
			}


		}
		edgeExplorationFlag = 1;
		normalizationFlag = 1;
	}


	/* Equation Line 16 on the right side
	*
	*/
	void calculateNodeQAndGetBestAction() {

		for (int i = 0; i < maxPlusGraph.size(); i++) {
			node nod = maxPlusGraph[i];
			int pickedActionIndex = 2;
			float BestQ = -10000;

			for (int actionIndex = 0; actionIndex < availableActions; actionIndex++) {

				float q = nod.getQ()[actionIndex];// +sumOfMki(nod, NULL, actionIndex);

				if (nodeExplorationFlag == 0) {
					q = q + C * sqrt(log(vectorSum(nod.getNi()) + 1) / nod.getNi()[actionIndex]);
				}

				if (q > BestQ) {
					//std::cout << "CHECK FOR BIGGER" << std::endl;
					BestQ = q;
					pickedActionIndex = actionIndex;
				}
				if (q == BestQ) {
					//std::cout << "CHECK FOR TIES " << std::endl;
					bool pickOnTies = rand() > (RAND_MAX/2);
					if (pickOnTies) {
						//std::cout << "HAD A TIE" << std::endl;
						BestQ = q;
						pickedActionIndex = actionIndex;
					}
				}
				
			}
			nod.setTemporaryBestActionIndex(pickedActionIndex);
			nod.setTemporaryBestActionValue(BestQ);

			for (int p = 0; p < nod.getAdjacencyList().size(); p++) {
				edge* e = nod.getAdjacencyList()[p];

				if (e->getBestActionIndexI() == nod.getCar().getCarNumber()) {
					e->setBestActionIndexI(pickedActionIndex);
					e->setBestActionValueI(BestQ);
				}
				else {
					e->setBestActionIndexJ(pickedActionIndex);
					e->setBestActionValueJ(BestQ);
				}

				nod.setEdge(p, e);
			}
			maxPlusGraph[i] = nod;

		}

	}




	int getBestActionFromNode(int carNumber) {
		float bestQ = -1000;
		int index = 0, i = 0;

		// Get the car from the graph
		for (node nod : maxPlusGraph) {
			if (nod.getCar().getCarNumber() == carNumber) {
				index = i;
				break;
			}
			i++;
		}

		// Get the best action from the list of actions
		i = 0;
		int actionIndex = 0;
		for (float qi : maxPlusGraph[index].getQ()) {
			if (qi > bestQ) {
				bestQ = qi;
				actionIndex = i;
			}
			i++;
		}

		return actionIndex;
	}


	/* Third part of the equation(5)
	*
	* */
	float sumOfMki(node agent, edge* edgeToConclude, int action) {
		float sumofmki = 0;

		for (edge* e : agent.getAdjacencyList()) {
			if (e == edgeToConclude) {
				continue;
			}
			bool is_i = agent.getCar().getCarNumber() == e->getCarNumberI();
			sumofmki = sumofmki + is_i ? e->getBestM_ifJgetsActionI(action) : e->getBestM_ifIgetsActionJ(action);

		}
		return sumofmki;
	}

	/*	Getter and setter for the number of agents  */
	void setNumberOfAgents(int numberOfAgents) {
		this->numberOfAgents = numberOfAgents;
	}

	int getNumberOfAgents() {
		return numberOfAgents;
	}


	/*Utility function
	*
	*/
	Car getCarWithNumber(int number) {
		for (node n : maxPlusGraph) {
			if (n.getCar().getCarNumber() == number) {
				return n.getCar();
			}
		}
	}

};

#endif