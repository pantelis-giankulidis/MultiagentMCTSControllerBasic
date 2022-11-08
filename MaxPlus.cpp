#ifndef MPLUS

#define MPLUS

#include "carAction.hpp";
#include "utils.hpp"

class MaxPlus {
private:
	/*1 for Exploration and 0 for no exploration*/
	int edgeExplorationFlag;

	/*1 for Exploration and 0 for no exploration*/
	int nodeExplorationFlag;

	/*1 for Normalization and 0 for not normalization*/
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
		this->edgeExplorationFlag = 0;
		this->nodeExplorationFlag = 1;
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
			for (int j = i; j < maxPlusGraph.size(); j++) {
				if (i == j) {
					continue;
				}
			
				if (abs(maxPlusGraph[i].getCar().getPositionX() - maxPlusGraph[j].getCar().getPositionX()) <= DISTANCE_FOR_CREATING_EDGE) {
					edge* e = new edge(maxPlusGraph[i].getCar().getCarNumber(), maxPlusGraph[j].getCar().getCarNumber());
					maxPlusGraph[i].addEdge(e);
					maxPlusGraph[j].addEdge(e);
				}
				//std::cout << "INSERT" << std::endl;
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
		int initActionIndex = 1;
		float initActionValue = 0;
		for (int i = 0; i < maxPlusGraph.size(); i++) {
			node nod = maxPlusGraph[i];
			nod.setTemporaryBestActionIndex(initActionIndex); // zero acceleration in both directions
			nod.setTemporaryBestActionValue(initActionValue);

			for (edge* e : nod.getAdjacencyList()) {
				if (e->getCarNumberSender() == nod.getCar().getCarNumber()) {
					e->setBestActionIndexSender(initActionIndex);
					e->setBestActionValueSender(initActionValue);
				}
				else {
					e->setBestActionIndexReceiver(initActionIndex);
					e->setBestActionValueReceiver(initActionValue);
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
	void coordinateActions() {

		// Initially,pick random actions for the simulation.
		generateRandomActions();
		setInitialMessageValues();

		/*Apply the MaxPlus to get one coordinated action at one leaf of the Monte Carlo Tree */
		for (int i = 0; i < maxPlusMCSimulationRounds; i++) {
			messagePassing();
			computeQwithMessages();
		}
		
		//TODO message padding with edge exploration
		this->edgeExplorationFlag = 1;
		messagePassing();
		computeQwithMessages();
		calculateBestq();
	}



	/* Between two iterations, the message values stored in the graph, must be set to zero.
	*
	*/
	void setInitialMessageValues() {
		for (int i = 0; i < maxPlusGraph.size(); i++) {
			for (edge* e : maxPlusGraph[i].getAdjacencyList()) {
				e->initialiseMSender();
				e->initialiseMReceiver();
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

			// For all agents i
			for (int i = 0; i < maxPlusGraph.size(); i++) {
				node agent = maxPlusGraph[i];

				// For all edges that the node has
				// 
				for (int j = 0; j < agent.getAdjacencyList().size(); j++) {
					edge* e = agent.getAdjacencyList()[j];
					bool is_sender = (agent.getCar().getCarNumber() == e->getCarNumberSender());

					for (int action = 0; action < availableActions; action++) {

						//UPDATE
						for (int actionj = 0; actionj < availableActions; actionj++) {
							float newMij = agent.getQ()[action] + e->getQij()[is_sender ? action : actionj][is_sender ? actionj : action]  - sumOfMki(agent, e, action);
							
							if (is_sender) {
								if (normalizationFlag == 1) {
									newMij = newMij - (e->getMSenderRowSum(action) / availableActions);
								}
								if (edgeExplorationFlag == 1) {
									newMij = newMij + C * sqrt(log(agent.getN() + 1) / (e->getNij()[action][actionj] + 1));
								}
								e->setMSender(action, actionj, newMij);
							}
							else {
								if (normalizationFlag == 1) {
									newMij = newMij - (e->getMReceiverRowSum(action) / availableActions);
								}
								if (edgeExplorationFlag == 1) {
									newMij = newMij + C * sqrt(log(agent.getN() + 1) / (e->getNij()[actionj][action] + 1));
								}
								e->setMReceiver(action, actionj, newMij);
							}
						}
					}
					agent.setEdge(j, e);
				}

				maxPlusGraph[i] = agent;
			}
		}
	}


	/* 
	* Compute the values of q, for every message passing phase 
	*
	*/
	void computeQwithMessages() {

		for (int i = 0; i < maxPlusGraph.size(); i++) {
			node nod = maxPlusGraph[i];
			//nod.printTempQ();
			std::vector<float> tempQ = nod.getTempQForMaxPlus();

			for (int actionIndex = 0; actionIndex < availableActions; actionIndex++) {
				float q = nod.getQ()[actionIndex] - sumOfMki(nod, NULL, actionIndex);
				if (nodeExplorationFlag==1) {
					q = q + C * sqrt(log(nod.getSumN()+ 1) / (nod.getNi()[actionIndex] + 1));
				}
				tempQ[actionIndex] = q;
			}
			nod.setTempQForMaxPlus(tempQ);
			maxPlusGraph[i] = nod;
		}

	}

	/*
	* After the message passing phase, we take the best action of each agent
	*/
	void calculateBestq() {
		float bestQ = -1000;
		int indexQ = 0;
		for (node & n:maxPlusGraph) {
			for (int action = 0; action < availableActions; action++) {
				if (n.getTempQForMaxPlus()[action] > bestQ) {
					bestQ = n.getTempQForMaxPlus()[action];
					indexQ = action;
				}
			}
			n.setTemporaryBestActionIndex(indexQ);
			n.setTemporaryBestActionValue(bestQ);
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
			bool is_sender = agent.getCar().getCarNumber() == e->getCarNumberSender();
			sumofmki = sumofmki + is_sender ? e->getBestResponceSendAction(action) : e->getBestResponceReceiveAction(action);
		}
		return sumofmki/availableActions;
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