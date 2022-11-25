#ifndef MCTS_CPP
#define MCTS_CPP
#define LONGITUDINAL_ACCELERATION_VALUES 7
#define LATERAL_ACCELERATION_VALUES 3
#include "MCTS.hpp"
#include "LaneFree_win.h"
#include "laneFreeSimulation.hpp"
#include "FactoredValueMCTS.hpp"
#include <fstream>

/*IMPLEMENTATIONS FOR THE FUNCTIONS OF THE carAction CLASS IN THE carAction.hpp FILE

	@carAction() -- The default constructor
	@carAction(double valueLong, double valueLat) -- Constructor
	@execute(laneFreeState & state) -- Applies the action to the state given.
	@setLongitudinalAccelerationValue(double value) --setter for the longitudinal acceleration
	@setLateralAccelerationValue(double value)   -- setter for the lateral acceleration
	@getLongitudinalAccelerationValue() --getter for the longitudinal acceleration
	@getLateralAccelerationValue() -- getter for the lateral acceleration
*/


//extern std::vector<node> maxPlusGraph;
std::ofstream thats("../final.txt");

carAction::carAction(double valueLong, double valueLat) {
	accValueLongitudinal = valueLong;
	accValueLateral = valueLat;
}

void carAction::execute(laneFreeState& state) {
	Car controlledCar = state.getControlledCar();

	// Change controlled car's dynamics
	int accelX = getLongitudinalAccelerationValue();
	int accelY = getLateralAccelerationValue();

	if (accelX == DBL_MIN) {
		accelX = 0;
	}
	if (accelY == DBL_MIN) {
		accelY = 0;
	}
	double oldVelocityX = controlledCar.getVelocityX();
	double oldVelocityY = controlledCar.getVelocityY();

	controlledCar.setVelocity(oldVelocityX + accelX * SIMULATION_CONSTANT_TIME, oldVelocityY + accelY * SIMULATION_CONSTANT_TIME);
	controlledCar.setPosition(controlledCar.getPositionX() + oldVelocityX * SIMULATION_CONSTANT_TIME + (1 / 2) * accelX * SIMULATION_CONSTANT_TIME_SQUARED, controlledCar.getPositionY() + oldVelocityY * SIMULATION_CONSTANT_TIME + (1 / 2) * accelY * SIMULATION_CONSTANT_TIME_SQUARED);

	// Compute other car's dynamics
	// Assume no acceleration applied
	int index = 0;
	for (Car c : state.getParticipatingCars()) {
		c.setPosition(c.getPositionX() + c.getVelocityX() * SIMULATION_CONSTANT_TIME, c.getPositionY() + c.getVelocityY() * SIMULATION_CONSTANT_TIME);
		state.updateCar(c, index);
		index++;
	}

	state.setControlledCar(controlledCar);
}

void carAction::setLongitudinalAccelerationValue(double value) {
	accValueLongitudinal = value;
}

void carAction::setLateralAccelerationValue(double value) {
	accValueLateral = value;
}

double carAction::getLongitudinalAccelerationValue() {
	return accValueLongitudinal;
}

double carAction::getLateralAccelerationValue() {
	return accValueLateral;
}


/*IMPLEMENTATIONS FOR THE FUNCTIONS OF THE laneFreeState CLASS IN THE laneFreeState.hpp FILE
	@laneFreeState() -- The constructor of the class.It takes the controlled car as input
	@setAgentState() -- setter for the data regarding the controlled car
	@setControlledCar() -- setter for the controlled car
	@getControlledCar() -- getter for the controlled car
	@getParticipatingCars() -- getter for the other cars
	@addCar(Car c) -- adds a new car in the participating cars vector
	@updateCar(Car c, int index) -- updates a car in the participating cars vector
	@getNumberOfCarsInRoads()  -- getter for the size of the participating cars vector
	@play(int l1, int l2) -- calculates the controlled car's velocity and position,if we apply
							 longitudinal acceleration equal to:  longitudinalAccelerationValues[l1] m/s^2 and
							 lateral acceleration equal to: lateralAccelerationValues[l2] m/s^2.
	@isTerminal()   --  checks if the state is a terminal state
	@isAccidentState--  checks if the state is final and represents an accident involving the controlled car.
*/
laneFreeState::laneFreeState() {
	controlledCar = Car();
	score = 0.0;
	numOfVisits = 0;
}

void laneFreeState::updateScore(float scor) {
	score += scor;
}

void laneFreeState::increaseVisits() {
	numOfVisits += 1;
}

// Add controlled agent's position and other parameters
void laneFreeState::setAgentState(Car controlled) {
	controlledCar = controlled;
}

void laneFreeState::setControlledCar(Car c) {
	controlledCar = c;
}

Car laneFreeState::getControlledCar() const {
	return controlledCar;
}

std::vector<Car> laneFreeState::getParticipatingCars() const {
	return participatingCars;
}

void laneFreeState::addCar(Car c) {
	participatingCars.push_back(c);
}

void laneFreeState::updateCar(Car c, int index) {
	participatingCars[index] = c;
}

int laneFreeState::getNumberOfCarsInRoads() const {
	return static_cast<int>(participatingCars.size());
}

void laneFreeState::play(int l1, int l2) {

	carAction a = carAction(l1, l2);

	// Change controlled car's dynamics
	double accelX = a.getLongitudinalAccelerationValue();
	double accelY = a.getLateralAccelerationValue();
	double oldVelocityX = controlledCar.getVelocityX();
	double oldVelocityY = controlledCar.getVelocityY();
	//double sx = controlledCar.getPositionX();

	controlledCar.setVelocity(oldVelocityX + accelX * SIMULATION_CONSTANT_TIME, oldVelocityY + accelX * SIMULATION_CONSTANT_TIME);
	controlledCar.setPosition(controlledCar.getPositionX() + oldVelocityX * SIMULATION_CONSTANT_TIME + (1 / 2) * accelX * SIMULATION_CONSTANT_TIME_SQUARED, controlledCar.getPositionY() + oldVelocityY * SIMULATION_CONSTANT_TIME + (1 / 2) * accelY * SIMULATION_CONSTANT_TIME_SQUARED);

	// Compute other car's dynamics
	// Assume no acceleration applied
	for (Car c : participatingCars) {
		c.setPosition(c.getPositionX() + c.getVelocityX() * SIMULATION_CONSTANT_TIME, c.getPositionY() + c.getVelocityY() * SIMULATION_CONSTANT_TIME);
	}
}

bool laneFreeState::isTerminal() {
	finalState f = finalState();
	return f.isTerminal(*this);
}

bool laneFreeState::isAccidentState() const {
	finalState f = finalState();
	if (f.isTerminal(*this)) {
		return f.getAccidentFinalState();
	}
	else {
		return false;
	}
}

float laneFreeState::getScore() {
	return score/numOfVisits;
}

int laneFreeState::getNumOfVisits() {
	return numOfVisits;
}

/*IMPLEMENTATIONS FOR THE FUNCTIONS OF THE resultBackpropagation CLASS IN THE laneFreeSimulation.hpp FILE
* @updateScore -- backpropagates the score: backpropScore for the state given.
*
*/
float resultBackPropagation::updateScore(const laneFreeState& state, float backpropScore) {
	return backpropScore;
}


/*IMPLEMENTATIONS FOR THE FUNCTIONS OF THE laneFree_MonteCarloSimulation_strategy CLASS IN THE laneFreeSimulation.hpp FILE
* @laneFree_MonteCarloSimulation_strategy -- The contructor
* @generateRandom(action) -- generates a action for the simulation
*
*/
laneFree_MonteCarloSimulation_strategy::laneFree_MonteCarloSimulation_strategy(laneFreeState* s) :PlayoutStrategy<laneFreeState, carAction>(s) {
	state = s;
}

void laneFree_MonteCarloSimulation_strategy::generateRandom(carAction& action) {
	double desiredSpeed = state->getControlledCar().getDesiredSpeed();

	/***
	* Case of diagonal movements
	*/
	//int x = longitudinalDistribution(state->getControlledCar().getVelocityX(), desiredSpeed).generateNext();
	//int y = lateralDistribution(state->getControlledCar().getVelocityY()).generateNext();
	//action.setLateralAccelerationValue(lateralAccelerationValues[y]);
	//action.setLongitudinalAccelerationValue(longitudinalAccelerationValues[x]);

	/*
	* Case of cross movements
	*/
	int x = 0, y = 0;
	int act = rand() % availableActions;
	if (act < 3) {
		x = act;
	}
	else {
		y = act;
	}
	action.setLateralAccelerationValue(AccelerationValues[y]);
	action.setLongitudinalAccelerationValue(AccelerationValues[x]);
}





/*IMPLEMENTATIONS FOR THE FUNCTIONS OF THE laneFree_TreeExpansionstrategy CLASS IN THE laneFreeSimulation.hpp FILE
* @setInitialValues -- The first action to expand in the expand stage of the MCTS.
* @laneFree_TreeExpansionStrategy -- The contructor
* @score(state) -- generates a action for the simulation
*
*/
void laneFree_TreeExpansionStrategy::setInitialValues(int iLongitude, int iLatitude) {
	longitudinalAccelerationIndex = iLongitude;
	lateralAccelerationIndex = iLatitude;
}

laneFree_TreeExpansionStrategy::laneFree_TreeExpansionStrategy(laneFreeState* s) : ExpansionStrategy<laneFreeState, carAction>(s) {
	setInitialValues(0, 0);
}

carAction laneFree_TreeExpansionStrategy::generateNext() {
	searchNextPossibleMove();
	int oldLongitudinalIndex = longitudinalAccelerationIndex;
	longitudinalAccelerationIndex++;

	//std::cout << "longitudinal acceleration is ... " << oldLongitudinalIndex << std::endl;
	//std::cout << "lateral acceleration is ... " << lateralAccelerationIndex << std::endl;

	return carAction(longitudinalAccelerationValues[oldLongitudinalIndex], lateralAccelerationValues[lateralAccelerationIndex]);
}

bool laneFree_TreeExpansionStrategy::canGenerateNext() const {
	if (longitudinalAccelerationIndex == LONGITUDINAL_ACTIONS && lateralAccelerationIndex == LATERAL_ACTIONS) {
		return false;
	}
	else {
		return true;
	}
}

void laneFree_TreeExpansionStrategy::searchNextPossibleMove() {
	if (longitudinalAccelerationIndex == LONGITUDINAL_ACTIONS) {
		longitudinalAccelerationIndex = 0;
		lateralAccelerationIndex++;
	}
	if (lateralAccelerationIndex == LATERAL_ACTIONS) {
		lateralAccelerationIndex = 0;
	}
}




/*IMPLEMENTATIONS FOR THE FUNCTIONS OF THE simulationResult CLASS IN THE laneFreeSimulation.hpp FILE
* score(state) -- the score of a given state.
*
*/
float simulationResult::score(const laneFreeState& state) {

	Car c = state.getControlledCar();

	//std::cout << "This is NOT an accidental state , Velocity=" << c.getVelocityX() << " , desired speed=" << c.getDesiredSpeed() << std::endl;

	float differenceFromDesiredSpeed = abs(c.getDesiredSpeed() - c.getVelocityX());

	return 10 / differenceFromDesiredSpeed;

}



/*IMPLEMENTATIONS FOR THE FUNCTIONS OF THE FinalState CLASS IN THE laneFreeState.hpp FILE
	@finalState() -- The constructor
	@isTerminal(state) -- checks if the given state is a terminal state
	@getAccidentFinalState() -- getter for accidental state or not
	@setAccidentFinalState(bool acc) -- setter for accidental state or not
*/

bool finalState::isTerminal(const laneFreeState& state) {
	Car c = state.getControlledCar();

	double minDistanceX, minDistanceY, diffInX, diffInY;
	// Check if we have a crash between the controlled car and the other cars in the road.
	const std::vector<Car> carsInTheRoad = state.getParticipatingCars();

	for (Car c2 : carsInTheRoad) {

		minDistanceY = ((c.getWidth() / 2) + (c2.getWidth() / 2)) + SAFETY_GAP/2;
		minDistanceX = ((c.getLength() / 2) + (c2.getLength() / 2)) + SAFETY_GAP;

		diffInX = c.getPositionX() > c2.getPositionX() ? c.getPositionX() - c2.getPositionX() : c2.getPositionX() - c.getPositionX();
		diffInY = c.getPositionY() > c2.getPositionY() ? c.getPositionY() - c2.getPositionY() : c2.getPositionY() - c.getPositionY();

		if (diffInX < minDistanceX && diffInY < minDistanceY) {
			accident = true;
			return true;
		}

	}
	//std::cout << " Cars DON'T crashed " << c.getPositionY()<< "  \n";

	//Check if the controlled car gets out of the road
	if (c.getPositionY() > ROAD_WIDTH - ((state.getControlledCar().getWidth()) / 2) || c.getPositionY() < (state.getControlledCar().getWidth()) / 2) {
		accident = true;
		return true;
	}
	//std::cout << "Car inside road " << std::endl;

	// Check if the controlled car has exited
	if (c.getPositionX() > 2000) {
		accident = false;
		return true;
	}

	//std::cout << "Car exited " << std::endl;
	//std::cout << "Not acciddent, nor false " << "\n";
	accident = false;
	return false;
}




/* IMPLEMENTATIONS FOR THE FUNCTIONS OF THE FactoredValueMCTS.hpp HEADER FILE.
* @ main - The initial function that executes the entire algorithm.
* @ updateGraphStats - Update the values in the edges and the nodes of the coordination graph, based on the results
*		of the simulations.
*
	*/
void factoredValueMCTS::main(laneFreeGlobalState s) {
	int simulations = SIMULATIONS_FROM_ROOT;
	MaxPlus graphSolver = MaxPlus(s.getNumberOfCarsInRoads());
	graphSolver.createGraph(s);

	while (simulations > 0) {
		simulate(s, graphSolver, 0);//initial depth=0
		simulations--;
	}

	// Run maxplus with no exploration to get the final coordinated action
	graphSolver.updateNodeExplorationFlag(1);
	graphSolver.setC(0);
	//graphSolver.printGraph();
	graphSolver.maxplus();

}

/* Updates the statistics of the graph, every time after a 'simulate' iteration
*
* @Params:
* State s, Action a,
*
* return: Void
*/

void factoredValueMCTS::updateGraphStats(laneFreeGlobalState s, MaxPlus step) {

	for (int i = 0; i < maxPlusGraph.size(); i++) {
		node nod = maxPlusGraph[i];
		//std::cout << "nod.getN()=" << nod.getN() << std::endl;
		//std::cout << "nod.getNi()[actionIndex] = " << nod.getNi()[actionIndex] << std::endl;
		//Update node statistics
		nod.setN(nod.getN() + 1);
		
		int actionIndex = nod.getTemporaryBestActionIndex();
		nod.setNi(actionIndex, nod.getNi()[actionIndex] + 1); // Ni=Ni+1
		nod.setQ(actionIndex, nod.getQ()[actionIndex] + (nod.getBestActionValue(actionIndex) - nod.getQ()[actionIndex]) / nod.getNi()[actionIndex]); // Q' = Q'+(qi-Q')/Ni

		//Update edge statistics
		std::vector<edge*> tmpAdjacencyList = nod.getAdjacencyList();
		for (int j = 0; j < tmpAdjacencyList.size(); j++) {
			edge* edg = tmpAdjacencyList[j];

			float qe = edg->getBestActionValueI() + edg->getBestActionValueJ(); //qe = qi+qj
			float QIJ = edg->getQij()[edg->getBestActionIndexI()][edg->getBestActionIndexJ()];

			edg->setNij(edg->getBestActionIndexI(), edg->getBestActionIndexJ(),
				edg->getNij()[edg->getBestActionIndexI()][edg->getBestActionIndexJ()] + 1);
			edg->setQij(edg->getBestActionIndexI(), edg->getBestActionIndexJ(),
				QIJ + ((qe - QIJ) / edg->getNij()[edg->getBestActionIndexI()][edg->getBestActionIndexJ()]));

			tmpAdjacencyList[j] = edg;
		}
		nod.setAdjacencyList(tmpAdjacencyList);
		maxPlusGraph[i] = nod;
	}
}


/* Given a global state s and a coordinated action a', computes and returns the new state s'
*
*/
laneFreeGlobalState factoredValueMCTS::generateNewState(laneFreeGlobalState s) {

	for (int i = 0; i < s.getNumberOfCarsInRoads(); i++) {
		Car c = s.getCar(i);

		//Looking for the action taken from car c
		int j = 0;
		int actionIndex = 0;
		node carAct = maxPlusGraph[j];
		while (carAct.getCar().getCarNumber() != c.getCarNumber()) {
			// The actions was not found in the list 'a', something went terribly wrong
			if (j == maxPlusGraph.size()) {
				std::cout << "FATAL ERROR: Car action for Car " << c.getCarNumber() << " not found " << std::endl;
				break;
			}
			carAct = maxPlusGraph[j + 1];
			actionIndex = carAct.getTemporaryBestActionIndex();
			j++;
		}

		/* E.g. action index=15, means that if we have 7 actions for longitudinal acceleration,
		* that we pick the action (2--lateral ,1--longitudinal ) (15 mod 7)
		*/
		std::div_t action = std::div(actionIndex, longitudinalAccelerationValues.size());

		c = applyDynamics(c, action.rem, action.quot);
		s.updateCar(c, i);

	}

	return s;
}


/* Computes the reward the entire system has, from moving from state s to the state s',
*  by taking the action a'
*/
void factoredValueMCTS::computeFactoredImediateReward(laneFreeGlobalState s, laneFreeGlobalState s_star, float gamma) {

	for (int i = 0; i < maxPlusGraph.size(); i++) {
		node r = maxPlusGraph[i];

		// Find the car in the state s and in the state s_star
		Car car_s = Car();
		Car car_s_star = Car();
		for (int j = 0; j < s.getNumberOfCarsInRoads(); j++) {
			if (s.getCar(j).getCarNumber() == r.getCar().getCarNumber()) {
				car_s = s.getCar(j);
			}
		}
		for (int j = 0; j < s_star.getNumberOfCarsInRoads(); j++) {
			if (s_star.getCar(j).getCarNumber() == r.getCar().getCarNumber()) {
				car_s_star = s_star.getCar(j);
			}
		}

		// Get nearby cars in state s_star
		std::vector<Car> adjecencyList;
		for (edge* e : r.getAdjacencyList()) {
			if (e->getCarNumberI() == r.getCar().getCarNumber()) {
				adjecencyList.push_back(s_star.getCarByNumber(e->getCarNumberJ()));
			}
			else {
				adjecencyList.push_back(s_star.getCarByNumber(e->getCarNumberI()));
			}
		}


		// Compute the reward of that car
		int actionIndex = r.getTemporaryBestActionIndex();
		float reward = imediateReward(car_s, car_s_star, adjecencyList);


		r.setActionValue(actionIndex, reward + gamma * r.getTemporaryBestActionValue());//q=r+g*SIMULATE

		maxPlusGraph[i] = r;
	}

}


/*One round of MCTS simulation,for the case of global state.
*
*/
int factoredValueMCTS::simulate(laneFreeGlobalState s, MaxPlus step, int depth) {
	if (depth == MAX_FVMCTS_DEPTH) {
		return 0;
	}

	step.maxplus(); //Apply max plus algorithm to the graph.

	laneFreeGlobalState s_star = generateNewState(s);
	
	simulate(s_star, step, depth + 1);

	computeFactoredImediateReward(s, s_star, pow(FVMCTS_GAMMA, depth));

	updateGraphStats(s, step);

}


bool finalState::getAccidentFinalState() {
	return accident;
}

void finalState::setAccidentFinalState(bool acc) {
	accident = acc;
}
#endif