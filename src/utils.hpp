#ifndef UTILITIES
#define UTILITIES

#include <random>
#include <iostream>
#include <chrono>
#include <fstream>
#include <cstdlib>
#include <map>
#include <tuple>


#define SIMULATION_CONSTANT_TIME 0.25
#define SIMULATION_CONSTANT_TIME_SQUARED 0.04
#define MAX_ACCELERATION 4
#define LONGITUDINAL_ACTIONS 5
#define LATERAL_ACTIONS 3
#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define ROAD_WIDTH 10.2
#define ROAD_SAFETY_GAP 0.25 
#define GAMMA 0.6
#define epsilon 1
#define SUMO_CAR_LENGTH 2

/*Factored value MCTS hyperparameters*/
#define MAX_FVMCTS_DEPTH 3
#define FVMCTS_GAMMA 0.95
#define SIMULATIONS_FROM_ROOT 12
#define DISTANCE_FOR_CREATING_EDGE 35
#define EXPLORATION_TERM 0.5
#define ALPHA 10
#define BETA 50


/*Max Plus hyperparameters */
#define maxPlusMessagePassingPerRound 2
#define maxPlusMCSimulationRounds 2


/*Potential field parameters*/
/*Parameters referring to the potential fields*/
#define MAX_DECEL -2
#define MAX_ACCEL 2
#define MAX_LAT_ACCEL 1.5

#define TREE_DEPTH 2

#define ACTUAL_MAX_ACCEL 3
#define ACTUAL_MAX_DECEL -3

#define ACTUAL_MAX_LAT_ACCEL 1

#define MAX_AGENTS_UPSTREAM 3
#define MAX_AGENTS_DOWNSTREAM 3


#define SAFETY_DIST_LONG_A 0.8
#define SAFETY_DIST_LAT_B 0.8
#define REACTION_TIME 0.3
#define REACTION_TIME_LAT 0.45
#define ELLIPSE_POW_X 2
#define ELLIPSE_POW_Y 4
#define ELLIPSE_POW_T 2

#define MAX_VELOCITY_LONG 40
#define MAX_VELOCITY_LAT 5

#define ELLIPSE_MAGNITUDE 15

#define TIMESTEP_LENGTH 0.25 //TODO, receive this value from SUMO

#define MONITOR_AGENT_1_ID 71
#define MONITOR_AGENT_2_ID 74


#define ROAD_WIDTH 10.2 // TODO receive this value from the API, also it will not be constant when working with multiple road segments

#define MAXIMUM(a, b) (((a) > (b))?(a):(b))
#define MINIMUM(a, b) (((a) <= (b))?(a):(b))

#define WALL_DN(point, safety, v, y, w, T, uymax_hard) U_lemma33(T, -(safety)*v, MAXIMUM(0, y-0.5*w - (point)), -uymax_hard)
#define WALL_UP(point, safety, v, y, w, T, uymax_hard) U_lemma33(T, +(safety)*v, MAXIMUM(0, (point) - (y+0.5*w)), -uymax_hard)

#define SAFETY_GAP 1


const std::vector<double> longitudinalAccelerationValues{ -5,-2,0,2,5 };
const std::vector<double> lateralAccelerationValues{ 0,-1,1 };
// For the case of cross movements
const std::vector<double> AccelerationValues{ 0,-2,2,-1,1 };

const int availableActions = 5;



class Car {
private:
	double position_x, position_y;
	double velocity_x, velocity_y;
	double length, width;
	double desired_speed;
	int carNumber;

public:
	Car() = default;

	Car(int no) {
		carNumber = no;
	}

	void setPosition(double x, double y) {
		position_x = x;
		position_y = y;
	}
	void setVelocity(double vx, double vy) {
		velocity_x = vx;
		velocity_y = vy;
	}

	void setDimensions(double l, double w) {
		length = l;
		width = w;
	}
	void setDesiredSpeed(double s) {
		desired_speed = s;
	}

	double getPositionX() {
		return position_x;
	}
	double getPositionY() {
		return position_y;
	}
	double getVelocityX() {
		return velocity_x;
	}
	double getVelocityY() {
		return velocity_y;
	}

	double getLength() {
		return length;
	}
	double getWidth() {
		return width;
	}
	double getDesiredSpeed() {
		return desired_speed;
	}

	int getCarNumber() {
		return carNumber;
	}

	// Check if the cars has a collision with another given car
	bool isCrashed(Car c);

	void print(std::ostream& stream) {
		stream << "Position:(" << position_x << "," << position_y << ")\nAcceleration:(" << velocity_x << "," << velocity_y << ")\n";
	}
};

// Class for generating random actions for monte carlo simulations , based on a distribution
class lateralDistribution {
private:
	double velocity;
public:
	lateralDistribution(double velocity) {
		this->velocity = velocity;
	}

	int generateNext() {
		return rand()%LATERAL_ACTIONS;
	}
};

class longitudinalDistribution {
private:
	double velocity;
	double desiredSpeed;
public:
	longitudinalDistribution(double velocity, double desiredSpeed) {
		this->velocity = velocity;
		this->desiredSpeed = desiredSpeed;
	}

	int generateNext(){
		return rand()%LONGITUDINAL_ACTIONS;
	}
};


class timestampStatistics {
public:
	double timestamp;
	int cars;
	int collisions;
	int carsOutOfBounds;
	double sumOfDifferencesFromDesiredSpeed;

	/*Case of car exampel trajectories*/
	double car1speedx;
	double car1speedy;
	double car1posx, car1posy;
	double car2speedx;
	double car2speedy;
	double car2posx, car2posy;

	double car3speedx;
	double car3speedy;
	double car3posx, car3posy;

	double car4speedx;
	double car4speedy;
	double car4posx, car4posy;


	timestampStatistics(double timestamp, int cars, int collisions, int carsOutOfBounds, double sumOfDifferencesFromDesiredSpeed,
		double c1x,double c1y,double c1xpos,double c1ypos,
		double c2x,double c2y,double c2xpos,double c2ypos,
		double c3x,double c3y,double c3xpos,double c3ypos,
		double c4x,double c4y,double c4xpos,double c4ypos) {
		this->timestamp = timestamp;
		this->cars = cars;
		this->collisions = collisions;
		this->carsOutOfBounds = carsOutOfBounds;
		this->sumOfDifferencesFromDesiredSpeed = sumOfDifferencesFromDesiredSpeed;

		this->car1speedx = c1x;
		this->car1speedy = c1y;
		this->car1posx = c1xpos;
		this->car1posy = c1ypos;

		this->car2speedx = c2x;
		this->car2speedy = c2y;
		this->car2posx = c2xpos;
		this->car2posy = c2ypos;

		this->car3speedx = c3x;
		this->car3speedy = c3y;
		this->car3posx = c3xpos;
		this->car3posy = c3ypos;

		this->car4speedx = c4x;
		this->car4speedy = c4y;
		this->car4posx = c4xpos;
		this->car4posy = c4ypos;
	}

	double getAverageDifferenceFromDesiredSpeed() {
		return (sumOfDifferencesFromDesiredSpeed / cars);
	}
};


class edge {
private:
	int carINumber;
	int carJNumber;

	/*Vector representing the joint temporary utility of all the actions of the agent i,for all available actions of the agent j.
	It is different than mji. It is used in the MaxPlus algorithm.
	*
	* First index is for the car with the car1Number number.
	* Second index for the car with the car2Number number.
	*/
	std::vector<std::vector<float>> mij;

	/*Vector representing the joint temporary utility of all the actions of the agent j,for all available actions of the agent i.
	It is different than mij. It is used in the MaxPlus algorithm.
	*
	* First index is for the car with the car1Number number.
	* Second index for the car with the car2Number number.
	*/
	std::vector<std::vector<float>> mji;


	/*Vector representing all possible couples of actions between the two agents and their value
	*
	* First index is for the car with the car1Number number.
	* Second index for the car with the car2Number number.
	*/
	std::vector<std::vector<float>> Qij;

	/*Times each combination of actions has been selected
	* Indexing is similar to Qij.
	*/
	std::vector<std::vector<int>> Nij;

	/* Variable storing the current best actions for the agents pointing to this edge.
	* The purpose for dong so, is to avoid looping in the coordinated graph to take the aj when
	* the algorithm is in the message passing phase.
	*/
	float bestActionValueI;
	float bestActionValueJ;
	int bestActionIndexI;
	int bestActionIndexJ;



public:
	edge(int carINumber, int carJNumber) {
		this->carINumber = carINumber;
		this->carJNumber = carJNumber;

		// Calculate all available actions aj for agent j.
		this->mij = std::vector<std::vector<float>>(availableActions, std::vector<float>(availableActions, 0.0));
		this->mji = std::vector<std::vector<float>>(availableActions, std::vector<float>(availableActions, 0));
		this->Qij = std::vector<std::vector<float>>(availableActions, std::vector<float>(availableActions, 0));
		this->Nij = std::vector<std::vector<int>>(availableActions, std::vector<int>(availableActions, 0));


	}

	std::vector<std::vector<float>> getMij() {
		return this->mij;
	}

	void setMij(int indexi, int indexj, float value) {
		this->mij[indexi][indexj] = value;
	}

	std::vector<std::vector<float>> getMji() {
		return this->mji;
	}

	void setMji(int indexi, int indexj, float value) {
		//std::cout << "SETTING mji for " << indexi << ", " << indexj << "  is " << value << std::endl;
		this->mji[indexi][indexj] = value;
	}



	void initialiseMij() {
		for (int i = 0; i < availableActions; i++) {
			for (int j = 0; j < availableActions; j++) {
				this->mij[i][j] = 0;
			}
		}
	}

	void initialiseMji() {
		for (int i = 0; i < availableActions; i++) {
			for (int j = 0; j < availableActions; j++) {
				mji[i][j] = 0;
			}
		}
	}



	std::vector<std::vector<float>> getQij() {
		return Qij;
	}

	void setQij(int indexI, int indexJ, float value) {
		Qij[indexI][indexJ] = value;
	}

	void setNij(int indexI, int indexJ, int value) {
		Nij[indexI][indexJ] = value;
	}

	std::vector<std::vector<int>> getNij() {
		return Nij;
	}

	void setCarNumberI(int carINumber) {
		this->carINumber = carINumber;
	}

	int getCarNumberI() {
		return carINumber;
	}

	void setCarNumberJ(int carJNumber) {
		this->carJNumber = carJNumber;
	}

	int getCarNumberJ() {
		return carJNumber;
	}

	void setBestActionValueI(float value) {
		this->bestActionValueI = value;
	}

	float getBestActionValueI() {
		return bestActionValueI;
	}

	void setBestActionValueJ(float value) {
		this->bestActionValueJ = value;
	}

	float getBestActionValueJ() {
		return bestActionValueJ;
	}



	void setBestActionIndexI(int index) {
		this->bestActionIndexI = index;
	}

	float getBestActionIndexI() {
		return bestActionIndexI;
	}
	void setBestActionIndexJ(int index) {
		this->bestActionIndexJ = index;
	}

	float getBestActionIndexJ() {
		return bestActionIndexJ;
	}

	float getBestM_ifIgetsActionJ(int index) {
		float max = -1000;
		for (std::vector<float> row : mij) {
			if (row[index] > max) {
				max = row[index];
			}
		}
		return max;
	}

	float getBestM_ifJgetsActionI(int index) {
		float max = -1000;
		for (std::vector<float> row : mji) {
			//std::cout << "For action " << index << " best responce: " << row[index] << std::endl;
			if (row[index] > max) {
				max = row[index];
			}
		}
		return max;
	}

	float getMijRowSum(int indexRow) {
		float totalSum = 0;
		for (int i = 0; i < availableActions; i++) {
			totalSum = totalSum + mij[indexRow][i];
		}
		return totalSum;
	}

	float getMjiRowSum(int indexRow) {
		float totalSum = 0;
		for (int i = 0; i < availableActions; i++) {
			totalSum = totalSum + mji[indexRow][i];
		}
		return totalSum;
	}
};


class node {
private:
	Car car;

	int temporaryBestActionIndex;// The best action for an agent at a given moment im maxPlus
	float temporaryBestActionValue; // The score q,of the action above

	int bestAction; // The best action of an agent after the simulation phase
	std::vector<float> actionValues; // The best action value of an agent after the simulation phase


	int N;//The number N, of all actions this node has taken
	std::vector<float> Q; // The quality of each possible move
	std::vector<int> Ni; // The times each possible move has been selected 
	std::vector<edge*> adjacencyList; // Edges of the graph

public:
	node(Car car) {
		this->car = car;
		this->N = 0;
		this->Q = std::vector<float>(availableActions, 1);//Initialize quality of move
		this->Ni = std::vector<int>(availableActions, 1);
		this->temporaryBestActionIndex = 2;//zero acceleration
		this->temporaryBestActionValue = 0;
		this->actionValues = std::vector<float>(availableActions, 0.0);
		this->bestAction = 2;//zero acceleration
	}

	void setAdjacencyList(std::vector<edge*> list) {
		this->adjacencyList = list;
	}

	void addEdge(edge* e) {
		this->adjacencyList.push_back(e);
	}

	int getCarNo() {
		return car.getCarNumber();
	}

	Car getCar() {
		return car;
	}

	std::vector<float> getQ() {
		return Q;
	}

	void setQ(int index, float q) {
		this->Q[index] = q;
	}

	std::vector<int> getNi() {
		return Ni;
	}

	void setNi(int index, int N) {
		this->Ni[index] = N;
	}

	std::vector<edge*> getAdjacencyList() {
		return adjacencyList;
	}

	void setTemporaryBestActionIndex(int actionIndex) {
		temporaryBestActionIndex = actionIndex;
	}

	int getTemporaryBestActionIndex() {
		return temporaryBestActionIndex;
	}

	void setTemporaryBestActionValue(float value) {
		temporaryBestActionValue = value;
	}

	float getTemporaryBestActionValue() {
		return temporaryBestActionValue;
	}


	void setBestActionIndex(int actionIndex) {
		bestAction = actionIndex;
	}

	int getBestActionIndex() {
		return bestAction;
	}

	void setActionValue(int actionIndex, float value) {
		actionValues[actionIndex] = value;
	}

	std::vector<float> getBestActionValues() {
		return actionValues;
	}

	float getBestActionValue(int index) {
		return actionValues[index];
	}


	void setN(int n) {
		this->N = n;
	}

	int getN() {
		return N;
	}

	int getBestAction() {
		int action = 0;
		float actionValue = 0;
		//std::cout << "///NEW CAR////" << std::endl;
		for (int f = 0; f < actionValues.size(); f++) {
			//std::cout << "Action " << f << " , value: " << actionValues[f] << std::endl;

			if (actionValues[f] > actionValue) {
				actionValue = actionValues[f];
				action = f;
			}
		}
		return action;
	}

	void setEdge(int index, edge* e) {
		this->adjacencyList[index] = e;
	}

};

/*Class representing statistics for individual node*/
class treeNodeStats {
private:
	unsigned int id;
	int numVisits = 0;
	float scoreSum;
public:
	treeNodeStats(unsigned int id) {
		this->numVisits = 0;
		this->scoreSum = 0.0F;
		this->id = id;
	}

	int getID() {
		return id;
	}

	float getAvgScore() {
		return scoreSum / numVisits;
	}

	void update(float score) {
		this->numVisits++;
		this->scoreSum += score;
	}
};


double differenceFromDesiredSpeed(double desiredSpeed, double actualSpeed);

/*Function that accumulates the values of a float vector
*
*/
float vectorSumFloat(std::vector<float> v);

/*Function that accumulates the values of an integer vector
*
*/
int vectorSumInt(std::vector<std::vector<int>> v);

int vectorSum(std::vector<int> v);

Car applyDynamics(Car c, int indexActionX, int indexActionY);


float imediateReward(Car c, Car c_star, std::vector<Car> adjacencyList);

float betweenCarsReward(Car c, Car c1);


node getNodeByCarNumber(int carNumber);

extern std::vector<node> maxPlusGraph;

void custom_regulate_forces(double* fx, double* fy, double y, double vx, double vy, double w, double T, double roadwid_meters, double uymax_hard);

double custom_pairwise_factor_function(double x1, double y1, double vx1, double vy1, double x2, double y2,
	double vx2, double vy2, double axx1, double ayy1, double axx2, double ayy2);



void initializeScores();
void insertNewNodeScore(unsigned int id);
int getNumNodeVisits(unsigned int id);
float getAvgNodeScore(unsigned int id);
void updateNodeScore(unsigned int id, float score);


/*
Init cars functions
*/
void initializeSimulationOne();
void initializeSimulationTwo();
void initializeSimulationThree();
void initializeSimulationFour();


#endif