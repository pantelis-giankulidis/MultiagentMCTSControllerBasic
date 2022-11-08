#ifndef STATESS
#define STATESS
#include "MCTS.hpp"
#include "utils.hpp"

/*
Represents the state of the simulation,as it appears for a particular car.(A car's world)
*/
class laneFreeState : public State {
private:
	/*The controlled car.The state is the world as this car "sees" it */
	Car controlledCar;

	/*Instances of the cars that are near the controlled car and affect its state and actions*/
	std::vector<Car> participatingCars;

	float score;
	int numOfVisits;

public:

	laneFreeState();

	void updateScore(float scor);

	void increaseVisits();


	// Add controlled agent's instance
	void setAgentState(Car controlled);

	Car getControlledCar() const;

	void setControlledCar(Car c);

	// get the vector of instances for all the other cars that the controlled car can see
	std::vector<Car> getParticipatingCars() const;

	// Add a new car to the state
	void addCar(Car c);

	// update a car's state
	void updateCar(Car c, int index);

	int getNumberOfCarsInRoads() const;

	// Apply current action to a given state of the road
	void play(int l1, int l2);

	void print(std::ostream& stream) override {
		stream << "Car's position: " << controlledCar.getPositionX() << "," << controlledCar.getPositionY() << "\n" << "Cars in the road " << getNumberOfCarsInRoads() << "\n";
	}

	// Check if the state is a terminal state
	bool isTerminal();

	// check if the state has an accident occured
	bool isAccidentState() const;

	float getScore();

	int getNumOfVisits();
};

/*
Represents the final state of the simulation,for a car.A final state is when a car exits the road, collides with another car or finishes
*/
class finalState : public TerminationCheck<laneFreeState> {
private:
	/*Indicates whether a final state represents an accident or not*/
	bool accident;
public:
	finalState() = default;

	/*Checks if the given state is final state.If it is, it also determines if it represents an accident.
	*
	* state: The state to check
	* return: true or false,whether the state given is final.
	*/
	bool isTerminal(const laneFreeState& state) override;

	/*Checks if the final state is final because of an accident*/
	bool getAccidentFinalState();

	void setAccidentFinalState(bool acc);
};


/* Represents the global state of the roadmap,for the multi-agent coordination algorithm*/
class laneFreeGlobalState :public State {
private:
	std::vector<Car> participatingCars;

public:
	// Add a new car to the state
	void addCar(Car c);

	// update a car's state
	void updateCar(Car c, int index);

	int getNumberOfCarsInRoads() const;

	// Apply coordination of actions to the state
	void play(std::vector<std::pair<int, int>> coordActions);

	// Check if the state is a terminal state
	bool isTerminal();

	// check if the state has an accident occured
	bool isAccidentState() const;

	// get a specific car from the state
	Car getCar(int index);

	// get a car based on its car number
	Car getCarByNumber(int carNumber);


	// Get all cars of the state
	std::vector<Car> getCars();
};
#endif