#ifndef GLOBALSTATE
#define GLOBALSTATE
#include "utils.hpp"
#include "laneFreeState.hpp"

/*Implementation of the functions in the global state class*/
void laneFreeGlobalState::addCar(Car c) {
	participatingCars.push_back(c);
}

void laneFreeGlobalState::updateCar(Car c, int index) {
	participatingCars[index] = c;
}

int laneFreeGlobalState::getNumberOfCarsInRoads() const {
	return static_cast<int>(participatingCars.size());
}

Car laneFreeGlobalState::getCar(int index) {
	return participatingCars[index];
}

Car laneFreeGlobalState::getCarByNumber(int carNumber) {
	for (Car c : participatingCars) {
		if (c.getCarNumber() == carNumber) {
			return c;
		}
	}
}

void laneFreeGlobalState::play(std::vector<std::pair<int, int>> coordActions) {
	/* TODO */
}

bool laneFreeGlobalState::isTerminal() {
	/* TODO */
	return true;
}

bool laneFreeGlobalState::isAccidentState() const {
	/* TODO */
	return true;
}

std::vector<Car> laneFreeGlobalState::getCars() {
	return participatingCars;
}


#endif