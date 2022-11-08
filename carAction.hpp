#ifndef CAR_ACTION
#define CAR_ACTION
#include "laneFreeState.hpp"

class carAction : public Action<laneFreeState> {
private:
	double accValueLongitudinal;
	double accValueLateral;
public:
	carAction() = default;
	carAction(double valueLong, double valueLat);

	void execute(laneFreeState& state) override;

	void setLongitudinalAccelerationValue(double value); 

	void setLateralAccelerationValue(double value);

	double getLongitudinalAccelerationValue();

	double getLateralAccelerationValue();

	void print(std::ostream& stream) override {
		stream << "Action is: Longitude =" << accValueLongitudinal << "m/s^2   ***   Latitude=" << accValueLateral << std::endl;
	}
};

class coordinatedAction {
private:
	std::vector<carAction> actions;
public:
	coordinatedAction();

	void addAction(carAction a) {
		actions.push_back(a);
	}

	std::vector<carAction> getActionPairs(int i, int j) {
		std::vector<carAction> coord;
		coord.push_back(actions[i]);
		coord.push_back(actions[j]);
		return coord;
	}
};

#endif