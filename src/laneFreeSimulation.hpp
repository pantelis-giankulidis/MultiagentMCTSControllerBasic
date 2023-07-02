#ifndef SIMULATION
#define SIMULATION
//#include "utils.hpp"
#include "carAction.hpp"


/*------------------------------------------------------------------------------------------------------*/
/*--------------- Lane free simulation strategy class --------------------------------------------------*/
class laneFree_MonteCarloSimulation_strategy : public PlayoutStrategy<laneFreeState, carAction> {
private:
	laneFreeState* state;
public:
	explicit laneFree_MonteCarloSimulation_strategy(laneFreeState* s);

	void generateRandom(carAction& action) override;

};

/*-----------------------------------------------------------------------------------------------------*/
/*-------------------- Lane free tree expansion strategy ----------------------------------------------*/
class laneFree_TreeExpansionStrategy : ExpansionStrategy<laneFreeState, carAction> {
private:
	int longitudinalAccelerationIndex;
	int lateralAccelerationIndex;

public:
	explicit laneFree_TreeExpansionStrategy(laneFreeState* s);

	void setInitialValues(int iLongitude, int iLatitude);

	laneFree_TreeExpansionStrategy() = default;

	carAction generateNext() override;

	bool canGenerateNext() const override;

	void searchNextPossibleMove();
};

/*---------------------------------------------------------------------------------------*/
/*---------- Result backpropagation class -----------------------------------------------*/
class resultBackPropagation : public Backpropagation<laneFreeState> {
public:
	resultBackPropagation() = default;

	float updateScore(const laneFreeState& state, float backpropScore) override;
};


/*----------------------------------------------------------------------------------------*/
/*------------ Simulation result class --------------------------------------------------*/
class simulationResult : public Scoring<laneFreeState> {
public:
	~simulationResult() = default;

	float score(const laneFreeState& state) override;
};

#endif
