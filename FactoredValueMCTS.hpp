#ifndef FACTORED
#define FACTORED

#include "carAction.hpp"
#include "MaxPlus.cpp"

class factoredValueMCTS {
public:
	/*The starting point for the entire FV-MCTS algorithm*/
	void main(laneFreeGlobalState s);

	/* A simulation step for the algorithm. */
	int simulate(laneFreeGlobalState s, MaxPlus step, int depth);

	/*Update node and edge statistics after one simulation*/
	void updateGraphStats(laneFreeGlobalState s, MaxPlus step);

	/* Generate new global state based on current state and the coordinated action */
	laneFreeGlobalState generateNewState(laneFreeGlobalState s);

	/* Compute the reward each car takes when the system goes from state s to the state s' */
	void computeFactoredImediateReward(laneFreeGlobalState s, laneFreeGlobalState s_star, float gamma);

};

#endif