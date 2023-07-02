#ifdef CONTROLLER_H
#define EXTERN_C /* nothing */
#else
#define EXTERN_C extern
#endif /* DEFINE_VARIABLES */

#ifndef CONTROL
#define CONTROL
#include "laneFreeSimulation.hpp"
#include "MCTS.hpp"
#include "laneFreeState.hpp"

using laneFreeMCTS = MCTS<laneFreeState, carAction, laneFree_TreeExpansionStrategy, laneFree_MonteCarloSimulation_strategy>;
class MCTSInstance {
public:
	static carAction calculateAction(const laneFreeState& state);

private:
	static laneFreeMCTS createMCTS(const laneFreeState& state);
};
#endif


