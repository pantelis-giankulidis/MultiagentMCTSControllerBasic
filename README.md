# Multiagent MCTS Controller Basic Solution
Lane-Free traffic is a novel paradigm that lifts the notion of lanes in traffic en-
vironments populated (as a first step, only) with autonomous vehicles, resulting in
much higher efficiency since the road capacity is better exploited. Despite its novelty,
a significant amount of research on lane-free autonomous driving has been already
performed. However, well-known Artificial Intelligence (AI) intelligent search and
decision planning algorithms have not been yet explored in this setting.

To this end, we expand upon the research in lane-free vehicle movement strate-
gies by introducing a different approach to the problem. Monte Carlo Tree Search
(MCTS), a popular search algorithm for decision planning in games, is adopted for
the problem at hand. We introduce a formulation for the task of lane-free driving
using this algorithm and examine its efficiency under two different settings, which
differ with respect to the existence of communication among vehicles, and the very
constituents of the basic MCTS algorithm.
In the first setting, each vehicle acts independently from the others according on
the formulation of the lane-free environment that is suitable for the MCTS algorithm.
The formulation we introduce addresses the two objectives of the vehicles, namely
collision avoidance and reaching or preserving a desired speed of choice.
While this approach gives satisfactory results, it does not take into consideration
online interactions among vehicles. For every vehicle, other vehicles are observed
as moving obstacles with constant speed. If we consider communication among
vehicles as well, we can additionally model these interactions and examine their
influence in their decision making.
As such, in the second setting we address the multiagent nature of the lane-
free environment. For that, we adopt a recently introduced multiagent planning
algorithm based on MCTS and Coordination Graphs. Essentially, vehicles exchange
messages that affect their planning, consequently resulting in a coordinated decision
making process. Then, we provide an extensive set of experiments in order to eval-
uate our proposed approach under these two settings. Our experimental procedure
correspondingly involves two distinct phases. First, the single-agent performance is
investigated in scenarios populated with a large number of vehicles in a highway,
all employing the MCTS algorithm independently. We observe the performance by
evaluating collision occurrences and deviation from the desired speed in demanding
scenarios that exceed the capacity of equivalent lane-based environments. Finally,
we evaluate the multiagent algorithm in three lane-free scenarios that involve a few
vehicles, and showcase its increased coordination capabilities compared to the plain
MCTS algorithm at the expense of computational time.

*** The official submitted and successfully defentend script can be found at <a href='https://drive.google.com/drive/folders/1K-viCYhWWIa8G9jzv9-jOVf5STpybHr3'>THIS LINK</a> ***
