#ifndef UTILSCPP
#define UTILSCPP
#include "utils.hpp"
#include <tuple>

std::vector<node> maxPlusGraph{};
std::vector<std::tuple<int, float>> nodescores;
std::ofstream nodestats;
std::ifstream nodestatsread;

void initializeScores() {
	nodescores = std::vector<std::tuple<int, float>>(1000);
	for (int i = 0; i < 1000; i++) {
		nodescores[i] = std::make_tuple<int, float>(0, 0.0F);
	}
}

void insertNewNodeScore(unsigned int id) {
	std::get<0>(nodescores[id]) = 0;
	std::get<1>(nodescores[id]) = 0.0F;
}

int getNumNodeVisits(unsigned int id) {
	return std::get<0>(nodescores[id]);
}

float getAvgNodeScore(unsigned int id) {
	return std::get<1>(nodescores[id]) / std::get<0>(nodescores[id]);
}

void updateNodeScore(unsigned int id, float score) {
	std::get<0>(nodescores[id]) = std::get<0>(nodescores[id]) + 1;
	std::get<1>(nodescores[id]) = std::get<1>(nodescores[id]) + score;

}

Car applyDynamics(Car c, int indexActionX, int indexActionY) {
	int accelX = 0,accelY=0;
	
	/**
	* Case of diagonal movements
	*/
	//accelX = longitudinalAccelerationValues[indexActionX];
	//accelY = lateralAccelerationValues[indexActionY];

	/**
	* Case of non-diagonal movements
	*/
	if (indexActionX < 3) {
		accelX = longitudinalAccelerationValues[indexActionX];
	}
	else {
		accelY = longitudinalAccelerationValues[indexActionX];
	}

	double oldVelocityX = c.getVelocityX();
	double oldVelocityY = c.getVelocityY();

	c.setVelocity(oldVelocityX + accelX * SIMULATION_CONSTANT_TIME, oldVelocityY + accelY * SIMULATION_CONSTANT_TIME);
	c.setPosition(c.getPositionX() + oldVelocityX * SIMULATION_CONSTANT_TIME + (1 / 2) * accelX * SIMULATION_CONSTANT_TIME_SQUARED, c.getPositionY() + oldVelocityY * SIMULATION_CONSTANT_TIME + (1 / 2) * accelY * SIMULATION_CONSTANT_TIME_SQUARED);

	return c;
}

float imediateReward(Car c, Car c_star, std::vector<Car> adjacencyList) {

	float newDistanceFromDesiredSpeed = abs(c_star.getDesiredSpeed() - c_star.getVelocityX());
	float oldDistanceFromDesiredSpeed = abs(c.getDesiredSpeed() - c.getVelocityX());

	float score = oldDistanceFromDesiredSpeed - newDistanceFromDesiredSpeed;

	float res = (7 / (newDistanceFromDesiredSpeed+epsilon));// +score;//elegxos gia 0
	if (res > 10) {
		res = 10;
	}
	//std::cout << "Score ->" << score << std::endl;
	/*if (c_star.getVelocityX() > c_star.getDesiredSpeed()) {
		res = res - oldDistanceFromDesiredSpeed;
	}*/

	// Check if a collision in eminent
	float collision = 0;

	// Check for the adjacency list if necessary 
	/*
	@@

	*/
	for (Car c2 : adjacencyList) {
		collision = collision + betweenCarsReward(c, c2);
	}
	if (adjacencyList.size() > 0) {
		collision = collision / adjacencyList.size();
	}

	float penalty = exitingRoadPenalty(c_star);

	float score2 = ALPHA * res + BETA * collision + penalty;
	
	return score2;
}

float exitingRoadPenalty(Car c) {
	if (c.getPositionY() >= 10 - SUMO_CAR_WIDTH - ROAD_SAFETY_GAP) {
		if (c.getVelocityY() > 0) {
			return -50;
		}
		else {
			return -10;
		}
	}
	if (c.getPositionY() <= SUMO_CAR_WIDTH + ROAD_SAFETY_GAP) {
		if (c.getVelocityY() < 0) {
			return -50;
		}
		else {
			return -10;
		}
	}
	return 0;
}

/* Function that computes the reward based on the likelihood of collision betweeen two cars*/
float betweenCarsReward(Car c, Car c1) {
	float distanceX = abs(c.getPositionX() - c1.getPositionX());
	float distanceY = abs(c.getPositionY() - c1.getPositionY());

	
	//float score = (10 / distanceX) + (10 / distanceY);
	float score = custom_pairwise_factor_function(c.getPositionX(), c.getPositionY(), c.getVelocityX(), c.getVelocityY(),
		c1.getPositionX(), c1.getPositionY(), c1.getVelocityX(), c1.getVelocityY(), 1, 0, 0, 0);
	
	return 3*score;
}


/*Function that accumulates the values of a float vector
*
*/
float vectorSumFloat(std::vector<float> v) {
	float s = 0;
	for (float f : v) {
		s = s + f;
	}
	return s;
}

/*Function that accumulates the values of an integer vector
*
*/
int vectorSumInt(std::vector<std::vector<int>> v) {
	int s = 0;
	for (std::vector<int> vi : v) {
		for (int i : vi) {
			s = s + i;
		}
	}
	return s;
}


int vectorSum(std::vector<int> v) {
	int s = 0;
	for (int i : v) {
		s = s + i;
	}
	return s;
}

node getNodeByCarNumber(int carNumber) {
	for (node n : maxPlusGraph) {
		if (n.getCar().getCarNumber() == carNumber) {
			return n;
		}
	}
}

double U_lemma33(double T, double v, double d, double ubar) {
	double nom, den;

	double ret;

	nom = T * ubar - 2 * v + sqrt(pow(T, 2) * pow(ubar, 2) - (4 * T * ubar * v) + 8 * ubar * (T * v - d));
	den = 2 * T;

	if (fpclassify((ret = nom / den)) == FP_NAN) {
		ret = ubar;
	}

	return ret;
}

void custom_regulate_forces(double* fx, double* fy, double y, double vx, double vy, double w, double T, double roadwid_meters, double uymax_hard) {
	//return;
	/* y wall */

	//printf("before regulation %f %f\n", *fx, *fy);

	/* keeping vehicles within the road */
	*fy = MAXIMUM(*fy, -WALL_DN(0, 1.05, vy, y, w, T, uymax_hard));
	*fy = MINIMUM(*fy, +WALL_UP(roadwid_meters, 1.05, vy, y, w, T, uymax_hard));



	/* [umin, umax] ranges */
	//*fx = MIN(*fx, uxmax_hard);
	//*fx = MAX(*fx, uxmin_hard);

	*fy = MINIMUM(*fy, +uymax_hard);
	*fy = MAXIMUM(*fy, -uymax_hard);

	/* non-negative speed */
	*fx = MAXIMUM(*fx, -vx / T);


	/* lat. speed never exceeds 0.5 x lon. speed, i.e.
	 * enforcing: |vy[k+1]| <= 0.5 vx[k+1]
	 */
	*fy = MINIMUM(*fy, +(0.5 * vx - vy) / T);
	*fy = MAXIMUM(*fy, -(0.5 * vx + vy) / T);
	// printf("after regulation %f %f\n", *fx, *fy);


}




// input args are: {x1,y1,vx1,vy1,vd1,l1,w1, x2,y2,vx2,vy2,vd2,l2,w2, ax1,ay1,ax2,ay2} only four (last) control variables are normalized within the range [0,1]
double custom_pairwise_factor_function(double x1, double y1, double vx1, double vy1, double x2, double y2,
	double vx2, double vy2, double axx1, double ayy1, double axx2, double ayy2) { // This is the potential field related factor

	double vd1 = 0;
	double l1 = 3.2;
	double w1 = 2;


	double vd2 = 0;
	double l2 = 3.2;
	double w2 = 2;

	double ax1 = axx1;// *(MAX_ACCEL - MAX_DECEL) + MAX_DECEL;
	double ay1 = ayy1;// *(2 * MAX_LAT_ACCEL) - MAX_LAT_ACCEL;

	//discretize_control(&ax1, &ay1);

	double ax2 = axx2;// *(MAX_ACCEL - MAX_DECEL) + MAX_DECEL;
	double ay2 = ayy2;// *(2 * MAX_LAT_ACCEL) - MAX_LAT_ACCEL;
	//std::cout << "Pairwize params:(" << ax1 << "," << ay1 <<") ("<<ax2<< ","<< ay2 << ")\n";
	//discretize_control(&ax2, &ay2);

	// bound the values based on how they may be filtered due to the boundary controls
	custom_regulate_forces(&ax1, &ay1, y1, vx1, vy1, w1, TIMESTEP_LENGTH, ROAD_WIDTH, MAX_LAT_ACCEL);
	custom_regulate_forces(&ax2, &ay2, y2, vx2, vy2, w2, TIMESTEP_LENGTH, ROAD_WIDTH, MAX_LAT_ACCEL);

	double dx_next = x2 - x1;// (x2 - x1) + (vx2 - vx1) * TIMESTEP_LENGTH + 0.5 * (ax2 - ax1) * pow(TIMESTEP_LENGTH, 2);

	double dy_next = y2 - y1;// (y2 - y1) + (vy2 - vy1) * TIMESTEP_LENGTH + 0.5 * (ay2 - ay1) * pow(TIMESTEP_LENGTH, 2);

	double dvx_next = (vx2 - vx1);// +(ax2 - ax1) * TIMESTEP_LENGTH;
	double dvy_next = (vy2 - vy1);// +(ay2 - ay1) * TIMESTEP_LENGTH;

	double a_c = (l1 + l2) / 2 + SAFETY_DIST_LONG_A;

	double b_c = (w1 + w2) / 2 + SAFETY_DIST_LAT_B;

	double dvx_k = (dvx_next * dx_next < 0) ? dvx_next : 0;
	double dvy_k = ((dvy_next * dy_next < 0) && (abs(dy_next) > b_c - SAFETY_DIST_LAT_B)) ? dvy_next : 0;

	double a_b = pow(dvx_k, 2) / (2 * abs(ACTUAL_MAX_DECEL)) + REACTION_TIME * (vx1 + vx2) / 2 + a_c;

	double dvy_k_curr = (((vy2 - vy1) * (y2 - y1) < 0) && (abs(y2 - y1) > b_c - SAFETY_DIST_LAT_B)) ? vy2 - vy1 : 0;

	double b_b = pow(dvy_k, 2) / (2 * abs(ACTUAL_MAX_LAT_ACCEL)) + REACTION_TIME_LAT * (abs(vy2) + abs(vy1)) * (abs(dvy_k_curr) > 0) + b_c;



	// create the field here
	double field_util = -ELLIPSE_MAGNITUDE / pow((pow(abs(dx_next) / a_b, ELLIPSE_POW_X) + pow(abs(dy_next) / b_b, ELLIPSE_POW_Y) + 1), ELLIPSE_POW_T) - ELLIPSE_MAGNITUDE / pow((pow(abs(dx_next) / a_c, ELLIPSE_POW_X) + pow(abs(dy_next) / b_c, ELLIPSE_POW_Y) + 1), ELLIPSE_POW_T);





	return field_util;
}
#endif

