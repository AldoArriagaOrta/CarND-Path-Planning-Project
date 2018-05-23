#  Path Planning

---

**Path Planning Project**

The goals of this project are the following:

* The code should compile.
* The car is able to drive at least 4.32 miles without incident..
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes

[//]: # (Image References)
[image1]: ./Success.png

## [Rubric](https://review.udacity.com/#!/rubrics/1020/view) Points


---

### Reflection

#### Reflection on how to generate paths


My implementation followed the guidelines provided in the walthrough video:

1. The main principle is to create a list of widely spaced waypoints, interpolate between them with a spline or a polynomial (to ensure smooth trajectories with minimal jerk) and fill up with more points to control the speed.
2. The last two points from the previous trajectory are incorporated into the first points of the new trajectory in order to smoothen the transition by making it tangent to the previous trajectory.
3. The helper function getXY is used to determine 3 waypoints 30 m apart  (in Frenet coordinates)
4. The points list is transformed into car coordinates to facilitate the computations.
5. The spline library is used to create a smooth line connecting the waypoints.
6. Speed is determined by the spacing between points.
7. The result is transformed back into world coordinates.
8. Lane changes are achieved by specifying the lane number and associating a lateral distance (Frenet coordinate d) range to each lane. When switching lane numbers, the center of the new lane in d is used to generate the spline for the new path.

The smooth trajectories were already covered by these points, the next step was to define the set of behaviours and transitions for a Finite State Machine: Keep Lane (KL), Change Lane Left (CLL) and Change Lane Right (CLR) (If a lane change is not possible, then the ego speed is reduced), depending on the lane the ego vehicle is:

```
// Finite State Machine 
switch (lane)
{
	case 0: // Ego vehicle in leftmost lane
		if (not_enough_gap_right) //KL
		{
			lane = 0; 
		}
		else if ((!not_enough_gap_right ) && (too_close)) //CLR
		{
			lane = 1;
		}
		break;

	case 1: // Ego vehicle in center lane			
		if ( ((not_enough_gap_left) && (not_enough_gap_right) ) ) //Keep Lane
		{
			lane = 1; 
		}
		else if (!not_enough_gap_left && too_close) //CLL
		{
			lane = 0;
		}
		else if (!not_enough_gap_right && too_close)//CLR
		{
			lane = 2;
		}
		break;

	case 2: // Ego vehicle in rightmost lane
		if (not_enough_gap_left) //KL
		{
			lane = 2;
		}
		else if ((!not_enough_gap_left) && (too_close)) //CLL
		{
			lane = 1;
		}
		break;

	default:
		break;

}

if (too_close)
{
	ref_vel -= 0.224;
}
else if (ref_vel < 49.5) 
{
	ref_vel += 0.224;		
}
```

The main idea for this design is to check if there are vehicles ahead of the ego vehicle and if there is enough room for a safe lane change in the adjacent lanes, reducing the safety distance will result in more audacious driving and incrementing it results in a more moderate behavior. Sensor fusion data is used to predict in a very simple fashion the position of surrounding vehicles and determine whether or not there is room for lane changes. The logic for setting the flags for the FSM is covered in this snippet:

```
// Adapted from Walkthrough part 2: Sensor Fusion
if (prev_size > 0) 
{
	car_s = end_path_s;
}

bool too_close = false;
bool not_enough_gap_left = false;
bool not_enough_gap_right = false;

//check the position and speed of surrounding relevant vehicles to set FSM transition flags
for (int i = 0; i < sensor_fusion.size(); i++)
{
	float d = sensor_fusion[i][6];
	double vx = sensor_fusion[i][3];
	double vy = sensor_fusion[i][4];
	double check_speed = sqrt(vx*vx + vy*vy);
	double check_car_s = sensor_fusion[i][5];

	check_car_s += ((double)prev_size*0.02*check_speed); //prediction of s coordinate of [i] vehicle

	if (d<(2 + 4 * lane + 2) && d>(2 + 4 * lane - 2)) // check for vehicles in ego lane
	{
		if ((check_car_s > car_s) && ((check_car_s - car_s) < 20))
		{
			too_close = true;					
		}
	}

	if (lane>=1) 
	{
		if ((d<(2 + 4 * (lane - 1) + 2) && d>(2 + 4 * (lane - 1) - 2))) // check for vehicles in left lane
		{
			if (((check_car_s - car_s) >= -15) && ((check_car_s - car_s) <= 25))
			{
				not_enough_gap_left = true;
			}
		}
	}

	if (lane<=1)
	{
		if ((d<(2 + 4 * (lane + 1) + 2) && d>(2 + 4 * (lane + 1) - 2))) // check for vehicles in right lane
		{
			if (((check_car_s - car_s) >= -15) && ((check_car_s - car_s) <= 25))
			{
				not_enough_gap_right = true;
			}
		}
	}
}

```

 This has been a very minimalistic implementation, a simple Finite State Machine was enough to meet the project specifications. There is room for plenty of improvements, just to mention a few:
 
* Use naive Bayes to predict more accurately the position of surrounding vehicles and determine a more suitable path.
* Extend the states and transitions to consider cases like: switching two lanes consecutively, monitor not only adjacent lanes to predict when vehicles are likely to switch lanes and interfere with the current ego path (already in motion), emergency brake state.
* Implement a set of cost functions to choose the most appropriate state/transition.
* Implement a speed controller for a smoother drive following cars ahead of the ego car.

![alt text][image1]








