## Reflection: ##

**Attribution:** I have completed this project with the help of Udacity's Project Overview Video on this project by Aaron Brown and Mind the Line Quiz which is provided as classroom material. I would like to thank Aaron and Udacity team for the insightful module and project review video which has enabled me to implement this project. 

**Disclaimer:** I have tuned the cost parameter using trial and error method.

### Model Selection ###

The model was selected based on the classroom modules and no changes were made to it. The following equations are of the chosen model:

**State Equations**

```
x' = x + v*cos(psi)*dt
y' = y + v*sin(psi)*dt
psi' = psi + v/Lf*delta*dt
v' = v + a*dt
```

In the above equation, the variables on the left side indicates the value next state based on the current state.
The state x, y, psi and v are actuated by steering angle `delta` and throttle control `a` which indicates acceleration/retardation.

**Error Equations**

```
cte' = cte + vt*sin(e_psi)*dt
cte' = f(x_t) - y_t + vt*sin(e_psi)*dt
```

```
e_psi' = e_psi + v/Lf * delta * dt
psi_des = arctan(f'(x_t)) i.e. tangential angle of polynomial f evaluated at x_t.
e_psi = psi - arctan(f'(x_t))
```

In the above equation, the variables on the left side indicates the value next state based on the current state.

### Timestep length and Elapsed Duration ###

**Values taken from Project Overview Video**

```
N = 10 (taken from Project Overview Video)
dt = 0.1
T = 1 second
```

I have taken these values from project overview video as a starting point. These value made my solution work fine, then I tried increasing and decreasing these values to see which other values can give me desired solution. Please find below some analysis of these different values:

`N = 5, dt = 0.1, T = 0.5 seconds:` Since the discrete points were only 5, the controller was not able to account for what would come next and was driving the car straigth only even on the curves. Ultimately the vehicle crashed on a turn.

`N = 5, dt = 0.05, T = 0.25 seconds:` Fortunately vehicle didn't crashed with this setting and it was able to complete lap for sure, however, it was always on the edges of road and was on the verge of collision. This setting has increased the number of steps at which we are computing the next state, however still the overall prediction duration is less i.e. 0.25 seconds because of which controller was not able to estimate the future state as accurately as needed.

`N = 15, dt = 0.05, T = 0.75 seconds:` With this setting, the predicted trajectory length was increased as expected, however since the discretization sampling was higher, controller was giving large steering angle actuations and it was steering the vehicle left and right in very short intervals. As a result, vehilce just lost the track and stopped moving in few seconds.

`N = 15, dt = 0.1, T = 1.5 seconds:` This setting started of very nicely and I thought I would keep this as the optimum solution as the predictions were really aligned to the center of lane and the car was driving really great. However, before the completion of lap, as the car approached sharp turn, it the steering actuation had oscillatory behavior and ultimately car crashed so I had to reject this setting as well. Although I believe that if I try to optimize the cost factors on this settings of Timestep and Elapsed Duration, I might be able to achieve an optimum solution with this setting as well.

`N = 12, dt = 0.1, T = 1.2 seconds:` This was the most optimum solution I could find with my cost parameters tuned. Please take a note that I tuned my cost parameters while keeping the timestep and elapsed duration same as the ones in Project Overview Video and then later I tuned these values. With this setting, the car was driven accurately by the controller without leaving the lane or any oscillatory actuations. It was able to achieve speed upto 50 mph also. I tried with higher speeds and sometimes the car was able to complete the lap while other times it just crashed. I strongly believe that with this setting, my cost factors can be tuned even further to achieve the speed near to 100 mph, however I will try that later as I need to complete Term 2 and get myself enrolled in Term 3.

### Latency Handling ###

I got the approach to implement latency handling from slack channel of this project. The idea was pretty simple and it was to consider latency value as time change in state equations of our model. Following code is implemented in main.cpp for the same:

```
double latency = 0.1;
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;
psi = psi - v*steer_value/Lf*latency;
v = v + throttle_value*latency;
```

As one can observe in the above equation that dt is replaced by latency, rest of all is similar to state equations of model.

### Optimum Solution: ###

**Values chosen by Trial and Error:**

```
ref_v = 50
Cross-track Error Factor = 1000
Orientation Error Factor = 1000
Velocity Error Factor = 1
Delta Actuation Factor = 5
Acceleration Actuation Factor = 5
Delta Actuation Change Factor = 600
Acceleration Actuation Change Factor = 10
```

**Please Note:** The coefficient values which are multiplied to the various cost computations are labelled as Factor in above description.