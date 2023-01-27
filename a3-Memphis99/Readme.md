# Assignment 3 - Boids

VIDEO LINK: https://www.youtube.com/watch?v=LfQSQydeSPY

To better test the tasks, I made possible to select directly from the app the integration methods to use, the forces to apply and the step size used to update positions and velocities. To do so, I also had to modify the initialization of the boids and the visualizations (e.g. to display the circle or to differentiate the leader or the 2 boid groups).
I also applied a limitation to the velocity that boids can reach, as it improved the simulation and avoided the velocities to diverge too much.
I had to tune many parameters, as the radii for the neighbours search, or how much force to apply.

NOTES:
CIRCULAR MOTION: as seen in the clip 2, using explicit euler the boid positions diverge in time, increasing their distance to the center of rotation (put at 0.5, 0.5 for visualization reasons). This doesn't happen with sympletic and explicit midpoint euler methods.

SEPARATION: I divided the forces for the relative distances between two boids in order to have great repulsive forces when two boids were really close to each others.

COLLISION AVOIDANCE: I drawed an arbitrary cicrle with fixed center and radius, and I supposed them known. This task works better with explicit and sympletic euler, as midpoint euler calls 2 times the function with 2 different positions, causing some conflicts. I multiplied the force with a fixed constant, so it could not work well with step sizes under a certain value. To avoid to trap some boids inside the circle, and to have a smoother impact on the obstacle, I have put the condition that the velocity's direction should point toward the center of the circle and not outside. This improved a bit the results.

LEADER FOLLOWING: here we have different force components. One that repulse the boids if they are too cloe to the leader, one that avoid overcrowding, one for collision avoidance, one that makes boids follow leader's position and another one that makes them allign with its velocity. Lastly, the leader follows the mouse cursor. I had to add a dumping force to make it reach the target precisely, without oscillations. I chose the first boid as leader for convenience.

COLLABORATIVE AND ADVERSARIAL BEHAVIORS: for this task, I firstly divided the boids in 2 groups, splitting them in 2 halves (20 and 20). To do so, I assigned 1 or 0 based on the group, saving values in a vector. Other than this, I created a vector where I stored the simulation time at wich the boids lastly made spawn a mate. I used this in order to make a "timer" of 10 frames between two consecutive spawns from the same boid. If two boids from the same group are close enough, one boid from the same group will spawn with random velocity and position (to avoid overcrowding). If a group of enemies is close to a boid, and have at least 3 more components than his group, the boid is removed (this made the deletion more even and random in the two groups). If in a larger radius the enemies are 3 more than the mates, the boid will run away from them. In the opposite case, the boid will chase them. Moreover, every boid will try to group with near mates.
In clip 8, I disabled every control strategy: boids contuined to move randomly, not many boids spawned or were removed.
In clip 9, I created a predator prey model: only group 0 (red) could eat and chase the enemy, while group 1 (blue) could only escape. The red group clearly dominated every time.
In clip 10, I enabled every control for both groups: the results were pretty even, sometimes one group dominated, sometimes the other, and in some cases like the last one I recorded no group took over the other one. In general, both tried to group together and chased/escaped the other groups based on their number.






