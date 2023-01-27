# Assignment 6 - Trajectory Optimization

**Hand-in:** May 27, 2022, 18:00 CEST 

First Name: Andrea

Last Name: Bionda

Solution to Question 2:
![question 2 answer](/images/question2.jpg)

Solution to Question 5: 
Newton's method takes only 1 step to converge to the final objective value. This happens because our objective function is quadratic and convex, so the optimization will converge directly to the minima (the one of the objective function and the Newton approximation coincide).

Solution to Question 7:
![question 7 answer](/images/question7.PNG)

Solution to Question 9: 
Gradient descend method doesn't work well with this problem. This is because having to optimize 3 different variables together, and having a single learning rate and potentially big coefficients, at every optimization step we will have big numerical differences in the update quantity for every variable, regardless the distance from their minimum. This will cause an under or overshooting of the variables, making then the optimization diverge or explode.

Explanation of your approach to Question 10 (required for full credit):
For this task, I set a target for both position and velocity, and used them in the objective function.
I decided for example to take the position at "-45°" from the top of the orbit. Then, I computed the velocity target to be tangential to the orbit in the target point, and set its module based on the orbital speed equation (in this case, I divided by the square root of the orbit ray, as the numerator was given as =1).
Moreover, I decided to add 3 regularizers, both for the objective function and for the velocity target's x and y components, as the resulting spaceship movement was not perfectly circular but more elliptical.

---

Assignment writeup: http://crl.ethz.ch/teaching/computational-motion-22/slides/Tutorial-Write-Up-6.pdf

---

- NOTE: Don't forget to switch to Release mode or Release With Debug Info otherwise it's gonna be just unimaginably slow.
- NOTE: Tested builds: GCC 9.3 (Linux), Visual Studio 2022 (Windows), Clang 12.0.0 (Mac OS)
