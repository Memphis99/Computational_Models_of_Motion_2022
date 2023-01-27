First Name: Andrea

Last Name: Bionda

Video link: https://www.youtube.com/watch?v=ZFshVjQd5xs

Solution to Question 4: 
See the image in the following link https://ibb.co/M1JkTKC

Solution to Question 10:
The current objective function just considers the distance between the target and deformed position of the two points we can drag. This is why the only deformation the bar will incur is to bring the 2 points in their target position. In order to do so, the pins just need to translate coherently with the rest of the bar, but rotation doesn't bring the objective function value down. Furthermore, energy is also not considered, and this is another reason why the triangles near to the extremes can be deformed more than necessary in the positions we are considering.


Solution to Question 11:
I implemented a regularizer that tries to align the direction of the handles to the direction defined by the line passing through the targets of the 2 feature points that we can drag. Doing so, when the bar is stretched, the targets are reached as they normally did before, but the extremes now follow a more coherent behaviour and get stretched too.
To do so, I computed the angle defined by the targets using the atan 2 function on their positions difference, and I calculated the sum of the squared differences between that and the 2 handle orientations.
Then, I multiplied the sum with a coefficient. The sum of squared differences performed better than the regular sum, as it behaves similar to the original objective function, other than being differentiable (if we wanted to implement the gradient). The coefficient lets us trade between the original objective function and the regularizer, to be sure that the main task is satisfied well.
This gives some problems when you do more than a pi/2 rotation, due to the incompatibility between the domains of the handle rotations and atan2 function.
---

Please find the assignment write-up on the course webpage.

---

Could use ./build.sh on Linux/MacOS
