# Mini Project 1

![](https://github.com/bgrantboards/hbird_webots/GIF.gif)

## What did you learn from this? What did you not know before this assignment?

We learned a lot about working with ROS, which was a relatively new coding environment for some of us. Prior to this assignment we did not know a lot about the PID controllers coupled system,  translational and rotational,  and how it affects the drone response. We got to learn more about how they are controlled, two PID controllers, one controlling the linear thrust set point, and the other controlling the angular position based on those set points. 

## What was the most difficult aspect of the assignment?

The most difficult aspect for us was with the import statement in the controller file, specifically with accessing the pid_gains in the controller file that was preventing us from testing our code. There were also minor setbacks debugging our code using ROS command-line tools since it was difficult to trace our issues.

## What was the easiest or most straightforward aspect of the assignment?

Translating the formulas given to us in class into code in our controller was relatively easy and quick to do, we ran into some issues with the formatting but it didn’t take a lot to fix. Adjusting the z rise without getting an overshoot, along with the overall tuning PID controls,  although it did take us some time, was not too difficult since we were already familiar with it.  

## How long did this assignment take? What took the most time (Setup? Figuring out the codebase/ROS2? Coding in Python? Exploring the questions?)?

It took us around 10 hours to complete the assignment. What took the most time was understanding the codebase, since we didn’t write it, and with issues running our code due to what we were running in the source command. Tracing our error outputs in terminal also took a lot of time because the error is not highlighted in the output, we had to go through each line to find the error.  

## What did you learn about PID controllers that we didn’t explicitly cover in class or in this assignment?

One thing we learned about the PID controllers in the assignment was how the controllers phi, theta, and psi have to work together with  x, y, and z in order for it to execute properly. If the P values for phi or theta are low, it results in a delayed response even if the P values for x or y are set to high.

## What more would you like to learn about controls?

Lulu: I would like to learn more about how the two different controllers affect each other. Although we were exposed to it with tuning,  and I think I have a good understanding on how each works individually, I am just still not certain on how exactly they affect each other. 

Carlo: I would like to learn how to calibrate multiple controllers at the same time without controller information. In this assignment, it took me some time to understand how the different controllers affect each other and because we were given information about the controllers, I was able to understand it, now I would like to try it without being provided information. 

Ben: I would like to learn more about the ROS environment and develop my skills in debugging. It was challenging and time consuming to trace errors, especially when they come from separately executed coupled processes. I am also interested in learning about integrating more software tools like RViz which allows for visualizing geospatial data from our robot control algorithms and the simulation environment, which is what I used for comprobo last year and was much easier to use as a debugging tool.  
