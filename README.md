# Computed-torque-C
Computed torque control of a pendulum in C

Code created to support a Linkedin post. Follow me on Linkedin! https://www.linkedin.com/in/simone-bertoni-control-eng/

Original post: https://www.linkedin.com/posts/simone-bertoni-control-eng_computed-torque-c-activity-7023560026476699648-KtA0?utm_source=share&utm_medium=member_desktop

If you are interested in robotics this is for you.

Computed torque control implemented in C.

I showed the theory in a couple of previous posts (links in the comments).

Recap:

The computed torque control technique is a typical approach to controlling robot manipulators.

As the pendulum is a simple case of a robot manipulator, we can control it using this technique.

The equation of motion is:

tau = m*l^2*ddtheta + k*dtheta + m*g*l*sin(theta)

The computed torque control law is:

tau = m*l^2*(ddtheta_des + u) + m*g*l*sin(theta) + k*dtheta

with u = kp*e + kd*de + ki*int(e)

where e = theta_des - theta

To implement this example in C I have used 6 functions:

1/ PID_Step

To implement the PID term in the control law u = kp*e + kd*de + ki*int(e).

Uses a filtered derivative for the D component.

2/ Filtered_Derivative

The filtered derivative is used to:

- Estimate dtheta from theta
- Calculate ddtheta_des by applying twice the function

3/ First_Order_LP_Filter

The first-order low pass filter is applied to the setpoint, to make it smoother and ready to be derived twice without spikes.

4/ Computed_Torque_Control_Algorithm

The function calculates the computed torque term tau as follows:

- Filter the setpoint
- Calculate the second derivative of the setpoint
- Calculate the PID term
- Calculate dtheta from theta
- Calculate the computed torque term

5/ Pendulum_Step

This function implements the model of the pendulum.

6/ Main

This function actually implements the simulation:

Runs a while loop 6000 times (60 seconds with 0.01s sample time)

For each loop runs the computed torque algorithm using the previous value of theta and the current setpoint, runs the pendulum step using the output and then logs in a text file

To give some "real world" flavour to the simulation, the computed torque term is calculated using m_est = 1.1*m, l_est = l*1.1 and k_est = k*1.1.

Essentially the control algorithm thinks that all the parameters are 10% bigger.

The simulation result is shown on the first slide.

Parameters used:

- kp = 10
- ki = 6
- kd = 10
- T = 0.01
- T_c = 0.5
- m = 0.5
- l = 1
- k = 0.5
- m_est = 0.55
- l_est = 1.1
- k_est = 0.55
- g = 9.81

If you enjoyed this follow me for more tips on control and embedded software engineering.

Hit the 🔔 on my profile to get a notification for all my new posts.

Feel free to ask anything in the comments, I'll do my best to answer.

#controlsystems #embeddedsystems #softwareengineering #embeddedsoftware #controltheory #robotics
