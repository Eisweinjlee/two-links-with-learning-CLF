# two-links-with-learning-CLF

updated on April 24, 2019

### There are two Matlab program in the project:

1. `main.m` is a naive PD feedback controller for two-link manipulator
  
    It is finished. It can be improved by adding feedforward term, but I am not going to do this.
  
  
2. `main_two_links.m` is an input-output linearization CLF-based controller for two-link manipulator
  
    It is developing and there are many problems.
  

### On going works and plans:

1. finish a very basic CLF-based controller[1] for two-link

2. Model an estimated system by inducing uncertainties

3. Using the episodic learning method to train CLF for the estimated system control[2]


### References:
[1] Ames, Aaron & Galloway, Kevin & Sreenath, Koushil & W. Grizzle, Jessy. (2014). Rapidly Exponentially Stabilizing Control Lyapunov Functions and Hybrid Zero Dynamics. Automatic Control, IEEE Transactions on. 59. 876-891. 10.1109/TAC.2014.2299335. 

[2] Andrew J. Taylor, Victor D. Dorobantu, Hoang M. Le, Yisong Yue, Aaron D. Ames. (2019). Episodic Learning with Control Lyapunov Functions for Uncertain Robotic Systems. arXiv:1903.01577.
