# two-links-with-learning-CLF

updated on July 27, 2019. This project is paused for the time being.

### There are two Matlab program in this project:

1. `main_naive.m` is a naive PD feedback controller for two-link manipulator
  
    It is finished. It can be improved by adding feedforward term, but I am not going to do this.
  
  
2. `main_clf.m` is a CLF-based controller for two-link manipulator
  
    It is not finished yet.

    New Features: tuning the control parameter by epsilon for rapid exponentially stability[1].
  

### On going works and plans:

1. (finished) finish an input-output linearization system with PD controller

2. Coding a optimization algorithm for CLF-based controller[1] design.

3. Model an estimated system by inducing uncertainties

4. Using the episodic learning method to train CLF for the estimated system control[2]


### References:
[1] Ames, Aaron & Galloway, Kevin & Sreenath, Koushil & W. Grizzle, Jessy. (2014). Rapidly Exponentially Stabilizing Control Lyapunov Functions and Hybrid Zero Dynamics. Automatic Control, IEEE Transactions on. 59. 876-891. 10.1109/TAC.2014.2299335. 

[2] Andrew J. Taylor, Victor D. Dorobantu, Hoang M. Le, Yisong Yue, Aaron D. Ames. (2019). Episodic Learning with Control Lyapunov Functions for Uncertain Robotic Systems. arXiv:1903.01577.
