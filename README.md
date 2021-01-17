# Throwing_robot

This is a deep reinforcement learning project for throwing robot.\
The robot has 3 DoF that is similar with huhan's arm.\
It learns 6 parameters, 3 joint angles and 3 joint angular velocities.\
It operates with these 6 parameters.\
When the robot throws a ball, goal post senses success or failure.\
If it is a success, arduino connected with impact sensor sends the "Goal" and result is recorded as integer 1.\
If else, result is recorded as integer 0.\
Artificial neaural network(ANN) is trained by Binary Cross Entrophy(BCE) as loss.\
ANN input consists of 8 dimensions; the 6 parameters of robot policies and 2 goal post position (X, Y).\
Output consists of 1 dimension; Success or Failure (1 or 0).\
BCE is effective for binary output learning model.\
In policy searching step, the robot selects the policy with max probability of success.\
After each throwing, new training data accumulates.\
So gradually the performance of throwing enhances.\

Deep Reinforcement Learning of Ball Throwing Robot’s Policy Prediction.\
DOI : 10.7746/jkros.2020.15.4.398.
