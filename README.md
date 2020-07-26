# Tossing_robot

This project is machine learning of tossing ball.\
Artificial neaural network(ANN) is trained Binary Cross Entrophy(BCE) as loss.\
Robot has 3 DoF that is similar with huhan's arm.\
Robot learns the 6 parameters, 3 joint angles and 3 joint angular velocity.\
Robot operate with these 6 parameters.\
When robot tossing a ball, goal post senses and decides the success or failure.\
If robot scores a goal, arduino connected with sensor sends the "Goal" and result is recorded as integer 1.\
If robt fails, result is recorded as integer 0.\
6 parameters of robot policies and 2 goal post position (X, Y) is the 8 learning input (8 Dim).\
Output is Success or Failure (1 or 0) (l Dim).\
BCE is effective to binary output learning like this learning molel.\
In policy searching code, robot selects the policy that has max probability of success.\
So robot is trained to direction of success.
