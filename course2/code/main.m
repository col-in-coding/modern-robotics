
clear; clc;
W1 = 0.109;
W2 = 0.082;
L1 = 0.425;
L2 = 0.392;
H1 = 0.089;
H2 = 0.095;

% Home configuration of end-effector
M = [
    -1 0 0 L1+L2;
    0 0 1 W1+W2;
    0 1 0 H1-H2;
    0 0 0 1
];

% The screw axes
B = [
    0 1 0 W1+W2 0 L1+L2;
    0 0 1 H2 -L1-L2 0;
    0 0 1 H2 -L2 0;
    0 0 1 H2 0 0;
    0 -1 0 -W2 0 0;
    0 0 1 0 0 0;
]';

% Desired end-effector configuration
T = [
    0 1 0 -0.5;
    0 0 -1 0.1;
    -1 0 0 0.1;
    0 0 0 1;
];

% user threshold
eomg = 0.0001;
ev = 0.00001;

% first guess
thetalist0 = [-0.143 2.198 1.570 -0.628 -3.140 1.571]';

[thetalist, success] = IKinBodyIterates(B, M, T, thetalist0, eomg, ev);




