addpath('mr')

clear;
clc;
% constant parts
M01 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.089159; 0, 0, 0, 1];
M12 = [0, 0, 1, 0.28; 0, 1, 0, 0.13585; -1, 0, 0, 0; 0, 0, 0, 1];
M23 = [1, 0, 0, 0; 0, 1, 0, -0.1197; 0, 0, 1, 0.395; 0, 0, 0, 1];
M34 = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.14225; 0, 0, 0, 1];
M45 = [1, 0, 0, 0; 0, 1, 0, 0.093; 0, 0, 1, 0; 0, 0, 0, 1];
M56 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.09465; 0, 0, 0, 1];
M67 = [1, 0, 0, 0; 0, 0, 1, 0.0823; 0, -1, 0, 0; 0, 0, 0, 1];
G1 = diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7]);
G2 = diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393]);
G3 = diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275]);
G4 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
G5 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
G6 = diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879]);
Glist = cat(3, G1, G2, G3, G4, G5, G6);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67); 
Slist = [0,         0,         0,         0,        0,        0;
         0,         1,         1,         1,        0,        1;
         1,         0,         0,         0,       -1,        0;
         0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491;
         0,         0,         0,         0,  0.81725,        0;
         0,         0,     0.425,   0.81725,        0,  0.81725];


% initial states
thetalist = [0; -1; 0; 0; 0; 0;]; % starts with join2 at -1 radian, others are 0
dthetalist = [0; 0; 0; 0; 0; 0;]; % zero velocity
taulist = [0; 0; 0; 0; 0; 0;]; % zero torques applied to the joint
Ftip = [0; 0; 0; 0; 0; 0;]; % zero wrench applied to the end-effector
g = [0; 0; -9.81]; % gravity
% dt is 1/200s
dt = 1/200;

% collect the joint vectors
buffer(1, :) = thetalist';

% 5 seconds
for sec = 0:4
    % 100 iterations per sec
    for i = 1:200
        % calculate ddthetalist in Forward Dynamics
        ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip, Mlist,Glist,Slist);
        % calculate the joint motions
        [thetalistNext, dthetalistNext] = EulerStep(thetalist, dthetalist, ddthetalist, dt);
        thetalist = thetalistNext;
        dthetalist = dthetalistNext;
        index = i + 1 + sec * 200;
        % storing the results for this iteration
        buffer(index, :) = thetalist';

    end
end

% write to csv file
csvwrite('simulation2.csv', buffer);