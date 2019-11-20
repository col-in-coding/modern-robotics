function [thetalist, success] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
% ***
% This is doing the same thing in IKinBody, except report the details of
% each iterate and out the results
%
% Example Inputs:
% 
% clear; clc;
% Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% T = [[0, 1, 0, -5]; [1, 0, 0, 4]; [0, 0, -1, 1.6858]; [0, 0, 0, 1]];
% thetalist0 = [1.5; 2.5; 3];
% eomg = 0.01;
% ev = 0.001;
% [thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)
% 
% Output:
% thetalist =
%    1.5707
%    2.9997
%    3.1415
% success =
%     1
%
% Joint vectors saved as rows in file "iterates.csv"

thetalist = thetalist0;
i = 0;
maxiterations = 20;
Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
report(i, thetalist, T, Vb);
% collect joint vectors
iterates(1, :) = thetalist';
while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    i = i + 1;
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    report(i, thetalist, T, Vb);
    iterates(i+1, :) = thetalist';
end
success = ~ err;
% this would write to csv file
csvwrite('iterates.csv', iterates);
end


%% Print out the report for each iteration
function report(i, thetalist, T, Vb)
fprintf('------------\n');
fprintf('Iteration: %d\n\n', i);
fprintf('joint vector:\n');
disp(thetalist');
fprintf('SE(3) end-effector config:\n');
disp(T);
fprintf('error twist V_b:\n');
disp(Vb');
fprintf('angular error magnitude ||omega_b||: %.8f \n\n', norm(Vb(1: 3)));
fprintf('linear error magnitude ||v_b||: %.8f \n\n', norm(Vb(4: 6)));
end