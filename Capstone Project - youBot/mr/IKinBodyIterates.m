function IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector
%           position error less than ev.
% Returns thetalist: Joint angles that achieve T within the specified 
%                    tolerances,
%         success: A logical value where TRUE means that the function found
%                  a solution and FALSE means that it ran through the set 
%                  number of maximum iterations without finding a solution
%                  within the tolerances eomg and ev.
% Uses an iterative Newton-Raphson root-finding method.
% The maximum number of iterations before the algorithm is terminated has 
% been hardcoded in as a variable called maxiterations. It is set to 20 at 
% the start of the function, but can be changed if needed.  
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

thetalist = thetalist0;
i = 0;
t =thetalist';
maxiterations = 20;
Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
while err && i < maxiterations
    disp(' ');
    disp(' ');
    disp('----------- ');
    disp(['iteration: ', num2str(i+1)]);        %Iteration number
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;  %joint vector
    t = [t;thetalist'];         %Collection of joint vectors
    fprintf('\njoint vector: ');
    fprintf('%d, ', thetalist(:)'); %Print joint vector
    T_curr = FKinBody(M,Blist,thetalist);       %Current state of end effector
    fprintf('\n\nSE(3)end - effector config: ');            
    fprintf('%d %d %d    ', T(:)' - T_curr(:)');        %Print difference between current state and desired state
    %disp(['SE(3)end - effector config: ',num2str(T(:)' - T_curr(:)')]);
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));     %End effector twist
    fprintf('\n\nerror twist V_b: ');
    fprintf('%d, ', Vb(:)');                
    %disp(['error twist V_b: ',num2str(Vb(:)')]);
    aerr = sqrt((Vb(1,1)^2) + (Vb(2,1)^2) + (Vb(3,1)^2));       %angular error magnitude ||omega_b|| 
    verr = sqrt((Vb(4,1)^2) + (Vb(5,1)^2) + (Vb(6,1)^2));       %linear error magnitude ||v_b||
    fprintf('\n\nangular error magnitude ||omega_b|| ');        % print angular error magnitude ||omega_b|| 
    fprintf('%d, ', aerr);
    %disp(['angular error magnitude ||omega_b|| ',num2str(aerr)]);      % print linear error magnitude ||v_b| 
    fprintf('\n\nlinear error magnitude ||v_b|| ');
    fprintf('%d, ', verr);
    %disp(['linear error magnitude ||v_b|| ',num2str(verr)]);
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    i=i+1;
end
 csvwrite('iterates.csv',t);
 

end