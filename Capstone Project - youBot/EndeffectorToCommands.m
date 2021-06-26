function current_vel = EndeffectorToCommands(Ve, current_config, Jb_arm, Tse_curr_act)



l = 0.47/2;
w = 0.3/2;
% r = 0.0475 meters
r = 0.0475;


phi = current_config(1,1);
x = current_config(1,2);
y = current_config(1,3);
H = (1/r)*[(-l-w) 1 -1; (l+w) 1 1; (l+w) 1 -1; (-l-w) 1 1];
Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];

F = pinv(H);
%disp(F);
F6 = [zeros(1,size(F,2)); zeros(1,size(F,2)); F; zeros(1,size(F,2))];
%disp(F6);
J_base = Adjoint((inv(Tse_curr_act)) * Tsb) * F6;

Je = [J_base Jb_arm];
%disp(Je);
%disp(pinv(Je));
%disp(Ve);
current_vel = pinv(Je)*Ve;

current_vel = current_vel';
w_sp = current_vel(1,1:4);
e_sp = current_vel(1,5:9);

current_vel = [e_sp w_sp];

end

