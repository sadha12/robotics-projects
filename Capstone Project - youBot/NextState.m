function new_config = NextState(current_config, current_vel, del_t, speed_lim)

phi = current_config(1,1);
x = current_config(1,2);
y = current_config(1,3);

curr_j_angles = current_config(1, 4:8);
curr_j_speeds = current_vel(1,1:5);

%for i = 1:length(curr_j_speeds)
 %   if curr_j_speeds(1,i)>abs(speed_lim(1,i))
  %      curr_j_speeds(1,i) = abs(speed_lim(1,i));
   % end
    %if curr_j_speeds(1,i)< (-1*abs(speed_lim(1,i)))
  %      curr_j_speeds(1,i) = (-1*abs(speed_lim(1,i)));
   % end
%end

        
new_j_angles = curr_j_angles + curr_j_speeds*del_t;


curr_w_angles = current_config(1, 9:12);
curr_w_speeds = current_vel(1,6:9);

%for i = length(curr_w_speeds)
 %   if curr_w_speeds(1,i)>abs(speed_lim(1,i+5))
  %      curr_w_speeds(1,i) = abs(speed_lim(1,i+5));
   %%if curr_w_speeds(1,i)< (-1*abs(speed_lim(1,i+5)))
     %   curr_w_speeds(1,i) = (-1*abs(speed_lim(1,i+5)));
    %end
%end

new_w_angles = curr_w_angles + curr_w_speeds*del_t;

gs = current_config(1, end);

l = 0.47/2;
w = 0.3/2;
% r = 0.0475 meters
r = 0.0475;

H = (1/r)*[(-l-w) 1 -1; (l+w) 1 1; (l+w) 1 -1; (-l-w) 1 1];

curr_twist = pinv(H)*curr_w_speeds';

t6 = [0; 0; curr_twist; 0];
t6se3 = VecTose3(t6);

T_k_k1 = MatrixExp6(del_t*t6se3);

Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
%disp(Tsb);
T_b_k = eye(4);

T_b_k1 = T_b_k*T_k_k1;

T_s_k1 = Tsb*T_b_k1;

new_chassis = [atan2(T_s_k1(2,1),T_s_k1(1,1)) T_s_k1(1,4) T_s_k1(2,4)];

new_config = [new_chassis new_j_angles new_w_angles gs];

end
