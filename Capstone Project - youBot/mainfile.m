current_config = csvread('config.csv');
current_config = current_config(1,:);

arm_vel = 0.25*zeros(1,5);
base_vel = 10*[0 0 0 0];

current_vel = [arm_vel base_vel];

arm_lim = ones(1,5);
base_lim = 12.5*ones(1,4);

speed_lim = [arm_lim base_lim];

del_t = 0.01;

phi = current_config(1,1);
x = current_config(1,2);
y = current_config(1,3);

Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];

Tse_init = Tsb*Tb0*M0e;

Tsc_init = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];

Tce_standoff = [-1 0 0 0; 0 1 0 0; 0 0 -1 0.1; 0 0 0 1];

Tce_grasp = [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1];

Blist_arm = [0 0 0 0 0; 0 -1 -1 -1 0; 1 0 0 0 1; 0 -0.5076 -0.3526 -0.2176 0; 0.033 0 0 0 0; 0 0 0 0 0];


TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_standoff, Tce_grasp);

global intg_X_err;
intg_X_err = 0;



t_comp = csvread('trajectory.csv');

for i = 1:(size(t_comp,1)-1)
    T_details_curr = t_comp(i,:);
    r = [T_details_curr(1,1:3); T_details_curr(1,4:6); T_details_curr(1,7:9); 0 0 0];
    p = [T_details_curr(1,10);T_details_curr(1,11);T_details_curr(1,12);1];
    Tse_curr = [r p];
    
    
    T_details_next = t_comp(i+1,:);
    r = [T_details_next(1,1:3); T_details_next(1,4:6); T_details_next(1,7:9); 0 0 0];
    p = [T_details_next(1,10);T_details_next(1,11);T_details_next(1,12);1];
    Tse_next = [r p];
    
    
    T_details_curr_act = current_config(i,:);
    current_config(i,end) = T_details_curr(1,end);
    phi = T_details_curr_act(1,1);
    x = T_details_curr_act(1,2);
    y = T_details_curr_act(1,3);
    Blist_arm = [0 0 0 0 0; 0 -1 -1 -1 0; 1 0 0 0 1; 0 -0.5076 -0.3526 -0.2176 0; 0.033 0 0 0 0; 0 0 0 0 0];

    Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
    T0e_curr_act = FKinBody(M0e, Blist_arm,(current_config(i,4:8))');
    Tse_curr_act = Tsb*(Tb0*T0e_curr_act);
    
    curr_j_angles = current_config(i, 4:8);
    Jb_arm = JacobianBody(Blist_arm,curr_j_angles');
    
    Ve = FeedbackControl(Tse_curr_act, Tse_curr, Tse_next, 1, 0, del_t);
    current_vel = EndeffectorToCommands(Ve, current_config(i,:), Jb_arm,Tse_curr_act);
    new_config = NextState(current_config(i,:), current_vel, del_t, speed_lim);
    p = 1;
    while p
        [p, Jb_arm] = testJointLimits(new_config, Jb_arm);
        current_vel = EndeffectorToCommands(Ve, current_config(i,:), Jb_arm,Tse_curr_act);
        %disp(p);
        new_config = NextState(current_config(i,:), current_vel, del_t, speed_lim);
        break;
    end

    
        
       
    current_config = [current_config; new_config];
   
    %if mod(i,50) == 0
       %disp('Tse_curr');
       %disp(Tse_curr);
       %disp('Tse_next');
       %disp(Tse_next);
       %disp('Tse_curr_act');
       %disp(Tse_curr_act);
       %disp('T0e_curr_act');
       %disp(T0e_curr_act);
    %end
    
        
    
    %disp(current_config);
end

csvwrite('config.csv', current_config);
disp('Succesfully generated trajectory and Control inputs! Use CSV file for simulation');



