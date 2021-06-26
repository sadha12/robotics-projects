function TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_standoff, Tce_grasp)

%Go to standoff pose1
Tse_init_goal = Tsc_init*Tce_standoff;
traj1 = CartesianTrajectory(Tse_init,Tse_init_goal,8,800,3);
T1_col = [];
for i= 1:length(traj1)
    r = traj1{i}(1:3,1:3);
    p = traj1{i}(1:3,4);
    r =r';
    r = r(:);
    T1_col = [T1_col; [r' p'] 0];
end

%Go to grasp pose1
Tse_init_grasp = Tsc_init*Tce_grasp;
traj2 = ScrewTrajectory(Tse_init_goal,Tse_init_grasp,3,300,3);
T2_col = [];

for i= 1:length(traj2)
    r = traj2{i}(1:3,1:3);
    p = traj2{i}(1:3,4);
    r =r';
    r = r(:);
    T2_col = [T2_col; [r' p'] 0];
end

%Close the fingers
T3_col=[];
for i=1:65
    T3_col = [T3_col; T2_col(end,1:end-1) 1];
end

%Go to standoff after grasp
traj4 = ScrewTrajectory(Tse_init_grasp,Tse_init_goal,3,300,3);
T4_col = [];

for i= 1:length(traj4)
    r = traj4{i}(1:3,1:3);
    p = traj4{i}(1:3,4);
    r =r';
    r = r(:);
    T4_col = [T4_col; [r' p'] 1];
end

%Go to standoff for drop
Tse_final_standoff = Tsc_final*Tce_standoff;
traj5 = ScrewTrajectory(Tse_init_goal,Tse_final_standoff,8,800,3);
T5_col = [];

for i= 1:length(traj5)
    r = traj5{i}(1:3,1:3);
    p = traj5{i}(1:3,4);
    r =r';
    r = r(:);
    T5_col = [T5_col; [r' p'] 1];
end

%Go to drop pose final
Tse_final_drop = Tsc_final*Tce_grasp;
traj6 = ScrewTrajectory(Tse_final_standoff,Tse_final_drop,3,300,3);
T6_col = [];

for i= 1:length(traj6)
    r = traj6{i}(1:3,1:3);
    p = traj6{i}(1:3,4);
    r =r';
    r = r(:);
    T6_col = [T6_col; [r' p'] 1];
end

%Open gripper
T7_col=[];
for i=1:65
    T7_col = [T7_col; T6_col(end,1:end-1) 0];
end

%Go to standoff
traj8 = ScrewTrajectory(Tse_final_drop,Tse_final_standoff,3,300,3);
T8_col = [];

for i= 1:length(traj8)
    r = traj8{i}(1:3,1:3);
    p = traj8{i}(1:3,4);
    r =r';
    r = r(:);
    T8_col = [T8_col; [r' p'] 0];
end



t_comp = [T1_col; T2_col; T3_col; T4_col; T5_col; T6_col; T7_col; T8_col];

csvwrite('trajectory.csv', t_comp);
end


    