current_config = csvread('config.csv');
current_config = current_config(1,:);
disp(current_config);
arm_vel = 0.25*ones(1,5);
base_vel = 10*[0 0 0 0];

current_vel = [arm_vel base_vel];

arm_lim = ones(1,5);
base_lim = 12.5*ones(1,4);

speed_lim = [arm_lim base_lim];

del_t = 0.01;

for i=1:0.01:5
    new_config = NextState(current_config(end,:), current_vel, del_t, speed_lim);
    current_config = [current_config; new_config];
end
csvwrite('config2.csv', current_config);