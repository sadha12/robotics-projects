function [chk, J_arm] = testJointLimits(new_config, Jb_arm)
chk =0;

if new_config(1,6)<-1.7
    Jb_arm(:,3) = zeros(6,1);
    
    chk =1;
end



if new_config(1,7)<-1.7
    Jb_arm(:,4) = zeros(6,1);
    
    chk =1;
end



J_arm = Jb_arm;
end
