function chk = formclosure(M)
chk = 0;
v = M(:,1:2);
v = [v zeros(size(v,1),1)];
v = v';
n = [cos(M(:,3)) sin(M(:,3)) zeros(size(v,2),1)];
n =n';
F = [];
for i = 1:size(v,2)
    curr_v = v(:,i);
    so3_v_curr = VecToso3(curr_v);
    m = so3_v_curr * n(:,i);
    f = n(:,i);
    F_curr = [m(3,1);f(1:2,1)];
    F = [F F_curr];
end
f = ones(size(F,2),1);
Aeq = F;
beq = zeros(3,1);
b = -1*ones(size(F,2),1);
A = -1*eye(size(F,2));
[k, t, flag] = linprog(f,A,b,Aeq,beq);
if rank(F) == 3 && all(k,'all') && flag ==1
    disp('k=');
    disp(k);
    disp('Closure Binary');
    chk = 1;
end
disp(chk);
end


    
    
