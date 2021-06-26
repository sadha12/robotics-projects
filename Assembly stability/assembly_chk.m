function chk = assembly_chk()
M = csvread('mass.csv');
C = csvread('contacts.csv');
chk =1;
F_ext = [];
for i = 1:size(M,1)
    com_so3 = VecToso3([M(i,1); M(i,2); 0]);
    f_ext= [0; M(i,3)*9.81; 0];
    m_ext = com_so3*f_ext;
    w_ext = [m_ext(3,1); f_ext(1:2,1)];
    F_ext = [F_ext w_ext];
end

F =[];
for i = 1:size(C,1)
    so3_v_curr = VecToso3([C(i,3); C(i,4); 0]);
    N1 = [cos( C(i,5) + atan(C(i,6)) ) ; sin(C(i,5)+atan(C(i,6))); 0];
    N2 = [cos(C(i,5)-atan(C(i,6))) ; sin(C(i,5)-atan(C(i,6))); 0];
    m1 = so3_v_curr*N1;
    m2 = so3_v_curr*N2;
    F1 = [m1(3,1); N1(1:2,1)];
    F2 = [m2(3,1); N2(1:2,1)];
    F = [F F1 F2];
end

for i=1:size(C,1)
    if C(i,1)==1 || C(i,2) ==1
        F_curr = F(:,1:2*i);
    end
end
%disp(F_curr);
f = ones(size(F_curr,2),1);
Aeq = F_curr;
beq = F_ext(:,1);
b = -1*ones(size(F_curr,2),1);
A = -1*eye(size(F_curr,2));
[s, t, flag] = linprog(f,A,b,Aeq,beq);
k = [s];
if flag ~=1
    chk =0;
    disp('Assembly not stable');
    return;
end
        

F_curr = F_curr*k;
F(:,1:size(F_curr,2)) = F_curr;
F_curr =[];
F_prev =[];

for i=2:size(M,1)
    for j = 1:size(C,1)
        if C(j,1) == i
            F_curr = [F_curr F(:,(2*j)-1:2*j)];
        end
        if C(j,2) == i
            F_prev = [F_prev F(:,(2*j)-1:2*j)];
        end
    end
    F_ext_curr = F_ext(:,i) - F_prev;
    f = ones(size(F_curr,2),1);
    Aeq = F_curr;
    beq = F_ext_curr(:,i);
    b = -1*ones(size(F_curr,2),1);
    A = -1*eye(size(F_curr,2));
    [s, t, flag] = linprog(f,A,b,Aeq,beq);
    k = [k s];
    if flag ~=1
        chk =0;
        disp('Assembly not stable');
        break;
    end
    %disp(s);
    %disp(F_curr);
    F_curr = F_curr*s;
    F(:,1:size(F_curr,2)) = F_curr;
    F_curr =[];
end
if chk ==1
    disp('Assembly stable');
end
    
end