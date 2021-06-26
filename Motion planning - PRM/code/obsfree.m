function chk = obsfree(x_chk, y_chk, current_x, current_y)
m = (current_y - y_chk)/(current_x - x_chk);
c = y_chk - m*x_chk;
chk =1;
obs = csvread('obstacles.csv');
for k = 1:length(obs)
    a = obs(k,1);
    b = obs(k,2);
    r = obs(k,3)/2;
    rts = roots([(1+(m*m)) (2*m*c - 2*m*b - 2*a) (a*a + b*b + c*c - r*r - 2*b*c)]);
    %disp(rts);
    if isreal(rts)
        xr1 = rts(1,1);
        yr1 = m*xr1 + c;
        xr2 = rts(2,1);
        yr2 = m*xr2 + c;
        if (xr1 > min(x_chk, current_x)&& xr1<max(x_chk, current_x)) || (xr1 > min(x_chk, current_x)&& xr2<max(x_chk, current_x))
            chk = 0;
        end
    end
end
        
        