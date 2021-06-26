function edgecalc()
nodes = csvread('nodes.csv');
m = nodes(:,1:3);
n=[];

for i = length(m):-1:1
    
    
    
    s = size(n,1);
    current_x = m(i,2);
    current_y = m(i,3);
    curr_dist = sqrt((m(:,2) - current_x).^2 + (m(:,3) - current_y).^2);
    curr_dist = [m(:,1) curr_dist];
    %disp(curr_dist);
    curr_dist = sortrows(curr_dist,2);
    %disp(curr_dist);
    j = 2;
    while  (size(n,1) < s+3) && (j <= length(curr_dist))
        flag = 1;
        %disp('in while');
        %disp(i);
        x_chk = nodes(curr_dist(j,1),2);
        y_chk = nodes(curr_dist(j,1),3);
        chk = obsfree(x_chk, y_chk, current_x, current_y);
        for o = 1:size(n,1)
            if (i == n(o,2)) && (curr_dist(j,1) == n(o,1))
                flag = 0;
                break;
            end
        end
        if flag && chk
            n = [n ; [i curr_dist(j,:)] ];
            
            %disp(n);
        end
        j=j+1;
    end
    %disp('out from while');
    
    
end
csvwrite('edges.csv',n);
end

        