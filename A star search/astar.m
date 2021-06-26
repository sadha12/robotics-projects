function astar()
ndetails = csvread('nodes.csv');
e = csvread('edges.csv');

h_cost = ndetails(:,4);     
cost = zeros(length(ndetails));     
cost = cost -1;

for i=1:size(e,1)
    cost(floor(e(i,1)),floor(e(i,2))) = e(i,3);
    if i<=12
        cost(i,i)=0;
    end
    
end

past_cost = zeros(length(ndetails),1);
past_cost = past_cost + 1000;
past_cost(1,1) =0;

est_cost = past_cost;
est_cost(1,1) =0;
est_cost = past_cost + h_cost;

est_mat = [ndetails(:,1) est_cost];
%disp(est_mat(:,2));
open1 = zeros(size(sortrows(est_mat, 2)));

closed = [];

parent = zeros(size(ndetails,1),1);

goal = length(cost);
open1 = [ndetails(:,1) est_cost];
open1 = sortrows(open1, 2);

while ~isempty(open1)
    current_node = open1(1,1);
    %disp(current_node);
    if current_node == goal
        break;
    end
    
    for nbr = 1:size(cost,1)
        if cost(nbr,current_node)>0
            current_nb_cost = past_cost(current_node) + cost(nbr,current_node);
            %disp(nbr);
            
             
            if current_nb_cost < past_cost(nbr)
                past_cost(nbr) = current_nb_cost;
                est_cost(nbr) = past_cost(nbr) + h_cost(nbr);
                parent(nbr) = current_node;
                %disp(est_cost(nbr));
                for h = 1: size(open1,1)
                    if open1(h,1) == nbr
                       open1(h,2) = est_cost(nbr);
                    end
                end
            end
        end
        
    
    end
    closed = [closed current_node];
    %disp(est_mat(:,2));
    open1 = sortrows(open1, 2);
    open1 = open1(2:end,1:end);
    %disp(open1);
    
    
    
end
if current_node == goal
    path = current_node;
while parent(path(1,end))~=0
    k = path(1,end);
    %disp(k);
    path = [path parent(k)];
end
    
path = flip(path);    

csvwrite('path.csv',path);
end

