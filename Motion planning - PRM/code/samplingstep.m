function nodes = samplingstep()
obs = csvread('obstacles.csv');
x = [-0.5];
y = [-0.5];

while length(x)<19
    x1 = -0.49 + (0.98)*rand([1,1]);
    y1 = -0.49 + (0.98)*rand([1,1]);
    k =1;
    for i = 1:length(obs)
        dist = ((x1 - obs(i,1))^2 + (y1 - obs(i,2))^2 - (obs(i,3)/2)^2);
        if dist < 0
            k=0;
            break;
        end
    end
    if k==1
        x = [x;x1];
        y = [y;y1];
    end
end

x = [x; 0.5];
y = [y; 0.5];
n = [1:20]';
h_dist = sqrt((x-0.5).^2 + (y-0.5).^2);
nodes = [n x y h_dist];


end


