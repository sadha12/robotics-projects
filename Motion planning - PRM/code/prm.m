function prm()
nodes = samplingstep();
csvwrite('nodes.csv',nodes);

edgecalc();
astar();
end