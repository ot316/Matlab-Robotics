
%rtbdemo % run the demo to get familiar with toolbox

%%% PRM %%%
load map1 % load map
goal = [50,30]; % goal point
start = [20, 10]; % start point
prm = PRM(map); % create navigation object
prm.plan('npoints', 150) % create roadmaps
prm.query(start, goal) % animate path from this start location


%%% RRT %%%
goal = [0,0,0];
start = [0,2,0];
veh = Bicycle('steermax', 1.2); % create an agent
rrt = RRT(veh, 'goal', goal, 'range', 5);
rrt.plan() % create navigation tree
rrt.query(start, goal) % animate path from this start location
%rrt.plot()
