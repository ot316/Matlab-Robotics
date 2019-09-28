
rtbdemo % run the demo to get familiar with toolbox
%return

%%% PRM %%%
load house % load map
goal = place.kitchen; % goal point
start = place.br3; % start point
prm = PRM(house); % create navigation object
prm.plan('npoints', 150) % create roadmaps
prm.query(start, goal) % animate path from this start location
figure;
prm.plot();
%return

% RRT
load road; % load the map
car = Bicycle('steermax', 0.5); % create the agent
rrt = RRT(car, road, 'npoints', 3000, 'root', [50 22 0], 'simtime', 4);
rrt.plan()
p = rrt.query([40 45 0], [50 22 0]);
about p
figure;
rrt.plot(p)
plot_vehicle(p, 'box', 'size', [20 30], 'fill', 'r', 'alpha', 0.1);
