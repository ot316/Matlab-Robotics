
%Oli Thompson Matlab Tutorial 4 Submission

%%% PRM %%%
image = imread('World_map.jpg');
imageBW = im2bw(image,0.8);

goal = [200,400]; % goal point
start = [400,100]; % start point
prm = PRM(imageBW); % create navigation object
prm.plan('npoints', 150) % create roadmaps
prm.query(start, goal) % animate path from this start location
figure;
prm.plot();
axis equal


% RRT

car = Bicycle('steermax', 0.9,'speedmax',1, 'L', 3, 'rdim', 2); % create the agent with specified size and max steering speed
rrt = RRT(car, imageBW, 'npoints', 1000,'root', [250 300 0], 'simtime', 2);
rrt.plan('ntrials', 100)
p = rrt.query([250 300 0], [350 200 0]);
about p
figure;
rrt.plot(p)
%plot_vehicle(p, 'box', 'size', [2 3], 'fill', 'r', 'alpha', 0.1); % plot the car with the same size as created
