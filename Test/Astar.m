function [isSuccess] = Astar(imgPath, startX, startY, goalX, goalY)
%ASTAR Summary of this function goes here
%   Detailed explanation goes here
    % Load the map of the environment
    isSuccess = true;
    image = imread(imgPath);
%     imageCropped = image(40:140, 2:117);
    imageCropped = image;
    imshow(imageCropped);
    
    imageBW = imageCropped < 100;
    imshow(imageBW);
    
    figure;
    tb3map = binaryOccupancyMap(imageBW);
    
    
    % Define parameters
    vMap = validatorOccupancyMap;
    vMap.Map = tb3map;
    
    % Astar planner
    planner = plannerHybridAStar(vMap, 'MinTurningRadius', 2.0);
    
    % Set start and goal points
    start = [startX startY 0];
    goal = [goalX goalY 0];

%     start = [63, 77 0];
%     goal = [100, 65 0];
    
    % Plan the path
    route = plan(planner, start, goal);
    route = route.States;
    
    % Get poses from the route.
    rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
    startPoses = route(1:end-1,:);
    endPoses = route(2:end,:);
    
    rsPathSegs = connect(rsConn, startPoses, endPoses);
    poses = [];
    for i = 1:numel(rsPathSegs)
        lengths = 0:0.1:rsPathSegs{i}.Length;
        [pose, ~] = interpolate(rsPathSegs{i}, lengths);
        poses = [poses; pose];
    end

%     disp(poses)
    
    % Display the map
    figure
    show(planner)
    title('A* path')
end

