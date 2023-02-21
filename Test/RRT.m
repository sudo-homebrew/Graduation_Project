function [isSuccess] = RRT(imgPath, startX, startY, goalX, goalY)
%RRT Summary of this function goes here
%   Detailed explanation goes here
    isSuccess = true;
    image = imread(imgPath);
    imageCropped = image;
    
    imageBW = imageCropped < 100;

    tb3map = binaryOccupancyMap(imageBW);

    start = [double(startX), double(startY), 0];
    goal = [double(goalX), double(goalY), 0];
    

    
    bounds = [tb3map.XWorldLimits; tb3map.YWorldLimits; [-pi pi]];
    
    ss = stateSpaceDubins(bounds);
    ss.MinTurningRadius = 10;
    
    stateValidator = validatorOccupancyMap(ss); 
    stateValidator.Map = tb3map;
    stateValidator.ValidationDistance = 0.05;
    
    planner = plannerRRT(ss, stateValidator);
    planner.MaxConnectionDistance = 2.0;
    planner.MaxIterations = 20000;
    
    
    [pthObj, ~] = plan(planner, start, goal);
    

    interpolate(pthObj,300)

    
    Path = pthObj.States(:, 1:2);
    Path(:, 1) = Path(:, 1) - startX;
    Path(:, 2) = Path(:, 2) - startY;

    %%%%%%%%%%%%    Path Smoothing    %%%%%%%%%%%%

    S_Path = Path(1:10:end, :);

%     disp(S_Path)
    
    filename = [pwd, '/Path/smooded_path.csv'];

    writematrix(S_Path, filename)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    filename = [pwd, '/Path/path.csv'];

    writematrix(Path, filename)

end

