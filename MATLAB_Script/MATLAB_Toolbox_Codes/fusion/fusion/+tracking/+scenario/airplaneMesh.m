function mesh = airplaneMesh
% TRACKING.SCENARIO.AIRPLANEMESH Mesh representation of an airplane 
% mesh = tracking.scenario.airplaneMesh returns an extendedObjectMesh which
% defines an airplane mesh to use with trackingScenario. 
%
% Example 1: Create and visualize the mesh
% ----------------------------------------
% % Visualize the mesh
% mesh = tracking.scenario.airplaneMesh;
% ax = axes('ZDir','reverse');
% show(mesh,ax);
%
% Example 2: Use the mesh for lidar simulation
% --------------------------------------------
% % Create the prebuilt airplane mesh
% mesh = tracking.scenario.airplaneMesh;
% 
% % Generate point cloud for an airplane in tracking scenario
% scene = trackingScenario;
% 
% % Create the tower
% tower = platform(scene);
% h = 50;
% tower.Trajectory.Position = [0 0 -h];
% tower.Dimensions = struct('Length',10,'Width',10,'Height',h,'OriginOffset',[0 0 -h/2]);
% tower.Sensors = monostaticLidarSensor('SensorIndex',1,...
%                                        'MaxRange',200,...
%                                        'HasINS',true,...
%                                        'DetectionCoordinates','scenario',...
%                                        'AzimuthLimits',[-75 75],...
%                                        'ElevationLimits',[-10 30]);
% 
% tower2 = platform(scene);
% h = 50;
% tower2.Trajectory.Position = [0 500 -h];
% tower2.Dimensions = struct('Length',10,'Width',10,'Height',h,'OriginOffset',[0 0 -h/2]);
% tower2.Sensors = monostaticLidarSensor('SensorIndex',2,...
%                                        'MaxRange',200,...
%                                        'HasINS',true,...
%                                        'DetectionCoordinates','scenario',...
%                                        'AzimuthLimits',[-75 75],...
%                                        'ElevationLimits',[-10 30]);
% 
% % Create the target airplane. 
% airplane = platform(scene);
% airplane.Mesh = mesh;
% 
% % Setting the dimensions will auto-scale the mesh
% airplane.Dimensions = struct('Length',40,...
%                               'Width',40,...
%                               'Height',12.5,...
%                               'OriginOffset',[0 0 12.5/2]);
% 
% % Create a landing trajectory for the plane
% x = 50*ones(10,1);
% y = linspace(-500,1000,10)';
% yToLand = max(0,-y);
% z = -1e4*(2.*(yToLand./50e3).^3 + 3*(yToLand./50e3).^2);
% wps = [x y z];
% toa = linspace(0,30,10)';
% traj = waypointTrajectory(wps,toa);
% airplane.Trajectory = traj;
% 
% % Display plotters
% lp = scatter3(nan,nan,nan,6,nan,'o','DisplayName','Lidar data');
% tp = theaterPlot('Parent',lp.Parent,...
%                   'XLimits',[0 100],...
%                   'YLimits',[-500 1000],...
%                   'ZLimits',[-75 0]);
% lp.Parent.ZDir = 'reverse';
% view(lp.Parent,169,5);
% pp = platformPlotter(tp,'DisplayName','Platforms','Marker','^');
% cp = coveragePlotter(tp,'DisplayName','Lidar coverage');
% hold on;
% 
% % Auto update rate
% scene.UpdateRate = 0;
% 
% % Advance simulation, generate data and visualize
% while advance(scene)
%   % Generate point cloud
%   ptCloud = lidarDetect(scene);  
%   
%   % Coverage configurations
%   cfgs = coverageConfig(scene);
%   
%   % Plot coverage
%   cp.plotCoverage(cfgs);
%   
%   % Plot platforms
%   platPoses = platformPoses(scene);
%   pos = vertcat(platPoses.Position);
%   mesh = cellfun(@(x)x.Mesh,scene.Platforms);
%   orient = vertcat(platPoses.Orientation);
%   pp.plotPlatform(pos,mesh,orient);
% 
%   % Concatenate all point clouds as they are all in the same coordinate
%   % system
%   s = vertcat(ptCloud{:});
%   % Plot lidar data
%   set(lp,'XData',s(:,1),...
%          'YData',s(:,2),...
%          'ZData',s(:,3),...
%          'CData',s(:,3));
%   drawnow;
% end
% 
% See also: extendedObjectMesh, trackingScenario, monostaticLidarSensor,
% lidarDetect

% Copyright 2020 The MathWorks, Inc.

persistent airplanemesh

if isempty(airplanemesh)
    % Create multiple parts and join them together
    airplanemesh = body;
    airplanemesh = join(airplanemesh,horizontalstabilizers);
    airplanemesh = join(airplanemesh,verticalstabilizer);
    airplanemesh = join(airplanemesh,landinggear);
    airplanemesh = join(airplanemesh,wings);
    airplanemesh = join(airplanemesh,jetengines);

    % Flip to Forward, Right and Downward coords
    airplanemesh = rotate(airplanemesh,[0 0 180]);
end

mesh = airplanemesh;

end

function mesh = body()

% Airplane body. Looking at an airplane from the side, values of concetric
% circles as x, z coordinate of the center and their respective radius
x = [-18 -12.5 -5 12 16.5];
z = [1.5 0.5 0 0 0 0];
r = [0.1 1.5 1.9 1.9 1.5];

% Smoothen the noise with an elliptical contour in x, z
theta = linspace(0,90,4);
xEnd = 16.5 + (19.8-16.5).*sind(theta);
rEnd = 0.01 + 1.49*cosd(theta);
x = [x xEnd(2:end)];
z = [z zeros(1,numel(xEnd)-1)];
r = [r rEnd(2:end)];

% n = number of vertices per circle
n = 10;
az = linspace(0,360,n);
v = zeros(0,3);

% Convert values of x,z and r into into circular vertices
for i = 1:numel(x)
    xi = x(i)*ones(1,n);
    ri = r(i);
    yi = ri*cosd(az);
    zi = ri*sind(az) + z(i);
    vi = [xi(:) yi(:) zi(:)];
    v = [v;vi]; %#ok<AGROW>
end

% Connecting two circles together to form the mesh
f = zeros(0,3);
k = 0;

% number of connections = numel(x) - 1;
for i = 1:(numel(x) - 1)
    fi = zeros(0,3);
    for z = (1:(n-1)) + k
        fi = [fi;z z+1 z+1+n;z+1+n z+n z]; %#ok<AGROW>
    end
    k = k + n;
    f = [f;fi]; %#ok<AGROW>
end

% Create the mesh
mesh = extendedObjectMesh(v,f);

end

function mesh = horizontalstabilizers
v = [-18 0.1 1.5;
    -14 0.1 1.5;
    -18 6 3.5;
    -19 6 3.5];

f = [1 2 3;3 4 1];

up = extendedObjectMesh(v,f);
down = translate(up,[0 0 -0.2]);

% Normals facing downwards
down.Faces = [1 3 2;3 1 4];
leftWing = join(up,down);

% Fill the gap by supplying more faces
fCover = [1 5 6;6 2 1;2 6 7;7 3 2;3 7 8;8 4 3;1 4 8;8 5 1];
leftWing.Faces = [leftWing.Faces;fCover];

% Create the right wing
rightWing = leftWing;
rightWing.Vertices(:,2) = -rightWing.Vertices(:,2);

% Correct the normals
f = rightWing.Faces;
rightWing.Faces(:,2:3) = [f(:,3) f(:,2)];
mesh = join(leftWing,rightWing);
end


function mesh = jetengines
% Each engine is defined using a hollow cylinder with a thickness
cylIn = extendedObjectMesh('cylinder');
cylOut = scale(cylIn,[1.4 1.4 1]);

% Flip normals inside
cylIn.Faces(:,2:3) = [cylIn.Faces(:,3) cylIn.Faces(:,2)];

% Join in and out
cylinder = join(cylIn,cylOut);

% Create faces to join the cylinders
fJoin = zeros(0,3);
for i = 1:1:20
    f1 = i;
    f2 = f1 + 2;
    f3 = f2 + 22;
    f4 = f1 + 22;
    if mod(i,2) == 1
        fi = [f1 f2 f3;f3 f4 f1];
    else
        fi = [f1 f3 f2;f3 f1 f4];
    end
    fJoin = [fJoin;fi];
end

% Flip normals of back side

cylinder.Faces = [cylinder.Faces;fJoin];

cylinder = scale(cylinder,[1 1 5]);
jet1 = rotate(cylinder,[0 90 0]);
jet1 = translate(jet1,[3 -5 0]);

jet2 = jet1;
jet2.Vertices(:,2) = -jet2.Vertices(:,2);

mesh = join(jet1,jet2);

end

function mesh = verticalstabilizer
v = [-18 -0.5 1.55
    -12.5 -0.5 1.25
    -7.5 -0.5 1.1
    -12.6 -0.5 4
    -17.5 -0.5 10
    -19.6 -0.5 10];

f = [1 2 4;2 3 4;1 4 5;1 5 6];

right = extendedObjectMesh(v,f);
left = translate(right,[0 1 0]);

% Correct normals
f = left.Faces;
left.Faces(:,2:3) = [f(:,3) f(:,2)];

% Join left and right to create the mesh
mesh = right;
mesh = join(mesh,left);

fCover = lidarsim.internal.mesh.utilities.f4Tof3([3 9 10 4;10 11 5 4;11 12 6 5;12 7 1 6]);
mesh.Faces = [mesh.Faces;fCover];
end

function mesh = landinggear()
% Create a full cylinder
cylinder = extendedObjectMesh('cylinder');
dims = struct('Length',0.2,'Width',0.2,'Height',2.5,'OriginOffset',[0 0 1]);
holder = scaleToFit(cylinder,dims);

% Full cylinder
bottom = 1:2:20;
top = 2:2:20;
f = lidarsim.internal.mesh.utilities.fNTof3(top);
cylinder.Faces = [cylinder.Faces;f];
f = lidarsim.internal.mesh.utilities.fNTof3(bottom);
% Normals outside
f(:,2:3) = [f(:,3) f(:,2)];
cylinder.Faces = [cylinder.Faces;f];

wheel = rotate(cylinder,[0 0 90]);
dims = struct('Length',2,'Width',0.4,'Height',2,'OriginOffset',[0 0 0]);
wheel = scaleToFit(wheel,dims);
wheel = translate(wheel,[0 0 -2]);
mesh = join(holder,wheel);

leftWheel = translate(mesh,[1 -4 0]);
rightWheel = translate(mesh,[1 4 0]);

rearWheels = join(leftWheel,rightWheel);

frontWheel = scale(mesh,0.8);
frontWheel = translate(frontWheel,[16.5 0 -1]);
mesh = join(rearWheels,frontWheel);

end

function mesh = wings()

% Vertices of the upper side of a wing
v = [-1.4 -1.5 1.2;
     -1.4 -5 1.2;
     -4.7 -17 1.2;
     -3.6 -17 1.2;
        5 -1.5 1.2];
    
% Face order
f = [1 2 5;
     2 3 4;
     2 4 5];

% Create a mesh
leftUp = extendedObjectMesh(v,f);

% Downside of the left wing
vDown = v;

% Downward slanting towards inner side.
% Drop of 0.6 meter in 15.5 meters. 
% In 12 meters, that's 0.6/15.5*12 = 0.4645
vDown(:,3) = vDown(:,3) - [1 0.8645 0.4 0.4 1]';

% Normals facing down
fDown = [1 5 2;2 4 3;2 5 4];

% Left down mesh
leftDown = extendedObjectMesh(vDown,fDown);

% Join to create the wing
leftWing = join(leftDown,leftUp);

% Join the up and down part by supplying more faces in the mesh
fCover = zeros(0,3);
for i = 1:4
    fCover = [fCover;i i+1 i+1+5;i+5+1 i+5 i]; %#ok<AGROW>
end
leftWing.Faces = [leftWing.Faces;fCover];

% Mirror left wing to create right wing
rightWing = leftWing;
rightWing.Vertices(:,2) = -rightWing.Vertices(:,2);
rightWing.Faces(:,2:3) = [rightWing.Faces(:,3) rightWing.Faces(:,2)];

% Join left and ring wings to create wings
mesh = join(rightWing,leftWing);

end


