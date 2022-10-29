% triangulateLOS Triangulate multiple line-of-sight(LOS) detections
%
% Multiple angle-only or line-of-sight measurements result in lines in
% space, which may or may not intersect because of measurement noise. The
% function uses a sub-optimal linear least-squares method to minimize the
% distance of miss between multiple detections. The least-squares
% formulation inherently assumes that all detections report measurements
% with approximately same accuracy in azimuth and elevation (if measured).
% It also assumes that distances of sensors from the triangulated target
% are of the same order.
%
% For diverging angle-only measurements, the distance is minimum at the
% center of positions of sensors. Hence, the estimated position is returned
% as the center of sensor positions.
%
% estPos = triangulateLOS(detections) estimates the position of the target
% in global Cartesian coordinate frame by triangulating the set of
% angle-only detections provided as an input.
%
% [estPos,estCov] = triangulateLOS(detections) also provides the error
% covariance of the target position. The covariance of the error in 
% position is determined by using Taylor-series approximations.
%
% estPos is a 3-by-1 vector denoting the estimated position of the target.
%
% estCov is a 3-by-3 matrix denoting the estimated error covariance of the 
% target position.
%
% detections is a cell array of objectDetection objects containing
% angle-only measurement. Each detection in the cell array must define the
% measurement parameters, as an array of struct with the following fields.
% Default values are used if a field is missing.
%
%      Frame           - either 'rectangular' or 'spherical' or an enum
%                        with the same values. The first structure must
%                        specify the Frame as 'spherical' or enum with the
%                        same value.
%      OriginPosition  - a 3-by-1 real vector.
%      OriginVelocity  - a 3-by-1 real vector.
%      Orientation     - a 3-by-3 orthonormal orientation matrix.
%      HasAzimuth      - a logical scalar, true if azimuth is measured.
%                        Default: true, must be set to true if specified as
%                        a field.
%      HasElevation    - a logical scalar, true if elevation is measured.
%                        Default: true, can be set to true if elevation is
%                        measured.
%      HasVelocity     - a logical scalar, true if velocity is measured.
%                        Default: false, must be set to false if specified
%                        as a field.
%      HasRange        - a logical scalar, true if range is measured.
%                        Default: true, must be specified as a field and
%                        set to false.
%      IsParentToChild - a logical scalar, true if the orientation is
%                        given as a parent to child frame rotation.
%  
%   Example: Triangulate Line-of-sights measurements from 3 sensors.
%   
%   % Load line-of-sight detections from a MAT file
%   load('angleOnlyDetectionFusion.mat','detectionSet')
%   
%   % Visualize angle-only detections
%   % range for plotting direction vector
%   rPlot = 5000;
%   plotData = zeros(3,numel(detectionSet)*3);
%   for i = 1:numel(detectionSet)
%       az = detectionSet{i}.Measurement(1);
%       el = detectionSet{i}.Measurement(2);
%       [xt,yt,zt] = sph2cart(deg2rad(az),deg2rad(el),rPlot);
%       % The sensor is co-located at platform center, therefore use
%       % the position from the second measurement parameter
%       originPos = detectionSet{i}.MeasurementParameters(2).OriginPosition;
%       positionData(:,i) = originPos(:);
%       plotData(:,3*i-2) = [xt;yt;zt] + originPos(:);
%       plotData(:,3*i-1) = originPos(:);
%       plotData(:,3*i) = [NaN;NaN;NaN];
%   end
%   plot3(plotData(1,:),plotData(2,:),plotData(3,:),'r-');
%   hold on;
%   plot3(positionData(1,:),positionData(2,:),positionData(3,:),'o','MarkerSize',12,'MarkerFaceColor','g');
%   
%   % Triangulate detections using TRIANGULATELOS
%   [estPos, estCov] = TRIANGULATELOS(detectionSet);
%   
%   plot3(estPos(1),estPos(2),estPos(3),'ko','MarkerSize',12, 'MarkerFaceColor','k');
%   legend('Angle-only Detections','Sensor Position','Triangulated Position');
%   title('Line-of-sights Detection Triangulation'); xlabel('x [m]'); ylabel('y [m]')
%   view(2);
%
%   See also: staticDetectionFuser, objectDetection.

 
% Copyright 2018 The MathWorks, Inc.

