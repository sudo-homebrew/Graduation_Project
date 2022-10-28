classdef trackingGlobeViewer< handle
%trackingGlobeViewer Virtual globe for tracking scenario visualization
% viewer = trackingGlobeViewer creates a trackingGlobeViewer, viewer.
% viewer = trackingGlobeViewer(fig) creates a trackingGlobeViewer,
% viewer, in the uifigure fig.
% viewer = trackingGlobeViewer(..., 'Name', value) creates a
% trackingGlobeViewer object by specifying its properties as
% name-value pair arguments. Unspecified properties have
% default values. See the list of properties below.
%
% trackingGlobeViewer properties:
%  ReferenceLocation       - Reference location for non Earth-centered
%                            scenarios (Constrution Only)
%  CoverageMode            - Option for sensor coverage display
%  CoverageRangeScale      - Coverage range scaling factor
%  NumCovarianceSigma      - Covariance ellipse size in number of sigma
%  PlatformHistoryDepth    - Length of platform trajectory history line
%  TrackHistoryDepth       - Length of track history line
%  TrackLabelScale         - Track label scaling factor
%  ShowDroppedTracks       - Show dropped tracks on the globe
%  Basemap                 - Basemap of the globe (Construction Only)
%  Terrain                 - Terrain of the globe (Construction Only)
%
% trackingGlobeViewer methods:
%  plotScenario            - plots a tracking scenario object
%  plotPlatform            - plots platforms
%  plotTrajectory          - plots trajectory objects
%  plotCoverage            - plots sensor coverages
%  plotDetection           - plots detection objects
%  plotTrack               - plots tracks
%  clear                   - clears the globe viewer
%  snapshot                - creates a snapshot of the globe viewer
%  campos                  - controls camera position
%  camorient               - controls camera orientation
%
% % Example 1:
% viewer = trackingGlobeViewer;
%
% % move camera above Paris, France
% parisLLA = [48.85 2.32 1.2e4];
% campos(viewer, parisLLA);
%
% % plot the trajectory of a boat on the Seine river
% coords = [48.829 2.386 0 ; 48.841 2.373 0; 48.849 2.360 0; ...
%     48.857 2.345 0 ; 48.861 2.328 0 ; 48.863 2.317 0 ; 48.863 2.298 0;...
%     48.858 2.290 0 ; 48.852 2.283 0 ; 48.846 2.275 0; 48.833 2.262 0];
% boattraj = geoTrajectory(coords, linspace(0,2*3600,11));
% plotTrajectory(viewer, boattraj);
%
% % Example 2:
%
% % Visualize a tracking scenario
% load('ATCScenario.mat')
%
% % This scenario is using local coordinates. Define a reference location to
% % visualize it
% refloc = [42.363 -71 0];
% viewer = trackingGlobeViewer('ReferenceLocation',refloc);
%
% % Simulate scenario and visualize on the globe
% while advance(scenario)
%     detections = detect(scenario);
%     plotScenario(viewer, scenario, detections);
% end
%
% See also: theaterPlot, trackingScenario

 
%   Copyright 2021 The MathWorks, Inc.

    methods
        function out=trackingGlobeViewer
        end

        function out=camorient(~) %#ok<STOUT>
            %camorient Set or query camera orientation
            % orient = camorient(viewer) returns the current orientation of
            % the camera as a 3-element vector, orient, of the form
            % [heading, pitch, roll].
            %
            % camorient(viewer, orientation) sets the orientation of
            % the camera for the specified trackingGlobeViewer. orientation
            % is specified as a 1-by-3 vector of degree angle rotations [heading,
            % pitch, roll].
            %
            % Example
            % viewer = trackingGlobeViewer;
            % orient = [250 -44 0];
            % camorient(viewer, orient);
        end

        function out=campos(~) %#ok<STOUT>
            %campos Set or query camera position
            % pos = campos(viewer) returns the current position of the
            % camera as a 3-element vector of the form [lat, lon, height].
            %
            % campos(viewer, lat, lon) sets the latitude and longitude of
            % the camera for the specified trackingGlobeViewer.
            %
            % campos(viewer, lat, lon, height) additionally sets the height
            % of the camera.
            %
            % campos(viewer, [lat, lon, height]) is equivalent to the above
            % definition.
            %
            % Example
            % % Move camera position to Boston, Massachusetts
            % viewer = trackingGlobeViewer;
            % pos = [42.33598, -71.03103 1.5e4];
            % campos(viewer, pos);
        end

        function out=clear(~) %#ok<STOUT>
            %clear Clear graphics on the globe
            % clear(viewer) removes all graphics from the globe
        end

        function out=delete(~) %#ok<STOUT>
            %delete delete trackingGlobeViewer
        end

        function out=plotCoverage(~) %#ok<STOUT>
            %plotCoverage plot coverage configuration
            % plotCoverage(viewer, configs) plots an array of coverageConfig
            % structs onto the viewer.
            %
            % plotCoverage(..., ref) specifies the reference frame used to
            % interpret the 'Position' field of the config struct. Choose
            % between 'NED' (default), 'ENU', and 'ECEF'. When using 'NED'
            % or 'ENU', the ReferenceLocation property of the viewer is
            % used as the origin of the frame.
            %
            % When CoverageMode is set to 'Beam', only the sensor/emitter
            % beam is plotted. When CoverageMode is set to 'Coverage', both
            % beam and coverage are plotted.
            %
            % plotCoverage(..., 'Name', value) also lets you specify
            % additional plotting options for the coverage surface:
            %
            %  Color          - unique beam and coverage color. Specified
            %                   as an N-by-3 matrix of RGB triplets.
            %  Alpha          - coverage transparency, specified as a
            %                   scalar between 0 and 1.
            %
            % % Example
            % % Plot coverage in ECEF coordinates
            % scene = trackingScenario('IsEarthCentered',true);
            % r = fusionRadarSensor(1);
            % radarTowerLLA = [10 10 1000];
            % platform(scene,'Position',radarTowerLLA,'Sensors',r);
            % % Call coverageConfig on scenario to obtain the config struct
            % covcon = coverageConfig(scene);
            % viewer = trackingGlobeViewer;
            % % Plot Coverage
            % plotCoverage(viewer, covcon, 'ECEF', 'Color', [1 0 0]);
            %
            % See also: coverageConfig
        end

        function out=plotDetection(~) %#ok<STOUT>
            %plotDetection Plot point detections on the viewer
            % plotDetection(viewer, detections) plots a cell array of
            % objectDetection, detections, on the globe.
            %
            % plotDetection(..., ref) lets you specify the reference frame
            % for the detection position coordinates. Choose between 'NED'
            % (default), 'ENU', and 'ECEF'. When using 'NED' or 'ENU', the
            % ReferenceLocation property of the viewer is used as the
            % origin of the frame.
            %
            % plotDetection(..., 'Name', value) also lets you specify
            % additional plotting options using Name-Value pairs:
            %
            %     Color       - Color of the detection marker and
            %                   covariance ellipse, specified as a 1-by-3
            %                   RGB triplet (unique color for all tracks)
            %                   or a N-by-3 matrix of RGB triplets. N is
            %                   the number of unique SensorIndex in the
            %                   detections.
            %
            % %Example
            % refloc = [42.366978, -71.022362, 50];
            % viewer = trackingGlobeViewer('ReferenceLocation',refloc);
            % campos(viewer, 42.3374,  -71.0605,  872.7615);
            % camorient(viewer,[39 0 -2.7]);
            %
            % % plot a cartesian measurement from a sensor located at
            % % coordinates [1000 0 0] in local NED axes centered at the ReferenceLocation. The
            % % sensor measurement is reported in sensor centered NED axes.
            % det1 = objectDetection(0,[0 ; 100; -300],'MeasurementParameters',struct('Frame','rectangular',...
            %     'OriginPosition',[1000 0 0])); % Sensor origin with respect to Parent axes
            % plotDetection(viewer, det1, 'NED'); % sensor Parent axes are NED
            %
            %
            % % plot a cartesian measurement [0 100 350] from a sensor located at
            % % coordinates [1000 0 0] in a local ENU frame at the ReferenceLocation. The
            % % sensor reports measurements in its own ENU frame.
            % det2 = objectDetection(0,[0 ; 100; 350],'MeasurementParameters',...
            %     struct('Frame','rectangular',... cartesian measurement
            %     'OriginPosition',[1000 0 0],... sensor own frame origin
            %     'Orientation',eye(3)),...
            %     'SensorIndex',2); % sensor own frame ENU already has the same orientation as the local ENU frame
            % plotDetection(viewer, det2,'ENU','Color',[0.5 0.5 0]); % Sensor Parent axes are ENU
            %
            %
            % % plot a cartesian measurement reported in ECEF coordinates [1.5349 -4.4634 4.2761]*1e6
            % det3 = objectDetection(0,[1.5349; -4.4634; 4.2761]*1e6,'MeasurementParameters',...
            %     struct('Frame','rectangular',... cartesian measurement
            %     'OriginPosition',[0 0 0],... sensor own frame origin
            %     'Orientation',eye(3)),...
            %     'SensorIndex',3);
            % plotDetection(viewer, det3,'ECEF','Color',[0 0.5 0]); % Sensor Parent axes are ECEF
            %
            % See also: objectDetection
        end

        function out=plotPlatform(~) %#ok<STOUT>
            %plotPlatform Plot platforms on the globe
            % plotPlatform(viewer, platforms) plots tracking scenario
            % platforms on the trackingGlobeViewer viewer, specified as an
            % array of platform objects. 
            % 
            % plotPlatform(viewer, platstructs) plots platform on the globe, 
            % specified as an array of structs. Each struct must contain
            % the fields 'Position' and 'PlatformID'.
            %
            % plotPlatform(viewer, platstructs, ref) specifies the
            % reference frame used to interpret the 'Position' field of the
            % platform structure. Choose between 'NED' (default), 'ENU',
            % and 'ECEF'. When using 'NED' or 'ENU', the ReferenceLocation
            % property of the viewer is used as the origin of the frame.
            %
            % plotPlatform(..., 'Name', value) also lets you specify the
            % following Name-Value pairs:
            %
            %   TrajectoryMode   - Display mode for platform trajectories.
            %                      Choose between 'History' (default),
            %                      'Full', or 'None'. When selecting
            %                      'History', past positions from the
            %                      previous calls to plotPlatform are
            %                      displayed, up to PlatformHistoryDepth
            %                      number of calls. When selecting 'None',
            %                      no trajectory is displayed. When
            %                      selecting 'Full', the entire trajectory
            %                      is displayed, including future
            %                      positions. 'Full' can only be specified
            %                      for platform objects with a
            %                      waypointTrajectory or geoTrajectory with
            %                      multiple waypoints.
            %
            %   Marker           - Marker style for the current platform
            %                      location. Choose between:
            %                         '^'(default)  'd'    's'
            %
            %   Width            - Width of the trajectory line. 
            %                      Specified as a scalar integer (unique
            %                      width for all platforms).
            %
            %   Color            - Color of the trajectory line.
            %                      Specfied as a 1-by-3 RGB triplet (unique
            %                      color for all platforms) or a N-by-3
            %                      matrix of RGB triplets. N is the number
            %                      of platforms.
            % % Example
            %
            % refloc = [42.366978, -71.022362, 50];
            % viewer = trackingGlobeViewer('ReferenceLocation',refloc);
            % campos(viewer, refloc + [0 0 820]);
            % camorient(viewer,[210 -9 0]);
            % % Plot individual platforms from a scenario
            % s = trackingScenario;
            % % By default everything is defined in NED
            % p1= platform(s, 'Trajectory',waypointTrajectory([0 0 0; 0 100 -100; 0 200 -500],[0 60 120]));
            % p2 = platform(s, 'Position',[0 50 -50]);
            % % Plot platform with its entire trajectory and a '^' point marker
            % plotPlatform(viewer,p1,'TrajectoryMode','Full');
            % % Plot second platform in a different color
            % plotPlatform(viewer, p2, 'Color',[1 0 0]);
            %
            % See also: trackingScenario/platform
        end

        function out=plotScenario(~) %#ok<STOUT>
            %plotScenario Plot a tracking scenario on the viewer
            % plotScenario(viewer, scene) displays the tracking scenario
            % scene on the globe. Platforms with kinematicTrajectory or
            % static geoTrajectory are plotted with 'TrajectoryMode' set to
            % 'History'. Platforms with waypointTrajectory or other
            % geoTrajectory are plotted with 'TrajectoryMode' set to
            % 'Full'. See <a href=" matlab:help trackingGlobeViewer/plotPlatform">plotPlatform</a> for more
            % information.
            %
            % plotScenario(viewer, scene, detections) additionally displays
            % an array or cell-array of objectDetection, detections, on the
            % globe. See plotDetection for more information on the
            % detections input format.
            %
            % plotScenario(viewer, scene, detection, tracks) additionally
            % displays an array of objectTrack or track structs, tracks, on
            % the globe. See plotTrack for more information on the tracks
            % input format.
            %
            % % Example 1
            % % Create an Earth-Centered scenario with two platforms and one sensor
            % scene = trackingScenario('IsEarthCentered', true);
            % poslla = [10 10 500];
            % r = fusionRadarSensor(1);
            % platform(scene,'Position',poslla, 'Sensors',r);
            % trajlla = geoTrajectory([10 11 500; 10 12 500],[0 3600]);
            % platform(scene, 'Trajectory',trajlla);
            % viewer = trackingGlobeViewer;
            % % Plot scenario
            % plotScenario(viewer, scene);
            %
            % % Example 2
            % % Build the equivalent scenario using NED convention and a reference location
            % refloc = [10 10 0];
            % viewer = trackingGlobeViewer('ReferenceLocation', refloc);
            % scene2 = trackingScenario('IsEarthCentered', false);
            % poslla2 = [0 0 -500];
            % r = fusionRadarSensor(1);
            % platform(scene2, 'Position',poslla2, 'Sensors',r);
            % trajlla = geoTrajectory([10 11 500; 10 12 500],[0 3600]);
            % trajned = waypointTrajectory(lla2ned(trajlla.Waypoints, refloc, 'ellipsoid'),[0 3600]);
            % platform(scene2, 'Trajectory',trajned);
            % % Plot scenario
            % plotScenario(viewer, scene2)
            %
            % See also: trackingScenario
        end

        function out=plotTrack(~) %#ok<STOUT>
            %plotTrack Plot tracks
            % plotTrack(viewer, tracks) plots tracks specified as an array
            % of ObjectTracks or track structs on the globe.
            % plotTrack(viewer, {tracks1, ..., tracksP}) plots tracks
            % specified as a cell array of tracks array. Use this syntax
            % when plotting tracks with different state definition.
            % plotTrack(..., ref) specifies the reference frame used to
            % interpret the position coordinates of the track state. Choose
            % between 'NED' (default), 'ENU', and 'ECEF'. When using 'NED'
            % or 'ENU', the ReferenceLocation property of the viewer is
            % used as the origin of the frame.
            % plotTrack(..., 'Name' value) also lets you specify additional
            % options for the track specified as Name-Value pairs. 
            % Options are:
            %   PositionSelector - Position selector specified as a D-by-N
            %                      matrix, with 1 in the matrix
            %                      representing the position element in the
            %                      state, and where D is the number of
            %                      dimensions of the motion. If not
            %                      specified, the default value used is 
            %                      [1 0 0 0 0 0 ; 0 0 1 0 0 0; 0 0 0 0 1 0] 
            %                      which selects [x;y;z] from a 
            %                      six-dimenional state [x;vx;y;vy;z;vz].
            %                      See <a href=" matlab:help getTrackPositions">getTrackPositions</a> for more
            %                      information.
            %                      Alternatively, specify PositionSelector
            %                      as a cell array of position selectors
            %                      when 'tracks' is specified as a cell
            %                      array. Use this syntax when plotting
            %                      tracks with different state definitions.
            %   VelocitySelector - Velocity selector specified as a D-by-N
            %                      matrix, with 1 in the matrix
            %                      representing the position element in the
            %                      state, and where D is the number of
            %                      dimensions of the motion. If not
            %                      specified, the default value used is 
            %                      [0 1 0 0 0 0 ; 0 0 0 1 0 0; 0 0 0 0 0 1]
            %                      which selects [vx;vy;vz] from a 
            %                      six-dimenional state [x;vx;y;vy;z;vz].
            %                      See <a href=" matlab:help getTrackPositions">getTrackVelocities</a> for more
            %                      information.
            %                      Alternatively, specify VelocitySelector
            %                      as a cell array of velocity selectors
            %                      when 'tracks' is specified as a cell
            %                      array. Use this syntax when plotting
            %                      tracks with different state definitions.
            %   Width            - Width of the track history line. 
            %                      Specified as a scalar integer (unique
            %                      width for all tracks).
            %   Color            - Color of the track marker and history line.
            %                      Specfied as a 1-by-3 RGB triplet (unique
            %                      color for all tracks), a N-by-3 matrix
            %                      of RGB triplets, or a P-by-3 matrix of
            %                      RGB triplets. N is the number of tracks.
            %                      P is the number of cell elements when
            %                      'tracks' is specified as a cell array.
            %                      Alternativel, specify Color as a
            %                      P-element cell array of RGB triplets.
            %   LabelStyle       - Style used for track labels. Set to 'ID' 
            %                      to display the track ID and source
            %                      index. Set to 'ATC' to use an Air
            %                      Traffic Control style showing track ID,
            %                      heading, climb-rate, and groundspeed of
            %                      the track. Set to 'Custom' to pass your
            %                      own label. The default is 'ID'.
            %  CustomLabel       - Used when LabelStyle is set to 'Custom'.
            %                      Specify your own track labels as an
            %                      N-length cell array or string array. N
            %                      is the number of tracks. Each cell
            %                      element must be a scalar string or a
            %                      character vector. Each label is applied
            %                      to their respective track in order.
            %                      Alternatively, specify 'CustomLabel' as
            %                      a scalar string or a character vector to
            %                      use a unique label for all tracks.
            %
            %
            % The length of all track history lines is determined by the
            % TrackHistoryDepth property of the viewer.
            % The viewer maintains each track internally by its TrackID and
            % SourceIndex. If a (TrackID,SourceIndex) pair was previously
            % passed and is not found in the current call to plotTrack,
            % this track is considered dropped. Dropped tracks can be
            % removed from the globe by setting ShowDroppedTracks to false.
            %
            % % Example
            % refloc = [42.366978, -71.022362, 50];
            % viewer = trackingGlobeViewer('ReferenceLocation',refloc);
            % campos(viewer, 42.3374,  -71.0605,  872.7615);
            % camorient(viewer,[39 0 -2.7]);
            %
            % % Track state is [x, vx, y, vy, z, vz] with all coordinates expressed in
            % % the NED axes with origin ReferenceLocation
            % track1 = objectTrack('TrackID',1,'State',[10; 0; 10; 0;-50; 0], 'StateCovariance',1000*eye(6));
            % plotTrack(viewer, track1);
            %
            % % Track state is [x, y, z, vx, vy, vz] with all coordinates expressed in
            % % ENU axes with origin ReferenceLocation
            % track2 = objectTrack('TrackID',2,'State',[100; 10; 80; 0; 0; 0], 'StateCovariance',1000*eye(6));
            % % Define position and velocity selectors for non default state definitions.
            % possel = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
            % velsel = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
            % plotTrack(viewer, track2,'ENU', 'PositionSelector',possel,'VelocitySelector', velsel);
            %
            % % Track state is [x, y, z, d] where x, y, z are the ECEF coordinates and d is a non-positional track state.
            % track3 = objectTrack('TrackID',3,'State',[1.5349; -4.4634; 4.2761; 1e-5]*1e6, 'StateCovariance',2000*eye(4));
            % possel = [1 0 0 0; 0 1 0 0; 0 0 1 0];
            % velsel = []; % no velocity
            % plotTrack(viewer, track3, 'ECEF', 'PositionSelector',possel,'VelocitySelector',velsel);
            %
            % See also: objectTrack
        end

        function out=plotTrajectory(~) %#ok<STOUT>
            %plotTrajectory Plot trajectory objects
            % plotTrajectory(viewer, trajectory) plots a trajectory on the
            % viewer. Specify trajectory as a geoTrajectory, a
            % waypointTrajectory, or a cell array of geoTrajectory and
            % waypointTrajectory. WaypointTrajectory coordinates will be
            % plotted using the viewer ReferenceLocation as the
            % ReferenceFrame origin.
            %
            % plotTrajectory(..., 'Name',value) also lets you specify
            % plotting options for the trajectory line:
            %   Width            - Width of the trajectory line. 
            %                      Specified as a scalar integer (unique
            %                      width for all trajectories).
            %   Color            - Color of the trajectory line.
            %                      Specfied as a 1-by-3 RGB triplet (unique
            %                      color for all tracks) or a N-by-3 matrix
            %                      of RGB triplets. N is the number
            %                      of trajectories.
            % % Example
            % refloc = [1, -5, 50];
            % viewer = trackingGlobeViewer('ReferenceLocation',refloc);
            % campos(viewer, refloc + [-0.02 0 1000]);
            % camorient(viewer, [10 -15 0]);
            % % Plot trajectories with 3 possible frames, NED, ENU, LLA/ECEF
            % traj1 = waypointTrajectory([0 0 -400; 100 0 -400; 0 100 -400], [0 200 400]); % Coordinates are NED
            % traj2 = waypointTrajectory([0 0 400; 100 0 400; 0 100 400], [0 200 400], 'ReferenceFrame','ENU'); % Coordinates are ENU
            % traj3 = geoTrajectory([1 -5 0; 1.001 -5 0; 1.002 -5 100],[0 3600 7200]); % Coordinates are LLA
            %
            % plotTrajectory(viewer, traj1, 'Color', [1 0 0]);
            % plotTrajectory(viewer, traj2, 'Color', [0 1 0]);
            % plotTrajectory(viewer, traj3, 'Color', [0 0 1]);
            % 
            % See also: waypointTrajectory, geoTrajectory
        end

        function out=snapshot(~) %#ok<STOUT>
            %snapshot Take a snapshot
            % snapshot(viewer) creates a snapshot of the viewer
            % img = snapshot(viewer) returns the screenshot as a variable img.
            % Use imshow to display it.
            %
            % %Example:
            % viewer = trackingGlobeViewer;
            % im = snapshot(viewer);
            % imshow(im);
        end

    end
    properties
        %Basemap   Basemap used on the globe
        % Map used to visualize scenarios, specified as a character vector
        % or string scalar matching one of the following options:
        %    'satellite'     - Satellite imagery provided by ESRI
        %    'topographic'   - Topographic imagery provided by ESRI
        %    'streets'       - Street maps provided by ESRI
        %    'streets-light' - Light street maps provided by ESRI
        %    'streets-dark'  - Dark street maps provided by ESRI
        %    'darkwater'     - Two-tone map with light gray for land and dark gray for water
        %    'grayland'      - Two-tone map with gray for land and white for water
        %    'bluegreen'     - Two-tone map with green for land and blue for water
        %    'colorterrain'  - Shaded relief map derived from elevation and climate
        %    'grayterrain'   - Shaded relief map in shades of gray
        %    'landcover'     - Shaded relief map derived from satellite data
        % This property is available on contruction only.
        % Default value: 'satellite'
        Basemap;

        %CoverageMode Display option for coverages
        % Choose between 'Beam' and 'Coverage'. When set to 'Beam', only
        % the beam of each sensor is displayed. When set to 'Coverage',
        % both beam (field of view) and coverage (field of regard) are
        % displayed.
        % Default value: 'Beam'
        CoverageMode;

        %CoverangeRangeScale Scaling factor for sensor coverage
        % Scaling factor of range in drawing beam and/or coverage of each
        % sensor.
        % Default value: 1
        CoverageRangeScale;

        %NumCovarianceSigma Covariance ellipse size
        % Number of standard deviation to use to draw covariance
        % ellipses. Set to 0 to hide covariance ellipses.
        % Default value: 2
        NumCovarianceSigma;

        OnGlobeClosedListener;

        %PlatformHistoryDepth Length of platform trajectory history line
        % Defines the number of previous updates shown in the platform
        % history plot. 
        % Default value: 1000
        PlatformHistoryDepth;

        %ReferenceLocation Reference location on Earth
        % Specify the reference frame origin location in WGS84 coordinates
        % for displaying scenario elements that are not Earth-centered.
        % Default value: [0 0 0]
        ReferenceLocation;

        %ShowDroppedTracks Show dropped tracks on the globe
        % Show history line of dropped tracks. A track is dropped if it was
        % passed to plotTrack in the previous call and is not present in
        % the current call to the function. Changing the value of
        % ShowDroppedTracks will only affect subsequent calls to plotTrack.
        % To remove all graphics from the globe, use clear.
        % Default value: true
        ShowDroppedTracks;

        %Terrain Terrain used on the globe
        % Terrain data on which to visualize scenarios, specified as a
        % character vector or string scalar matching the name of custom
        % terrain data previously added using addCustomTerrain or one of
        % the following options:
        %    'none'      - Terrain elevation is 0 everywhere
        %    'gmted2010' - Global terrain derived from USGS GMTED2010 (Internet required)
        % This property is available on contruction only.
        % Default value: 'none'
        Terrain;

        %TrackHistoryDepth Length of track history line
        % Defines the number of previous updates shown in the track history
        % plot.
        % Default value: 1000
        TrackHistoryDepth;

        %TrackLabelScale Scaling factor for track labels
        % Scaling factor of labels displayed with each track.
        % Default value: 1
        TrackLabelScale;

        Viewer;

    end
end
