classdef (Sealed,StrictDefaults) monteCarloLocalization < matlab.System
%MONTECARLOLOCALIZATION Localize robot with range sensor data and map
%   MONTECARLOLOCALIZATION creates a Monte Carlo Localization (MCL)
%   object. The Monte Carlo Localization algorithm is used to estimate
%   the position and orientation of a robot in its environment using a
%   known map of the environment, range sensor data and odometry sensor
%   data.
%
%   The Monte Carlo Localization algorithm uses particles to represent
%   the distribution of the likely states, where each particle
%   represents a possible robot state. The particles converge around a
%   single location as the robot moves in the environment and senses
%   different parts of the environment using a range sensor. The robot
%   motion is sensed using the odometry sensor.
%
%   The step function of the monteCarloLocalization class takes the
%   pose and range sensor data as inputs. The pose is relative to an
%   arbitrary starting position and orientation, which is computed by
%   integrating the odometry sensor data. If the change in pose is more
%   than any of the update thresholds, then the particles are updated
%   and a new state estimate from the particle filter is computed.
%   The particles are updated in a three step process. First, the
%   particles are propagated as per the change in the pose and the
%   motion model. Next, the particles are assigned weights based on the
%   likelihood of receiving the range sensor observation for each particle.
%   Finally, based on the resampling interval, the particles are resampled
%   from the posterior distribution, where the particles of low weight
%   are eliminated. For example, if the resampling interval is 2, then
%   the particles are resampled after every second update.
%
%   The outputs of step function are ISUPDATED, ESTIMATEDPOSE and
%   ESTIMATEDCOVARIANCE. The ESTIMATEDPOSE and ESTIMATEDCOVARIANCE are
%   computed as mean and covariance of the cluster of particles that
%   has the highest weight. If the change in pose is more than any of
%   the update thresholds, the particles are updated during the step
%   function call and a new ESTIMATEDPOSE and ESTIMATEDCOVARIANCE are
%   returned along with the ISUPDATED flag as true. If the change in
%   pose is not more than any of the update thresholds, then the
%   particle update step is a no-op and the step function returns with
%   previously updated ESTIMATEDPOSE and ESTIMATEDCOVARIANCE and
%   ISUPDATED flag is false.
%
%   MCL = monteCarloLocalization returns a Monte Carlo
%   Localization object, MCL, that estimates robot pose using a
%   map, range sensor and odometry data. By default, an empty map is
%   assigned and a valid map assignment is required before using the
%   MCL object.
%
%   MCL = monteCarloLocalization(MAP) returns a Monte Carlo
%   Localization object, MCL, with occupancy grid MAP assigned to the
%   default likelihood field sensor model in MCL object.
%
%   MCL = monteCarloLocalization('PropertyName', PropertyValue, ...)
%   returns a Monte Carlo Localization object, MCL, with each specified
%   property set to the specified value.
%
%   MCL = monteCarloLocalization(MAP, 'PropertyName',
%   PropertyValue, ...) returns a Monte Carlo Localization object, MCL,
%   with each specified property set to the specified value and the
%   occupancy grid MAP assigned to the likelihood field sensor model in
%   MCL object.
%
%   Step method syntax:
%
%   [ISUPDATED, MEAN, COV] = step(MCL, POSE, SCAN) returns an
%   estimate of the robot pose MEAN, and covariance of the estimated pose
%   COV. The MEAN and COV are updated only when the ISUPDATED flag is
%   true. When ISUPDATED flag is false, the MEAN and COV are from the
%   previous step. The inputs are odometry reading POSE and range sensor
%   readings, SCAN, as lidarScan object.
%   To use this step method syntax, the "UseLidarScan" property has to be
%   set to "true".
%
%   [ISUPDATED, MEAN, COV] = step(MCL, POSE, RANGES, ANGLES) allows
%   you to pass range sensor readings as RANGES and ANGLES. The vectors RANGES and
%   ANGLES are range sensor data in a polar coordinate system and have
%   the same number of elements. The input RANGES are in meters, the
%   ANGLES are in radians.
%   To use this step method syntax, the "UseLidarScan" property has to be
%   set to "false".
%
%   The vector POSE represents the current pose of the robot estimated
%   by integrating the odometry sensor data, with respect to some
%   arbitrary starting point.  The POSE has elements [X Y THETA], where X
%   and Y are in meters and THETA is in radians. Positive angles are
%   measured counter-clockwise from the forward direction.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   monteCarloLocalization methods:
%
%   step         - Compute robot pose and covariance using range data
%   <a href="matlab:help matlab.System/reset   ">reset</a>        - Re-initialize the monteCarloLocalization System object
%   release      - Releases system resources of monteCarloLocalization System object
%   clone        - Create monteCarloLocalization object with same property values
%   getParticles - Get particles and their weights
%
%   monteCarloLocalization properties:
%
%   InitialPose         - Initial pose to start localization
%   InitialCovariance   - Covariance of initial pose
%   GlobalLocalization  - Flag to start global localization
%   ParticleLimits      - Minimum and maximum number of particles
%   SensorModel         - Likelihood field sensor model
%   MotionModel         - Odometry motion model for differential drive
%   UpdateThresholds    - Minimum change in states [x, y, theta] to trigger update
%   ResamplingInterval  - Resampling interval
%   UseLidarScan        - Use lidarScan object instead of ranges and angles
%
%   Example:
%
%       % Create a Monte Carlo Localization object
%       mcl = monteCarloLocalization('UseLidarScan', true);
%
%       % Assign sensor model to Monte Carlo Localization object
%       sm = likelihoodFieldSensorModel;
%       sm.Map = binaryOccupancyMap(10,10,20);
%       mcl.SensorModel = sm;
%
%       % Example laser scan data input
%       ranges = 10*ones(1, 300);
%       ranges(1, 130:170) = 1.0;
%       angles = linspace(-pi/2, pi/2, 300);
%       scan = lidarScan(ranges, angles);
%       odomPose = [0 0 0];
%
%       % Estimate robot pose and covariance
%       [isUpdated, pose, covariance] = mcl(odomPose, scan)
%
%   See also stateEstimatorPF, controllerVFH.

%   Copyright 2015-2021 The MathWorks, Inc.
%
%   References:
%
%   [1] S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics.
%   Cambridge, MA: MIT Press, 2005.

%#codegen

    properties (Dependent, Nontunable)
        %InitialPose Initial pose for particle filter
        %   A vector representing initial pose of the robot as [x y theta],
        %   where x, y are in meters, and theta is in radians. Initializing
        %   Monte Carlo Localization with an initial pose estimate will
        %   allow using a smaller value for maximum number of particles
        %   without compromising localization output.
        %
        %   Default: [0 0 0]
        InitialPose

        %InitialCovariance Covariance of initial pose
        %   Covariance of the Gaussian distribution for the InitialPose,
        %   which is an estimate of uncertainty in knowledge of initial
        %   pose. A scalar, 3 element vector or a diagonal matrix input are
        %   supported. Valid scalar and vector inputs are converted to a
        %   diagonal matrix.
        %
        %   Default: diag([1 1 1])
        InitialCovariance

        %GlobalLocalization Uniformly distribute particles in map
        %   Logical indicating if the global localization is to be
        %   performed. A TRUE value will result in initialization with
        %   uniformly distributed particles in the entire map. A FALSE
        %   value will result in initialization with InitialPose and
        %   InitialCovariance. Global localization requires a large number
        %   of particles to cover the entire workspace.
        %
        %   Default: false
        GlobalLocalization

        %ParticleLimits Minimum and maximum number of particles
        %   A vector [MIN MAX] representing the limits on the number of
        %   particles to use during Monte Carlo Localization.
        %
        %   Default: [500 5000]
        ParticleLimits

        %SensorModel Sensor model for laser range finder
        %
        %   Default: likelihoodFieldSensorModel
        SensorModel
    end

    properties (Dependent)
        %MotionModel Motion model for the robot
        %   An odometry motion model object for differential drive robots.
        %
        %   Default: odometryMotionModel
        MotionModel

        %UpdateThresholds Minimum change in state required to update filter
        %   The minimum change in states of the robot [x, y, theta] needed
        %   to update particle filter. The update will be applied if any of
        %   the thresholds are met and a new pose estimate will be available.
        %
        %   Default: [0.2 0.2 0.2]
        UpdateThresholds

        %ResamplingInterval Number of filter updates before resampling
        %   The interval representing number of filter updates to be done
        %   before resampling particles.
        %
        %   Default: 1
        ResamplingInterval
    end

    properties (Dependent, Nontunable)
        %UseLidarScan Use lidarScan object instead of ranges and angles
        %   By default, you pass ranges and angles as numeric arrays into
        %   the step function. Setting UseLidarScan to "true" allows you to
        %   pass a lidarScan object instead.
        %
        %   Default: false
        %
        %   See also: lidarScan
        UseLidarScan
    end
    
    properties (Hidden)
        %Seed Random seed used at construction
        %   The intent of the "Seed" is to allow reproducibility of the MCL
        %   behavior if the user sets the global random number generator
        %   to a fixed seed before using your object.
        %   Default: 67
        Seed = 67
    end

    properties (Constant, Access = private)
        %RecoveryAlphaSlow Recovery parameter
        RecoveryAlphaSlow = 0

        %RecoveryAlphaFast Recovery parameter
        RecoveryAlphaFast = 0

        %KLDError Maximum KLD distribution error
        KLDError = 0.05

        %KLDZ KLD parameter
        KLDZ = 0.99
    end

    properties (Transient , Access = {?nav.algs.internal.InternalAccess})
        %MCLObj Handle for internal monteCarloLocalization object
        MCLObj
    end

    properties (Access = protected)
        %PrivateMotionModel Private property for MotionModel
        PrivateMotionModel

        %PrivateSensorModel Private property for SensorModel
        PrivateSensorModel

        %PrivateParticleLimits Private property for ParticleLimits
        PrivateParticleLimits = [500 5000]

        %PrivateInitialPose Private property for InitialPose
        PrivateInitialPose = [0 0 0]

        %PrivateInitialCovariance Private property for InitialCovariance
        PrivateInitialCovariance = eye(3);

        %PrivateResamplingInterval Private property for ResamplingInterval
        PrivateResamplingInterval = 1

        %PrivateUpdateThresholds Private property for UpdateThresholds
        PrivateUpdateThresholds = [0.2 0.2 0.2]

        %PrivateGlobalLocalization Private property for GlobalLocalization
        PrivateGlobalLocalization = false
    end
    
    properties (Access = protected, Nontunable)
        %PrivateUseLidarScan Private property for UseLidarScan
        PrivateUseLidarScan = false       
    end

    methods
        function obj = monteCarloLocalization(varargin)
        %monteCarloLocalization Constructor
            obj.PrivateSensorModel = likelihoodFieldSensorModel;
            obj.PrivateMotionModel = odometryMotionModel;
            
            if coder.target('MATLAB')
                % Read the seed from MATLAB global seed
                % Note that this only works in MATLAB, not in code
                % generation. If you want a consistent random seed in both
                % cases, use the 'Seed' name-value pair input to the
                % constructor.
                randomState = rng;
                obj.Seed = double(randomState.Seed);            
            end

            % Allow map as first optional value-only argument
            if nargin > 0 && isa(varargin{1}, 'binaryOccupancyMap')
                validateattributes(varargin{1}, ...
                                   {'binaryOccupancyMap'},{'nonempty', 'scalar'});
                setProperties(obj,nargin-1,varargin{2:end});
                obj.PrivateSensorModel.Map = varargin{1};
            else
                setProperties(obj,nargin,varargin{:});
            end
                                   
            obj.MCLObj = nav.algs.internal.MonteCarloLocalizationBuiltins(obj.Seed);            
        end

        function set.InitialPose(obj, pose)
            validateattributes(pose, {'double'}, ...
                               {'vector', 'numel', 3, 'real', 'finite'});
            obj.PrivateInitialPose = pose;
        end

        function value = get.InitialPose(obj)
            value = obj.PrivateInitialPose;
        end

        function set.InitialCovariance(obj, val)
            validateattributes(val, {'double'}, ...
                               {'real', 'finite'});

            % Allow scalar, vector of size 3 or a diagonal matrix
            switch numel(val)
              case 1
                validateattributes(val, {'double'}, {'nonnegative'});
                covVal = val*eye(3);
              case 3
                validateattributes(val, {'double'}, {'vector', 'nonnegative'});
                covVal = diag(val);
              otherwise
                validateattributes(val, {'double'}, ...
                                   {'diag', 'size', [3,3], 'nonnegative'});
                covVal = val;
            end

            obj.PrivateInitialCovariance = covVal;
        end

        function value = get.InitialCovariance(obj)
            value = obj.PrivateInitialCovariance;
        end

        function set.GlobalLocalization(obj, val)
            validateattributes(val, {'numeric', 'logical'}, ...
                               {'binary', 'scalar', 'real'});
            obj.PrivateGlobalLocalization = logical(val);
        end

        function val = get.GlobalLocalization(obj)
            val = obj.PrivateGlobalLocalization;
        end

        function set.ResamplingInterval(obj, value)
            validateattributes(value, {'double'}, ...
                               {'scalar', 'real', 'positive', 'finite', 'integer'});
            obj.PrivateResamplingInterval = value;

            if obj.isLocked
                obj.MCLObj.setResamplingInterval(value);
            end
        end

        function value = get.ResamplingInterval(obj)
            value = obj.PrivateResamplingInterval;
        end

        function set.UseLidarScan(obj, val)
            validLogical = robotics.internal.validation.validateLogical(val);
            obj.PrivateUseLidarScan = validLogical;
        end

        function value = get.UseLidarScan(obj)
            value = obj.PrivateUseLidarScan;
        end

        function set.ParticleLimits(obj, value)

            validateattributes(value, {'double'}, ...
                               {'numel', 2, 'positive', 'real', 'finite', 'integer', ...
                                '<=', intmax('int32')});
            obj.PrivateParticleLimits = [min(value) max(value)] ;
        end

        function value = get.ParticleLimits(obj)
            value = obj.PrivateParticleLimits;
        end

        function set.UpdateThresholds(obj, value)
            validateattributes(value, {'double'}, ...
                               {'numel', 3, 'vector', 'real', 'nonnegative', 'finite'});
            obj.PrivateUpdateThresholds = value;

            if obj.isLocked
                obj.MCLObj.setUpdateThresholds(value(1),value(2),value(3));
            end
        end

        function value = get.UpdateThresholds(obj)
            value = obj.PrivateUpdateThresholds;
        end

        function set.MotionModel(obj, mm)
            validateattributes(mm, {'odometryMotionModel'}, ...
                               {'scalar'}, 'MotionModel');
            obj.PrivateMotionModel = clone(mm);

            if obj.isLocked
                obj.MCLObj.setMotionModel(mm.Noise);
            end
        end

        function val = get.MotionModel(obj)
            val = obj.PrivateMotionModel;
        end

        function set.SensorModel(obj, sm)
            validateattributes(sm, {'likelihoodFieldSensorModel'}, ...
                               {'scalar'}, 'SensorModel');

            coder.internal.errorIf(isempty(sm.Map), 'nav:navalgs:mcl:EmptyMap');

            obj.PrivateSensorModel = clone(sm);
        end

        function value = get.SensorModel(obj)
            value = obj.PrivateSensorModel;
        end

        function [particles, weights] = getParticles(obj)
        %getParticles Get particles from localization algorithm
        %   [PARTICLES, WEIGHTS] = getParticles(MCL) returns an N-by-3
        %   array PARTICLES representing [x y yaw] poses of current
        %   particles used by the monteCarloLocalization object along
        %   with an N-by-1 vector WEIGHTS representing current weight
        %   of each particle. The size of the arrays PARTICLES and
        %   WEIGHTS can change after each particle filter update.
        %
        %   Example:
        %
        %       % Create a Monte Carlo Localization object
        %       mcl = monteCarloLocalization('UseLidarScan', true);
        %
        %       % Assign sensor model to Monte Carlo Localization object
        %       sm = likelihoodFieldSensorModel;
        %       sm.Map = binaryOccupancyMap(10,10,20);
        %       mcl.SensorModel = sm;
        %
        %       % Example laser scan data input
        %       ranges = ones(1, 10);
        %       angles = linspace(-pi, pi, 10);
        %       scan = lidarScan(ranges, angles);
        %       odomPose = [0 0 0];
        %
        %       % Estimate robot pose and covariance
        %       [isUpdated, pose, covariance] = mcl(odomPose, scan);
        %
        %       % Get particles and weights
        %       [particles, weights] = getParticles(mcl);

            coder.internal.errorIf(~obj.isLocked, 'nav:navalgs:mcl:ParticlesNotAllocated');
            
            [particles, weights] = ...
                nav.algs.internal.AccessMCL.getParticles(obj.MCLObj);
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
        %setupImpl Initialize the system object
            if isempty(obj.PrivateSensorModel.Map)
                error(message('nav:navalgs:mcl:EmptyMap'));
            end

        end

        function resetImpl(obj)
        %resetImpl Reset the object to initial state
            if ~isempty(obj.MCLObj)
                obj.initialize();
            end
        end

        function releaseImpl(obj)
        %releaseImpl Release resources held with the object
            obj.MCLObj.cleanup;
        end

        function validateInputsImpl(obj, pose, varargin)
        % Validate inputs to the step method at initialization

            obj.checkInputs(pose, varargin{:});
        end

        function [isUpdated, estPose, estCov] = stepImpl(obj, pose, varargin)
        %step Estimate robot pose using sensor data

        % Validate inputs
            scan = obj.checkInputs(pose, varargin{:});

            % Combined predict, correct and resampling step
            numranges = length(scan.Ranges);
            obj.MCLObj.update(numranges, scan.Ranges, scan.Angles, pose)

            % Get the outputs
            isUpdated = obj.MCLObj.isUpdated;
            [estPose, estCov] = ...
                nav.algs.internal.AccessMCL.getHypothesis(obj.MCLObj);
        end

        function num = getNumInputsImpl(obj)
            if obj.UseLidarScan
                % Step input is lidarScan object
                num = 2;
            else
                % Step inputs are ranges and angles (legacy)
                num = 3;
            end
        end

        function num = getNumOutputsImpl(~)
            num = 3;
        end

        function flag = isInputSizeMutableImpl(~, ~)
            flag = true;
        end
        
        function s = saveObjectImpl(obj)
        %saveObjectImpl Custom save implementation
            s = saveObjectImpl@matlab.System(obj);

            if isLocked(obj)
                % Cannot save internal state of this object
                % Issue a warning to the user, so he can act appropriately
                warning(message('nav:navalgs:mcl:InvalidSaveLoad', ...
                                class(obj)));
            end

            % Save all protected properties
            mco = ?monteCarloLocalization;
            propList = mco.PropertyList;

            for i = 1:length(propList)
                propName = mco.PropertyList(i).Name;
                if robotics.internal.isProtectedProperty(mco, propList(i))
                    s.(propName) = obj.(propName);
                end
            end
        end

        function loadObjectImpl(obj,svObj,wasLocked)
        %loadObjectImpl Custom load implementation

            mco = ?monteCarloLocalization;
            propList = mco.PropertyList;

            % Re-load all protected properties
            for i = 1:length(propList)
                propName = mco.PropertyList(i).Name;
                if robotics.internal.isProtectedProperty(mco, propList(i)) && ...
                        isfield(svObj, propName)
                    obj.(propName) = svObj.(propName);
                end
            end

            loadObjectImpl@matlab.System(obj,svObj,wasLocked);

            if wasLocked
                % Cannot load internal state of this object
                % Issue a warning to the user, so he can act appropriately
                warning(message('nav:navalgs:mcl:InvalidSaveLoad', ...
                                class(obj)));
            end
        end
    end

    methods (Access = private)
        function initialize(obj)
        %initialize Initialize by assigning all data to internal object
            obj.MCLObj.setUpdateThresholds(obj.PrivateUpdateThresholds(1), ...
                                           obj.PrivateUpdateThresholds(2), obj.PrivateUpdateThresholds(3));
            obj.MCLObj.setResamplingInterval(obj.PrivateResamplingInterval);

            % Set sensor model
            nav.algs.internal.AccessMCL.setSensorModel( ...
                obj.MCLObj, obj.PrivateSensorModel);

            % Set motion model
            obj.MCLObj.setMotionModel(obj.PrivateMotionModel.Noise);

            % Initialize particle filter
            obj.MCLObj.initializePf(obj.PrivateParticleLimits(1), ...
                                    obj.PrivateParticleLimits(2), obj.RecoveryAlphaSlow, ...
                                    obj.RecoveryAlphaFast, obj.KLDError, obj.KLDZ);


            if obj.PrivateGlobalLocalization
                % Global initialization
                obj.MCLObj.globalLocalization();
            else
                % Initialize with pose and covariance
                obj.MCLObj.setInitialPose(obj.PrivateInitialPose, ...
                                          obj.PrivateInitialCovariance);
            end
        end

    end

    methods (Access = protected)
        function scan = checkInputs(obj, pose, varargin)
        %checkInputs Validate inputs

            if obj.UseLidarScan
                % Only lidarScan input
                scan = robotics.internal.validation.validateLidarScan(...
                    varargin{1}, 'step', 'scan');

            else
                % Scan as ranges and angles
                scan = robotics.internal.validation.validateLidarScan(...
                    varargin{1}, varargin{2}, 'step', 'ranges', 'angles');
            end

            validateattributes(pose, {'double'}, ...
                               {'vector', 'numel', 3, 'real', 'finite'}, 'step', 'pose');

        end
    end
end
