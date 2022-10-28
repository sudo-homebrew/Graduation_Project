% Robotics System Toolbox
% Version 4.0 (R2022a) 13-Nov-2021
%
% Robot Manipulators
%   rigidBodyJoint                 - Create a joint
%   rigidBody                      - Create a rigid body
%   rigidBodyTree                  - Create a tree-structured robot
%   interactiveRigidBodyTree       - Interact with rigid body tree robot models
%   importrobot                    - Import rigid body tree model from URDF file, text, or Simscape Multibody model
%   loadrobot                      - Load robot model as rigidBodyTree
%   jointSpaceMotionModel          - Model rigid body tree motion given joint-space inputs
%   taskSpaceMotionModel           - Model rigid body tree motion given task-space reference inputs
%   manipulatorRRT                 - Plan motion for rigid body tree using bidirecitonal RRT
%   manipulatorStateSpace          - State space of a rigid body tree
%   manipulatorCollisionBodyValidator - Validate a state of a rigid body tree against a cell-array of convex collision bodies
%   workspaceGoalRegion            - Define a workspace region of end-effector goal poses
% 
% Inverse Kinematics
%   analyticalInverseKinematics    - Create an analytical inverse kinematics solver
%   generalizedInverseKinematics   - Create multiconstraint inverse kinematics solver
%   inverseKinematics              - Create inverse kinematics solver
%   constraintCartesianBounds      - Constraint to keep a body origin inside Cartesian bounds
%   constraintDistanceBounds       - Constrain a body's distance from another body
%   constraintJointBounds          - Bounds on joint positions of a rigidBodyTree
%   constraintAiming               - Aiming constraint for pointing at a target location
%   constraintOrientationTarget    - Constraint on the relative orientation of a body
%   constraintPositionTarget       - Constraint on the relative position of a body
%   constraintPoseTarget           - Constraint on the relative pose of a body
%   constraintPrismaticJoint       - Prismatic joint constraint between two bodies.
%   constraintRevoluteJoint        - Revolute joint constraint between two bodies.
%   constraintFixedJoint           - Fixed joint constraint between two bodies.
%
% Collision Checking
%   collisionBox          - Create a collision geometry as a box primitive
%   collisionCylinder     - Create a collision geometry as a cylinder primitive
%   collisionSphere       - Create a collision geometry as a sphere primitive
%   collisionMesh         - Create a collision geometry as a convex mesh
%   checkCollision        - Report collision status between two convex geometries
%
% Mobile Robots
%   ackermannKinematics           - Create car-like steering vehicle model            
%   bicycleKinematics             - Create bicycle vehicle model
%   differentialDriveKinematics   - Create differential-drive vehicle model
%   unicycleKinematics            - Create unicycle vehicle model
%   binaryOccupancyMap            - Create an occupancy grid map with binary values
%   controllerPurePursuit         - Create a controller to follow a set of waypoints
%   mobileRobotPRM                - Create probabilistic roadmap path planner
%   stateEstimatorPF              - Create a particle filter state estimator                  
%   rangeSensor                   - Simulate range-bearing sensor readings
%
% Small UAVs
%   roboticsAddons        - Install add-on for UAV development
%
% Trajectory Generation
%   cubicpolytraj     - Generate third-order polynomial trajectories through multiple waypoints
%   quinticpolytraj   - Generate fifth-order polynomial trajectories through multiple waypoints
%   bsplinepolytraj   - Generate multi-axis trajectories through control points using B-splines
%   transformtraj     - Generate trajectory between two homogeneous transforms
%   trapveltraj       - Generate piecewise polynomials through multiple waypoints using trapezoidal velocity profiles
%   rottraj           - Generate trajectory between two orientations
%   minjerkpolytraj   - Generate minimum jerk trajectories through multiple waypoints
%   minsnappolytraj   - Generate minimum snap trajectories through multiple waypoints
%
% Coordinate Transformations
%   axang2quat         - Convert axis-angle rotation representation to quaternion
%   axang2rotm         - Convert axis-angle rotation representation to rotation matrix
%   axang2tform        - Convert axis-angle rotation representation to homogeneous transform
%   cart2hom           - Convert Cartesian coordinates to homogeneous coordinates
%   eul2quat           - Convert Euler angles to quaternion
%   eul2rotm           - Convert Euler angles to rotation matrix
%   eul2tform          - Convert Euler angles to homogeneous transformation
%   hom2cart           - Convert homogeneous coordinates to Cartesian coordinates
%   quat2axang         - Convert quaternion to axis-angle rotation representation
%   quat2eul           - Convert quaternion to Euler angles
%   quat2rotm          - Convert quaternion to rotation matrix
%   quat2tform         - Convert quaternion to homogeneous transformation
%   rotm2axang         - Convert rotation matrix to axis-angle representation
%   rotm2eul           - Convert rotation matrix to Euler angles
%   rotm2quat          - Convert rotation matrix to quaternion
%   rotm2tform         - Convert rotation matrix to homogeneous transform
%   tform2axang        - Extract axis-angle rotation from homogeneous transformation
%   tform2eul          - Extract Euler angles from homogeneous transformation
%   tform2quat         - Extract quaternion from homogeneous transformation
%   tform2rotm         - Extract rotation matrix from homogeneous transformation
%   tform2trvec        - Extract translation vector from homogeneous transformation
%   trvec2tform        - Convert translation vector to homogeneous transformation
%   transformTree      - Define coordinate frames and relative transformations
%
% Angular Operations
%   angdiff            - Calculate difference between two angles
%
% Utilities
%   plotTransforms     - Plot 3-D transforms described by translations and rotations
%   roboticsAddons     - Install add-ons for robotics
%   rateControl        - Execute loop at a fixed frequency
%                                     
% <a href="matlab:demo('toolbox','Robotics System')">View examples</a> for Robotics System Toolbox.

% Copyright 2013-2021 The MathWorks, Inc.
