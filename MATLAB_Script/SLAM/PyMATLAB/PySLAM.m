function isSuccess = PySLAM(domainID, ROS2NodeName, duration)
%PUBLISHMAP Summary of this function goes here
%   Detailed explanation goes here

%     arguments
%         domainID (1, 1) int64
%         ROS2NodeName (1, 1) string
%         duration (1, 1) int64 = 100;
%     end
    
    
    isSuccess = true;
    setenv('ROS_DOMAIN_ID', int2str(domainID))
    
    
    n = ros2node(ROS2NodeName, domainID);
    
    
    lidarSub = ros2subscriber(n, "/scan","sensor_msgs/LaserScan", "Reliability","besteffort","Durability","volatile","Depth",5);

    
    
    %%  SLAM Section  %%
%     sc = {};
    maxLidarRange = 8;
    mapResolution = 20;
    maxNumScans = 360;
    global map
    global slamALg
    slamAlg = lidarSLAM(mapResolution, maxLidarRange, maxNumScans);
    
    
    slamAlg.LoopClosureThreshold = 210;  
    slamAlg.LoopClosureSearchRadius = 8;
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Pause Option
    % pause(4)
    
    
    
    

    tic
%     cnt = 0;
    
    while toc <= duration
    %%%%%%%%%%%%%%%%%%%%%%%    SLAM Section     %%%%%%%%%%%%%%%%%%%%%%%%%%%

        [scanMsge, isScaned] = receive(lidarSub);

%         before = toc;
        scan_temp = rosReadLidarScan(scanMsge);
%         sc = [sc, scan_temp];
        isScanAccepted = addScan(slamAlg, scan_temp);

        [scans, optimizedPoses]  = scansAndPoses(slamAlg);
        map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
%         occMatrix = getOccupancy(map);
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        if(~isScanAccepted || ~isScaned)
            fprintf(2, "Error occured");
            break;
        end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    end
    
    
    


%%%%%%%%%%%%%%%%%%%%%%%%    Plot Section     %%%%%%%%%%%%%%%%%%%%%%%%%%%
%     figure;
%     show(slamAlg);
%     title({'Map of the Environment','Pose Graph'});
%     
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
%     
%     
%     [scans, optimizedPoses]  = scansAndPoses(slamAlg);
%     map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
%     
%     
%     
%     figure; 
%     show(map);
%     hold on
%     show(slamAlg.PoseGraph, 'IDs', 'off');
%     hold off
%     title('Occupancy Grid Map Built Using Lidar SLAM');
%     
end
