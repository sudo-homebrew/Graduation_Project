function success = leftT(step, slamAlg, sc, velMsg, velPub, lidarSub, odomSub, velocity)
    velMsg.angular.z = velocity;
    send(velPub,velMsg)

    odomMsg_new = receive(odomSub);
    odomMsg_origin = receive(odomSub);

    ang = abs(odomMsg_new.pose.pose.orientation.z - odomMsg_origin.pose.pose.orientation.z);

    tic;
    
    while ang < step && toc < 100
      scanMsge = receive(lidarSub);
      rosPlot(scanMsge)
      scan_temp = rosReadLidarScan(scanMsge);
      sc = [sc, scan_temp];
      [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan_temp)

      if(~isScanAccepted)
          fprintf("Error occured");
          break;
      end

      odomMsg_new = receive(odomSub);

    ang = abs(odomMsg_new.pose.pose.orientation.z - odomMsg_origin.pose.pose.orientation.z);
    end
    
    velMsg.angular.z = 0;
    send(velPub,velMsg)

    success = isScanAccepted;
end
