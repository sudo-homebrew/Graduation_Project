function success = forwardS(step, slamAlg, sc, velMsg, velPub, lidarSub, odomSub, velocity)
    velMsg.linear.x = velocity;
    send(velPub,velMsg)

    odomMsg_new = receive(odomSub);
    odomMsg_origin = receive(odomSub);

    dist = sqrt((odomMsg_new.pose.pose.position.x - odomMsg_origin.pose.pose.position.x)^2 + ...
        (odomMsg_new.pose.pose.position.y - odomMsg_origin.pose.pose.position.y)^2);
    
    
    tic;
    while dist < step && toc < 100
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
      dist = sqrt((odomMsg_new.pose.pose.position.x - odomMsg_origin.pose.pose.position.x)^2 + ...
        (odomMsg_new.pose.pose.position.y - odomMsg_origin.pose.pose.position.y)^2);
    end

    
    
    velMsg.linear.x = 0;
    send(velPub,velMsg)

    success = isScanAccepted;
end