function mainFile = generateMain(folder,nodeName)
%This function is for internal use only. It may be removed in the future.

%GENERATEMAIN Generate main.cpp

%   Copyright 2021 The MathWorks, Inc.
    s = StringWriter;
    s.addcr('#include "ros/ros.h"');
    s.addcr('#include <thread>');
    s.addcr('#include "%s.h"',nodeName); % Contains function signature for top-level entry function
    s.addcr();
    s.addcr('bool threadTerminating = false;')
    s.addcr();
    s.addcr('void threadFunction(void)');
    s.addcr('{');
    s.addcr('   try');
    s.addcr('   {');
    s.addcr('       %s();',nodeName);
    s.addcr('   }');
    s.addcr('   catch (std::runtime_error e)');
    s.addcr('   {');
    s.addcr('       std::cout << "Caught exception: " << e.what() << std::endl;');
    s.addcr('   }');
    s.addcr('   catch (...)');
    s.addcr('   {');
    s.addcr('       std::cout << "Caught unknown exception, terminating the program." << std::endl;');
    s.addcr('   }');
    s.addcr('    threadTerminating = true;');
    s.addcr('    ros::shutdown();');     % User function terminated. Shutdown ROS
    s.addcr('}');
    s.addcr();
    s.addcr('int main(int argc, char** argv)');
    s.addcr('{');
    s.addcr('    ros::init(argc, argv, "%s");',nodeName);
    s.addcr('    ros::NodeHandlePtr MLROSNodePtr = ros::NodeHandlePtr(new ros::NodeHandle);');
    s.addcr('    std::thread threadObj(threadFunction);');
    s.addcr();
    s.addcr('    ros::spin();');
    s.addcr('    if (threadTerminating) {'); % threadFunction called ros::shutdown
    s.addcr('    threadObj.join();');
    s.addcr('    }');
    s.addcr();
    s.addcr('    return 0;');
    s.addcr('}');

    % Write out the main.cpp file and return full-path to the file
    mainFile = fullfile(folder,'main.cpp');
    s.write(mainFile);
end
