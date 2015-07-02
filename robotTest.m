function robotTest()
   pause on;
   disp('Program started');
   vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
   vrep.simxFinish(-1); % just in case, close all opened connections
   clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    if clientID == -1
           disp('Failed connecting to remote API server');
    end
    
     [retVal,leftMotor] = vrep.simxGetObjectHandle(clientID,'remoteApiControlledBubbleRobLeftMotor',vrep.simx_opmode_oneshot_wait);
     check(vrep,retVal, 'failed to get left motor', 'got left motor');
     [retVal,rightMotor] = vrep.simxGetObjectHandle(clientID,'remoteApiControlledBubbleRobRightMotor',vrep.simx_opmode_oneshot_wait);
     check(vrep,retVal, 'failed to get right motor', 'got right motor');
     [retVal,noseSensor] = vrep.simxGetObjectHandle(clientID,'remoteApiControlledBubbleRobSensingNose',vrep.simx_opmode_oneshot_wait);
     check(vrep,retVal, 'failed to get nose sensor', 'got nose sensor');
     
     [retVal, vel] = vrep.simxGetObjectFloatParameter(clientID, leftMotor, 2012, vrep.simx_opmode_oneshot_wait);
      vel
     
     [retVal]=vrep.simxSetJointTargetVelocity(clientID,leftMotor,3*pi,vrep.simx_opmode_oneshot_wait);
     check(vrep,retVal, 'velocity left failed', 'velocity left set');
     
     [retVal]=vrep.simxSetJointTargetVelocity(clientID,rightMotor,pi,vrep.simx_opmode_oneshot_wait);
     check(vrep,retVal, 'velocity right failed', 'velocity right set');
    
    vrep.simxAddStatusbarMessage(clientID, 'this is a test', vrep.simx_opmode_oneshot_wait);
    
     [retVal, vel] = vrep.simxGetObjectFloatParameter(clientID, leftMotor, 2012, vrep.simx_opmode_oneshot_wait);
      vrep.simxAddStatusbarMessage(clientID, 'this is a test', vrep.simx_opmode_oneshot_wait);
     
   try
      while (vrep.simxGetConnectionId(clientID)>-1)
          pause(.001)
        %[retVal, nose] = vrep.simReadProximitySensor(noseSensor);
        %nose
      end
        disp('exiting');
      vrep.delete(); % call the destructor!
   catch err
      vrep.simxFinish(clientID); % close the line if still open
      vrep.delete(); % call the destructor!
   end;
   
   disp('Program ended');
end

function check(vrep, retVal, textError, textNoError)
     if (retVal==vrep.simx_error_noerror)
         disp(textNoError);
     else
        disp(textError);
     end
end