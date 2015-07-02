function bubbleRobClient(actionFunction, functionInput)

% Connection parameters
IPADDRESS                           = '127.0.0.1';
PORT                                = 19999;
WAIT_UNTIL_CONNECTED                = true;
DO_NOT_RECONNECT_ONCE_DISCONNECTED  = true;
TIME_OUT_IN_MSEC                    = 5000;
COMM_THREAD_CYCLE_IN_MS             = 1;

% Arbitrary speed multiplier for motors
MOTOR_SPEED = 1;

% Bozo filter for input args
if nargin < 1
    fprintf('Usage:   bubbleRobClient(actionFunction)\n')
    fprintf('Example: bubbleRobClient(@actionFromKeyboard)\n')
    return
end

% Read WAV file for reporting success
%[success_sound,success_fs] = audioread('success.wav');

% Use this to build the library the first time through
%loadlibrary('remoteApi','extApi.h','mfilename','remoteApiProto')
%vrep = remApi('remoteApi', 'extApi.h');

% Load the V-REP remote API
vrep = remApi('remoteApi');

% Start the simulation on the server
clientID = vrep.simxStart(IPADDRESS, PORT, ...
    WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, ...
    TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS);

% We use try/catch to avoid fatal errors messing up the server
try
    if (clientID > -1)
        
        fprintf('Connected to remote API server as client %d\n', clientID);
        
        % Grab handles from V-REP server
        leftMotorHandle = getObjectHandle(vrep, clientID, 'remoteApiControlledBubbleRobLeftMotor');
        rightMotorHandle = getObjectHandle(vrep, clientID, 'remoteApiControlledBubbleRobRightMotor');
        noseSensorHandle = getObjectHandle(vrep, clientID, 'remoteApiControlledBubbleRobSensingNose');
        %leftAntennaHandle = getObjectHandle(vrep, clientID, 'remoteApiControlledBubbleRobSensingNose3');
        %rightAntennaHandle = getObjectHandle(vrep, clientID, 'remoteApiControlledBubbleRobSensingNose2');
        
        % Start the simulation
        if vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            
            fprintf('Failed to start simulation\n')
            return
            
        end
        
        % Initialize previous motor commands for tracking movement update
        leftMotorCommandPrev = 0;
        rightMotorCommandPrev = 0;
        
        % Loop until the action function returns an empty action for the
        % left motor command
        while true
            
            % Read from the three sensors
            noseSensor = readSensor(vrep, clientID, noseSensorHandle);
            %leftSensor = readSensor(vrep, clientID, leftAntennaHandle);
            %rightSensor = readSensor(vrep, clientID, rightAntennaHandle);
            
            % Action function converts sensor readings (0,1) into left and
            % right motor commands (-1,0,1)
            if nargin > 1
                [leftMotorCommand, rightMotorCommand] = actionFunction(noseSensor, leftSensor, rightSensor, functionInput);
            else
                %[leftMotorCommand, rightMotorCommand] = actionFunction(noseSensor, leftSensor, rightSensor);
                [leftMotorCommand, rightMotorCommand] = actionFunction(noseSensor);

            end
            
            % Done!
            if isempty(leftMotorCommand)
                break
            end
            
            % Set motor speeds using new action commands
            if leftMotorCommand ~= leftMotorCommandPrev || ...
                    rightMotorCommand ~= rightMotorCommandPrev
                setMotorSpeed(vrep, clientID, leftMotorHandle,  leftMotorCommand*MOTOR_SPEED)
                setMotorSpeed(vrep, clientID, rightMotorHandle, rightMotorCommand*MOTOR_SPEED)
            end
            
            % Track previous motor commands for movement update
            leftMotorCommandPrev = leftMotorCommand;
            rightMotorCommandPrev = rightMotorCommand;
            
            % Check success
            if leftSensor && rightSensor
                sound(success_sound, success_fs)
                fprintf('Success!!!\n')
                setMotorSpeed(vrep, clientID, leftMotorHandle,  0)
                setMotorSpeed(vrep, clientID, rightMotorHandle, 0)
                pause(3)
                break
            end
            
        end
        
        % Stop the simulation on the server
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
        
    else
        error('Failed connecting to remote API server')
    end
    
    % Clean up
    close(gcf)
    display('Done')
    vrep.simxFinish(clientID)
    vrep.delete(); % call the destructor
    
    % Handle all failures with a helpful stack trace
catch err
    close(gcf)
    fprintf('Error: %s\nTrace:\n', err.message)
    for k = 1:length(err.stack)
        level = err.stack(k);
        fprintf('On line %d of function %s in file %s\n', ...
            level.line, level.name, level.file)
    end
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(clientID); % close the line if still open
    vrep.delete(); % call the destructor
end

% Helper functions --------------------------------------------------------

function handle = getObjectHandle(vrep, clientID, objectName)
[errorCode,handle] = vrep.simxGetObjectHandle(clientID, objectName, vrep.simx_opmode_oneshot_wait);
if errorCode
    error('Failed to get object named %s', objectName)
end

function detectionState = readSensor(vrep, clientID, sensorHandle)
[errorCode,detectionState] = vrep.simxReadProximitySensor(clientID, sensorHandle, vrep.simx_opmode_continuous);
if errorCode
    detectionState = 0;
end

function setMotorSpeed(vrep, clientID, jointHandle, motorSpeed)
vrep.simxSetJointTargetVelocity(clientID, jointHandle,  motorSpeed, vrep.simx_opmode_oneshot);




