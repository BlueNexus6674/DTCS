%% Matlab setup -------------------------------------
clear all
close all

% MATLAB Params
LData = load("Left_Camera_Params.mat");
RData = load("Right_Camera_Params.mat");

%MATLAB Vars
ComputationInfo = [0, (0.5), 0];
ComputationInfo(3) = 1/ComputationInfo(2);

% April Tag Info
TagInfo = ["tag36h11", 97];

% Table Calib
Table = [1200, 600, 15]; % [Width, Height, Angle]
DvRange = [0, 800; 0, 280]; % Dx Dy Range
Scale(1) = Table(1) / (DvRange(1, 2)-DvRange(1, 1));
Scale(2) = Table(2) / (DvRange(2, 2)-DvRange(2, 1));

% ROS
rosshutdown
rosinit

%ROS
SubscriberError = true;
while (SubscriberError)
    fprintf("\nConnecting to ROS Image src...\n");

    try
        LeftCam = rossubscriber('/stereo/left/image_raw', 'DataFormat', 'struct');
        RightCam = rossubscriber('/stereo/right/image_raw', 'DataFormat', 'struct');
        LCMsg = receive(LeftCam, 1);
        RCMsg = receive(RightCam, 1);

        LCMsgStruct = isa(LCMsg, "struct");
        RCMsgStruct = isa(LCMsg, "struct");
    catch ERROR1
        LCMsg = 0;
        RCMsg = 0;
    end

    % Check Struct
    LCMsgStruct = isa(LCMsg, "struct");
    RCMsgStruct = isa(LCMsg, "struct");
    
    if (LCMsgStruct & RCMsgStruct)
        SubscriberError = false;
        fprintf("Connection Successfull\n");
    elseif (RCMsgStruct)
        fprintf("Left Camera Connection Failed\n")
        fprintf("Connection Unsuccessful\n");
    elseif (LCMsgStruct)
        fprintf("Right Camera Connection Failed\n")
        fprintf("Connection Unsuccessful\n");
    else
        fprintf("No Camera Messages Recieved\n");
        fprintf("Connection Unsuccessful\n");
    end
    
    pause(1);
end

fprintf("\n--- Setup Complete ---\n")

%% Main Loop -------------------------------------
%figure(f1)
while (true)
    fprintf('\n--- ---\nCycle: %d \n', ComputationInfo(1));
   
    %Get Latest Message 
    LCMsg = receive(LeftCam, 1);
    RCMsg = receive(RightCam, 1);

    % Check Struct
    LCMsgStruct = isa(LCMsg, "struct");
    RCMsgStruct = isa(LCMsg, "struct");

    if (LCMsgStruct & RCMsgStruct)
        LeftImg = rosReadImage(LCMsg);
        RightImg = rosReadImage(RCMsg);
        
        %Undistort
        [LUImg, LeftOrigin] = undistortImage(LeftImg, LData.Left_Camera_Params);
        [RUImg, RightOrigin] = undistortImage(RightImg, RData.Right_Camera_Params);
        
        try
            % Read April Tags
            [LTagID, LTagLocation, LTagPose] = readAprilTag(LUImg, TagInfo(1), LData.Left_Camera_Params.Intrinsics, TagInfo(2));
    
            % Read April Tags
            [RTagID, RTagLocation, RTagPose] = readAprilTag(RUImg, TagInfo(1), RData.Right_Camera_Params.Intrinsics, TagInfo(2));
            
            % Sort TagID's 
            [LTagIDRows, LTagIDCols] = size(LTagID);
            for ForLoopNum = 1:LTagIDCols
                if (LTagID(ForLoopNum) == 1)
                    LOriginTagID = ForLoopNum;
                elseif (LTagID(ForLoopNum) == 4)
                     LCube1TagID = ForLoopNum;
                else
                end
            end
            LOriginTagPose = LTagPose(:, LOriginTagID);
            LOriginTagLoc = LTagLocation(:, :, LOriginTagID);
    
            LCube1TagPose = LTagPose(:, LCube1TagID);
            LCube1TagLoc = LTagLocation(:, :, LCube1TagID);
    
            [RTagIDRows, RTagIDCols] = size(RTagID);
            for ForLoopNum = 1:RTagIDCols
                if (RTagID(ForLoopNum) == 1)
                    ROriginTagID = ForLoopNum;
                elseif (RTagID(ForLoopNum) == 4)
                     RCube1TagID = ForLoopNum;
                else
                end
            end
            
            %Get Poses
            ROriginTagPose = RTagPose(:, ROriginTagID);
            RCube1TagPose = RTagPose(:, RCube1TagID);
           
            %Calculate Displacements
            [Dx, Dy] = TablePos(LOriginTagPose.Translation - LCube1TagPose.Translation);
            
            % Print
            fprintf("Displacement X: %d \n", round(Dx));
            fprintf("Displacement Y: %d \n", round(Dy));
    
        catch ERROR1
            %rethrow (ERROR1);
            fprintf("ERROR: AprilTag Not Found\n")
            fprintf("Waiting for next cycle")
        end
      
        imshow(LUImg); 
        
        %Sleep ---------------
        pause(ComputationInfo(3));
        ComputationInfo(1) = ComputationInfo(1) + 1;
    else
        fprintf("\nERROR - No Message Recieved\n")
    end
end

function [Dx, Dy] = TablePos(Dv)
            RDx = Dv(1);
            RDy = abs(Dv(3)*cosd(Angle));
            
            Dx = RDx*ScaleWidth;
            Dy = RDy*ScaleHeight;
end
