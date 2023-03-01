% Matlab setup
close all

global LeftImg
global RightImg

f1 = figure;
%f2 = figure;
f3 = figure;
f4 = figure;

Cycle = 0;
ScriptWait = 0;
ComputationRateHz = 0.5;
ComputationPause = 1/ComputationRateHz;


% Disparity SGM setup
%DisparityRange = [0, 48]; %{}
%UniquenessThreshold = 15;

% Disparity BM Setup
MinDisparity = 30;
MaxDisparity = ((1)*16)+MinDisparity;
DisparityRange = [MinDisparity, MaxDisparity];   % 0,48
UniquenessThreshold = 10;   %15
BlockSize = 11;             %15
ContrastThreshold = 0.5;    %0.5
DistanceThreshold = [];     %[]
TextureThreshold = 0.0002;  %0.0002




%Main Script
rosshutdown
rosinit

fprintf("\n--- Setup Complete ---\n")
fprintf("\n--- Script Start ---\n\n")
fprintf("Starting Countdown...\n")
pause(ScriptWait)
fprintf("Countdown Complete\n\n")
fprintf("")

LeftCam = rossubscriber('/stereo/left/image_raw', 'DataFormat', 'struct');
RightCam = rossubscriber('/stereo/right/image_raw', 'DataFormat', 'struct');

LeftCam.NewMessageFcn = @LeftNewImg;
RightCam.NewMessageFcn = @RightNewImg;

while (true)
    fprintf('Cycle: %d \n', Cycle);
    %Computation ---------------
    
    %Undistort
    [LUImg, LeftOrigin] = undistortImage(LeftImg, Left_Camera_Params);
    [RUImg, RightOrigin] = undistortImage(RightImg, Right_Camera_Params);
    
    %Rectify
    [LRImg, RRImg, reprojectionMatrix] = rectifyStereoImages(LUImg, RUImg, Stereo_Params);
    
    %Red-Cyan StereoAnalglyph
    %RCImg = stereoAnaglyph(LRImg, RRImg);

    %Grayscale
    LGImg = rgb2gray(LRImg);
    RGImg = rgb2gray(RRImg);
    
    %Disparity Map
    DisparityMap = disparityBM( ...
        LGImg, RGImg, ...
        'DisparityRange', DisparityRange, ...
        'UniquenessThreshold', UniquenessThreshold, ...
        'BlockSize', BlockSize, ...
        'ContrastThreshold', ContrastThreshold, ...
        'DistanceThreshold', DistanceThreshold, ...
        'TextureThreshold', TextureThreshold);

    %DisparityMap = disparitySGM(...
    %    LGImg, RGImg,...
    %    'DisparityRange', DisparityRange,...
    %    'UniquenessThreshold', UniquenessThreshold);
    
    % Calculate Points
    Points = detectHarrisFeatures(LGImg);
    StrongestPoints = selectStrongest(Points, 10);
    
    % Triangulate
    [ID, LOC] = readAprilTag(I, TagFamily);

    % Show Images ---------------
    figure(f1)
    imshow(LeftImg, 'Border', 'tight');
    
    %figure(f2);
    %imshow(RightImg, 'Border', 'tight');

    figure(f3);
    imshow(DisparityMap, DisparityRange, 'Border', 'tight');
    title("Disparity Map")
    colormap jet
   
    %figure(f4);
    %imshow(LRImg)
    %hold on
    %plot(StrongestPoints)
    
    %Sleep ---------------
    pause(ComputationPause);
    Cycle = Cycle + 1;
end

function LeftNewImg(src, msg)
    global LeftImg
    LeftImg = rosReadImage(msg);
end

function RightNewImg(src, msg)
    global RightImg
    RightImg = rosReadImage(msg);
end