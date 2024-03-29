classdef FunctionContainer
    methods (Static)
        function [Camera, Error] = ConnectToROSCameras(MaxAttempts, HardError)
            % Argument and Var Creation
            arguments
                MaxAttempts (1, 1) {mustBeInteger} = 9999;
                HardError (1, 1) {mustBeInteger} = 1;
            end
            Attempts = 0;
            SubscriberError = true;
            
            % Main While Loop
            while (SubscriberError & (Attempts<MaxAttempts))
                fprintf("\nConnecting to ROS Image src (Attempt %d/%d)...\n", Attempts+1, MaxAttempts);
                
                % Try Connect To ROS
                try
                    Camera = rossubscriber('/stereo/left/image_raw', 'DataFormat', 'struct');
                    CMsg = receive(Camera, 1);
            
                    IsMsgStruct = isa(CMsg, "struct");
                catch ERROR1
                    %rethrow(ERROR1)
                    CMsg = 0;
                end
            
                % Check Struct
                IsMsgStruct = isa(CMsg, "struct");
                
                if (IsMsgStruct)
                    SubscriberError = false;
                    fprintf("Connection Successfull\n");
                else
                    fprintf("No Camera Messages Recieved\n");
                    fprintf("Connection Unsuccessful\n");
                end
                pause(1);
                Attempts = Attempts+1;
            end
            
            % Error Block
            if (Attempts >= MaxAttempts)
                if (HardError)
                    error("FunctionContainer Error: ROS Connection Attempts Exceed MaxAttempts Input")
                end
                Error = 1;
            else
                Error = 0;
            end

        end
        
        function [Img, Error] = GetROSImages(Camera)
            try
                %Get Latest Message 
                CMsg = receive(Camera, 1);
          
                % Check Struct
                IsMsgStruct = isa(CMsg, "struct");

            catch ERRORInternal
                CMsg = 0;
            end

            IsMsgStruct = isa(CMsg, "struct");

            % Create Error Code
            if (IsMsgStruct(1))
                Img = rosReadImage(CMsg);
                Error = 0;
            else                                   
                fprintf("No Images Recieved\n")
                Error = 3;
            end
        end

        function [TagTranslations] = FindAprilTags(UImg, Camera_Params, TagInfo)
            % Var
            TagTranslations = zeros(5, 3);
            OriginExist = 0;
            ObjectExist = [0, 0, 0, 0];

            % Origin Tag
            OriginTagNum = TagInfo(3);
            
            % Object Tags
            if (string(OriginTagNum) == "1")
                ObjectTagNum = [2, 3, 4, 5];
            else
                ObjectTagNum = [1, 2, 3, 4];
            end

            % Read April Tags
            [TagID, TagLocation, TagPose] = readAprilTag(UImg, TagInfo(1), Camera_Params.Intrinsics, str2double(TagInfo(2)));

            % Sort TagID's 
            [TagIDRows, TagIDCols] = size(TagID);

            for i = 1:TagIDCols
                switch TagID(i)
                    case str2double(OriginTagNum)
                        TagTranslations(1, :) = TagPose(:, i).Translation;
                        OriginExist = 1;

                    case ObjectTagNum(1)
                        TagTranslations(2, :) = TagPose(:, i).Translation;
                        ObjectExist(1) = 1;

                    case ObjectTagNum(2)
                        TagTranslations(3, :) = TagPose(:, i).Translation;
                        ObjectExist(2) = 1;

                    case ObjectTagNum(3)
                        TagTranslations(4, :) = TagPose(:, i).Translation;
                        ObjectExist(3) = 1;

                    case ObjectTagNum(4)
                        TagTranslations(5, :) = TagPose(:, i).Translation;
                        ObjectExist(4) = 1;
                end
            end
                
            for i = 1:4
                if (ObjectExist(i) ~= 1)
                    TagTranslations(i+1, :) = [-999, -999, -999];
                end
            end

            if (OriginExist ~= 1)
                TagTranslations(1, :) = [-999, -999, -999];
                warning("Origin Tag Not Found");
            end
        end

        function [Dx, Dy] = ObjectDisplacements(TagTranslations)
            Dx = TagTranslations(:, 1);
            Dy = TagTranslations(:, 3);
        end

        function [OutDx, OutDy] = CalibrateDisplacements(Dx, Dy, Scale, CalibParamsX, CalibParamsY)
            OutDy = [-999, -999, -999, -999];
            OutDy = [-999, -999, -999, -999];
            [Rows, Cols] = size(Dx);
            for i = 2:Rows
                if ((Dx(i) ~= -999) & (Dy(i) ~= -999))
                    %Displacement
                    OutDx(i-1) = Dx(1) - Dx(i);
                    OutDy(i-1) = Dy(1) - Dy(i);
                    
                    % Zero and Span Calibration
                    OutDx(i-1) = OutDx(i-1)*Scale(1);
                    OutDy(i-1) = abs(OutDy(i-1)*cosd(15))*Scale(2);

                    % Calib
                    OutDx(i-1) = CalibrateXY(OutDx(i-1), CalibParamsX);
                    OutDy(i-1) = CalibrateXY(OutDy(i-1), CalibParamsY);
                %else
                    %OutDx(i-1) = -999;
                    %OutDy(i-1) = -999;
                end
            end
        end

        function PrintObjectDisplacements(Dx, Dy)
            [Rows, Cols] = size(Dx);
            fprintf("Object Displacements:\n")
            for i = 1:Cols
                if ((Dx(i)) ~= -999) && (Dy(i) ~= -999)
                    fprintf("Object(%d) Displacement: X: %d\tY: %d\n", i, Dx(i), Dy(i));
                end
            end
            fprintf("\n")
        end

        function ROSPublish(ROSPublisher, Dx, Dy)
            [Rows, Cols] = size(Dx);
            for i = 1:Cols
                if ((Dx(i)) ~= -999) && (Dy(i) ~= -999)
                    ObjMsg(i) = rosmessage(ROSPublisher(i));
                    ObjMsg(i).X = double(Dx(i));
                    ObjMsg(i).Y = double(Dy(i));
                    ObjMsg(i).Z = double(0);
                    send(ROSPublisher(i), ObjMsg(i));
                end
            end
            fprintf("Publish Complete\n")
        end
    end
end


function Dc = CalibrateXY(Dc, CalibParams)
            InputDc = Dc;
            [Rows, Cols] = size(CalibParams);

            for i = 2:Rows
                if ((CalibParams(i-1, 2) < InputDc) & (InputDc <= CalibParams(i, 2)))
                    % Calc Error Range
                    ErrorRange = CalibParams(i, 4);

                    % Add Base Error
                    Dc = InputDc + CalibParams(i-1, 3);

                    % Add Linear Error
                    Dc = Dc + ((InputDc/CalibParams(i, 2))*ErrorRange);
                end
            end
        end