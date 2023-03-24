%set up the PID parameters
Kp = 0.8;
Ki = 0.01;
Kd = 0.0;
dt = 0.01;
error = 0.0;
intergral_error = 0.0;
last_error = 0.0;
pre_velocity = zeros(1,3,'uint32');
desired_angles = {[20, 3.0], [25, 3.0], [20, 3.0]};
desired_angle_index = 0;
current_duration = 0.0;
cumulative_duration = 0.0;
last_desired_angle_reached = false;
time = 0.0;

%initialize lists for storing data
time_values = [];
angle_values = [];
desired_angle_values = [];
desired_angle_duration_values = [];
control_signal_values = [];









XRed(2, 1) = 0;
YRed(1, 1) = 0; 
% Accquire video source
vid = videoinput('winvideo', 1, 'MJPG_1024x576');
src = getselectedsource(vid);

vid.FramesPerTrigger = Inf;% frames per trigger--> set to inf so that can obtain real time data
set(vid, 'ReturnedColorspace', 'rgb') %return the RGB format
vid.FrameGrabInterval = 5;

% start the video aquisition here
start(vid)
% XRed(2, 1) = 0;
% YRed(1, 1) = 0; 
% Set a loop that stop after certain frames of aquisition
while(vid.FramesAcquired<=Inf)
    
    % init the colors
%     XRed = "";
%     YRed = "";
    
    data = getsnapshot(vid); % Get the snapshot of the current frame
    
    % Track red objects in real time
    % subtract the red component
    % from the grayscale image to extract the red components in the image.
    diff_im = imsubtract(data(:,:,1), rgb2gray(data));
    % Use a median filter to filter out noise
    diff_im = medfilt2(diff_im, [3 3]);
    % Convert the resulting grayscale image into a binary image.
    diff_im = im2bw(diff_im,0.18);
    
    % Remove all those pixels less than 300px
    diff_im = bwareaopen(diff_im,300);
    
    % Label all the connected components in the image.
    bw = bwlabel(diff_im, 8);
    
    % image blob analysis.
    % obtain a set of properties for each labeled region.
    stats = regionprops(logical(bw), 'BoundingBox', 'Centroid');
    % Display the image
    imshow(data)
    
    hold on
    
    % This is a loop to bound the red objects in a rectangular box.

    
    for object = 1:length(stats)
        disp(length(stats));
        maxXRed = -Inf;
        minXRed = Inf;
        maxYRed = -Inf;
        minYRed = Inf;
        
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
        
        % Convert the bounding box coordinates to the new coordinate system
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
        plot(bc(1),bc(2), '-m+')
        a=text(bc(1)+15,bc(2), strcat('X: ', num2str(abs( round(bc(1)-size(data,2) ))), '    Y: ', num2str(abs(round(bc(2)-size(data,1))))));
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
        XRed(object, 1) = abs(round(bc(1)-size(data,2)));
        YRed(object, 1) = abs(round(bc(2)-size(data,1))); 
%         disp(XRed);
%         disp(YRed);
%         class(XRed)

    end
    disp(XRed);
    disp(YRed);
    error = atan((YRed(1,1)-YRed(2,1))/(XRed(1,1)-XRed(2,1)));
    error = (error / pi) * 180;
    disp(error)
    disp(desired_angles{1}(2))
    %PID controller
    
    
    hold off
end


%   Both the loops end here.

%   Stop the video aquisition.
stop(vid);

%   Flush all the image data stored in the memory buffer.
flushdata(vid);

%   Clear all variables
clear all

