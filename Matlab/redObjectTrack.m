%set up the PID parameters
Kp = 0.8;
Ki = 0.01;
Kd = 0.0;
dt = 0.1;
error = 0.0;
intergral_error = 0.0;

FingerTipPositionPrev = [0; 0];
last_error = 0.0;
pre_velocity = zeros(1,3,'uint32');
desired_angles = {[20, 3.0], [25, 3.0], [20, 3.0]};
desired_angle_index = 1;
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
position_values = [];

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

% Set a loop that stop after certain frames of aquisition
while(vid.FramesAcquired<=Inf)
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
%         disp(length(stats));        
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
        
        % Convert the bounding box coordinates to the new coordinate system
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
        plot(bc(1),bc(2), '-m+')
        a=text(bc(1)+15,bc(2), strcat('X: ', num2str(abs( round(bc(1)-size(data,2) ))), '    Y: ', num2str(abs(round(bc(2)-size(data,1))))));
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
        XRed(object, 1) = abs(round(bc(1)-size(data,2)));
        YRed(object, 1) = abs(round(bc(2)-size(data,1))); 

    end
    %calculate current angle(degree) and current velocity
    FingerTipPosition = [XRed(1,1);YRed(1,1)];
    BasePoint = [XRed(2,1);YRed(2,1)];
    [current_angle, current_velocity] = calculateAngleAndVelocity(FingerTipPosition, BasePoint, FingerTipPositionPrev, dt);
    disp(current_angle)
    disp(current_velocity)
    
    % Update previous finger tip position
    FingerTipPositionPrev = FingerTipPosition;
    
%     pause(dt);
%     position_values = [position_values FingerTipPosition]; %append the fingertip position into array
    
    %check if the last desired angle has been reached
    if desired_angle_index >= length(desired_angles)
        last_desired_angle_reached = true;
        return;
    end
    
    %get the current desired angle
    current_desired_angle = desired_angles{desired_angle_index}(1);
    current_duration = desired_angles{desired_angle_index}(2);

%     % get the current desired angle and its duration
%     [current_desired_angle, current_duration] = desired_angles{desired_angle_index};
%     
%     % update the current duration for the current desired angle if it hasn't reached the last desired angle
%     if ~last_desired_angle_reached
%         current_duration = current_duration + dt_value;
%     end
%     
%     % check if the current desired angle has been reached for the specified duration
%     if current_duration >= current_duration
%         % move on to the next desired angle
%         desired_angle_index = desired_angle_index + 1;
%         current_duration = 0.0;
%     end
%     % calculate the error between the current desired angle and the current angle
%     error = current_desired_angle - current_angle;
%     
%     %calculate the integral error & derivative error
%     
%     
%     %PID controller
%     FingerTipPositionPrev = FingerTipPosition; % Store current finger tip position as previous for next iteration

    hold off
end




%   Both the loops end here.

%   Stop the video aquisition.
stop(vid);

%   Flush all the image data stored in the memory buffer.
flushdata(vid);

%   Clear all variables
clear all

function[current_angle, current_velocity] = calculateAngleAndVelocity(FingerTipPosition, BasePoint, FingerTipPositionPrev, dt)
%calculate finger tip Velocity
FingerTipVelocity = [0;0];
if exist('FingerTipPositionPrev', 'var')
    FingerTipVelocityX = (FingerTipPosition(1) - FingerTipPositionPrev(1)) / dt;
    FingerTipVelocityY = (FingerTipPosition(2) - FingerTipPositionPrev(2)) / dt;
    FingerTipVelocity = [-FingerTipVelocityX; FingerTipVelocityY];    
end

%calculate finger tip angle
current_angle_pi = atan((FingerTipPosition(2)-BasePoint(2))/(FingerTipPosition(1)-BasePoint(1)));
current_angle = (current_angle_pi / pi) * 180;

current_velocity = FingerTipVelocity;
end

% function[current_velocity] = calculateVelocity()
% FingerTipPosition = [XRed(1,1);YRed(1,1)];
% FingerTipVelocity = [0;0]; % Initialize velocity as zero
% FingerTipPrev = FingerTipPosition; % Store previous finger tip position
%     
% % Measure time elapsed since last iteration
% tic;
% elapsedTime = toc;
% 
% % Calculate velocity if current and previous positions are available
% if exist('FingerTipPositionPrev', 'var')
%     FingerTipVelocityX = (FingerTipPosition(1) - FingerTipPositionPrev(1)) / toc;
%     FingerTipVelocityY = (FingerTipPosition(2) - FingerTipPositionPrev(2)) / toc;
%     FingerTipVelocity = [-FingerTipVelocityX;FingerTipVelocityY];
%     current_velocity = FingerTipVelocity;
% end
% 
% FingerTipPositionPrev = FingerTipPosition; % Store current finger tip position as previous for next iteration
% 
% end

