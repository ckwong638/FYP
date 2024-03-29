delete(instrfind({'Port'},{'COM4'}));
Kp = 0.01;
Ki = 0.00;
Kd = 0.0;
dt = 0.01;
error = 0.0;
intergral_error = 0.0;

FingerTipPositionPrev = [0; 0];
prev_velocity = [0; 0];
desired_angles = 40;
t_start = tic;

%creat a serial object for the Arduino
s = serial('COM4', 'BaudRate', 9600);
fopen(s);


%initialize lists for storing data
time_values = [];
angle_values = [];
desired_angle_values = [];
desired_angle_duration_values = [];
control_signal_values = [];
position_values = [];

XRed(2, 1) = 0;
YRed(1, 1) = 0;

%initialize persistent values




% Accquire video source
vid = videoinput('winvideo', 1, 'MJPG_1024x576');
src = getselectedsource(vid);

vid.FramesPerTrigger = inf;% frames per trigger--> set to inf so that can obtain real time data
set(vid, 'ReturnedColorspace', 'rgb') %return the RGB format
vid.FrameGrabInterval = 5;

% start the video aquisition here
start(vid)

% Set a loop that stop after certain frames of aquisition
while(vid.FramesAcquired<=Inf)
    
    data = getsnapshot(vid); % Get the snapshot of the current frame
    %     t_elapsed = toc(t_start);
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
    
    % measure the elapsed time since the last desired angle
    %     t_elapsed = toc(t_start);
    %calculate current angle(degree) and current velocity
    FingerTipPosition = [XRed(1,1);YRed(1,1)];
    BasePoint = [XRed(2,1);YRed(2,1)];
    [current_angle, current_velocity] = calculateAngleAndVelocity(FingerTipPosition, BasePoint, FingerTipPositionPrev, dt);
    %     disp(current_angle)
    %     disp(current_velocity)
    
    % Update previous finger tip position and velocity
    FingerTipPositionPrev = FingerTipPosition;
    prev_velocity = current_velocity;
             
    % calculate the error between the current desired angle and the current angle
    error = desired_angles - current_angle;
    %
    %calculate the integral error & derivative error
    intergral_error = intergral_error + error * dt;
    derivative_error = (current_velocity - prev_velocity) / dt;
    
    %PID controller
    output = Kp * error + Ki * intergral_error + Kd * derivative_error;
    fprintf('output is %d\n', round(output,3));
    %     disp('Output is ',output)
    
    % Update previous finger tip position and velocity
    FingerTipPositionPrev = FingerTipPosition;
    prev_velocity = current_velocity;
    %     fprintf('duration remain: %d\n', t_elapsed);
    fprintf('desired angle is %d\n', desired_angles);
    fprintf('current  angle is %d\n', current_angle);
    %     disp('current time is ',t_elapsed)
    %     disp('current desired angle is ',current_desired_angle)
    %     disp(current_period)
    %     position_values = [position_values FingerTipPosition]; %append the fingertip position into array
        
    %define the voltage range of the smc itv
    Vmin = 0;
    Vmax = 5;
    
    %Define the min and max output values of pid controller
    PIDmin = -10;
    PIDmax = 10;
        
    PIDout = output;
    
    Vout_mapped = mapminmax(PIDout, Vmin, Vmax);
    
    Vout = Vmin + ((Vmax - Vmin) * (Vout_mapped - PIDmin)) / (PIDmax - PIDmin);
    disp(Vout)
     
    fprintf(s, '%d\n', Vout);
 
end

fclose(s);

%   Both the loops end here.

%   Stop the video aquisition.
stop(vid);

delete(vid);

%   Flush all the image data stored in the memory buffer.
flushdata(vid);

%   Clear all variables
clear all


function[current_angle, current_velocity] = calculateAngleAndVelocity(FingerTipPosition, BasePoint, ~, ~)
%calculate finger tip Velocity
persistent prev_time prev_pos
persistent prev_error
persistent prev_integral_error
persistent prev_derivative_error

%get the current time using datatime function
curr_time = datetime('now');
if isempty(prev_pos) || isempty(prev_time)
    prev_pos = [FingerTipPosition(1); FingerTipPosition(2)];
    prev_time = curr_time;
    FingerTipVelocity = [0;0];
else
    curr_pos = [FingerTipPosition(1); FingerTipPosition(2)];
    
    dt = seconds(curr_time - prev_time);
    
    curr_pos(1) = curr_pos(1)*-1;
    displacement = curr_pos - prev_pos;
    FingerTipVelocity = displacement / dt; %pixels per second
    
    prev_pos = curr_pos;
    prev_time = curr_time;
end

%calculate finger tip angle
current_angle_pi = atan((FingerTipPosition(2)-BasePoint(2))/(FingerTipPosition(1)-BasePoint(1)));
current_angle = (current_angle_pi / pi) * 180;

current_velocity = norm(FingerTipVelocity);
end


