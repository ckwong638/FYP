delete(instrfind({'Port'},{'COM4'}));
Kp = 0.8;
Ki = 0.01;
Kd = 0.0;
dt = 0.001;
error = 0.0;
intergral_error = 0.0;

FingerTipPositionPrev = [0; 0];
prev_velocity = [0; 0];
desired_angles = {[0, 5.0],[10, 5.0], [70, 5.0], [25, 5.0]};
% desired_angles = {[60, 10.0]};
desired_angle_index = 1;
current_duration = 0.0;
last_desired_angle_reached = false;
t_start = tic;

%creat a serial object for the Arduino 
s = serial('COM4', 'BaudRate', 115200);
fopen(s);


%initialize lists for storing data
time_values = [];
angle_values = [];
desired_angle_values = [];
desired_angle_duration_values = [];
control_signal_values = [];
position_values = [];

XRed(1, 1) = 0;
XRed(2, 1) = 0;
YRed(1, 1) = 0;
YRed(2, 1) = 0;

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
        
        
        %get the current desired angle and its duration
        current_desired_angle = desired_angles{desired_angle_index}(1);
        current_period = desired_angles{desired_angle_index}(2);
        t_elapsed = toc(t_start);
        % check if the current desired angle has been reached for the specified duration
        if t_elapsed >= desired_angles{desired_angle_index}(2)
            % move on to the next desired angle
            desired_angle_index = desired_angle_index + 1;
            
            % reset the timer for the new desired angle
            t_start = tic;
            
            % check if all desired angles have been reached
            if desired_angle_index > length(desired_angles)
                break;
            end
        end
        
        % adjust the PID controller output based on the remaining time for the current desired angle
        t_remaining = desired_angles{desired_angle_index}(2) - t_elapsed;
        % calculate the error between the current desired angle and the current angle
        error = current_desired_angle - current_angle;
        %
        %calculate the integral error & derivative error
        intergral_error = intergral_error + error * dt;
        derivative_error = (current_velocity - prev_velocity) / dt;
        
        %PID controller
        output = Kp * error + Ki * intergral_error + Kd * derivative_error;
%         fprintf('output is %d\n', round(output,3));
        %     disp('Output is ',output)
        
        % Update previous finger tip position and velocity
        FingerTipPositionPrev = FingerTipPosition;
        prev_velocity = current_velocity;
        %     disp(current_velocity)
        %     fprintf('duration remain: %d\n', t_elapsed);
        fprintf('desired angle is %d\n', current_desired_angle);
        fprintf('current  angle is %d\n', current_angle);
        %     disp('current time is ',t_elapsed)
        %     disp('current desired angle is ',current_desired_angle)
        %     disp(current_period)
        %     position_values = [position_values FingerTipPosition]; %append the fingertip position into array
        time_values = [time_values t_elapsed];
        angle_values = [angle_values current_angle];
        desired_angle_values = [desired_angle_values current_desired_angle];
        desired_angle_duration_values = [desired_angle_duration_values current_period];
        control_signal_values = [control_signal_values output];
        
        %define the voltage range of the smc itv
        Vmin = 0;
        Vmax = 5;
        
        %Define the min and max output values of pid controller
        PIDmin = -50;
        PIDmax = 50;
        
        PIDout = output;
        
        Vout_mapped = mapminmax(PIDout, Vmin, Vmax);
        
        Vout = Vmin + ((Vmax - Vmin) * (Vout_mapped - PIDmin)) / (PIDmax - PIDmin);
%         disp(Vout)
        
        fprintf(s, '%d\n', Vout);
    end
    
    %update the figure
    hold off
    drawnow
    
   
    
    
    
    
    
end

fclose(s);

%   Both the loops end here.

%   Stop the video aquisition.
stop(vid);

plot(time_values, angle_values, '-o', 'LineWidth', 2);
hold on;
plot(time_values, desired_angle_values, '-x', 'LineWidth', 2);
hold off;

% Add axis labels and title
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('Angle vs. Time');
legend('Angle', 'Desired Angle');

delete(vid);

%   Flush all the image data stored in the memory buffer.
flushdata(vid);

%   Clear all variables
clear all


function[current_angle, current_velocity] = calculateAngleAndVelocity(FingerTipPosition, BasePoint, ~, ~)
%calculate finger tip Velocity
persistent prev_time prev_pos

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


