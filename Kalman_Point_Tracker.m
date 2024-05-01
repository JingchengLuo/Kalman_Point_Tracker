clear;
clc;
close all;
%Created by Jingcheng Luo

%% Custom Area
video_path='/Users/diphda/Desktop/ERCI/Kalman_Point_Tracker/example_video.mp4';%full file path
search_size = 10;%yellow box size

%% Pre-processing
video_reader = VideoReader(video_path);
object_frame = readFrame(video_reader);
video_player = vision.VideoPlayer('Position',[100,100,size(object_frame,2)/2,size(object_frame,1)/2]);
[h,w,~]=size(object_frame);

%% Start
disp('Welcome!')
disp('You are using the Kalamen Point Tracker.')
disp('Use the box to select an area of ​​interest. Double-click the box to complete.')
figure;
imshow(object_frame);
posi = imrect;
wait(posi);
interest_region = round(getPosition(posi));
close;

%detect feature points
% object_frame = imadjust(object_frame, [0.3, 0.7], [0, 1]);
select_points = detectMinEigenFeatures(im2gray(object_frame),'ROI',interest_region);

%configure point tracker
tracker = vision.PointTracker('MaxBidirectionalError',2);
initialize(tracker,select_points.Location,object_frame);

%configure kalman filter
MotionModel = 'ConstantVelocity';
InitialEstimateError = [1 1]*1e5;
MotionNoise = [100 100];
MeasurementNoise = 20;

kf = cell(size(select_points, 1), 1);
sum_points=cell(size(select_points, 1), 1);
pred_idx=cell(size(select_points, 1), 1);

for i = 1:length(kf)
    kf{i} = configureKalmanFilter(...
        MotionModel,...
        select_points.Location(i,:),...
        InitialEstimateError,...
        MotionNoise,...
        MeasurementNoise);
end

search_region={};
limit_points=[];

while hasFrame(video_reader)
    frame = readFrame(video_reader);

    % frame = imadjust(frame, [0.3, 0.7], [0, 1]);
    % frame = im2gray(frame);
    
    [mov_points, validity] = tracker(frame);

    for i = 1:length(kf)

        %Limit points within the image
        if isempty(sum_points{i})
            limit_points=[1 1];
        else
            limit_points=sum_points{i};
            limit_points=limit_points(end,:);
        end

        if all(limit_points(1,:) > 0) && limit_points(1,1) < w && limit_points(1,2) < h

            %save validity for each point
            if isempty(pred_idx{i})
                pred_idx{i}=validity(i);
            else
                old_points=pred_idx{i};
                new_points=[old_points;validity(i)];
                pred_idx{i}=new_points;
            end

            if validity(i) == 0
                mov_points(i,:) = predict(kf{i});

                %search region
                center = mov_points(i,:);
                top_bottom = center - search_size;
                search_region{i} = [top_bottom(1), top_bottom(2), 2*search_size, 2*search_size];

            else
                predict(kf{i});
                correct(kf{i}, mov_points(i,:));
            end

            %save the trajectory of the points
            if isempty(sum_points{i})
                sum_points{i}=mov_points(i,:);
            else
                old_points=sum_points{i};
                new_points=[old_points;mov_points(i,:)];
                sum_points{i}=new_points;
            end
        else
            mov_points(i,:)=[-1,-1];
            validity(i)=-1;
        end
    end

    %mark the moving points and predicted points
    out = insertMarker(frame, mov_points(validity==1,:), '+', 'MarkerColor', 'green');
    out = insertMarker(out, mov_points(validity==0,:), '+', 'MarkerColor', 'red');

    %mark the searching regions
    for s=1:length(search_region)
        if ~isempty(search_region)
            out = insertShape(out, 'Rectangle', search_region{s}, 'Color', 'yellow');
        end
    end

    video_player(out);
    % pause(1)
end


%plot the movement trajectory of the points
figure;
hold on;
for s=1:length(sum_points)
    raw_data = sum_points{s};
    valid = pred_idx{s};
    y_data = h - raw_data(:, 2) + 1;
    x_data = raw_data(:,1);
    plot(x_data(valid==1),y_data(valid==1),'Color','green','LineWidth',2);
    plot(x_data(valid==0),y_data(valid==0),'Color','red','LineWidth',2);
    xlim([0,w]);
    ylim([0,h]);
end
grid;
title('Points Trajectories')
xlabel('X position')
ylabel('Y position')

