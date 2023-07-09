clc; clear;

%% Given an image folder

imageDir = fullfile('C:\Users\HUANG\Desktop\TUM\CV\project\delivery_area\images\dslr_images_undistorted');
% imageDir = fullfile('C:\Users\HUANG\Desktop\TUM\CV\project\kicker\images\dslr_images_undistorted');%Please change
imds = imageDatastore(imageDir);


%% Initialization

images = cell(1, numel(imds.Files));
num_img =numel(imds.Files);
Points =cell(1, num_img);
Features=cell(1, num_img);
gray_img=cell(1, num_img);

%% Loading and pre-processing images

for i = 1:num_img
    I = readimage(imds, i);
    gray_img{i}= medfilt2(im2gray(I));
end


%% Giving the camera inside reference
%Please change yourself
% imageSize =[4137 6211];
% focalLength =[3410.34 3409.98];
% principalPoint=[3121.33 2067.07];
imageSize =[4135 6208];
focalLength =[3408.59 3408.87];
principalPoint=[3117.24 2064.07];
intrinsics =  cameraIntrinsics(focalLength,principalPoint,imageSize);
%% Feature finding, matching, triangulation

% Give the features of the first picture
Points{1}= detectSIFTFeatures(gray_img{1});
[Features{1}, Points{1}]= extractFeatures(gray_img{1}, Points{1});

% Create an ensemble to store the features, matches, and camera positions for each image

vSet = imageviewset;
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d, Points=Points{1});

for i = 2:numel(images)
    Points{i}= detectSIFTFeatures(gray_img{i});
    [Features{i}, Points{i}]= extractFeatures(gray_img{i}, Points{i});
    max_Pairs=0;
    % Match the j graph with the previous 1~j-1 graphs and find the pair with the highest match
    for j =1:i-1
        indexPairs  = matchFeatures(Features{i-j}, Features{i});
        if numel(indexPairs) >max_Pairs
            max_Pairs=numel(indexPairs);
            k=j;
            indexPairs_k=indexPairs;
        end
    end
    % Select matched points.
    matchedPoints1 = Points{i-k}(indexPairs_k(:, 1));
    matchedPoints2 = Points{i}(indexPairs_k(:, 2));

    % estimateFundamentalMatrix
    [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1.Location, matchedPoints2.Location,...
        intrinsics);
    inlierPoints1 = matchedPoints1.Location(inlierIdx, :);
    inlierPoints2 = matchedPoints2.Location(inlierIdx, :);  
    
    % Estimate the camera pose of current view relative to the previous view.
    [relPose, validPointFraction] = ...
    estrelpose(E, intrinsics, inlierPoints1(1:2:end, :),...
    inlierPoints2(1:2:end, :));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-k).AbsolutePose;
    currPose = rigidtform3d(prevPose.A*relPose(1).A);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, currPose, Points=Points{i});
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-k, i, relPose, Matches=indexPairs_k(inlierIdx,:));
    
    tracks = findTracks(vSet);

    camPoses = poses(vSet);
    
    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, intrinsics, FixedViewId=1, ...
        PointsUndistorted=true);
    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);
end
%% Reconstruction shows

 % Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, Size=0.2);
hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, VerticalAxis='y', VerticalAxisDir='down', MarkerSize= 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.AbsolutePose(1).Translation;
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

%% Point cloud processing


ptCloud = pointCloud(xyzPoints);
% parameter, adjustable, which determines which points are considered to be part of the same object
distanceThreshold = 1.0; 
% parameter, which can be adjusted, determines the minimum number of points needed for an object
numClusterPoints = 100;  

% Splitting point clouds using the pcsegdist function
labels = pcsegdist(ptCloud, distanceThreshold, 'NumClusterPoints', numClusterPoints);

% Using labels to segment point clouds
segmentedClouds = cell(max(labels),1);
for i = 1:max(labels)
    segmentedClouds{i} = select(ptCloud, labels == i);
end

figure;
for i = 1:numel(segmentedClouds)
    pcshow(segmentedClouds{i});
    hold on;
end
hold off;
% parameter that determines the minimum number of points needed to create an alpha shape
minNumPointsForAlphaShape = 4;  

% Define the colour mapping (this can be modified to use a different colour)
colors = jet(numel(segmentedClouds));

% Create a new figure
figure;

% Create an alpha shape for each cluster and plot it on the same graph
for i = 1:numel(segmentedClouds)
    % Extracts the points of the current cluster and converts them to double type
    xyzPointsCurrent = double(segmentedClouds{i}.Location);

    % Verify that the number of points
    if size(xyzPointsCurrent, 1) < minNumPointsForAlphaShape
        continue;
    end

    % Create alpha shape
    shp = alphaShape(xyzPointsCurrent(:,1), xyzPointsCurrent(:,2), xyzPointsCurrent(:,3));

    % Draws an alpha shape and assigns a colour to it
    plot(shp, 'FaceColor', colors(i, :), 'EdgeColor', 'k');
    hold on;
end
hold off;

title('3D Alpha Shapes for All Clusters');


