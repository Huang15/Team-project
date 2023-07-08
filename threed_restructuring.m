imageDir = fullfile('C:\Users\84235\Desktop\tum 2023ss\computer vision\delivery_area_dslr_undistorted\delivery_area\images\dslr_images_undistorted');
% imageDir = fullfile('C:\Users\HUANG\Desktop\TUM\CV\project\kicker\images\dslr_images_undistorted');

imds = imageDatastore(imageDir);

images = cell(1, numel(imds.Files));
num_img =numel(imds.Files);
Points =cell(1, num_img);
Features=cell(1, num_img);
% indexPairs=cell(1, num_img);


for i = 1:num_img
    I = readimage(imds, i);
    I= im2gray(I);
    
    Points{i}= detectSIFTFeatures(I);
    [Features{i}, Points{i}]= extractFeatures(I, Points{i});
end

% max_indexPairs=cell(2,num_img);
% maxnum_indexPairs =0;
% for i =1:num_img
%     for j=1:num_img 
%         if j==i
%             continue
%         end
%         indexPairs = matchFeatures(Features{i}, Features{j},MaxRatio=0.7);
%         if numel(indexPairs) > numel(max_indexPairs{1,i})
%             max_indexPairs{1,i}=indexPairs;
%             max_indexPairs{2,i}=[i,j];
%             % max_indexPairs{3,i}=numel(indexPairs);
%         end
%     end
%     if numel(max_indexPairs{1,i}) > maxnum_indexPairs
%         maxnum_indexPairs=numel(max_indexPairs{1,i});
%         first_img =i;
%     end
% end

imageSize =[4135 6208];
focalLength =[3408.59 3408.87];
principalPoint=[3117.24 2064.07];
intrinsics =  cameraIntrinsics(focalLength,principalPoint,imageSize);


vSet = imageviewset;
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d, Points=Points{1});

for i = 2:numel(images)
    max_Pairs=0;
    for j =1:i-1
        indexPairs  = matchFeatures(Features{i-j}, Features{i}, Method="Approximate",...
            MaxRatio=0.6, Unique=true);

        if numel(indexPairs) >max_Pairs
            max_Pairs=numel(indexPairs);
            k=j;
            indexPairs_k=indexPairs;
        end

        % if size(indexPairs,1) >= 500 
        %     break
        % end
            
        % matchedPoints1 = Points{i-j}(indexPairs(:, 1));
        % matchedPoints2 = Points{i}(indexPairs(:, 2));
              
        % try
        % 
        %     [relPose, inlierIdx] = helperEstimateRelativePose(...
        %         matchedPoints1, matchedPoints2, intrinsics);
        % catch ME
        %     continue
        % end


    end
    
    matchedPoints1 = Points{i-k}(indexPairs_k(:, 1));
    matchedPoints2 = Points{i}(indexPairs_k(:, 2));


    [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1.Location, matchedPoints2.Location,...
        intrinsics);

    inlierPoints1 = matchedPoints1.Location(inlierIdx, :);
    inlierPoints2 = matchedPoints2.Location(inlierIdx, :);  
    
    [relPose, validPointFraction] = ...
    estrelpose(E, intrinsics, inlierPoints1(1:2:end, :),...
    inlierPoints2(1:2:end, :));


    % [relPose, inlierIdx] = helperEstimateRelativePose(...
    %     matchedPoints1, matchedPoints2, intrinsics);
    
        
    
    prevPose = poses(vSet, i-k).AbsolutePose;
    currPose = rigidtform3d(prevPose.A*relPose.A);
    
    vSet = addView(vSet, i, currPose, Points=Points{i});

    vSet = addConnection(vSet, i-k, i, relPose, Matches=indexPairs_k(inlierIdx,:));
    
    tracks = findTracks(vSet);

    camPoses = poses(vSet);

    xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);

    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, intrinsics, FixedViewId=1, ...
        PointsUndistorted=true);
    
    vSet = updateView(vSet, camPoses);
end

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


ptCloud = pointCloud(xyzPoints);
distanceThreshold = 1.0;  % 参数，可调整，它决定了哪些点被认为是同一对象的一部分
numClusterPoints = 100;  % 参数，可调整，它决定了一个对象最少需要多少个点

% 使用 pcsegdist 函数分割点云
labels = pcsegdist(ptCloud, distanceThreshold, 'NumClusterPoints', numClusterPoints);

% 使用labels来分割点云
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

minNumPointsForAlphaShape = 4;  % 参数，决定了创建 alpha shape 需要的最少点数

% 定义颜色映射（可修改这个，使用不同的颜色）
colors = jet(numel(segmentedClouds));

% 创建一个新的 figure
figure;

% 为每个簇创建一个 alpha shape，并在同一图中绘制出来
for i = 1:numel(segmentedClouds)
    % 提取当前簇的点并将其转换为double类型
    xyzPointsCurrent = double(segmentedClouds{i}.Location);
    
    % 检查点的数量
    if size(xyzPointsCurrent, 1) < minNumPointsForAlphaShape
        continue;
    end
    
    % 创建 alpha shape
    shp = alphaShape(xyzPointsCurrent(:,1), xyzPointsCurrent(:,2), xyzPointsCurrent(:,3));
    
    % 绘制 alpha shape，并为它指定颜色
    plot(shp, 'FaceColor', colors(i, :), 'EdgeColor', 'k');
    hold on;
end
hold off;

% 设置图的标题
title('3D Alpha Shapes for All Clusters');


% Helper function
function [relPose, inlierIdx] = ...
    helperEstimateRelativePose(matchedPoints1, matchedPoints2, intrinsics)

if ~isnumeric(matchedPoints1)
    matchedPoints1 = matchedPoints1.Location;
end

if ~isnumeric(matchedPoints2)
    matchedPoints2 = matchedPoints2.Location;
end

for i = 1:100
    % Estimate the essential matrix.    
    [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
        intrinsics);

    % Make sure we get enough inliers
    if sum(inlierIdx) / numel(inlierIdx) < .3
        continue;
    end
    
    % Get the epipolar inliers.
    inlierPoints1 = matchedPoints1(inlierIdx, :);
    inlierPoints2 = matchedPoints2(inlierIdx, :);    
    
    % Compute the camera pose from the fundamental matrix. Use half of the
    % points to reduce computation.
    [relPose, validPointFraction] = ...
        estrelpose(E, intrinsics, inlierPoints1(1:2:end, :),...
        inlierPoints2(1:2:end, :));

    % validPointFraction is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
    if validPointFraction > .8
       return;
    end
end
% After 100 attempts validPointFraction is still too low.
error('Unable to compute the Essential matrix');
end

