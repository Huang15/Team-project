imageDir = fullfile('C:\Users\HUANG\Desktop\TUM\CV\project\kicker\images\dslr_images_undistorted');
% imageDir = fullfile('C:\Users\HUANG\Desktop\TUM\CV\project\kicker\images\dslr_images_undistorted');
imds = imageDatastore(imageDir);

images = cell(1, numel(imds.Files));
num_img =numel(imds.Files);
Points =cell(1, num_img);
Features=cell(1, num_img);
gray_img=cell(1, num_img);


for i = 1:num_img
    I = readimage(imds, i);
    gray_img{i}= medfilt2(im2gray(I));
    
end



% imageSize =[4135 6208];
% focalLength =[3408.59 3408.87];
% principalPoint=[3117.24 2064.07];

imageSize =[4137 6211];
focalLength =[3410.34 3409.98];
principalPoint=[3121.33 2067.07];
intrinsics =  cameraIntrinsics(focalLength,principalPoint,imageSize);

Points{1}= detectSIFTFeatures(gray_img{1});
[Features{1}, Points{1}]= extractFeatures(gray_img{1}, Points{1});

vSet = imageviewset;
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d, Points=Points{1});

for i = 2:numel(images)
    Points{i}= detectSIFTFeatures(gray_img{i});
    [Features{i}, Points{i}]= extractFeatures(gray_img{i}, Points{i});
    max_Pairs=0;
    for j =1:i-1
        indexPairs  = matchFeatures(Features{i-j}, Features{i});

        if numel(indexPairs) >max_Pairs
            max_Pairs=numel(indexPairs);
            k=j;
            indexPairs_k=indexPairs;
        end


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
% camPoses = poses(vSet);
% figure;
% plotCamera(camPoses, Size=0.2);
% hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
% pcshow(xyzPoints, VerticalAxis='y', VerticalAxisDir='down', MarkerSize= 45);
% grid on
% hold off

% % Specify the viewing volume.
% loc1 = camPoses.AbsolutePose(1).Translation;
% xlim([loc1(1)-5, loc1(1)+4]);
% ylim([loc1(2)-5, loc1(2)+4]);
% zlim([loc1(3)-1, loc1(3)+20]);
% camorbit(0, -30);


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


