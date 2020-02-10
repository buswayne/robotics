clear all
close all
xyzPoints = load('pointCloud.mat');
ptCloud = pointCloud(xyzPoints.point);
figure
subplot(1,2,1)
pcshow(ptCloud)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Original Point Cloud')
% custom parameters of the algorithm
threshold = 0.005;
region = [0.4,0.6,-inf,0.2,0.1,inf];
sample = findPointsInROI(ptCloud,region);
cylinder_axis = [0,0,1];
% pcfitcylinder
[model, inliers, outliers] = pcfitcylinder(ptCloud,threshold,cylinder_axis,'SampleIndices',sample);
cylinder = select(ptCloud,inliers);
subplot(1,2,2)
pcshow(cylinder)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Cylinder')
fprintf('\n\nPosition of the cylinder:\n %i\n%i\n%i\n', model.Center);
fprintf('\n\nPose of the cylinder:\n %i\n%i\n%i\n', model.Orientation);
fprintf('\n\nRadius of the cylinder: %i\n', model.Radius);
fprintf('\n\nHeight of the cylinder: %i\n', model.Height);