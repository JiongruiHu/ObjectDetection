clear;
close all;
load('cameraParams108.mat')

imOrig = imread('chess_1.jpg');

[im,newOrigin] = undistortImage(imOrig,cameraParams108,'OutputView','same');

[imagePoints, boardSize] = detectCheckerboardPoints(im);
squareSize = 10; % centimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%gt_worldPoints(:,[1,2]) =  worldPoints(:,[2,1]);
worldPoints(:,2)= -worldPoints(:,2); 
gt_worldPoints = worldPoints -[10,-10];
% Note: R and t are the transformation from the world coordinate to the camera coordinate system.
[R,t] = extrinsics(imagePoints,gt_worldPoints,cameraParams108);
est_worldPoints = pointsToWorld(cameraParams108,R,t,imagePoints);
four_coners = [1,1; 224,1;1,172;224,172];
est = pointsToWorld(cameraParams108,R,t,four_coners);
% transformation from camera coodinate system to world coodinate
R_c2w = R';
t_c2w = -R_c2w*t';
%[roll,pitch,yaw] = rotation2RPY(R_c2w);
[roll,pitch,yaw] = rotation2RPY(R);

rDeg = rad2deg(roll)
pDeg = rad2deg(pitch)
yDeg = rad2deg(yaw)



imshow(im)
hold on
plot(imagePoints(:,1),imagePoints(:,2),'ro')
plot(imagePoints(end,1),imagePoints(end,2),'ro')
title("undistorted image")
hold off
figure;
hold on
%plot(gt_worldPoints(22,1), gt_worldPoints(22,2), 'ro')
plot(gt_worldPoints(1:2,1), gt_worldPoints(1:2,2), 'ro')
plot(est_worldPoints(1:4,1), est_worldPoints(1:4,2), 'gx')
plot(est_worldPoints(end,1), est_worldPoints(end,2), 'gx')
plot(est(:,1),est(:,2),'bx')

hold off
function [gamma, beta, alpha] = rotation2RPY(R)
% Roll: gamma
% Pitch: beta
% Yaw: alpha
beta = -asin(R(1,3));
gamma= atan2(R(3,2),R(3,3));
alpha = atan2(R(2,1),R(1,1));
end

