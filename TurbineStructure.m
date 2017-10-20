%Script for calculating stresses on a wind turbine

clear
clc

%Define points of turbine (all units in meters and kg and m/s)
points(1:11,1) = 1:11;

%area calculation
aerofoilDefinition = fopen('sg6042.txt', 'r');   %change text to load different set of points
fscanf(aerofoilDefinition,'%c %c %c',3);  %read first line + discard (just header titles)
aeroFoilPoints = fscanf(aerofoilDefinition, '%f %f %f', [3,Inf]);
aeroFoilPoints(3,:) = [];

plot(aeroFoilPoints(1,:),aeroFoilPoints(2,:));
figure


points(1:11,2) = 1:11;


density = 1070;
angVel = 10;
accuracy = 0.1;

betterPoints(:,1) = points(1,1):accuracy:points(end,1);
%spline interpolation of points
betterPoints(:,2) = spline(points(:,1), points(:,2), betterPoints(:,1));
plot(points(:,1), points(:,2), 'o', betterPoints(:,1), betterPoints(:,2));
%pass function into integration script
centIntFuncHandle = @centIntFunc;
centResult = simpsonInt(1, 11, points, centIntFuncHandle);
centResult = centResult * density * angVel^2;
fprintf('final centrifigual force is %2.2f \n', centResult)


%function for calculating centrifugal force at each dx
function vol = centIntFunc(points)
    vol = points(:,1) .* points(:,2);
end


%remember to devide by area at root to get stress
