%Script for calculating stresses on a wind turbine

clear
clc

%%
%Define location of section points on x (all units in meters and kg and m/s)
points(1:11,1) = 0:0.1:1; 
%%
%area calculation
aerofoilDefinition = fopen('sg6042.txt', 'r');   %change text to load different set of points
fscanf(aerofoilDefinition,'%c %c %c',3);  %read first line + discard (just header titles)
aeroFoilPoints = fscanf(aerofoilDefinition, '%f %f %f', [3,Inf]);
aeroFoilPoints(3,:) = [];



%Area Claculations
[~,minIndex] = min(aeroFoilPoints(1,:));
foilYFuncHandle = @foilYFunc; %just returns the y points
foilArea = -simpsonInt(1,minIndex, aeroFoilPoints', foilYFuncHandle);
foilArea = foilArea - simpsonInt(minIndex,length(aeroFoilPoints), aeroFoilPoints', foilYFuncHandle);
%plot aerofoil
plot(aeroFoilPoints(1,:),aeroFoilPoints(2,:));
figure
%%
points(1:11,2) = foilArea * 0.0001;  %area points
%%

density = 945;
angVel = 100;
accuracy = 0.1;
%%
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

function yVal = foilYFunc(points)
    yVal = points(:,2);
end

%remember to devide by area at root to get stress
