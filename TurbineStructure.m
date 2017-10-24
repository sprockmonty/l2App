%Script for calculating stresses on a wind turbine

clear
clc
close all

%%
%Define location of section points on x (all units in meters and kg and m/s)
points(:,1) = 0:0.05:0.55;  %in m
%%
%area calculation
aerofoilDefinition = fopen('sg6042.txt', 'r');   %change text to load different set of points
fscanf(aerofoilDefinition,'%c %c %c',3);  %read first line + discard (just header titles)
aeroFoilPoints = fscanf(aerofoilDefinition, '%f %f %f', [3,Inf]);
aeroFoilPoints(3,:) = [];

%Area Claculations
foilArea = aeroFoilAreaCalc(aeroFoilPoints);
%plot aerofoil
plot(aeroFoilPoints(1,:),aeroFoilPoints(2,:));
axis equal
figure

%%
points(:,2) = foilArea * ([0.5e-1,1.57670e-01,1.46300e-01,1.17330e-01,9.34700e-02,7.70200e-02,6.52559e-02,5.65000e-02,4.97700e-02,4.44400e-02,4.01320e-02,3.64840e-02]).^2;  %area points
%%
density = 945;
TSR=7; %tip speed ratio
R=0.55;
v=12;
angVel = TSR*v/R; %v= air velocity, R= radius
accuracy = 0.01;
%%
betterPoints(:,1) = points(1,1):accuracy:points(end,1);
%spline interpolation of points
betterPoints(:,2) = spline(points(:,1), points(:,2), betterPoints(:,1));
plot(points(:,1), points(:,2), 'o', betterPoints(:,1), betterPoints(:,2));
%pass function into integration script
centIntFuncHandle = @centIntFunc;
centResult = simpsonInt(1, length(points), points, centIntFuncHandle);
centResult = centResult * density * angVel^2;
fprintf('Final centrifigual force is %2.2f N \n and stress is %2.2f MPa \n', centResult, centResult*1e-6 / points(1,2));

for i=1:length(points)
    centvect(i,1) = simpsonInt(i, length(points), points, centIntFuncHandle);
    centvect(i,1)= centvect(i,1) .* (density * angVel^2);
    %fprintf('Section %1.f centrifigual force is %2.2f N \n and stress is %2.2f MPa \n', i,centvect(i,1), centvect(i,1)*1e-6 ./ points(i,2));
end
%function for calculating centrifugal force at each dx
function vol = centIntFunc(points)
    vol = points(:,1) .* points(:,2);
end



%remember to devide by area at root to get stress
