%Script for calculating stresses on a wind turbine

clear
clc
close all

%%
%Define location of section points on x (all units in meters and kg and m/s)
%0 is at the root
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
figure(1)
plot(aeroFoilPoints(1,:),aeroFoilPoints(2,:));
axis equal
figure(2)

%%
chord = [0.05,0.115,0.125,0.105,0.0935,0.077,0.0653,0.0565,0.0498,0.0444,0.0401,0.0365];
points(:,2) = foilArea * (chord).^2;  %area points

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
%%

%centroid
[xRoid, yRoid] = centroidCalc(foilArea, aeroFoilPoints);
figure(1)
hold on
scatter(xRoid,yRoid)


%is function of reynolds so remember to chech bruv
airDense = 1.225;
aoA = 7.4; %In degrees
cD = 0.02; 
cL = 1.4; 
velocity = ((points(:,1) .* angVel).^2 + (8)^2).^0.5; 
lift = cL .* 0.5 .* airDense .* velocity.^2 .* chord';
drag = cD * 0.5 * airDense .* velocity.^2 .* chord';


%%
%Torque calculations
xRoidDifference = xRoid .* chord - 0.25 .*chord;
yRoidDifference = yRoid .* chord;
lengthToCentroid = (xRoidDifference.^2 + yRoidDifference.^2).^0.5;
beta = rad2deg(atan(yRoidDifference(1) / xRoidDifference(1)));
gamma = beta - aoA;
react = lift .* cos(deg2rad(gamma)) - drag .* sin(deg2rad(gamma));
torque = lengthToCentroid' .* react;    %per unit span
%%
%Deflection Calculations
youngsMod = 2.18e6;%of abs
verticalDistReact = lift * cos(deg2rad(aoA)) + drag * sin(deg2rad(aoA));    %vertical distributed deflection 
horizontalDistReact = drag * cos(deg2rad(aoA)) - lift * sin(deg2rad(aoA));

momOfInertia = 4.98523914e-5; %for chord lenght of 1
scaledMomOfInertia = momOfInertia * chord .^4;

%vertical Deflec Integrations
yFuncHandle = @yFunc;
forceDistribution = intDistribution(points(:,1), verticalDistReact, yFuncHandle);
momentDistribution = intDistribution(points(:,1), forceDistribution', yFuncHandle);
d2vdz2 = intDistribution(points(:,1), (momentDistribution./(scaledMomOfInertia*youngsMod))', yFuncHandle);
dvdz = intDistributionFromRoot(points(:,1), d2vdz2', yFuncHandle);
deflection = intDistributionFromRoot(points(:,1), dvdz', yFuncHandle);

figure(3)
plot(points(:,1), deflection);


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

function intResult = intDistribution(radiusPoints,inputDistribution, yFuncHandle)
    for i = 1:length(inputDistribution)
        intResult(length(inputDistribution)-i+1) = simpsonInt(length(inputDistribution)-i+1,length(inputDistribution), [radiusPoints(:,1),inputDistribution], yFuncHandle);
    end
end

function intResult = intDistributionFromRoot(radiusPoints,inputDistribution, yFuncHandle)
    for i = 1:length(inputDistribution)
        intResult(i) = simpsonInt(1,i, [radiusPoints(:,1),inputDistribution], yFuncHandle);
    end
end

