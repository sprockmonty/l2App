function foilArea = aeroFoilAreaCalc(aeroFoilPoints)
    [~,minIndex] = min(aeroFoilPoints(1,:));
    foilYFuncHandle = @foilYFunc; %just returns the y points
    foilArea = -simpsonInt(1,minIndex, aeroFoilPoints', foilYFuncHandle);
    foilArea = foilArea - simpsonInt(minIndex,length(aeroFoilPoints), aeroFoilPoints', foilYFuncHandle); %dimensionless
    
    function yVal = foilYFunc(points)
        yVal = points(:,2);
    end
end

