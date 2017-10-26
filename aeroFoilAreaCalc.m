function foilArea = aeroFoilAreaCalc(aeroFoilPoints)
    foilYFuncHandle = @foilYFunc; %just returns the y points
    foilArea = -simpsonInt(1,length(aeroFoilPoints), aeroFoilPoints', foilYFuncHandle);
    
    function yVal = foilYFunc(points)
        yVal = points(:,2);
    end
end

