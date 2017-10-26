function foilArea = aeroFoilAreaCalc(aeroFoilPoints)
    yFuncHandle = @yFunc; %just returns the y points
    foilArea = -simpsonInt(1,length(aeroFoilPoints), aeroFoilPoints', yFuncHandle);
end

