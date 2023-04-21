function result = sqrt_of_quadratics(vector)
    %   This function takes a vector, pulls out the components, 
    %   and performs the square root of each of the components squared
    firstValue = vector(1);
    secondValue = vector(2);
    result = sqrt(firstValue^2 + secondValue^2);
    
end