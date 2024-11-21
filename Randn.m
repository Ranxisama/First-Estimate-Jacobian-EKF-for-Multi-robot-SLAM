function randset = Randn(rowNum,colNum) 
% This function is used to generate a more 'stable' random value
Randset = randn(rowNum,colNum);

for rowOrd = 1:rowNum
    for colOrd = 1:colNum
        while abs(Randset(rowNum,colNum)) > 1
            Randset(rowNum,colNum) = randn;
        end
    end
end

randset = Randset;

end


