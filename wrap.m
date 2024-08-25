function Theta = wrap(theta)
for Rnum_row = 1:size(theta,1)
    for Rnum_col = 1:size(theta,2)
        while theta(Rnum_row,Rnum_col) > pi
            theta(Rnum_row,Rnum_col) = theta(Rnum_row,Rnum_col) - 2 * pi;
        end
        while theta(Rnum_row,Rnum_col) < - pi
            theta(Rnum_row,Rnum_col) = theta(Rnum_row,Rnum_col) + 2 * pi;
        end
    end
end
Theta = theta;
end