function Theta = wrap(theta)
for Rnum = 1:size(theta,1)
while theta(Rnum,1) > pi
    theta(Rnum,1) = theta(Rnum,1) - 2 * pi;
end
while theta(Rnum,1) < - pi
    theta(Rnum,1) = theta(Rnum,1) + 2 * pi;
end
end
Theta = theta;
end