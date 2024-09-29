function x = Rcon(x,Rcond)
x(abs(x)<Rcond) = 0;
end