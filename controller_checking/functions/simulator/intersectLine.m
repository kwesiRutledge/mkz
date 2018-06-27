function line = intersectLine(x, y, psi)
    L = 1;
    t = [-L/2,L/2];
    a = 0.17+0.33;
    X = x + a*cos(psi) + t*cos(psi+pi/2);
    Y = y + a*sin(psi) + t*sin(psi+pi/2);
    line = [X; Y];
end