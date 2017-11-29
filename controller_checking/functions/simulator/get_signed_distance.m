function d = get_signed_distance(X, Y, psi, x, y)
    psi = psi + pi/2;
    a = cos(psi);
    b = sin(psi);
    c = -cos(psi)*X - sin(psi)*Y;
    d = a*x+b*y+c;
end