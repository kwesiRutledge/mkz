function plot_car(x, y, psi, color)
    a = 0.17 + 0.1;
    b = 0.16 + 0.1;
    W = 0.25;
    X = [a, -b, -b, a;
         W/2, W/2, -W/2, -W/2];
    T = [cos(psi), -sin(psi);
         sin(psi), cos(psi)];
    X = T*X;
    X(1,:) = X(1,:) + x;
    X(2,:) = X(2,:) + y;
    dp = (X(:,1) - X(:,2))/2;
    
    fill(X(1,:), X(2,:), color)
    hold on
    plot(x, y, 'ok', 'Linewidth',2)
    quiver(x,y,dp(1),dp(2),'MaxHeadSize',2,'Linewidth',2,'Color','k')
end