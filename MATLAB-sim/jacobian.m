function JA = jacobian(q, par)
    th1 = q(1); th2 = q(2); th3 = q(3); a1 = par(1); a2 = par(2); a3 = par(3);
    JA = [-cos(th1)*(a1+a2*cos(th2)+a3*cos(th2+th3)), sin(th1)*(a2*sin(th2)+a3*sin(th2+th3)),sin(th1)*a3*sin(th2+th3);
         -sin(th1)*(a1+a2*cos(th2)+a3*cos(th2+th3)), -cos(th1)*(a2*sin(th2)+a3*sin(th2+th3)),-cos(th1)*a3*sin(th2+th3);
         0                                          , a2*cos(th2)+a3*cos(th2+th3)            , a3*cos(th2+th3)];
end