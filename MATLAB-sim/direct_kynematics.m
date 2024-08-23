function [L1, L2, L3] = direct_kynematics(q, par)
    th1 = q(1); th2 = q(2); th3 = q(3); a1 = par(1); a2 = par(2); a3 = par(3); d1 = par(4);
    L1 = [-a1*sin(th1); a1*cos(th1); d1];
    L2 = L1 + [-a2*cos(th2)*sin(th1); a2*cos(th2)*cos(th1); a2*sin(th2)];
    L3 = L2 + [-a3*cos(th2+th3)*sin(th1); a3*cos(th2+th3)*cos(th1); a3*sin(th2+th3)];
    
end