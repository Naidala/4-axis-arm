init

q_out = q0;
S = [des_E(1)-start_E(1);
     des_E(2)-start_E(2);
     des_E(3)-start_E(3)];
vel = 25;

while 1
%     [L1, L2, L3] = direct_kynematics(q_out, par);
    Ja = jacobian(q_out, par);
    dq = Ja\S*(vel/norm(S))
    ee_vel = Ja*dq;
    clf
    hold on
    plot3([start_E(1) des_E(1)],[start_E(2) des_E(2)],[start_E(3) des_E(3)],'r','linewidth',2);
    plot_robot(q_out, par)
    pause(0.1)
    q_out = q_out + Ts*dq;
end
