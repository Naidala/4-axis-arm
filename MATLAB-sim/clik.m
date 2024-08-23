init

[L1, L2, L3] = direct_kynematics(q0, par);

q_out = q0;
steps = 0;

while norm(des_E-L3)>0.01
    [L1, L2, L3] = direct_kynematics(q_out, par);
    Ja = jacobian(q_out, par);
    [U, S, V] = svd(Ja);
    min(eig(S))
    K = [5,5,5];
    if norm(des_E - L3) > 11
        K = [0.5,0.5,0.5];
    end
    dq = Ja\(diag(K)*(des_E - L3));
    clf
    hold on
    plot3(des_E(1), des_E(2), des_E(3), 'ro', 'linewidth',2);
    plot_robot(q_out, par)
%     plot([step], [q_out(1)]);
    pause(0.05)
    q_out = q_out + Ts*dq;
    steps = steps + 1;
end