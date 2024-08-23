function plot_robot(q, par)
    scale = 1000;
    [L1, L2, L3] = direct_kynematics(q,par);
    
    axis([-0.30*scale 0.30*scale -0.30*scale 0.30*scale 0 0.60*scale])
    
    axis vis3d
    
    plot3([0 L1(1)],[0 L1(2)], [0 L1(3)],'b','linewidth',2)
    plot3([L1(1) L2(1)],[L1(2) L2(2)],[L1(3) L2(3)],'-g','linewidth',2)
    plot3([L2(1) L3(1)],[L2(2) L3(2)],[L2(3) L3(3)],'-yo','linewidth',2)
    
    hold on
    grid on
    view(3)
    
    
end