figure(1)
hold on;
grid on;
%limit = 11;
%axis([-limit limit -limit limit -limit limit]);

ax=5.82;
ay=4.53;
az=6.51;
a=norm([ax ay az]);


    quiver3(0,0,0,ax,0,0, 'r', 'LineWidth', 3, 'ShowArrowHead', 'on', 'MaxHeadSize', 0.2, 'AutoScale', 'off')
    quiver3(0,0,0,0,ay,0, 'g', 'LineWidth', 3, 'ShowArrowHead', 'on', 'MaxHeadSize', 0.2, 'AutoScale', 'off')
    quiver3(0,0,0,0,0,az, 'b', 'LineWidth', 3, 'ShowArrowHead', 'on', 'MaxHeadSize', 0.2, 'AutoScale', 'off')
    quiver3(0,0,0,ax,ay,az, 'c', 'LineWidth', 3, 'ShowArrowHead', 'on', 'MaxHeadSize', 0.2, 'AutoScale', 'off')
    quiver3(0,0,0,3,0,0, 'k', 'LineWidth', 2, 'ShowArrowHead', 'on', 'MaxHeadSize', 0.2, 'AutoScale', 'off')
    quiver3(0,0,0,0,3,0, 'k', 'LineWidth', 2, 'ShowArrowHead', 'on', 'MaxHeadSize', 0.2, 'AutoScale', 'off')
    quiver3(0,0,0,0,0,3, 'k', 'LineWidth', 2, 'ShowArrowHead', 'on', 'MaxHeadSize', 0.2, 'AutoScale', 'off')

    axis square;
    axis equal;


    legend(num2str(ax),num2str(ay),num2str(az),num2str(norm([ax ay az])));