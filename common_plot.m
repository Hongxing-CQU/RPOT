function common_plot( X, Y , title_name)
% plot function
    [x_points_size,x_dim] = size(X);
    [y_points_size,y_dim] = size(Y);

    if x_dim ~= y_dim
        disp('Please check pointcloud dimension between x and y');
        exit(-1);
    end
 
    figure
    if x_dim == 2
        plot(X(:,1),X(:,2),'*r',Y(:,1),Y(:,2),'ob');
    elseif x_dim == 3
        plot3(X(:,1),X(:,2),X(:,3),'.r',Y(:,1),Y(:,2),Y(:,3),'.b');
    end
    legend('X:blue','Y:red');
    axis off
    set(0,'defaultfigurecolor','w');
    title(title_name);
end

