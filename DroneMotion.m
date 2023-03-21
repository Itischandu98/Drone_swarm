function simulation=DroneMotion(N,pos,finalpointtaken,final)
    % matrix where we store the forces experienced by each robot for plotting
    force_collect = [];
    % temporary force matrix to be used for every robot/aircraft/drone
    force_temp = zeros(1,2);
    % activation radius for our force of repulsion
    rs = 3.0;
    % time chunk that we are simulating to be going through
    dt = 0.03; % 33 hz. 
    kp = 0.85; % formation_control_gain
    kc = 1.0; % formation_movement_gain
    %N = 3; % number of robots
    alpha = 0.5; % replusion_gain 
    beta = 0.3; % repulsion_ramp_gain
    plot(final(:,1),final(:,2),'r*'); % plotting our final destination/formation goal
    hold on; % don't erase the things on the plot. Keep them there. 
    for t=0:dt:10 % 0 seconds to 10 seconds 0,0.03,0.06....10
        dv=zeros(N,2); % velocity matrix for every instance
        force=zeros(N,2); % force vector matrix for every instance
        for i=1:1:N
            dv_temp = [0.0,0.0]; % temporary velocity variable
            for j=1:1:N % each robot
                dv_temp = dv_temp + pos(j,:)-pos(i,:)-(final(j,:)-final(i,:)); % velocity for formation control
                % force equation
                if norm(pos(j,:)-pos(i,:)) > 0 && norm(pos(j,:) - pos(i,:)) < rs % activate when below rs distance
                    force_repel = alpha*(exp(-beta*norm(pos(j,:)-pos(i,:))) - exp(-beta*rs));  % scalar force     
                    force_temp = force_temp + (pos(j,:)-pos(i,:))/norm(pos(j,:)-pos(i,:))*force_repel; % vector force
                else
                    force_temp = [0,0];
                end
            end
            dv(i,1) = dv_temp(1); % velocity x 
            dv(i,2) = dv_temp(2); % velocity y
            force(i,:) = force_temp;
            force_collect = [force_collect; force_temp];
        end
        % calculation of centroid
        centroid = zeros(1,2);
        for i=1:1:N
            centroid = centroid + pos(i,:);
        end
        centroid = centroid/N;
        for i=1:1:N
            pos(i,1) = pos(i,1) + (kp*dv(i,1) - centroid(1,1)*kc - force(i,1)+finalpointtaken(1))*dt;%with assumption of final point at origin
            pos(i,2) = pos(i,2) + (kp*dv(i,2) - centroid(1,2)*kc - force(i,2)+finalpointtaken(2))*dt;
        end
        c={'k','b','r','g','y','c','m'};
        for i=1:1:N
            plot(pos(i,1),pos(i,2),'color',c{i},'marker','.')
        end
        pause(dt);
    end