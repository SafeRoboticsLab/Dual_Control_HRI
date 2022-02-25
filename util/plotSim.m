function plotSim(planner, XR_in, XH_in, THETA, M_traj, option)
%	Plot simulation trajectories.
%
% Author: Haimin Hu (last modified 2021.11.11)

    params = planner.params;
    
    if option.N_interp == 1
        XR = XR_in;
        XH = XH_in;
    else
        % Interpolate data.
        XR = [];
        for i = 1:size(XR_in,1)
            XR = [XR; interp(XR_in(i,:), option.N_interp)];
        end

        XH = [];
        for i = 1:size(XH_in,1)
            XH = [XH; interp(XH_in(i,:), option.N_interp)];
        end
    end
    
    % Extend the road (default factor: 1.6).
    params.rd_len_ub = params.rd_len_ub*1.6;
    option.scale = 1;
    
    % Image rotation to align the front with the world-frame x-axis. 
    option.rotation = 0;
    
    % Image coordinates of the centre of the back wheels.
    option.centre = [348; 203];
    
    % Real world length for scaling.
    option.length = 4.5;
    option.fps = Inf;
    
    % Start and end time.
    if ~isempty(option.t_start)
        t_start = option.t_start;
        if t_start ~= 1
            t_start = t_start * option.N_interp;
        end
    else
        t_start = 1;
    end
        
    if ~isempty(option.t_end)
        t_end = option.t_end * option.N_interp;
    else
        t_end = length(XR);
    end

    % Transparency settings.
    if option.keep_traj && option.is_fading
%         alpha_vec = linspace(0.1,1,t_end-t_start+option.t_skip);
        alpha_vec = logspace(-1,0,t_end-t_start+option.t_skip);
    else
        alpha_vec = linspace(1,1,length(XR));
    end

    % Figure setup.
    fs = 25;
    f = figure('Color','white');
    
    % Plot in the relative coordinate.
    if strcmp(option.coordinate,'rel')
        f.Position = [1050 660 1500 665];
    % Plot in the absolute coordinate.
 	elseif strcmp(option.coordinate,'abs')
        f.Position = [0 660 3000 665];
    else
        error('Invalid coordinate option')
    end
    set(gca,'FontSize',fs)
    if ~option.UI
      set(gca,'Visible','off')
    end
    hold on
    daspect([1,1,1])
    
    % Initial (px,py) limits.
    road_h_lb = -XH_in(1,1)-4; % horizontal lower bound of the road
    if strcmp(option.coordinate,'rel')
        xlimSpan = [road_h_lb, params.rd_len_ub];
    elseif strcmp(option.coordinate,'abs')
        xlimSpan = [params.rd_len_lb + XH(1,t_start),...
            params.rd_len_ub + XR(1,t_start)];
    end
    ylimSpan = [params.rd_bd_min-1.5, params.rd_bd_max+1.5];
    xlim(xlimSpan)
    ylim(ylimSpan)
    
    % Plot the roads.
    if strcmp(option.coordinate,'rel')
        % Road color.
    	fill([road_h_lb,road_h_lb,params.rd_len_ub,params.rd_len_ub],...
             [params.rd_bd_min,params.rd_bd_max,params.rd_bd_max,...
             params.rd_bd_min],[191,191,191]/255,'LineStyle','none');
        % Road boundaries.
        plot(linspace(road_h_lb,params.rd_len_ub,2),...
             linspace(params.rd_bd_min,params.rd_bd_min,2),...
             'k-','LineWidth',5)
        plot(linspace(road_h_lb,params.rd_len_ub,2),...
             linspace(params.rd_bd_max,params.rd_bd_max,2),...
             'k-','LineWidth',5) 
        % Center line of the road.
        plot(linspace(road_h_lb,params.rd_len_ub,2),...
             linspace(0,0,2),'--','LineWidth',8,'Color',[255,255,255]/255) 
    elseif strcmp(option.coordinate,'abs')
        road_end = XR(1,end)+params.rd_len_ub;
        % Road color.
        fill([xlimSpan(1),xlimSpan(1),road_end,road_end],...
             [params.rd_bd_min,params.rd_bd_max,params.rd_bd_max,...
             params.rd_bd_min],[191,191,191]/255);
        % Road boundaries.
        plot(linspace(params.rd_len_lb,road_end,2),...
             linspace(params.rd_bd_min,params.rd_bd_min,2),...
             'k-','LineWidth',5)
        plot(linspace(params.rd_len_lb,road_end,2),...
             linspace(params.rd_bd_max,params.rd_bd_max,2),...
             'k-','LineWidth',5)
        % Center line of the road.
        plot(linspace(params.rd_len_lb,road_end,2),...
             linspace(0,0,2),'--','LineWidth',8,'Color',[255,255,255]/255)
    end

    % Plot vehicle movements.
    cnt = 1;
    for t = t_start:t_end
        
        if mod(t-t_start,option.t_skip)~=0 && t~=t_end 
            continue
        end
        
        % Human uncertainty model parameters.
        theta_distr_t = THETA(:, ceil(t / option.N_interp));
        M_distr_t = M_traj(:, ceil(t / option.N_interp));

        % Top-down view of the cars in the relative coordinate.
        if strcmp(option.coordinate,'rel')
            xR_plt = [XR(1,t)-XH(1,t); XR(2:3,t)];
            [option.image,~,option.alpha] = imread('car_robot_y.png');
            % Transparent snapshots.
            try
                option.alpha = option.alpha*alpha_vec(cnt);
            catch
                option.alpha = option.alpha*alpha_vec(end);
            end
            [~, hR] = plot_vehicle(xR_plt', 'model', option);
            xH_plt = [0; XH(2:3,t)];
            [option.image, ~, option.alpha] = imread('car_human.png');
            if ~option.keep_traj && t~=t_end && t>t_start
                delete(hH)
            end
            try
                option.alpha = option.alpha*alpha_vec(cnt);
            catch
                option.alpha = option.alpha*alpha_vec(end);
            end
            [~, hH] = plot_vehicle(xH_plt', 'model', option);
        % Top-down view of the cars in the absolute coordinate.
        elseif strcmp(option.coordinate,'abs')
            if t>t_start
                xlimSpan(1) = xlimSpan(1) + 0.8*(XH(1,t) -...
                    XH(1,t-option.t_skip));
                xlimSpan(2) = xlimSpan(2) + XR(1,t) -...
                    XR(1,t-option.t_skip);
                xlim(xlimSpan)
            end
            xR_plt = XR(1:3,t);
            [option.image,~,option.alpha] = imread('car_robot_y.png');
            % Transparent snapshots.
            option.alpha = option.alpha*alpha_vec(cnt);
            [~, hR] = plot_vehicle(xR_plt', 'model', option);
            xH_plt = XH(1:3,t);
            [option.image, ~, option.alpha] = imread('car_human.png');
            if ~option.keep_traj && t~=t_end && t>t_start
                delete(hH)
            end
            option.alpha = option.alpha*alpha_vec(cnt);
            [~, hH] = plot_vehicle(xH_plt', 'model', option);
        end

        % Axis and title.
        if option.UI
            if strcmp(option.coordinate,'rel')
                xlabel('$p_x^r$','Interpreter','latex','FontSize',1.2*fs)
            elseif strcmp(option.coordinate,'abs')
                xlabel('$p_x$','Interpreter','latex','FontSize',1.2*fs)
            end
            ylabel('$p_y$','Interpreter','latex','FontSize',1.2*fs)
            title(['$t = $',...
                num2str((t-1)*planner.ts/option.N_interp,'%.1f'),...
                ', $\mu^{\theta^l} =$ [',...
                num2str(theta_distr_t.l.mu(1),'%.2f'),...
                ',', num2str(theta_distr_t.l.mu(2),'%.2f'),...
                '], $\mu^{\theta^r} =$ [',...
                num2str(theta_distr_t.r.mu(1),'%.2f'),...
                ',', num2str(theta_distr_t.r.mu(2),'%.2f'),...
                '], $P(M) =$ [', num2str(M_distr_t(1),'%.2f'),...
                ',', num2str(M_distr_t(2),'%.2f'), ']'],...
                'Interpreter', 'latex', 'FontSize',1.2*fs)
        end
        
        % Pausing settings.
        if t == t_start
            pause(1.0)
        else
            pause(option.pause)
        end
        
        % Delete the last frame.
        if  t~=t_end
            if ~option.keep_traj
                delete(hR)
            end
        end
        cnt = cnt + option.t_skip;
    end
    if ~option.keep_traj
        delete(hH)
    end
end

