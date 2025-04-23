function bayes_mapping(grid_size, cell_size, max_iter)
    rosinit;
    laser = rossubscriber('/scan');
    odom = rossubscriber('/odom');

    % Inicialización del mapa: todos con probabilidad 0.5 (desconocido)
    l0 = 0;                        % log-odds inicial (0.5)
    map_log_odds = zeros(grid_size, grid_size); 
    center = grid_size / 2;

    % Parámetros de sensores
    l_occ = log(0.9 / 0.1);        % log-odds de celda ocupada
    l_free = log(0.3 / 0.7);       % log-odds de celda libre

    figure;

    for k = 1:max_iter
        scan = receive(laser);
        pose = odom.LatestMessage.Pose.Pose;
        [xr, yr, theta] = pose_robot(pose);

        ranges = scan.Ranges;
        angles = double(scan.AngleMin) + (0:length(ranges)-1) * double(scan.AngleIncrement);

        for i = 1:length(ranges)
            r = double(ranges(i));
            if isinf(r) || isnan(r) || r > 10
                continue;
            end

            angle = theta + angles(i);
            x_hit = xr + r * cos(angle);
            y_hit = yr + r * sin(angle);

            % Convertir a coordenadas de grilla
            [ix_r, iy_r] = world_to_grid(xr, yr, cell_size, center);
            [ix_h, iy_h] = world_to_grid(x_hit, y_hit, cell_size, center);

            % Línea entre el robot y el punto de impacto (celdas libres)
            line = bresenham(ix_r, iy_r, ix_h, iy_h);

            for j = 1:size(line,1)-1
                xi = line(j,1); yi = line(j,2);
                if dentro_mapa(xi, yi, grid_size)
                    map_log_odds(xi, yi) = map_log_odds(xi, yi) + l_free;
                end
            end

            % Celda final: obstáculo
            if dentro_mapa(ix_h, iy_h, grid_size)
                map_log_odds(ix_h, iy_h) = map_log_odds(ix_h, iy_h) + l_occ;
            end
        end

        % Visualización
        prob_map = 1 - 1./(1 + exp(map_log_odds));  % de log-odds a probabilidad
        imagesc(prob_map'); colormap(gray); axis equal tight;
        title(['Iteración ', num2str(k)]);
        drawnow;
    end

    save('bayes_map.mat', 'map_log_odds');
    rosshutdown;
end

