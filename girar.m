function girar(angulo, velocidad_angular)
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    msg = rosmessage(pub);
    if angulo < 0
        velocidad_angular = -abs(velocidad_angular); % Giro antihorario
    else
        velocidad_angular = abs(velocidad_angular);  % Giro horario
    end
    % Configurar velocidad angular
    msg.Linear.X = 0; % No avanzar
    msg.Angular.Z = velocidad_angular;

    % Convertir ángulo a tiempo de giro
    if angulo ~= 0
        tiempo = abs(deg2rad(angulo)) / abs(velocidad_angular);
    else
        tiempo = 5;
    end
    
    r = robotics.Rate(10);
    tic;
    while toc < tiempo
        send(pub, msg);
        waitfor(r);
    end

    % Detener giro
    msg.Angular.Z = 0;
    send(pub, msg);
end
