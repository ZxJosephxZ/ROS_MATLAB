function avanzar(distancia, velocidad)
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    msg = rosmessage(pub);

    % Configurar velocidad
    msg.Linear.X = velocidad;
    msg.Angular.Z = 0; % No giramos

    % Tiempo estimado de movimiento
    tiempo = distancia / velocidad;
    r = robotics.Rate(10); % 10 Hz

    tic; % Iniciar temporizador
    while toc < tiempo
        send(pub, msg);
        waitfor(r);
    end

    % Detener robot
    msg.Linear.X = 0;
    send(pub, msg);
end
