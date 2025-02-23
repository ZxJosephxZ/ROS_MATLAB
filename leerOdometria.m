function [x, y, theta] = leerOdometria()
    odom = rossubscriber('/robot0/odom');
    pause(1); % Esperar a recibir datos

    % Obtener posición
    pos = odom.LatestMessage.Pose.Pose.Position;
    x = pos.X;
    y = pos.Y;

    % Obtener orientación en cuaterniones
    ori = odom.LatestMessage.Pose.Pose.Orientation;
    q = [ori.W, ori.X, ori.Y, ori.Z];

    % Convertir cuaterniones a ángulos de Euler (yaw)
    euler = quat2eul(q);
    theta = mod(rad2deg(euler(1)), 360); % En grados
end
