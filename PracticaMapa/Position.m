function [x, y, theta] = pose_robot(pose)
    x = double(pose.Position.X);
    y = double(pose.Position.Y);
    q = pose.Orientation;
    eul = quat2eul([q.W q.X q.Y q.Z]);
    theta = eul(1); % Yaw
end


