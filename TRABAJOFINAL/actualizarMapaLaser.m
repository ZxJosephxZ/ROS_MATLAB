function logOddsMap = actualizarMapaLaser(logOddsMap, x0, y0, theta, scanMsg, resolution, l_occ, l_max, mapSize)
    minRange = 0.15;
    maxRange = 8.0;
    angleMin = scanMsg.AngleMin;
    angleInc = scanMsg.AngleIncrement;
    ranges = scanMsg.Ranges;
    
    for r = 1:length(ranges)
        range = ranges(r);
        if range < minRange || range > maxRange
            continue;
        end
        angle = angleMin + (r-1) * angleInc;
        worldTheta = theta + angle;

        x1 = x0 + range * cos(worldTheta);
        y1 = y0 + range * sin(worldTheta);

        [ix1, iy1] = world2map(x1, y1, resolution, mapSize);
        if ix1 > 0 && ix1 <= mapSize && iy1 > 0 && iy1 <= mapSize
            logOddsMap(iy1, ix1) = min(l_max, logOddsMap(iy1, ix1) + l_occ);
        end
    end
end

function [ix, iy] = world2map(x, y, resolution, mapHeight)
    ix = round(x / resolution) + 1;
    iy = mapHeight - round(y / resolution);
end

