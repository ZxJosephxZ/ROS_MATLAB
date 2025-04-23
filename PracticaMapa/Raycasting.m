function points = bresenham(x0,y0,x1,y1)
    x0 = round(x0); y0 = round(y0);
    x1 = round(x1); y1 = round(y1);

    dx = abs(x1 - x0); sx = sign(x1 - x0);
    dy = -abs(y1 - y0); sy = sign(y1 - y0);
    err = dx + dy;

    points = [];
    while true
        points = [points; x0 y0];
        if x0 == x1 && y0 == y1, break; end
        e2 = 2*err;
        if e2 >= dy
            err = err + dy;
            x0 = x0 + sx;
        end
        if e2 <= dx
            err = err + dx;
            y0 = y0 + sy;
        end
    end
end
