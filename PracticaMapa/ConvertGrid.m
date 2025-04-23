function [ix, iy] = world_to_grid(x, y, cell_size, center)
    ix = round(x / cell_size + center);
    iy = round(y / cell_size + center);
end

