import numpy as np

import cv2 as cv

from scipy.signal import convolve2d


MIN_X = -1.6
MAX_X =  1.6

MIN_Y = -0.8
MAX_Y =  2.4

RESOLUTION = 0.025


def filter_points(points: np.ndarray) -> np.array:
    mask_x = (points[:, 0] > MIN_X) & (points[:, 0] < MAX_X)
    mask_y = (points[:, 1] > MIN_Y) & (points[:, 1] < MAX_Y)

    return points[mask_x & mask_y]


def bresenham(grid, start, end):
    x0, y0 = start
    x1, y1 = end

    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    
    while True:
        try:
            grid[x0, y0] = 128
        except:
            break

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * error
        if e2 >= dy:
            if x0 == x1:
                break
            error = error + dy
            x0 = x0 + sx

        if e2 <= dx:
            if y0 == y1:
                break
            error = error + dx
            y0 = y0 + sy

    # grid[start[0], start[1]] = 255

    return grid


def match_grid(points: np.ndarray) -> np.ndarray:
    resolution = RESOLUTION

    dim_x = np.ceil((MAX_X - MIN_X) / resolution).astype(int)
    dim_y = np.ceil((MAX_Y - MIN_Y) / resolution).astype(int)

    grid = np.zeros((dim_x, dim_y), int)

    indices = np.round((points - np.array([MIN_X, MIN_Y])) / resolution).astype(int)
    origin = np.round(np.array([-MIN_X / resolution, -MIN_Y / resolution])).astype(int)

    for row in indices:
        grid = bresenham(grid, row, origin)
    
    grid[origin[0], origin[1]] = 255

    return grid


def remove_holes(grid: np.ndarray, iterations: int = 3) -> np.ndarray:
    kernel = np.array(
        [[0.0, 1.0, 0.0],
         [1.0, 1.0, 1.0],
         [0.0, 1.0, 0.0]], np.uint8)
    
    grid = np.sign(grid)
    grid = (255 * grid).astype(np.uint8)

    grid = cv.dilate(grid, kernel, iterations=iterations)
    grid = cv.erode(grid, kernel, iterations=iterations)
    grid = cv.erode(grid, kernel, iterations=5)
    
    return grid


def dynamic_window(grid: np.ndarray, length: int = 40) -> np.ndarray:
    ind = np.where(grid == 255)

    x = ind[0] * RESOLUTION + MIN_X
    y = ind[1] * RESOLUTION + MIN_Y

    angles = np.rad2deg(np.arctan2(y, x))

    valores = np.arange(0, 360, length)
    
    count = [np.count_nonzero((angles >= i) & (angles < i+length)) for i in valores]
    
    return np.deg2rad(valores[np.argmax(count)] + 0.5 * length)


def planning(angle) -> np.ndarray:
    t = np.linspace(0, 1.5, 10)
    
    x = t * np.cos(angle)
    y = t * np.sin(angle)
    
    path = np.concatenate((x[:, np.newaxis], y[:, np.newaxis]), axis=1)

    return path


def control(pointcloud):
    points = filter_points(pointcloud)

    grid = match_grid(points)
    grid = remove_holes(grid)
    angle = dynamic_window(grid)

    return angle
