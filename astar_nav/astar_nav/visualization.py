import cv2
import numpy as np
import matplotlib.pyplot as plt


def draw_path(
    image,
    path,
    start=None,
    goal=None,
    path_color=(255, 0, 0),
    thickness=2,
    show=False
):
    """
    Draws A* path on an image.

    :param image: BGR image
    :param path: list[(x, y)]
    :param start: (x, y)
    :param goal: (x, y)
    """
    vis = image.copy()

    if path:
        pts = np.array(path, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(vis, [pts], False, path_color, thickness)

    if start:
        cv2.circle(vis, start, 5, (0, 255, 0), -1)

    if goal:
        cv2.circle(vis, goal, 5, (0, 0, 255), -1)

    if show:
        plt.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
        plt.axis("off")
        plt.show()

    return vis
