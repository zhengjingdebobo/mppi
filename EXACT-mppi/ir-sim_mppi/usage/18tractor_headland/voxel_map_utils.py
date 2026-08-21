from typing import List, Tuple, Union
import numpy as np
from shapely.geometry import Polygon, MultiPolygon, Point, box, mapping
import shapely.affinity
from shapely.ops import unary_union
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import copy
import rasterio
from rasterio.features import rasterize

def create_box(width=1.0, height=1.0, center=[0.0, 0.0], angle=0.0, is_deg=True):
    """Create a box shape"""
    if width <= 0 or height <= 0:
        raise ValueError("Both width and height should be positive.")

    rectangle = box(-0.5, -0.5, 0.5, 0.5)
    rectangle_scale = shapely.affinity.scale(rectangle, width, height)
    if not is_deg:
        angle = np.rad2deg(angle)
    rectangle_scale_rot = shapely.affinity.rotate(rectangle_scale, angle)
    rectangle_scale_rot_trans = shapely.affinity.translate(
        rectangle_scale_rot, center[0], center[1]
    )
    
    return rectangle_scale_rot_trans


def create_circle(center=[0, 0], radius=1):
    """Create an circle shape"""
    if radius < 0:
        raise ValueError("The radius should be positive.")
    
    return Point(center[:2]).buffer(radius)


def create_ellipse(a=1, b=1, center=[0, 0], angle=0, is_deg=True):
    """Create an ellipse shape"""
    if a <= 0 or b <= 0:
        raise ValueError("Both a and b should be positive.")

    circle = create_circle([0, 0], 1)
    circle_scale = shapely.affinity.scale(circle, a, b)
    if not is_deg:
        angle = np.rad2deg(angle)
    circle_scale_rot = shapely.affinity.rotate(circle_scale, angle)
    circle_scale_rot_trans = shapely.affinity.translate(
        circle_scale_rot, center[0], center[1]
    )
    return circle_scale_rot_trans


def create_polygon(vertices):
    """Create a polygon shape"""
    if len(vertices) < 3:
        raise ValueError("The minimum vertices number is 3.")

    try:
        polygon = Polygon(vertices)
        if not polygon.is_valid:
            raise ValueError("The provided vertices do not form a valid polygon.")
        return polygon
    except Exception as e:
        raise Exception(f"Cannot create a polygon with the given vertices! Error: {e}")


def invert_polygon(polygon):
    """Invert a polygon"""
    inverted_vertices = [(y, x) for x, y in polygon.exterior.coords]
    inverted_polygon = Polygon(inverted_vertices)

    return inverted_polygon


def inflate_polygon(polygon, inflation=1.0):
    """Inflate a polygon"""
    if inflation < 0:
        raise ValueError("Inflation should be positive.")
    
    return polygon.buffer(inflation)


def combine_polygons(polygon_list):
    """Combine polygons using convex hull"""
    if len(polygon_list) == 0:
        return Polygon()

    unioned_polygon = unary_union(polygon_list)
    convex_hull = unioned_polygon.convex_hull

    return convex_hull

class FeatureMap2D:
    """
    Feature Map in 2D Class
    """

    def __init__(
        self,
        x_min: float = 0.0,
        y_min: float = 0.0,
        x_max: float = 20.0,
        y_max: float = 20.0,
    ):
        # create boundary
        self.create_boundary(x_min, y_min, x_max, y_max)

        # obstacle polygons
        self.obs_polygons = []

    def create_boundary(self, x_min: float, y_min: float, x_max: float, y_max: float):
        if x_min >= x_max:
            raise ValueError("x_min should be smaller than x_max")
        if y_min >= y_max:
            raise ValueError("y_min should be smaller than y_max")

        self.x_min, self.y_min = float(x_min), float(y_min)
        self.x_max, self.y_max = float(x_max), float(y_max)
        self.map_size_x = self.x_max - self.x_min
        self.map_size_y = self.y_max - self.y_min
        self.boundary_box = box(self.x_min, self.y_min, self.x_max, self.y_max)

    def cast_polygon_within_box(self, polygon: Polygon):
        """Cast the polygon within the boundary box"""
        cast_polygon = polygon.intersection(self.boundary_box)
        if not isinstance(cast_polygon, Polygon):
            return None
        if cast_polygon.area < 0.1:
            return None
        return cast_polygon

    def add_polygon(self, polygon: Polygon, is_relative: bool = False):

        if is_relative:
            translated_polygon = shapely.affinity.translate(
                polygon, self.x_min, self.y_min
            )
            casted_polygon = self.cast_polygon_within_box(translated_polygon)
        else:
            casted_polygon = self.cast_polygon_within_box(polygon)

        if casted_polygon is not None:
            self.obs_polygons.append(casted_polygon)

    def add_polygons(self, polygon_list: List[Polygon], is_relative: bool = False):
        for polygon in polygon_list:
            self.add_polygon(polygon, is_relative)

    def pop(self):
        self.obs_polygons.pop()

    def reset(self):
        self.obs_polygons = []

    def get_polys(self) -> List[Polygon]:
        return self.obs_polygons

    def get_polys_num(self) -> int:
        return len(self.obs_polygons)

    def get_boundary(self) -> Tuple:
        return (self.x_min, self.y_min, self.x_max, self.y_max)

    def draw(
        self,
        ax: plt.Axes,
        fill: bool = True,
        alpha: float = 1.0,
        facecolor: str = "k",
        edgecolor: str = "k",
        hatch: str = None,
        linewidth: float = 1.5,
        title: str = "2D Feature Map",
    ):
        # plot boundary
        ax.plot(*self.boundary_box.exterior.xy, color="k", linestyle="-", linewidth=2.0)
        # patch = patches.Polygon(list(self.boundary_box.exterior.coords), color='gray', fill=True, alpha=0.1)
        # ax.add_patch(patch)

        # plot obstacle polygons
        for polygon in self.obs_polygons:
            patch = patches.Polygon(
                list(polygon.exterior.coords),
                fill=fill,
                alpha=alpha,
                facecolor=facecolor,
                edgecolor=edgecolor,
                hatch=hatch,
                linewidth=linewidth,
            )
            ax.add_patch(patch)

        # leave a margin for better visualization
        margin_x = self.map_size_x * 0.05
        margin_y = self.map_size_y * 0.05
        ax.set_xlim([self.x_min - margin_x, self.x_max + margin_x])
        ax.set_ylim([self.y_min - margin_y, self.y_max + margin_y])
        ax.set_aspect(1)
        ax.set_title(title)
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")

    def to_gridmap(
        self, resolution: float = 0.2, use_raster: bool = True
    ) -> "GridMap2D":
        if resolution <= 0:
            raise ValueError("Resolution should be positive.")

        grid_map = GridMap2D(
            self.x_min,
            self.y_min,
            self.map_size_x,
            self.map_size_y,
            resolution,
        )

        # Initialize an empty grid
        grid_size = grid_map.get_grid_size()
        grid = np.zeros(grid_size, dtype=int)

        # 1) Use rasterio (fast but inaccurate)
        if use_raster:
            # Create a transform
            transform = rasterio.transform.from_bounds(
                self.y_min,
                self.x_max,
                self.y_max,
                self.x_min,
                grid_size[1],
                grid_size[0],
            )

            # Rasterize each polygon
            for polygon in self.obs_polygons:
                # The rasterize function converts the polygon into a binary mask
                mask = rasterio.features.rasterize(
                    [(mapping(invert_polygon(polygon)), 1)],
                    out_shape=grid_size,
                    transform=transform,
                    fill=0,
                    all_touched=True,
                    dtype=np.uint8,
                )
                grid += mask

        # 2) manual (slow but accurate)
        else:
            multi_polys = MultiPolygon(self.obs_polygons)

            for x in np.arange(
                self.x_min + resolution / 2,
                self.x_max - resolution / 2 + 1e-5,
                resolution,
            ):
                for y in np.arange(
                    self.y_min + resolution / 2,
                    self.y_max - resolution / 2 + 1e-5,
                    resolution,
                ):
                    if multi_polys.contains(Point(x, y)):
                        x_id = np.floor((x - self.x_min) / resolution).astype(int)
                        y_id = np.floor((y - self.y_min) / resolution).astype(int)
                        grid[x_id][y_id] += 1

        # grid is a grid map where each cell contains 1 if it is inside a polygon and 0 otherwise
        np.putmask(grid, grid > 0, 1)

        grid_map.set_value_from_grid(grid)
        return grid_map


class GridMap2D:
    """
    Grid Map in 2D Class
    """

    def __init__(
        self,
        origin_x: float = 0.0,  # bottom-left
        origin_y: float = 0.0,
        map_size_x: float = 20.0,
        map_size_y: float = 20.0,
        resolution: float = 0.2,
    ):
        if map_size_x <= 0 or map_size_y <= 0:
            raise ValueError("Map size should be positive.")

        if resolution <= 0:
            raise ValueError("Resolution should be positive.")

        self.origin = np.array([origin_x, origin_y], dtype=float)
        self.resolution = resolution

        self.grid_size = (
            np.ceil(map_size_x / self.resolution).astype(int),
            np.ceil(map_size_y / self.resolution).astype(int),
        )

        if self.grid_size[0] <= 0 or self.grid_size[1] <= 0:
            raise ValueError("Grid size error.")

        self.map_size = (
            float(self.grid_size[0] * resolution),
            float(self.grid_size[1] * resolution),
        )

        # initialize grid map
        self.grid = np.zeros(self.grid_size, dtype=float)

    def __getitem__(self, indices: List) -> Union[float, None]:
        if self.grid is None:
            return

        if len(indices) != 2:
            raise ValueError("Indices should have size 2.")

        if indices[0] > self.grid.shape[0] - 1 or indices[1] > self.grid.shape[1] - 1:
            raise IndexError(f"{indices} out of index range of grid.")

        return self.grid[indices[0]][indices[1]]

    def set_value_from_grid(self, grid: np.ndarray) -> bool:
        if grid.shape != self.grid.shape:
            print(
                f"Error! Input has a dimension of {grid.shape} while current grid has a dimension of {self.grid.shape}."
            )
            return False

        self.grid = copy.copy(grid).astype(float)
        return True

    def set_value_from_pos(self, pos: np.ndarray, val: float) -> bool:
        index = self.pos_to_index(pos)
        if not index:
            return False

        flag = self.set_value_from_index(index, val)
        return flag

    def set_value_from_index(self, index: Tuple, val: float) -> bool:
        if not self.is_index_in_map(index):
            return False

        self.grid[index[0]][index[1]] = val
        return True

    def get_resolution(self) -> float:
        return self.resolution

    def get_origin(self) -> np.ndarray:
        return self.origin

    def get_map_size(self) -> Tuple:
        return self.map_size

    def get_grid_size(self) -> Tuple:
        return self.grid.shape

    def get_grid(self) -> np.ndarray:
        return self.grid

    def pos_to_index(self, pos: np.ndarray) -> Union[Tuple, None]:
        index = (
            np.floor((pos[0] - self.origin[0]) / self.resolution).astype(int),
            np.floor((pos[1] - self.origin[1]) / self.resolution).astype(int),
        )
        if self.is_index_in_map(index):
            return index
        return

    def index_to_pos(self, index: Tuple) -> np.ndarray:
        if not self.is_index_in_map(index):
            return

        pos = np.array([self.origin[0], self.origin[1]])
        pos[0] += (index[0] + 0.5) * self.resolution
        pos[1] += (index[1] + 0.5) * self.resolution
        return pos

    def is_index_in_map(self, index: Tuple) -> bool:
        result = (
            index[0] >= 0
            and index[0] <= self.grid_size[0] - 1
            and index[1] >= 0
            and index[1] <= self.grid_size[1] - 1
        )
        return result

    def is_pos_in_map(self, pos: np.ndarray) -> bool:
        result = (
            pos[0] >= self.origin[0]
            and pos[0] <= self.origin[0] + self.map_size[0]
            and pos[1] >= self.origin[1]
            and pos[1] <= self.origin[1] + self.map_size[1]
        )
        return result

    def is_state_valid(self, pos: np.ndarray) -> bool:
        index = self.pos_to_index(pos)
        if index:
            return self.grid[index[0]][index[1]] == 0
        return False


    def draw(
        self,
        ax: plt.Axes,
        use_index: bool = True,
        cmap: str = "gray_r",
        alpha: float = 0.85,
        title: str = "2D Grid Map",
    ):
        if use_index:
            ax.imshow(self.grid.T, cmap=cmap, origin="lower", alpha=alpha)
            ax.set_xlabel("x id")
            ax.set_ylabel("y id")
        else:
            boundary_box = box(
                self.origin[0],
                self.origin[1],
                self.origin[0] + self.map_size[0],
                self.origin[1] + self.map_size[1],
            )
            ax.plot(*boundary_box.exterior.xy, color="k", linestyle="-", linewidth=1.5)

            x = np.array(
                [
                    self.origin[0] + i * self.resolution
                    for i in range(self.grid.shape[0] + 1)
                ]
            )
            y = np.array(
                [
                    self.origin[1] + j * self.resolution
                    for j in range(self.grid.shape[1] + 1)
                ]
            )
            ax.pcolormesh(
                x,
                y,
                self.grid.T,
                cmap=cmap,
                edgecolor="k",
                linewidth=0.01,
                alpha=alpha,
            )

            # leave a margin for better visualization
            margin_x = self.map_size[0] * 0.05
            margin_y = self.map_size[1] * 0.05
            ax.set_xlim(
                [
                    self.origin[0] - margin_x,
                    self.origin[0] + self.map_size[0] + margin_x,
                ]
            )
            ax.set_ylim(
                [
                    self.origin[1] - margin_y,
                    self.origin[1] + self.map_size[1] + margin_y,
                ]
            )
            ax.set_xlabel("x")
            ax.set_ylabel("y")

        ax.set_aspect(1)
        ax.set_title(title)
        ax.invert_yaxis()
