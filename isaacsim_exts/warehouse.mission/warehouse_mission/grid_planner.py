from __future__ import annotations

import heapq
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Point:
    x: float
    y: float


class GridRoutePlanner:
    """A* safety fallback used until Nav2 is actively connected."""

    def __init__(self, resolution: float = 0.25) -> None:
        self.resolution = resolution
        self.x_min, self.x_max = -7.0, 4.5
        self.y_min, self.y_max = -4.2, 4.2
        self._blocked = self._build_obstacles()

    def plan(self, start: Point, goal: Point) -> list[Point]:
        start_cell = self._to_cell(start)
        goal_cell = self._to_cell(goal)
        frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
        cost_so_far = {start_cell: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for neighbor in self._neighbors(current):
                new_cost = cost_so_far[current] + 1.0
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor, goal_cell)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        if goal_cell not in came_from:
            raise RuntimeError("No collision-free route was found.")

        cells = []
        current = goal_cell
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        points = [self._to_point(cell) for cell in cells]
        return self._simplify(points)

    def _build_obstacles(self) -> set[tuple[int, int]]:
        blocked = set()
        # Two pallet islands force the planner to make a visible route choice.
        rectangles = [(-0.5, 1.2, -1.0, 1.0), (1.8, 2.8, 0.5, 2.0)]
        for x0, x1, y0, y1 in rectangles:
            x = x0 - 0.45
            while x <= x1 + 0.45:
                y = y0 - 0.45
                while y <= y1 + 0.45:
                    blocked.add(self._to_cell(Point(x, y)))
                    y += self.resolution
                x += self.resolution
        return blocked

    def _neighbors(self, cell: tuple[int, int]):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            candidate = (cell[0] + dx, cell[1] + dy)
            point = self._to_point(candidate)
            if (
                self.x_min <= point.x <= self.x_max
                and self.y_min <= point.y <= self.y_max
                and candidate not in self._blocked
            ):
                yield candidate

    def _to_cell(self, point: Point) -> tuple[int, int]:
        return (
            round((point.x - self.x_min) / self.resolution),
            round((point.y - self.y_min) / self.resolution),
        )

    def _to_point(self, cell: tuple[int, int]) -> Point:
        return Point(
            self.x_min + cell[0] * self.resolution,
            self.y_min + cell[1] * self.resolution,
        )

    @staticmethod
    def _heuristic(left: tuple[int, int], right: tuple[int, int]) -> float:
        return abs(left[0] - right[0]) + abs(left[1] - right[1])

    @staticmethod
    def _simplify(points: list[Point]) -> list[Point]:
        if len(points) < 3:
            return points
        simplified = [points[0]]
        previous_direction = None
        for index in range(1, len(points)):
            direction = (
                math.copysign(1, points[index].x - points[index - 1].x)
                if points[index].x != points[index - 1].x
                else 0,
                math.copysign(1, points[index].y - points[index - 1].y)
                if points[index].y != points[index - 1].y
                else 0,
            )
            if previous_direction is not None and direction != previous_direction:
                simplified.append(points[index - 1])
            previous_direction = direction
        simplified.append(points[-1])
        return simplified

