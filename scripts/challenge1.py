#!/usr/bin/env python3
import fields2cover as f2c
import click

from romea_path_tools.path_planning_utils import discretize_swaths
from romea_path_tools.path_v2 import Path


class PathGenerator:

    def __init__(self, robot_width, operation_width, min_radius):
        self.swaths = None
        self.path = None
        self.polygon = None

        self.robot = f2c.Robot(robot_width, operation_width)
        self.robot.setMinRadius(min_radius)
        self.robot.cruise_speed = 1.
        self.robot.linear_curv_change = 0.4  # 1/m^2
        self.step_size = 0.1  # m

    def create_swaths_from_csv(self, filename):
        points = []
        with open(filename, 'r') as file:
            file.readline()
            for line in file.readlines():
                values = tuple(map(float, line.split(',')))
                points.append(values[3:6])
        print(f"points: {points}")

        self.swaths = f2c.Swaths()
        for a, b in zip(points[::2], points[1::2] + [points[0]]):
            line_string = f2c.LineString(
                f2c.Point(*a),
                f2c.Point(*b),
            )
            swath = f2c.Swath(line_string)
            self.swaths.emplace_back(swath)
        print(f"swaths: {self.swaths}")


    def path_planning(self):
        path_planner = f2c.PP_PathPlanning()

        turning = f2c.PP_DubinsCurves()
        # turning = f2c.PP_DubinsCurvesCC()
        # turning = f2c.PP_ReedsSheppCurves()
        # turning = f2c.PP_ReedsSheppCurvesHC()

        turning.discretization = self.step_size
        self.path = path_planner.searchBestPath(self.robot, self.swaths, turning)
        self.path = discretize_swaths(self.path, self.step_size)

    def visualize(self):
        f2c.Visualizer.figure()
        f2c.Visualizer.axis("equal")
        # f2c.Visualizer.plot(self.cell)
        f2c.Visualizer.plot(self.path)
        f2c.Visualizer.plot(self.swaths)
        f2c.Visualizer.show()

    def export_path(self, filename):
        tiara_path = Path()
        tiara_path.columns = ['x', 'y', 'speed']

        origin = (46.339159, 3.433923, 279.18)
        tiara_path.anchor = [origin[1], origin[0], origin[2]]

        TURN = f2c.PathSectionType_TURN
        SWATH = f2c.PathSectionType_SWATH
        previous_dir = None
        previous_type = TURN

        for i, state in enumerate(self.path.states):
            if previous_dir != state.dir:
                tiara_path.append_section([])

            if previous_type == SWATH and state.type == TURN:
                if i > 0:
                    tiara_path.append_annotation('zone_exit', 'work', i - 1)
                tiara_path.append_annotation('zone_enter', 'uturn', i)

            if previous_type == TURN and state.type == SWATH:
                if i > 0:
                    tiara_path.append_annotation('zone_exit', 'uturn', i - 1)
                tiara_path.append_annotation('zone_enter', 'work', i)

            state.velocity *= state.dir
            tiara_path.append_point((state.point.getX(), state.point.getY(), state.velocity))

            previous_dir = state.dir
            previous_type = state.type

        tiara_path.append_annotation('zone_exit', 'work', len(self.path.states) - 1)

        tiara_path.save(filename)


@click.command()
@click.argument('csv_file', required=True, type=click.Path(readable=True))
@click.option('-w', '--operation-width', default=1.58, type=click.FLOAT)
@click.option('-r', '--min-radius', default=4.0, type=click.FLOAT)
@click.option('-o', '--output', default='out.traj', type=click.Path(writable=True))
@click.option('--robot-width', default=1., type=click.FLOAT)
def main(csv_file, operation_width, min_radius, output, robot_width):
    if operation_width < robot_width:
        robot_width = operation_width

    pg = PathGenerator(robot_width, operation_width, min_radius)
    pg.create_swaths_from_csv(csv_file)
    pg.path_planning()
    pg.visualize()
    pg.export_path(output)


if __name__ == '__main__':
    main()
