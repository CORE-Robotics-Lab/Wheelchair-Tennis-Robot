from abc import abstractmethod

import geometry_msgs.msg

import planners.teb_planner as teb_planner

'''General planner wrapper'''
class Planner:
    @abstractmethod
    def best_path(self, start_pos: geometry_msgs.msg.Pose, goal_pos: list):
        return None


'''Interface for MoveIt TebPlanner'''
class TebPlanner(Planner):
    def __init__(self, init_node=False) -> None:
        super().__init__()

        teb_planner.spin_up(init_node)

    def best_path(self, start_pos: geometry_msgs.msg.Pose, goal_pos: list):
        return teb_planner.best_path(start_pos, goal_pos)
