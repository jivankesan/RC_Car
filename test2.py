import math


def is_goal_reached(current_position: list[float, float], current_goal: list[float, float]) -> bool:
    if current_goal is None:
        print("No current goal set")
        return False

    radius = 1.1

    distance_squared = (current_goal[0] - current_position[0]) ** 2 + (
        current_goal[1] - current_position[1]
        ) ** 2

    distance = math.sqrt(distance_squared)

    return distance < radius   



if __name__ == "__main__":
    current_goal = [3,3]
    current_position = [2,3]

    print(is_goal_reached(current_goal, current_position))
    
    