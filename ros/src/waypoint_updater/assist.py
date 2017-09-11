# Support routines
import math

# TODO: refine this initial implementation
def nearest_waypoint(msg, x, y, t):
    nearest_idx = None
    lowest_dist = 9999999.9
    for idx, wp in enumerate(msg.waypoints):
        delta = waypoint_distance(wp, x, y)
        if delta < lowest_dist:
            lowest_dist = delta
            nearest_idx = idx
    num_points = len(msg.waypoints)
    nearest_wp = msg.waypoints[nearest_idx]
    if not is_waypoint_positive(nearest_wp, x, y, t): # Is it in front?
        nearest_idx += 1
    return nearest_idx

def is_waypoint_positive(wp, x, y, t):
    tx, ty = wp.pose.pose.position.x, wp.pose.pose.position.y
    dx = tx - x
    dy = ty - y
    lx = math.cos(-t)*dx - math.sin(-t)*dy
    return lx > 0.0

def waypoint_distance(wp, x, y):
    px, py = wp.pose.pose.position.x, wp.pose.pose.position.y
    dx = px - x
    dy = py - y
    return math.sqrt(dx*dx + dy*dy)


