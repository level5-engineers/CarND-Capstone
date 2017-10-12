# Support routines
import math

# Works but very slow: goes through 10,000 waypoints each time
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
        nearest_idx = (nearest_idx + 1) % num_points
    return nearest_idx

# TODO: Debug this. It's fast (binary search) but makes mistakes.
def nearest_search(msg,x,y,t):
    not_found = True
    first = 0
    length = len(msg.waypoints)
    last = length - 1
    center = None
    while ((first <= last) and not_found):
        midpoint = int((first + last)/2)
        dist = waypoint_distance(msg.waypoints[midpoint], x, y)
        if (dist <= 2.8):
            center = midpoint
            not_found = False
        else:
            if x < msg.waypoints[midpoint].pose.pose.position.x:
                if msg.waypoints[midpoint].pose.pose.position.x <= 2339.44:
                    last = midpoint
                else:
                    first = midpoint
            else:
                if msg.waypoints[midpoint].pose.pose.position.x <= 2339.44:
                    first = midpoint
                else:
                    last = midpoint
    if center is None:
        return None
    nearest_idx = None
    lowest_dist = 9999999.9
    shorter_list = []
    start = 0
    #print "Length of base waypoints: ", length, "\n"
    #print "Center: ", center, "  "
    #print msg.waypoints[center].pose.pose.position.x, msg.waypoints[center].pose.pose.position.y, "\n"
    #print "Center-50:  ", msg.waypoints[center-50].pose.pose.position.x, msg.waypoints[center-50].pose.pose.position.y, "\n"
    #print "Center+50:  ", msg.waypoints[center+50].pose.pose.position.x, msg.waypoints[center+50].pose.pose.position.y, "\n"
    
    if center < 10:
        shorter_list = msg.waypoints[0:20]
    elif center > 10890:
        shorter_list = msg.waypoints[-30:]
        start = len-30
    else:
        shorter_list = msg.waypoints[center-10:center+10]
        start = center-10
    for idx, wp in enumerate(shorter_list):
        delta = waypoint_distance(wp, x, y)
        if (delta < lowest_dist and (x != wp.pose.pose.position.x and y != wp.pose.pose.position.y)):
            lowest_dist = delta
            nearest_idx = idx
    nearest_wp = msg.waypoints[nearest_idx + start]
    if not is_waypoint_positive(nearest_wp, x, y, t): # Is it in front?
        nearest_idx = (nearest_idx + start + 1) % length
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
