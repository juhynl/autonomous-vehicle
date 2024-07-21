import numpy as np
from scipy.interpolate import splev
from scipy.optimize import minimize


def normalize(v):
    norm = np.linalg.norm(v,axis=0) + 0.00001
    return v / norm.reshape(1, v.shape[1])

def curvature(waypoints):
    '''
    ##### TODO #####
    Curvature as the sum of the normalized dot product between the way elements
    Implement second term of the smoothin objective.

    args: 
        waypoints [2, num_waypoints] !!!!!
    '''
    curvature = 0
    waypoints = waypoints.reshape(2, -1)
    for n in range(1, waypoints[0].size-1):
        v1 = waypoints[:, n+1] - waypoints[:, n]
        v2 = waypoints[:, n] -  waypoints[:, n-1]
        numerator = np.dot(v1, v2)
        denominator = (np.sqrt(np.einsum("i, i", v1, v1)) * np.sqrt(np.einsum("i, i", v2, v2)))
        curvature += numerator / denominator
    
    return curvature


def smoothing_objective(waypoints, waypoints_center, weight_curvature=40):
    '''
    Objective for path smoothing

    args:
        waypoints [2 * num_waypoints] !!!!!
        waypoints_center [2 * num_waypoints] !!!!!
        weight_curvature (default=40)
    '''
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints)**2)

    # derive curvature
    curv = curvature(waypoints.reshape(2,-1))

    return -1 * weight_curvature * curv + ls_tocenter


def waypoint_prediction(roadside1_spline, roadside2_spline, num_waypoints=6, way_type = "smooth"):
    '''
    ##### TODO #####
    Predict waypoint via two different methods:
    - center
    - smooth 

    args:
        roadside1_spline
        roadside2_spline
        num_waypoints (default=6)
        parameter_bound_waypoints (default=1)
        waytype (default="smoothed")
    '''
    if way_type == "center":
        ##### TODO #####
     
        # create spline arguments
        t = np.linspace(0, 1, num_waypoints)
        
        # derive roadside points from spline
        point1 = splev(t, roadside1_spline)
        point2 = splev(t, roadside2_spline)
        point1[1] = point1[1] + 96 - 68
        point2[1] = point2[1] + 96 - 68
        
        # derive center between corresponding roadside points
        waypoint1 = np.concatenate(point1)
        waypoint2 = np.concatenate(point2)
        waypoints_center = ((waypoint1 + waypoint2) / 2)

        # output way_points with shape(2 x Num_waypoints)
        way_points = waypoints_center

        return way_points
    
    elif way_type == "smooth":
        ##### TODO #####

        # create spline points
        t = np.linspace(0, 1, num_waypoints)
        
        # roadside points from spline
        point1 = splev(t, roadside1_spline)
        point2 = splev(t, roadside2_spline)
        point1[1] = point1[1]
        point2[1] = point2[1]
        
        # center between corresponding roadside points
        waypoint1 = np.concatenate(point1)
        waypoint2 = np.concatenate(point2)
        waypoints_center = ((waypoint1 + waypoint2) / 2).flatten()
        
        # init optimized waypoints
        waypoints = waypoints_center.copy()
        
        # optimization
        optimization = minimize(smoothing_objective, waypoints, waypoints_center)
        way_points = optimization.x
        
        return way_points.reshape(2,-1)


def target_speed_prediction(waypoints, num_waypoints_used=6,
                            max_speed=60, exp_constant=4.5, offset_speed=30):
    '''
    ##### TODO #####
    Predict target speed given waypoints
    Implement the function using curvature()

    args:
        waypoints [2,num_waypoints]
        num_waypoints_used (default=6)
        max_speed (default=60)
        exp_constant (default=4.5)
        offset_speed (default=30)
    
    output:
        target_speed (float)
    '''
    target_speed = max_speed- offset_speed
    target_speed *= np.exp(-1 * exp_constant * abs(num_waypoints_used - 2 - curvature(waypoints.flatten())))
    target_speed += offset_speed
    
    return target_speed