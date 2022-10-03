import math
import numpy as np
from enum import Enum

D2R = np.pi/180.0
R2D = 180.0/np.pi

class myState(Enum):
  OFF          = 0
  ON           = 1
  OUTSIDE      = 2
  ENTRY_EXTENSION_PT = 3
  EXIT_EXTENSINON_PT = 4
  EXIT_TURN_PT = 5
  START        = 6
  END          = 7
  UTURN_PT1    = 8
  UTURN_PT2    = 9
  UTURN_PT3    = 10
  CORNER       = 11
  END_PLUS_ONE = 12
  LOOP         = 13

class route_pose_class:
  def __init__(self, pt=[0.0, 0.0, 0.0], heading_rad=0.0, state='a', w1_for_subsequent_segment=1.0, w2_for_subsequent_segment=1.0):
    self.pt = pt
    self.heading_rad = heading_rad
    self.state = state
    self.w1_for_subsequent_segment = w1_for_subsequent_segment
    self.w2_for_subsequent_segment = w2_for_subsequent_segment

class route_segment_class:
  def __init__(self, p0, p1, p2, p3, w1, w2, length, state):
    self.p0 = p0
    self.p1 = p1
    self.p2 = p2
    self.p3 = p3
    self.w1 = w1
    self.w2 = w2
    self.length = length
    self.state = state

#########################################################################
def create_route_segments(route_poses, want_loop):
  # inputs - route_poses: an array of 'route_pose_class'
  #
  # outputs- route_segments:  an array of 'route_segment_class'
  #          Note that there will ultimately be the same number of route_segments as there were route_poses.
  #          Either a stop extension segment will be added or a segment will be added to create the loop.
  
  route_segments = []
  
  num_points = len(route_poses)
  
  for i in range(num_points-1):  # there is one less route segment than there are route points (right now)
    p0 = route_poses[i].pt
    p3 = route_poses[i+1].pt
    
    w1 = route_poses[i].w1_for_subsequent_segment
    w2 = route_poses[i].w2_for_subsequent_segment
    
    L1L2dist = np.linalg.norm(p3-p0) / 4.0  # the route segment control points are 1/4 the distance between the ends (p0 & p3)
    if L1L2dist > 20.0:
      L1L2dist = 20.0
    
    S1 = np.array([math.cos(route_poses[i].heading_rad), math.sin(route_poses[i].heading_rad), 0.0])
    S2 = np.array([math.cos(route_poses[i+1].heading_rad), math.sin(route_poses[i+1].heading_rad), 0.0])
    
    p1 = p0 + L1L2dist * S1
    p2 = p3 - L1L2dist * S2
    
    # now get the length of the segment
    length = 0.0
    last_point = p0
    
    for u in np.arange(0.0, 1.001, 0.001):
      denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * w1 + 3.0 * u * u * (1.0 - u) * w2 + u * u * u;
      current_pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * w1 * p1 \
                  + 3.0 * u * u * (1.0 - u) * w2 * p2 + u * u * u * p3) / denom;
      length += np.linalg.norm(current_pt - last_point)
      last_point = current_pt
    
    route_segments.append(route_segment_class(p0, p1, p2, p3, w1, w2, length, route_poses[i].state))
 
  if(want_loop):
    # add a route segment from the last route point to the initial route point
    p0 = p3
    p3 = route_poses[0].pt
    w1 = route_poses[num_points-1].w1_for_subsequent_segment
    w2 = route_poses[num_points-1].w2_for_subsequent_segment
    S1 = np.array([math.cos(route_poses[num_points-1].heading_rad), math.sin(route_poses[num_points-1].heading_rad), 0.0])
    S2 = np.array([math.cos(route_poses[0].heading_rad), math.sin(route_poses[0].heading_rad), 0.0])
    L1L2dist = np.linalg.norm(p3-p0) / 4.0  # the route segment control points are 1/4 the distance between the ends (p0 & p3)
    if L1L2dist > 20.0:
      L1L2dist = 20.0
    p1 = p0 + L1L2dist * S1
    p2 = p3 - L1L2dist * S2
    
    # now get the length of the segment
    length = 0.0
    last_point = p0
    
    for u in np.arange(0.0, 1.001, 0.001):
      denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * w1 + 3.0 * u * u * (1.0 - u) * w2 + u * u * u;
      current_pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * w1 * p1 \
                  + 3.0 * u * u * (1.0 - u) * w2 * p2 + u * u * u * p3) / denom;
      length += np.linalg.norm(current_pt - last_point)
      last_point = current_pt
    
    route_segments.append(route_segment_class(p0, p1, p2, p3, w1, w2, length, route_poses[num_points-1].state))
    
  else:  # no loop; add a stop segment (make sure the distance (20 here) is larger than the look-ahead-distance
    p0 = p3
    p1 = p0 + 5.0 * S2
    p2 = p0 + 15.0 * S2
    p3 = p0 + 20.0 * S2
    w1 = 1.0
    w2 = 1.0
    length = 20.0
    #state = uf_dict["END_PLUS_ONE"]
    state = myState.END_PLUS_ONE.value

    route_segments.append(route_segment_class(p0, p1, p2, p3, w1, w2, length, state))
  
  return route_segments

#########################################################################
def get_point_on_route(route_segment:route_segment_class, \
                       u:float \
                      )-> np.array:
  #
  # inputs -
  #    route_segment: a route_segment
  #    u: the value of u along the route segment, 0 <= u <= 1
  # outputs -
  #    pt: the x,y,z coordinates of the point on the path

  denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * route_segment.w1 + 3.0 * u * u * (1.0 - u) * route_segment.w2 + u * u * u;
  
  pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * route_segment.p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * route_segment.w1 * route_segment.p1 + 3.0 * u * u * (1.0 - u) * route_segment.w2 * route_segment.p2 + u * u * u * route_segment.p3) / denom

  return pt  

#########################################################################
def get_route_length_to_u(route_segment:route_segment_class, \
                          uval:float \
                         )->float:
  #
  # inputs -
  #     route_segment: a route_segment_struct
  #     uval: the value of u on the path segment, 0 <= u <= 1
  # return - distance from point p0 of segment to the point designated by u

  length = 0.0
  last_pt = route_segment.p0

  for u in np.arange(0.0, uval, 0.001):
    denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * route_segment.w1 + 3.0 * u * u * (1.0 - u) * route_segment.w2 + u * u * u;
    pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * route_segment.p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * route_segment.w1 * route_segment.p1 + 3.0 * u * u * (1.0 - u) * route_segment.w2 * route_segment.p2 + u * u * u * route_segment.p3) / denom
    length += np.linalg.norm(pt-last_pt)
    last_pt = pt

  return length

#########################################################################
def get_u_value_for_dist(route_segment:route_segment_class, \
                          dist:float \
                         )->float:
  #
  # inputs -
  #     route_segment: a route_segment_struct
  #     dist: distance from point p0 of segment to the point designated by u
  # return - uval: the value of u on the path segment, 0 <= u <= 1, corresponding to dist
  #

  uval = 0.0
  mydist = 0.000000001  # did this in case input 'dist' is zero and then will output 'uval' equals zero
  last_pt = route_segment.p0

  while(mydist < dist and uval <= 1.0):
    uval += 0.001
    pt = get_point_on_route(route_segment, uval)
    mydist += np.linalg.norm(pt-last_pt)
    last_pt = pt

  return uval

#########################################################################
def get_heading_rad_at_u(route_segment:route_segment_class, u:float):
  # inputs:  route_segment
  #          u
  # outputs: heading_rad

  P0 = route_segment.p0
  P1 = route_segment.p1
  P2 = route_segment.p2
  P3 = route_segment.p3
  w1 = route_segment.w1
  w2 = route_segment.w2

  pyNumer = (-9 * P1 * w1 * w2 + 9 * P2 * w1 * w2 - 3 * w1 * P0 + 6 * P0 * w2 + 9 * w1 * P1 - 9 * P2 * w2 - 6 * P3 * w1 + 3 * P3 * w2 - 3 * P0 + 3 * P3) * u ** 4 \
          + (18 * P1 * w1 * w2 - 18 * P2 * w1 * w2 + 12 * w1 * P0 - 18 * P0 * w2 - 18 * w1 * P1 + 18 * P2 * w2 + 6 * P3 * w1 + 6 * P0 - 6 * P3) * u ** 3 \
          + (-9 * P1 * w1 * w2 + 9 * P2 * w1 * w2 - 18 * w1 * P0 + 18 * P0 * w2 + 18 * w1 * P1 - 18 * P2 * w2 - 3 * P0 + 3 * P3) * u ** 2 \
          + (12 * w1 * P0 - 6 * P0 * w2 - 12 * w1 * P1 + 6 * P2 * w2) * u \
          - 3 * w1 * P0 + 3 * w1 * P1


  pyDenom = 1 + (3 * w1 - 3 * w2) ** 2 * u ** 6 + 2 * (-6 * w1 + 3 * w2 + 3) * (3 * w1 - 3 * w2) * u ** 5 \
          + (2 * (3 * w1 - 3) * (3 * w1 - 3 * w2) + (-6 * w1 + 3 * w2 + 3) ** 2) * u ** 4 \
          + (6 * w1 - 6 * w2 + 2 * (3 * w1 - 3) * (-6 * w1 + 3 * w2 + 3)) * u ** 3 \
          + (-12 * w1 + 6 * w2 + 6 + (3 * w1 - 3) ** 2) * u ** 2 \
          + (6 * w1 - 6) * u

  dxdu = pyNumer[0]/pyDenom
  dydu = pyNumer[1]/pyDenom

  return (math.atan2(dydu, dxdu))

#########################################################################
def get_look_ahead_point(look_ahead_dist:float, \
                         vehicle_pt:np.array,   \
                         #route_segments:list[route_segment_class], \
                         route_segments, \
                         current_seg_num:int \
                        ) :
                        #)->list[
                        #        route_pose_class, \
                        #        np.array, \
                        #        int, \
                        #        int, \
                        #        int]:
  #
  # Find the closest point on the current vehicle path segment and the next path segment.
  # If the point on the next segment is closest, update the vehicle 'current' segment.
  #
  # inputs -
  #     look_ahead_dist: the distance from the closest point on the route to the look ahead point
  #     vehicle_pt: x,y,z coordinates of the vehicle at this instant
  #     route_segments: a list of route_segment_struct
  #     current_segment_num: the current route segment number to look for closest point
  #
  # outputs -
  #     look_ahead_pose:route_pose_class, 
  #     my_closest_pt:np.array(), 
  #     veh_seg_num:int, 
  #     look_ahead_seg_num:int, 
  #     stop_flag:int       Equals 1 if veh_seg_num has a state of END_PLUS_ONE
  #     

  num_segments = len(route_segments)

  #print('uf_nav_support; current seg = ', current_seg_num, ' num_segments = ', num_segments)

  if(current_seg_num > num_segments):
    print('The current segment number is ', current_seg_num,', but there are only ', num_segments, ' route segments.')
    return [0, 0, 0, 0, 0]
  
  closest_dist_current_seg = 99999.0
  closest_dist_next_seg    = 99999.0

  if (current_seg_num < num_segments):
    for u in np.arange(0.0, 1.001, 0.001):  # find closest pt on current and next segment
      pt_now = get_point_on_route(route_segments[current_seg_num], u)
      pt_next= get_point_on_route(route_segments[(current_seg_num+1)%num_segments], u)

      dist_now  = np.linalg.norm(pt_now  - vehicle_pt)
      dist_next = np.linalg.norm(pt_next - vehicle_pt)

      if (dist_now < closest_dist_current_seg):
        closest_dist_current_seg = dist_now
        closest_uval_current_seg = u
      if (dist_next < closest_dist_next_seg):
        closest_dist_next_seg = dist_next
        closest_uval_next_seg = u
    
    if(closest_dist_current_seg < closest_dist_next_seg):
      veh_seg_num = current_seg_num
      my_uval = closest_uval_current_seg
    else:
      veh_seg_num = (current_seg_num+1)%num_segments
      my_uval = closest_uval_next_seg

    # get the coordinates of the closest point
    my_closest_pt = get_point_on_route(route_segments[veh_seg_num], my_uval)

    # now get the look_ahead pose
    used_up_length = get_route_length_to_u(route_segments[veh_seg_num],my_uval)
    length_remaining_on_closest_seg = route_segments[veh_seg_num].length - used_up_length
    #print('vehicle seg num = ', veh_seg_num, ' uval' , my_uval)
    #print('used up length = ', used_up_length)
    #print('length remaining = ', length_remaining_on_closest_seg)

    if (length_remaining_on_closest_seg > look_ahead_dist):
      look_ahead_seg_num = veh_seg_num
      look_ahead_uval = get_u_value_for_dist(route_segments[veh_seg_num], used_up_length + look_ahead_dist)
      look_ahead_pt = get_point_on_route(route_segments[veh_seg_num], look_ahead_uval)
      look_ahead_heading_rad = get_heading_rad_at_u(route_segments[veh_seg_num], look_ahead_uval)

    else:  # look ahead pose is on a different segment
      mylength = look_ahead_dist - length_remaining_on_closest_seg
      look_ahead_seg_num = (veh_seg_num+1)%num_segments

      while (mylength > route_segments[look_ahead_seg_num].length):
        mylength -= route_segments[look_ahead_seg_num].length
        look_ahead_seg_num = (look_ahead_seg_num + 1) % num_segments

      look_ahead_uval = get_u_value_for_dist(route_segments[look_ahead_seg_num], mylength)
      look_ahead_pt = get_point_on_route(route_segments[look_ahead_seg_num], look_ahead_uval)
      look_ahead_heading_rad = get_heading_rad_at_u(route_segments[look_ahead_seg_num], look_ahead_uval)

    look_ahead_pose = route_pose_class(look_ahead_pt, \
                                     look_ahead_heading_rad, \
                                     route_segments[veh_seg_num].state, \
                                     route_segments[veh_seg_num].w1, \
                                     route_segments[veh_seg_num].w2)
  
  if (route_segments[veh_seg_num].state == myState.END_PLUS_ONE):
    stop_flag = 1  # STOP
  else:
    stop_flag = 0

  return [look_ahead_pose, my_closest_pt, veh_seg_num, look_ahead_seg_num, stop_flag]


#########################################################################
def get_slope_and_curvature_on_route():
  #  don't need this function right now
  return 1
#########################################################################
def get_radius_at_u(route_segment:route_segment_class, u:float):
  # inputs:  route_segment
  #          u
  # outputs: radius of curvature
  #
  P0 = route_segment.p0
  P1 = route_segment.p1
  P2 = route_segment.p2
  P3 = route_segment.p3
  w1 = route_segment.w1
  w2 = route_segment.w2

  pyNumer = (-9 * P1 * w1 * w2 + 9 * P2 * w1 * w2 - 3 * w1 * P0 + 6 * P0 * w2 + 9 * w1 * P1 - 9 * P2 * w2 - 6 * P3 * w1 + 3 * P3 * w2 - 3 * P0 + 3 * P3) * u ** 4 \
          + (18 * P1 * w1 * w2 - 18 * P2 * w1 * w2 + 12 * w1 * P0 - 18 * P0 * w2 - 18 * w1 * P1 + 18 * P2 * w2 + 6 * P3 * w1 + 6 * P0 - 6 * P3) * u ** 3 \
          + (-9 * P1 * w1 * w2 + 9 * P2 * w1 * w2 - 18 * w1 * P0 + 18 * P0 * w2 + 18 * w1 * P1 - 18 * P2 * w2 - 3 * P0 + 3 * P3) * u ** 2 \
          + (12 * w1 * P0 - 6 * P0 * w2 - 12 * w1 * P1 + 6 * P2 * w2) * u \
          - 3 * w1 * P0 + 3 * w1 * P1


  pyDenom = 1 + (3 * w1 - 3 * w2) ** 2 * u ** 6 + 2 * (-6 * w1 + 3 * w2 + 3) * (3 * w1 - 3 * w2) * u ** 5 \
          + (2 * (3 * w1 - 3) * (3 * w1 - 3 * w2) + (-6 * w1 + 3 * w2 + 3) ** 2) * u ** 4 \
          + (6 * w1 - 6 * w2 + 2 * (3 * w1 - 3) * (-6 * w1 + 3 * w2 + 3)) * u ** 3 \
          + (-12 * w1 + 6 * w2 + 6 + (3 * w1 - 3) ** 2) * u ** 2 \
          + (6 * w1 - 6) * u

  dxdu = pyNumer[0]/pyDenom
  dydu = pyNumer[1]/pyDenom

  pyNumer2 = 18 * (((3 * P1 - 3 * P2) * w2 + P0 - 3 * P1 + 2 * P3) * w1 + (-2 * P0 + 3 * P2 - P3) * w2 + P0 - P3) * (w1 - w2) * u ** 6 \
           - 108 * (((0.3e1 / 0.2e1 * P1 - 0.3e1 / 0.2e1 * P2) * w2 + P0 - 0.3e1 / 0.2e1 * P1 + P3 / 2) * w1 + (-0.3e1 / 0.2e1 * P0 + 0.3e1 / 0.2e1 * P2) * w2 + P0 / 2 - P3 / 2) * (w1 - w2) * u ** 5 \
           + 270 * (((0.3e1 / 0.5e1 * P1 - 0.3e1 / 0.5e1 * P2) * w2 + P0 - P1) * w1 + (-P0 + P2) * w2 + 0.2e1 / 0.15e2 * P0 - 0.2e1 / 0.15e2 * P3) * (w1 - w2) * u ** 4 \
           + (((-54 * P1 + 54 * P2) * w2 - 360 * P0 + 342 * P1 + 18 * P3) * w1 ** 2 + ((54 * P1 - 54 * P2) * w2 ** 2 + (540 * P0 - 324 * P1 - 216 * P2) * w2 + 42 * P0 - 18 * P1 - 24 * P3) * w1 + (-198 * P0 + 198 * P2) * w2 ** 2 + (-12 * P0 + 18 * P2 - 6 * P3) * w2 - 12 * P0 + 12 * P3) * u ** 3 \
           + ((270 * P0 - 270 * P1) * w1 ** 2 + ((-270 * P0 + 216 * P1 + 54 * P2) * w2 - 72 * P0 + 54 * P1 + 18 * P3) * w1 + (54 * P0 - 54 * P2) * w2 ** 2 + 18 * P0 - 18 * P3) * u ** 2 \
           + ((-108 * P0 + 108 * P1) * w1 ** 2 + 54 * (P0 - P1) * (w2 + 0.2e1 / 0.3e1) * w1 + (18 * P0 - 18 * P2) * w2 - 6 * P0 + 6 * P3) * u \
           + (18 * P0 - 18 * P1) * w1 ** 2 + (-6 * P0 + 6 * P1) * w1 - 6 * w2 * (P0 - P2)

  pyDenom2 = 27 * (0.1e1 / 0.3e1 + (w1 - w2) * u ** 3 \
           + (-2 * w1 + w2 + 1) * u ** 2 \
           + (w1 - 1) * u) ** 3

  d2xdu2 = pyNumer2[0]/pyDenom2
  d2ydu2 = pyNumer2[1]/pyDenom2

  radius_of_curvature = (dxdu ** 2 + dydu ** 2) ** (0.3e1 / 0.2e1) / (dxdu * d2ydu2 - d2xdu2 * dydu)

  if(radius_of_curvature > 100):
    radius_of_curvature = 100.0
  elif(radius_of_curvature < -100):
    radius_of_curvature = -100.0

  return radius_of_curvature

###########################################
def get_radius_at_u_equals_0(route_segment:route_segment_class):
  # inputs:  route_segment
  # outputs: radius of curvature at u=0
  #
  P0 = route_segment.p0
  P1 = route_segment.p1
  P2 = route_segment.p2
  P3 = route_segment.p3
  w1 = route_segment.w1
  w2 = route_segment.w2

  deriv_at_0 = -3 * w1 * (P0 - P1)
  dxdu = deriv_at_0[0]
  dydu = deriv_at_0[1]

  deriv2_at_0 = (18 * P0 - 18 * P1) * w1 ** 2 + (-6 * P0 + 6 * P1) * w1 - 6 * w2 * (P0 - P2)
  d2xdu2 = deriv2_at_0[0]
  d2ydu2 = deriv2_at_0[1]

  radius_of_curvature = (dxdu ** 2 + dydu ** 2) ** (0.3e1 / 0.2e1) / (dxdu * d2ydu2 - d2xdu2 * dydu)

  if(radius_of_curvature > 200):
    radius_of_curvature = 200.0
  elif(radius_of_curvature < -200):
    radius_of_curvature = -200.0

  return radius_of_curvature
#########################################################################
def update_vehicle_pose(old_position, old_heading_deg, rad_of_curvature, speed):
# outputs - new_position, np.array of floats
#           new_heading_deg, rad_of_curvature, speed, float
  #print(old_position, old_heading_deg, rad_of_curvature, speed)
  if (abs(rad_of_curvature) >= 150.0):
    Svec = np.array([math.cos(D2R*old_heading_deg), math.sin(D2R*old_heading_deg), 0.0])
    new_position = old_position + speed * Svec
    new_heading_deg = old_heading_deg
  else:
    rear_axle_dir_rad = (old_heading_deg + 90.0)*D2R
    center_pt = np.array([old_position[0] + rad_of_curvature * math.cos(rear_axle_dir_rad), \
                          old_position[1] + rad_of_curvature * math.sin(rear_axle_dir_rad), \
                          0.0])
    direction = np.sign(rad_of_curvature)    # positive is a left hand turn

    Svec = old_position - center_pt  # vector from center point to old vehicle position
    radius = np.linalg.norm(Svec)
    rotation_ang_rad = speed / radius  # speed represents dist to trave in this time step

    new_heading_deg = old_heading_deg + (direction * rotation_ang_rad)*R2D

    if (direction == 1):  # left turn
      temp = np.array([Svec[0] *math.cos(rotation_ang_rad) - Svec[1]*math.sin(rotation_ang_rad), \
                        Svec[0] *math.sin(rotation_ang_rad) + Svec[1]*math.cos(rotation_ang_rad), \
                        0.0])
      new_position = center_pt + temp
    elif (direction == -1):  # right turn
      temp = np.array([Svec[0] *math.cos(-rotation_ang_rad) - Svec[1]*math.sin(-rotation_ang_rad), \
                        Svec[0] *math.sin(-rotation_ang_rad) + Svec[1]*math.cos(-rotation_ang_rad), \
                        0.0])
      new_position = center_pt + temp

  return [new_position, new_heading_deg]

#########################################################################  
def get_rad_of_curvature_to_carrot(vehicle_point, vehicle_heading_rad, look_ahead_point, look_ahead_heading_rad, p1_ratio):
  # inputs -  vehicle_point, xyz coordinates of vehicle location
  #           vehicle_heading_rad, orientation of vehicle
  #           look_ahead_point, xyz coordinates of look-ahead point
  #           look_ahead_heading_rad, desired orientation at the look-ahead point
  #           p1_ratio, used to get distance to points p1 and p2
  # outputs-  radius of curvature
  #           driveme, the route segment that was created to move towards the carrot

  p0 = vehicle_point
  p3 = look_ahead_point

  pdist = np.linalg.norm(p3-p0)

  p1 = p0 + pdist*p1_ratio * np.array([math.cos(vehicle_heading_rad), math.sin(vehicle_heading_rad), 0.0])
  p2 = p3 - pdist*p1_ratio * np.array([math.cos(look_ahead_heading_rad), math.sin(look_ahead_heading_rad), 0.0])

  # p0, p1, p2, p3, w1, w2, length, state
  
  driveme = route_segment_class(p0, p1, p2, p3, 1.0, 1.0, 8.0, 0)

  radius_of_curvature = get_radius_at_u_equals_0(driveme)
  
  return [radius_of_curvature, driveme]

#########################################################################  