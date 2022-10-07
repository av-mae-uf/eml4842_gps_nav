# uf_support.py

# contains basic support files

import math
import numpy as np

D2R = np.pi/180.0
R2D = 180.0/np.pi

#########################################################################    
    
def get_intersection_of_two_lines(S1:np.array, SOL1:np.array, S2:np.array, SOL2:np.array)->np.array:
  # inputs -
  #   S1, SOL1 - coordinates of line 1 (if S1 and SOL1 define a screw, then the line along the screw will be determined)
  #   S2, SOL2 - coordinates of line 2
  # outputs - np.array([px, py, pz, w]) (coordinates of the point of intersection in homogeneou coordinates)
  #           if no intersection, it returns np.array([0.0,0.0,0.0, 999.0])

  mag1 = np.linalg.norm(S1)
  mag2 = np.linalg.norm(S2)
  S1   = S1/mag1
  SOL1 = SOL1/mag1
  S2   = S2/mag2
  SOL2 = SOL2/mag2
  
  h1   = np.dot(S1, SOL1)
  h2   = np.dot(S2, SOL2)

  SOL1 = SOL1 - h1*S1
  SOL2 = SOL2 - h2*S2   # now have line coordinates
  
  test1 = np.abs(np.dot(S1, S2))
  if (np.isclose(test1, 1.0, atol = 0.0001)):
    return np.append(S1,0)  # the lines are parallel, return the point at infinity
  
  mutual_moment = np.dot(S1, SOL2) + np.dot(S2, SOL1)
  
  if (not np.isclose(mutual_moment, 0.0, atol = 0.0001)):
    return np.array([0.,0.,0., 999.0])  # the lines do not intersect
  else:
    denom = 1 - test1*test1
    term1 = np.cross(S2, SOL2)
    term2 = -np.cross(np.dot(S1, S2)*S1, SOL2)
    term3 = np.dot(np.cross(S1, SOL1), S2) * S2
    intersection_pt = (term1+term2+term3)/denom
    
    return np.append(intersection_pt, 1)  # return the intersection point
#########################################################################
#def do_line_segments_intersect(line1_ptA:np.array, line1_ptB:np.array, line2_ptA:np.array, line2_ptB:np.array)->list[int, np.array2string]:
def do_line_segments_intersect(line1_ptA, line1_ptB, line2_ptA, line2_ptB):
  # inputs -
  #    [x,y,z] array of coordinates of points that define the line segments
  # outputs -
  #    [yes_no, np.array([px, py, pz])] - A true/false (1/0) value to indicate if the segments intersect, followed by an array of the coordinates of the intersection points

  S1   = line1_ptB - line1_ptA
  SOL1 = np.cross(line1_ptA, S1)
  S2   = line2_ptB - line2_ptA
  SOL2 = np.cross(line2_ptA, S2)

  int_pt = get_intersection_of_two_lines(S1, SOL1, S2, SOL2)
  
  if int_pt[3] != 1:
    # the infinite lines either do not intersect or are parallel
    return [0, np.array([0.0,0.0,0.0])]
  
  else:
    # see if the intersection point lies on both segments
    my_int_pt = int_pt[1:4]
    v1 = my_int_pt - line1_ptA
    v2 = my_int_pt - line1_ptB
    
    if np.sign(np.dot(S1, v1)) == np.sign(np.dot(S1, v2)):
      return [0, np.array([0.0,0.0,0.0])]  # the intersection point does not lie on the first segment
    
    v1 = my_int_pt - line2_ptA
    v2 = my_int_pt - line2_ptB

    if np.sign(np.dot(S2, v1)) == np.sign(np.dot(S2, v2)):
      return [0, np.array([0.0,0.0,0.0])]  # the intersection point does not lie on the second segment
      
    return [1, np.array(int_pt[0:3])]
#########################################################################
def is_point_in_planar_polygon(poly, pt):
  # inputs - poly, an n x 2 array of point coordinates  (all points are in the xy, plane)
  #          pt, an array containing the x, y coordinates of the point to be check 
  # outputs- 1 (true) if the point is within the polygon ;
  #          0 otherwise
  
  ang_rad = 0.0
  
  num_pts = poly.shape[0]
  
  for i in range(num_pts):
    v1 = poly[i,:] - pt
    v2 = poly[((i+1)%num_pts),:] - pt
    
    v1 = v1/np.linalg.norm(v1)
    v2 = v2/np.linalg.norm(v2)
    
    cosang = v1[0]*v2[0] + v1[1]*v2[1]  #np.dot(v1,v2)
    v1p = np.append(v1,0.0)
    v2p = np.append(v2,0.0)
    sinang = np.dot(np.array([0.0,0.0,1.0]), np.cross(v1p, v2p))
    
    ang_rad = ang_rad + math.atan2(sinang, cosang)
    
  if(np.isclose(ang_rad, 0.0, atol= 0.01)):
    return 0
  else:
    return 1
 
#########################################################################
def expand_planar_poly(old_poly, expand_dist):
  # inputs - old_poly: x,y coordinates of orginal polygon which is in xy plane
  #          expand_dist:  dist to offset the polygon
  # outputs- new_poly: x,y coordinates of the expanded polygon
  
  num_pts = old_poly.shape[0]
  
  new_poly = np.empty((num_pts,2))
  
  for i in range(num_pts):
    S1 = old_poly[(i+1)%num_pts,:] - old_poly[i,:]
    S2 = old_poly[(i+2)%num_pts,:] - old_poly[(i+1)%num_pts,:]
    
    S1 = S1/np.linalg.norm(S1)
    S2 = S2/np.linalg.norm(S2)
    
    S1perp = np.array([S1[1], -S1[0]])
    S2perp = np.array([S2[1], -S2[0]])
    
    test_pt = old_poly[i,:] + expand_dist*S1 + expand_dist*S1perp
    
    if is_point_in_planar_polygon(old_poly, test_pt):
      test_pt = old_poly[i,:] + expand_dist*S1 - expand_dist*S1perp
    
    SOL1 = np.cross(np.append(test_pt, 0.0), np.append(S1, 0.0))  # now have coordinates of lline offset from first line
    
    # now get second offset line
    test_pt = old_poly[(i+1)%num_pts,:] + expand_dist*S2 + expand_dist*S2perp
    
    if is_point_in_planar_polygon(old_poly, test_pt):
      test_pt = old_poly[i,:] + expand_dist*S2 - expand_dist*S2perp
    
    SOL2 = np.cross(np.append(test_pt, 0.0), np.append(S2, 0.0))  # now have coordinates of lline offset from second line
    
    int_pt = get_intersection_of_two_lines(np.append(S1,0.0), SOL1, np.append(S2,0.0), SOL2)
    
    new_poly[i,:] = int_pt[0:2]  # remember that we are working in the xy plane
  
  return new_poly
   

#########################################################################
def does_pt_lie_on_line_segment(check_pt, ptA, ptB):
  # inputs - x,y,z coordinates of pt A and pt B which define the line segment
  #          x,y,z coordinates of the point being checked to see if it is on the line segment
  # outputs- 1 (true) or 0 (false) depending on whether the check point is on the line segment
  
  temp = ptB - ptA
  S = temp/np.linalg.norm(temp)
  SOL = np.cross(ptA, S)
  
  SOLcheck = np.cross(check_pt, S)
  d_diff = np.linalg.norm(SOLcheck - SOL)
  
  if not np.isclose(d_diff, 0.0, atol=0.001):
    return 0  # the check_pt was not on the infinite line
  
  # now check if the check pt is on the line segment
  v1 = check_pt - ptA
  v2 = check_pt - ptB
  
  if (np.sign(np.dot(v1,v2)) == -1):
    return 1  #yes, the check point is on the line segment
  else:
    return 0
   
#########################################################################  
def FileCheck(fn):
    try:
      open(fn, "r")
      return 1
    except IOError:
      # the file did not exist
      return 0
#########################################################################
def value_near(val, goal, tol):
  if(val-tol < goal and val+tol>goal):
    return True
  else:
    return False

#########################################################################
def dist_between_point_and_line(P1, S1, SoL1):
  # P1 - coordinates of point 1 ; numpy array of floats ; length 3 ; units of length
  # S1 - direction of line ; numpy array of floats ; length 3 ; dimensionless
  # SoL1 - moment of line ; numpy array of floats ; length 3 ; units of length

  # will return a list with two items
  # first item is a float giving the distance from P1 to the line
  # second item is a numpy array of three float numbers which are the coordinates of the point on the line closest to the given point
  
  # will unitize S1
  mag = np.linalg.norm(S1)
  S1 = S1/mag
  SoL1 = SoL1/mag

  # check that S1 and SoL1 are perpendicular
  if (not value_near(np.dot(S1,SoL1), 0.0, 0.001)):
    return numpy.nan

  temp1 = np.cross(P1, S1)
  dvec  = np.cross(S1,SoL1-temp1)/np.dot(S1,S1)
  P2 = P1 + dvec

  return [numpy.linalg.norm(dvec), P2]
#########################################################################