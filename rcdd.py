
from numpy import *

class infini_plane : 

    c_wpos = array([0,0,0])
    # n_vec : normal vector of plane at world coords
    #n_vec = array([0,0,1])
    l2w    = array([[1,0,0],
                    [0,1,0],
                    [0,0,1]])
 
    def __init__ (self, wpos, rot) :
        self.c_wpos = wpos
        self.l2w = rot
        return

def chk_col_cylplane (plane, cyl) :

    #c2p = dot(plane.l2w, cyl.l2w.T)
    c_plane_pos = dot(plane.l2w.T, cyl.c_wpos)
    # c2p_dist distance to plate from center
    c2p_dist = c_plane_pos[2]

pattern2x2 = array([[+1.0, +1.0],
                 [+1.0, -1.0],
                 [-1.0, +1.0],
                 [-1.0, -1.0]])
pattern2x2 = pattern2x2[:,:,newaxis]
    
class cylinder :
    # c_wpos : center of volume at world coords
    # l2w    : local to world rotation matrix
    # t_lpos : top pos at loocal coords
    c_wpos = array([0,0,0])
    l2w    = array([[1,0,0],
                    [0,1,0],
                    [0,0,1]])
    r     = 1.0
    l     = 1.0

    #    2r
    #  |<-->|
    #   ____ ----
    #  (____)  ^
    #  |    |  | 2l
    #  |    |  v
    #  `----`----
    #

    def __init__ (self, wpos, r, l, rot) :
        self.c_wpos = wpos
        self.l2w = rot
        self.r   = r
        self.l   = l
        return

    def chk_col_plane (self, plane) :
        return self.chk_col2plane(plane.c_wpos, plane.l2w)

    # calc bottom point of cylinder at woorld coords.
    def chk_col2plane (self, wpos, l2w) :
        c2p = dot(l2w.T, self.l2w)

        # cylinder center to bottom circle center vector at plane local coords
        #tvec = array([0, 0, (self.l)])
        tvec = array([0, 0, 1])
        tvec = dot(c2p, tvec)

        # at plane local coords
        work = cross(tvec, array([0, 0, 1]))
        work = work / linalg.norm(work)
        if work.any() == False :
            work = array([1, 0, 0])
        rvec = cross(work, tvec)

        tvec = self.l * tvec
        rvec = self.r * rvec

        # cylinder center to 4vertex vector at plane local coords
        vertex_vec = sum(pattern2x2 * array([tvec, rvec]), axis=1)
        # cylinder center pos at plane local coords
        cpos = dot(l2w.T, self.c_wpos - wpos)
        # 4 vertex pos at plane local coords
        vertex_pos = cpos + vertex_vec
        #min_idx  = argmin(vertex_pos[:,2])
        min_z = min(vertex_pos[:,2])

        #print(vertex_pos)
        #print(vertex_vec)

        if  min_z <= 0 :
            min_idx, = where(vertex_pos[:,2] <= 0)
            vertex_vec = dot(c2p.T, vertex_vec[min_idx].T).T
            #print("vertex_pos : {0}".format(vertex_pos[min_idx, 2]))
            #vertex_pos =  array([0,0,1]) * vertex_pos[min_idx, 2]
            vertex_pos =  (array([0,0,1])[:,newaxis] * vertex_pos[min_idx, 2]).T
            return (True, vertex_vec, vertex_pos)
            #return (True, dot(c2p.T, vertex_vec[min_idx].T).T, array([0,0,1]) * vertex_pos[min_idx,2])
         
        return (True, None, min_z)


        #tpos = array([0, 0, (self.l)])
        #print("tpos : ", tpos);
        #tpos = dot(self.l2w, tpos)
        #print("tpos : ", tpos);
        #
        #l_nvec = dot(tpos, nvec)
        #l_pvec = sqrt(self.l * self.l - l_nvec * l_nvec)
        ## triangle similarity
        ##  l : l_pvec = r : r_nvec
        ##                 r * l_pvec
        ##   ==> r_nvec = ------------
        ##                     l
        #r_nvec = (self.r * l_pvec) / self.l
        ## c2s_dist distance to surface from center
        #c2s_dist = l_nvec + r_nvec

        #plate2cyl_vec = self.c_wpos - wpos
        ## c2p_dist distance to plate from center
        #c2p_dist = dot(plate2cyl_vec, nvec)

        #print("   c2p: {0}".format(c2p_dist))     # 2.0
        #print("   c2s: {0}".format(c2s_dist))     # 2.0
        #print("w_nvec: {0}".format(nvec))         # [0,0,1]
        #print("l_nvec: {0}".format(dot(self.l2w.T, nvec))) # [0,-1,0]
        #print("c_wpos: {0}".format(self.c_wpos))

        #if c2p_dist <= c2s_dist :
        #    return (True, dot(self.l2w.T, (c2p_dist * nvec)) - self.c_wpos)
        # 
        #return (False, None)

if __name__ == "__main__": 
    plane_rot = array([[1,0,0],[0,1,0],[0,0,1]])

    print ("test")
    cyl = cylinder(array([0,0,2]), 1, 1, array([[1,0,0],[0,1,0],[0,0,1]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    cyl = cylinder(array([0,0,2]), 1, 2, array([[1,0,0],[0,1,0],[0,0,1]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    rad = pi/4
    cyl = cylinder(array([0,0,2]), 1, 2, array([[cos(rad),-sin(rad),0],[sin(rad),cos(rad),0],[0,0,1]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    rad = pi/4
    cyl = cylinder(array([0,0,2]), 1, 2, array([[1,0,0],[0,cos(rad),-sin(rad)],[0,sin(rad),cos(rad)]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    rad = pi/2
    cyl = cylinder(array([0,0,2]), 1, 2, array([[1,0,0],[0,cos(rad),-sin(rad)],[0,sin(rad),cos(rad)]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    rad = pi/2
    cyl = cylinder(array([0,0,2]), 2, 1, array([[1,0,0],[0,cos(rad),-sin(rad)],[0,sin(rad),cos(rad)]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    rad = pi/2 + 1.0e-15
    cyl = cylinder(array([0,0,2]), 2, 1, array([[1,0,0],[0,cos(rad),-sin(rad)],[0,sin(rad),cos(rad)]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    #                       ^ y
    #      2*l = 1.0     z  |
    #  |<------------>|  <--o x
    #  _______________  ____
    # / \             \   ^
    # | |             |   |-2*r = 4.0
    # | |      o      |   |
    # | |    (0.0,2.0)|   |
    # \_/_____________/ __v_
    #                (*)
    #                W(0.0, 0.5, 0.0)
    #                L(0.0,-2.0,-0.5)
    #
    #  (*):collision point
    #
    #          ^ z
    #          |
    #        x o--> y
    #           world coordinate
    #
    rad = pi/2
    cyl = cylinder(array([0,0,2.0]), 2, 0.5, array([[1,0,0],[0,cos(rad),-sin(rad)],[0,sin(rad),cos(rad)]]))
    result, lpos = cyl.chk_col2plane(array([0,0,0]), plane_rot)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    ip = infini_plane(array([0,0,0]), plane_rot)
    result, lpos = cyl.chk_col_plane(ip)
    print("lpos : \n{0}".format(lpos))
    print("----------------------")

    #import time
    #count = 10000
    #stime = time.time()
    #for a in arange (count) :
    #    cyl.chk_col2plane(array([0,0,10/a]), plane_rot)
    #etime = time.time()
    #print(etime - stime) / count * 1000 * 1000



