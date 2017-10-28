#! coding=utf8
import svgwrite

from svgwrite import cm, pt, mm

import numpy as np
from numpy import *

dwg = svgwrite.Drawing(filename="out.svg", debug=True)

def mat3to4 (mat3) :
    mat4 = array([
            [ mat3[0][0], mat3[0][1], mat3[0][2], 0],
            [ mat3[1][0], mat3[1][1], mat3[1][2], 0],
            [ mat3[2][0], mat3[2][1], mat3[2][2], 0],
            [         0,          0,          0, 1]])
    return mat4

def circle (cx, cy, r, n=8) :
    r2 = r / np.cos(np.pi / n)

    path = dwg.path(d=
                  "M {x:.4f} {y:.4f}".format( x = float(r + cx), y = float(cy)))
    for i in np.arange(2, 2*n+1, 2) :
        srad = (i-1) * np.pi / n
        erad = (i  ) * np.pi / n
        sx = r2 * np.cos(srad) + cx
        sy = r2 * np.sin(srad) + cy
        ex = r  * np.cos(erad) + cx
        ey = r  * np.sin(erad) + cy
        path.push("Q {lx:.4f} {ly:.4f} {lx1:.4f} {ly1:.4f}".format(lx = sx, ly = sy, lx1 = ex, ly1 = ey))
 
    path.push("z")

    shape = dwg.g(fill='white', stroke='black', stroke_width=3.0)
    shape.add(path)

    return shape

#def pitch_joint (cxy,width=16,dydx=0.6) :
def pitch_joint (cxy,width=30,dydx=0.6) :
    cx = cxy[0]
    cy = cxy[1]
 
    width = width / 2
    cir = circle(cx, cy, width)

    path = dwg.path(d=
              "M  {x}  {y}".format(x=cx,y=cy))
    path.push("m   0  {dy}".format(dy=width))
    path.push("l   0 -{dy}".format(dy=width*2/3))
    path.push('z')

    cir.add(path)
    return cir

#def yaw_joint (cxy, width=20,dydx=0.6) :
def yaw_joint (cxy, width=50,dydx=0.6) :
    cx = cxy[0]
    cy = cxy[1]
    path = dwg.path(d=
              "M   {x}   {y}".format(x=cx,y=cy))
    dx = width / 2.0
    dy = dx * dydx

    path.push("m  {dx}     0".format(dx=dx))
    path.push("l -{dx}  {dy}".format(dx=dx,dy=dy))
    path.push("l -{dx} -{dy}".format(dx=dx,dy=dy))
    path.push("l  {dx} -{dy}".format(dx=dx,dy=dy))
    path.push("l  {dx}  {dy}".format(dx=dx,dy=dy))
    path.push('z')

    path2 = dwg.path(d="M {x} {y}".format(x=cx,y=cy))
    path2.push("m -{dx}   0".format(dx=dx))
    path2.push("l  {dx}   0".format(dx=width))
    path2.push('z')

    shape = dwg.g(fill='white', stroke='black', stroke_width=3.0)
    #shape = dwg.g(fill='white', stroke='black')
    shape.add(path)
    shape.add(path2)

    return shape

def roll_joint (cxy, width=50, dydx=0.6) :
    # followings are same as pitch joint
    # just dx,dy are swapped.

    yaw = yaw_joint (cxy, width, dydx)
    yaw.rotate(90, center = cxy)

    shape = dwg.g(fill='none', stroke='black', stroke_width=3.0)
    shape.add(yaw)

    #    |
    # +--+ __center (cx,cy)
    # |  ^/
    # +--o--+ (cx + dx + a)
    #    v  |
    #    +--+
    #    |


    cx = cxy[0]
    cy = cxy[1]

    height = width
    dy = height / 2.0
    dx = dy * dydx   
    l = dydx * 2

    # add line
    path = dwg.path(d="M {x} {y}".format(x=cx,y=cy))
    path.push("m -{dx}   0 ".format(dx=dx))
    path.push("l -{dx}   0 ".format(dx=l))
    path.push("l    0  {dy}".format(dy=dy+l))
    path.push("l  {dx}   0 ".format(dx=dx+l))
    #path.push('z')
    shape.add(path)

    path = dwg.path(d="M {x} {y}".format(x=cx,y=cy))
    path.push("m  {dx}   0".format(dx=dx))
    path.push("l  {dx}   0 ".format(dx=l))
    path.push("l    0 -{dy}".format(dy=dy+l))
    path.push("l -{dx}   0 ".format(dx=dx+l))
    #path.push('z')
    shape.add(path)


    return shape
 
#    height = width
#
#    cx = cxy[0]
#    cy = cxy[1]
#    path = dwg.path(d=
#              "M   {x}   {y}".format(x=cx,y=cy))
#    dy = height / 2.0
#    dx = dy * dydx
#
#    # followings are same as pitch joint
#    # just dx,dy are swapped.
#    path.push("m  {dx}     0".format(dx=dx))
#    path.push("l -{dx}  {dy}".format(dx=dx,dy=dy))
#    path.push("l -{dx} -{dy}".format(dx=dx,dy=dy))
#    path.push("l  {dx} -{dy}".format(dx=dx,dy=dy))
#    path.push("l  {dx}  {dy}".format(dx=dx,dy=dy))
#    path.push('z')
#
#    path2 = dwg.path(d="M {x} {y}".format(x=cx,y=cy))
#    path2.push("m  0 -{dy}".format(dx=width))
#    path2.push("l  0  {dx}   0".format(dx=dx))
#    path2.push('z')
#
#    shape = dwg.g(fill='white', stroke='black')
#    shape.add(path)
#    shape.add(path2)


def _link (sxy,dxy=None,exy=None) :
    path = dwg.path(d=
                "M  {x}  {y}".format( x=sxy[0], y=sxy[1]))
    if dxy != None :
      # relative position
      #print 'dxy:{0}'.format(dxy)
      path.push("l {dx} {dy}".format(dx=dxy[0],dy=dxy[1]))
    elif exy != None :
      # absolute position 
      #print 'exy:{0}'.format(exy)
      path.push("L {ex} {ey}".format(ex=exy[0],ey=exy[1]))
    else :
      return None

    path.push('z')

    shape = dwg.g(fill='white', stroke='black', stroke_width=3.0)
    shape.add(path)
    return shape

#path = dwg.path(d='M 80 20')
#path.push('l   0  20')
#path.push('z')
#robo.add(path)

def roll_rot (rad) :
    #print "roll {0}".format(rad)
    rot = array([[        1,        0,        0],
                 [        0, cos(rad),-sin(rad)],
                 [        0, sin(rad), cos(rad)]]);
    return rot

def pitch_rot (rad) :
    #print "pitch {0}".format(rad)
    rot = array([[ cos(rad),        0, sin(rad)],
                 [        0,        1,        0],
                 [-sin(rad),        0, cos(rad)]]);
    return rot

def yaw_rot (rad) :
    #print "yaw {0}".format(rad)
    rot = array([[ cos(rad),-sin(rad),        0],
                 [ sin(rad), cos(rad),        0],
                 [        0,        0,        1]]);
    return rot

def rp_rot (roll, pitch) :
    rot = array([[ cos(pitch), sin(pitch) * sin(roll),  sin(pitch) * cos(roll)],
                 [          0,              cos(roll),              -sin(roll)],
                 [-sin(pitch), cos(pitch) * sin(roll),  cos(pitch) * cos(roll)]]);
    return rot

def rpy_rot (roll, pitch, yaw) :
    #            |                       |                                                         |                                                           |
    rot = array([[  cos(yaw) * cos(pitch), cos(yaw) * sin(roll) * sin(pitch) - sin(yaw) * cos(roll),  cos(yaw) * sin(pitch) * cos(roll) - sin(yaw) * sin(roll)],
                 [  sin(yaw) * cos(pitch), sin(yaw) * sin(roll) * sin(pitch) + cos(yaw) * cos(roll),  sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)],
                 [            -sin(pitch),                                   cos(pitch) * sin(roll),                                    cos(pitch) * cos(roll)]]);
    return rot

def rpy2rad (rpy_rot) :
    # rad = [roll,pitch,yaw]
    rad = [0,0,0]
    rad[1] = arcsin(-rpy_rot[2][0])
    rad[0] = arctan2(rpy_rot[2][1],rpy_rot[2][2])
    rad[2] = arctan2(rpy_rot[1][0],rpy_rot[0][0])
    #rad[0] = arccos(rpy_rot[2][2] / cos(rad[1]))
    #rad[2] = arccos(rpy_rot[0][0] / cos(rad[1]))
    return rad

class joint :
    # angle [rad]
    angle = deg2rad(0)
    # axis [1,0,0] or [0,1,0] or [0,0,1]
    axis = array([0,0,0])

    def get_rot (self) :
        #print self.axis
        rad = self.angle
        if   (self.axis == array([1,0,0])).all() :
            return roll_rot(rad)
        elif (self.axis == array([0,1,0])).all():
            return pitch_rot(rad)
        elif (self.axis == array([0,0,1])).all():
            return yaw_rot(rad)

    def get_joint_func (self) :
        #print self.axis
        rad = self.angle
        if   (self.axis == array([1,0,0])).all() :
            return roll_joint
        elif (self.axis == array([0,1,0])).all():
            return pitch_joint
        elif (self.axis == array([0,0,1])).all():
            return yaw_joint

    # at local coordinate
    def get_rot_z2axis (self) :
        if   (self.axis == array([1,0,0])).all() :
            return pitch_rot(pi/2)
        elif (self.axis == array([0,1,0])).all():
            return roll_rot(-pi/2)
        elif (self.axis == array([0,0,1])).all():
            return identity(3)

class link :
    name = ""

    # link offset to tip of this link
    tip_offset = array([0,0,2])

    local2world = None
    pos = None

    # root joint of this link
    joint = None
    #joint = joint()
    # mass [kg]
    mass = 1.0
    # mass center offset [0,0,0] [m]
    #mass_offset = array([0,0,0])
    mass_offset = None
    # Inertia [ Ixx, Ixy, Ixy ],
    #         [ Iyx, Iyy, Iyz ],
    #         [ Izx, Izy, Izz ],
    inertia = array([[1,0,0],[0,1,0],[0,0,1]])

    # parent link
    plink = None
    # node (child) link
    nlink = None

    # total mass [kg] from cur to end
    total_mass = None
    # center of gravity position at local coordinate of total_mass
    total_mass_pos = None
    # moment by robot's own weight [kgm,kgm,kgm]
    total_mass_moment = None

    def __init__ (self, name='none', axis=array([0,1,0]), parent_link = None) :
        self.name  = name

        self.joint = joint()
        self.joint.axis = axis

        self.plink = parent_link

        local2world = array([0,0,0])
        pos = array([0,0,0])

        #mass_offset = array([0,0,0])
        self.mass_offset = self.tip_offset / 2

    def build_links (self) :
        if (self.plink) :
            #print " {0} <-- {1}".format(self.plink.name, self.name)
            self.plink.nlink = self
            self.plink.build_links()

    def print_hierarcy (self) :
        if (self.nlink) :
           print("{0} ==>".format(self.name))
           self.nlink.print_hierarcy()
        else :
           print("{0}".format(self.name))

    def print_svg2d (self, dwg, robo, base_pos, pos, origin) :
        skel = dwg.g()

        #origin = [40,20,1200]
        _sxy = (origin[0] + base_pos[0] * 100, origin[1] - base_pos[1] * 100)
        _exy = (origin[0] +      pos[0] * 100, origin[1] -      pos[1] * 100)
        #print "==== {0} {1} --> {2}".format(self.name, _sxy, _exy)
        skel.add(_link(sxy=_sxy, exy=_exy))
        print(rad2deg(self.joint.angle))
        jskel = self.joint.get_joint_func()(cxy=_sxy)

        rpyrad = rpy2rad(self.local2world)
        jskel.rotate(rad2deg(rpyrad[1]), center=_sxy)
        #jskel.rotate(rad2deg(self.joint.angle), center=_sxy)
        skel.add(jskel)

        #skel.rotate(self.joint.angle, center=_sxy)
        return skel


    def print_svg (self, dwg, roboxz, roboxy, base_pos) :

        ##origin = [30,100,100]
        #origin = [40,20,1200]
        #_sxy = (origin[0] + base_pos[0] * 100, origin[2] - base_pos[2] * 100)
        #_exy = (origin[0] + self.pos[0] * 100, origin[2] - self.pos[2] * 100)
        #print "==== {0} {1} --> {2}".format(self.name, _sxy, _exy)
        #skel.add(_link(sxy=_sxy, exy=_exy))
        #print rad2deg(self.joint.angle)
        #jskel = self.joint.get_joint_func()(cxy=_sxy)
        #
        #rpyrad = rpy2rad(self.local2world)
        #jskel.rotate(rad2deg(rpyrad[1]), center=_sxy)
        ##jskel.rotate(rad2deg(self.joint.angle), center=_sxy)

        xzskel = self.print_svg2d(dwg, roboxz, base_pos=array([base_pos[0],base_pos[2]]), pos=array([self.pos[0],self.pos[2]]), origin=array([40,1200]))
        roboxz.add(xzskel)

        xyskel = self.print_svg2d(dwg, roboxy, base_pos=array([base_pos[0],base_pos[1]]), pos=array([self.pos[0],self.pos[1]]), origin=array([40,1200]))
        xyskel.translate(tx=300)
        roboxy.add(xyskel)

        yzskel = self.print_svg2d(dwg, roboxy, base_pos=array([base_pos[1],base_pos[2]]), pos=array([self.pos[1],self.pos[2]]), origin=array([40,1200]))
        yzskel.translate(tx=600)
        roboxy.add(yzskel)

        #skel.rotate(self.joint.angle, center=_sxy)

        if (self.nlink) :
            self.nlink.print_svg(dwg, roboxz, roboxy, self.pos)

        return

    # forward kinematics
    def update_forward (self, base_pos) :
        if (self.plink == None) :
            self.local2world = identity(3)
            #self.pos = base_pos
            self.pos = base_pos + dot(self.local2world, self.tip_offset)
        else :
            self.local2world = dot(self.plink.local2world, self.joint.get_rot())
            self.pos = base_pos + dot(self.local2world, self.tip_offset)
            #pos = base_pos + dot(self.joint.get_rot(), self.tip_offset)

        #print "pos: {0} {1} @---".format(self.pos, self.name)
        if (self.nlink != None) :
            self.nlink.update(self.pos)

    # backward dinamics
    def update_backward (self) :
        if (self.nlink == None) :
            self.total_mass = self.mass
            self.total_mass_pos = self.mass_offset
        else :
            self.nlink.update_backward()

            nodes_mass = self.nlink.total_mass
            nodes_mass_pos = self.nlink.total_mass_pos
            self.total_mass = self.mass + self.nlink.total_mass
            #self.total_mass_pos = dot(self.nlink.joint.get_rot().T, nodes_mass_pos)
            self.total_mass_pos = self.tip_offset + dot(self.nlink.joint.get_rot(), nodes_mass_pos)
            self.total_mass_pos = (self.mass * self.mass_offset + nodes_mass * self.total_mass_pos) / (self.mass + nodes_mass)

        #print "================"
        #print " link name {0}".format(self.name)
        #print self.total_mass
        #print self.total_mass_pos

        g = 9.80665
        g_axis_at_world = array([0,0,1])
        #print " at world"
        #print dot(self.local2world, self.total_mass_pos)
        # gravity directinal vector at the point of view of local coordinate
        g_axis = dot(self.local2world.T, g_axis_at_world)
        #print self.total_mass
        #print g
        #print g_axis
        self.total_mass_moment = self.total_mass * g * cross(g_axis, self.total_mass_pos)
        #print "moment {0}".format(self.total_mass_moment)

        return

    def update (self, base_pos) :
        self.update_forward(base_pos)
        self.update_backward()
        return

    def func_links (self, base_pos, function) :
        function(self, base_pos, self.pos)
        if (self.nlink) :
            self.nlink.func_links(self.pos, function)

    def get_link (self, link_name) :
        if self.name == link_name :
            return self

        if (self.nlink) :
            return self.nlink.get_link(link_name)
        else :
            return None
    
    def get_num_of_links (self) :
        if self.nlink == None :
            return 1

        #print self.name
        return 1 + self.nlink.get_num_of_links()

    def get_link_orders (self) :
        if self.nlink == None :
            return [self]

        # TODO: it is slow. should be more smart
        #       [node0, node1, ..., nodeN] --> [self, node0, node1, ..., nodeN]
        nlinks = self.nlink.get_link_orders()
        nlinks.insert(0, self)
        return nlinks
        

def calc_local_jacobian_vec (base_link, end_link) :
    print("===== name {0} --> {1}".format(base_link.name, end_link.name))
    # r vector from base to end
    #    end_link.pos  : world coords
    #    base_link.pos : world coords
    #    base_link.tip_offset : local coords
    rb2e = end_link.pos - (base_link.pos - dot(base_link.local2world, base_link.tip_offset))
    # axis vector from base to end
    #print " name {0} : axis : {1}".format(base_link.name, base_link.joint.axis)
    ab2e = base_link.joint.axis
    #print "rb2e {0} = {1} - ({2} - {3})".format(rb2e, end_link.pos, base_link.pos, dot(base_link.local2world, base_link.tip_offset))
    #print "ab2e {0}".format(ab2e)
    
    # calc local jacobi_pos [3 x 1]
    # calc local jacobi_rot [3 x 1]
    rot = base_link.local2world
    rot_ab2e = dot(rot, ab2e)
    jacobi_pos = cross(rot_ab2e, rb2e)
    jacobi_rot = rot_ab2e
    #print "jaco pos {0}".format(jacobi_pos)
    #print "jaco rot {0}".format(jacobi_rot)
    #print "--- from b 2 here"
    #print rot
    #print "--- from here 2 end"
    #print end_link.local2world
 
    jacobi_vec = zeros(6)
    jacobi_vec[0:3] = jacobi_pos
    jacobi_vec[3:6] = jacobi_rot
    #print "==============================================="
    return jacobi_vec

class robo :
    name = None
    link = []
    base_pos = None

    def add_joint (self, link) :
        self.link.append(link)

    def construct_joints (self) :
        print(self.link)
        if len(self.link) == 0 :
            print("error robo does not have any link")
            return 

        # TODO:
        self.link[len(self.link) - 1].build_links()
        self.link[0].print_hierarcy()

    def set_base_pos (self, base_pos) :
        self.base_pos = base_pos

    def set_joint_angle_rad (self, linkno, rad) :
        self.link[linkno].joint.angle = rad

    def add_joint_angle_rad (self, linkno, rad) :
        self.link[linkno].joint.angle = self.link[linkno].joint.angle + rad

    def set_joint_angle_deg (self, linkno, deg) :
        self.link[linkno].joint.angle = deg2rad(deg)

    def update (self) :
        self.link[0].update(self.base_pos)

    def get_link (self, link_name) :
        return self.link[0].get_link(link_name)

    def get_pos (self, linkno) :
        return self.link[linkno].pos

    def __init__ (self, name='none', base_pos=array([0,0,0])) :
        self.name = name
        self.base_pos = base_pos

    # TODO: end_link ---> base_link --->  actual_end_link
    # TODO:       base_link ---> other_end_link
    #       other_base_link --->       end_link
    def calc_jacobian (self, base_link, end_link) :
        num_of_b2ae = base_link.get_num_of_links()
        num_of_e2ae = end_link.get_num_of_links()

        num_of_links = num_of_b2ae - num_of_e2ae + 1

        #print "num_of_b2ae {0}".format(num_of_b2ae)
        #print "num_of_e2ae {0}".format(num_of_e2ae)
        #if (num_of_links <= 0) :
        #    print "invalid arguments {0} {1]".format(base_link.name, end_link.name)

        jacobi_local = zeros(6*num_of_links).reshape(num_of_links, 6)

        print("orders {0}".format(base_link.get_link_orders()))
        for idx, link in enumerate(base_link.get_link_orders()[0:num_of_links]) :
            jacobian_vec = calc_local_jacobian_vec(link, end_link)
            #print "==={0} {1} {2}".format(idx, link.name, jacobian_vec)
            jacobi_local[idx] = jacobian_vec

        return jacobi_local.T
        
if __name__ == "__main__": 
    # TODO: blink should be set with axis=0,0,0 automatically
    base  = link(name='base', axis = array([0,0,0]))
    link0 = link(name='0', parent_link = base, axis = array([1,0,0]))
    link1 = link(name='1', parent_link = link0, axis = array([0,1,0]))
    link2 = link(name='2', parent_link = link1, axis = array([0,0,1]))
    link3 = link(name='3', parent_link = link2, axis = array([0,1,0]))
    link4 = link(name='4', parent_link = link3, axis = array([0,0,1]))
    link5 = link(name='5', parent_link = link4, axis = array([0,0,1]))
   
    if 0 :
        link5.build_links()
        base.print_hierarcy()
        
        link0.joint.angle = deg2rad(00)
        link2.joint.angle = deg2rad(30)
        link3.joint.angle = deg2rad(45)
        link4.joint.angle = deg2rad(90)
        link5.joint.angle = deg2rad(45)
        #link5.joint.angle = deg2rad(90)
        
        base.update (array([0,0,0]))
        
        roboxz = dwg.g(id='roboxz')
        roboxy = dwg.g(id='roboxy')
        
        base.print_svg(dwg, roboxz, roboxz, array([0,0,0]))
        
        dwg.add(roboxz)
  
        dwg.save()
    else :
        robo = robo()
        robo.add_joint(base)
        robo.add_joint(link0)
        robo.add_joint(link1)
        robo.add_joint(link2)
        robo.add_joint(link3)
        robo.add_joint(link4)
        robo.add_joint(link5)

        robo.construct_joints()
        robo.set_base_pos(array([0,0,0]))
        
        blink = link0
        tlink = link5
        #tlink = link2
        num = blink.get_num_of_links() - tlink.get_num_of_links() + 1
        print("{0} --> {1} : {2}".format(blink.name, tlink.name, num))

        #base_angles = array([20, 30, 20, 30, 20, 30])
        #base_angles = array([ 0, 0, 45, 0, 0, 0]) # OK.
        #base_angles = array([ 0, 0, 90, 0, 0, 0]) # OK.
        #base_angles = array([90, 0,  0, 0, 0, 0]) # OK.
        #base_angles = array([0, 45, 0, 0, 0, 0])  # OK.
        #base_angles = array([0, 90, 0, 0, 0, 0])  # OK.
        #base_angles = array([45, 45, 45, 0, 0, 0]) # OK.
        #base_angles = array([0, 0, 0, 0, 0, 0]) # OK.
        base_angles = array([20, 30, 40, 60, 80, -20]) # OK.
        base_angles = deg2rad(base_angles)
        for idx in range(0,num) :
          robo.set_joint_angle_rad(idx + 1, base_angles[idx])
        robo.update()

        jacobian = robo.calc_jacobian(blink, tlink)

        # delta_q at [deg]
        delta_q = 0.0001

        test_rads_p = (zeros(num) + delta_q)
       
        # diff @ world coordinate
        tip = dot(jacobian, test_rads_p)
        # TODO: should be use robo::get_pos
        offset = tlink.pos
        offset_rot = tlink.local2world
        wtip = tip[0:3] + offset
        wrot = tip[3:6]

        for idx in range(0,num) :
          robo.add_joint_angle_rad(idx, test_rads_p[idx])
        robo.update()
        test_pos_p = tlink.pos
        test_rot_p = dot(offset_rot.T, tlink.local2world)

        print("----- rot at origin ----")
        print(tlink.local2world)
        print(rpy2rad(tlink.local2world))
        print(rad2deg(rpy2rad(tlink.local2world)))
        print("--")
        print(dot(tlink.local2world, array([0,0,1.0])))
        print("------------------------")
        
        print("----- jacobian from kinematics -------")
        print(jacobian)
        print("----- (pseudo) inverse jacobian -----")
        jacobian_pinv = linalg.pinv(jacobian)
        print(np.round(jacobian_pinv, 3))
        print(np.round(dot(jacobian_pinv, jacobian), 3))
        print(np.round(dot(jacobian, jacobian_pinv), 3))
        print("-----  check  --------")
        print(" {0} {1}({2}) = {3} + {4}(offset)".format(wtip, tip[3:6], rad2deg(tip[3:6]), tip[0:3], offset))
        rads = rpy2rad(tlink.local2world)
        print(" {0} {1}({2}) ".format(tlink.pos, rads, rad2deg(rads)))
        print(" by yacobi")
        rpy = rpy_rot(tip[3:6][0], tip[3:6][1], tip[3:6][2])
        print(np.round(rpy, 8))
        print(" by kine forward")
        print(np.round(dot(offset_rot.T, tlink.local2world), 8))

        print("----------------------------------------------------------------------------")

        jacobi_from_plus = zeros(6*num).reshape(num, 6)
        jacobi_from_minus = zeros(6*num).reshape(num, 6)
        jacobi_from_plus_rot = zeros(3*3*num).reshape(num, 3, 3)
        jacobi_from_minus_rot = zeros(3*3*num).reshape(num, 3, 3)
        jacobi_from_diff = zeros(6*num).reshape(6, num)

        for idx in range(0,num) :
            angles = base_angles.copy()
            angles[idx] = angles[idx] + delta_q
            for jdx in range(0,num) :
                robo.set_joint_angle_rad(jdx + 1, angles[jdx])
            robo.update()
            jacobi_from_plus[idx][0:3] = tlink.pos
            jacobi_from_plus_rot[idx] = tlink.local2world

        for idx in range(0,num) :
            angles = base_angles.copy()
            angles[idx] = angles[idx] - delta_q
            for jdx in range(0,num) :
                robo.set_joint_angle_rad(jdx + 1, angles[jdx])
            robo.update()
            jacobi_from_minus[idx][0:3] = tlink.pos
            jacobi_from_minus_rot[idx] = tlink.local2world

        jacobi_from_diff = ((jacobi_from_plus.T - jacobi_from_minus.T) / (delta_q * 2))
        for idx in range(0,num) :
            rot = dot(jacobi_from_plus_rot[idx], jacobi_from_minus_rot[idx].T)
            angle = array(rpy2rad(rot)) / (delta_q * 2)
            jacobi_from_diff[3:6, idx] = angle
            
        print("----- angles -------")
        print(rad2deg(base_angles))
        print("----- jacobian from kinematics -------")
        print(jacobian)
        print("----- jacobian from diff -------")
        print(jacobi_from_diff)
        print("-------- norm --------")
        print(linalg.norm(jacobi_from_diff - jacobian))

