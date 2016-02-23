#! coding=utf8
import svgwrite

from svgwrite import cm, pt, mm

import numpy as np
from numpy import *

dwg = svgwrite.Drawing(filename="out.svg", debug=True)

I3 = identity(3)

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
      #print('dxy:{0}'.format(dxy))
      path.push("l {dx} {dy}".format(dx=dxy[0],dy=dxy[1]))
    elif exy != None :
      # absolute position 
      #print('exy:{0}'.format(exy))
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

base_roll = array([[ 1, 0, 0],
                  [ 0, 1, 0],
                  [ 0, 0, 1]])
base_pitch = array([[ 1, 0, 0],
                   [ 0, 1, 0],
                   [ 0, 0, 1]])
base_yaw = array([[ 1, 0, 0],
                  [ 0, 1, 0],
                  [ 0, 0, 1]])

def roll_rot (rad) :
    #print "roll {0}".format(rad)
    rot = array([[        1,        0,        0],
                 [        0, cos(rad),-sin(rad)],
                 [        0, sin(rad), cos(rad)]]);
    return rot
    #base_roll[1,1] = +cos(rad)
    #base_roll[1,2] = -sin(rad)
    #base_roll[2,1] = +sin(rad)
    #base_roll[2,2] = +cos(rad)
    #return base_roll

def pitch_rot (rad) :
    #print "pitch {0}".format(rad)
    rot = array([[ cos(rad),        0, sin(rad)],
                 [        0,        1,        0],
                 [-sin(rad),        0, cos(rad)]]);
    return rot
    #base_pitch[0,0] = +cos(rad)
    #base_pitch[0,2] = +sin(rad)
    #base_pitch[2,0] = -sin(rad)
    #base_pitch[2,2] = +cos(rad)
    #return base_pitch

def yaw_rot (rad) :
    #print "yaw {0}".format(rad)
    rot = array([[ cos(rad),-sin(rad),        0],
                 [ sin(rad), cos(rad),        0],
                 [        0,        0,        1]]);
    return rot
    #base_yaw[0,0] = +cos(rad)
    #base_yaw[0,1] = -sin(rad)
    #base_yaw[1,0] = +sin(rad)
    #base_yaw[1,1] = +cos(rad)
    #return base_yaw

def none_rot (rad) :
    return identity(3)


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
    # veloc [rad/sec]
    veloc = deg2rad(0)
    # accel [rad/sec/sec]
    accel = deg2rad(0)

    # actuational torque [Nm]
    torque_a = 0.0
    # external torque [Nm]
    torque_e = 0.0

    # viscosity
    visco = 0.0

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
        else :
            print("error get_rot() : {0}".format(self.axis))
            return none_rot(rad)

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
    mass_inertia = array([[1,0,0],[0,1,0],[0,0,1]])

    # TODO: default value
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

        if name == 'base' :
            self.tip_offset = array([0,0,0])

        #mass_offset = array([0,0,0])
        self.mass_offset = self.tip_offset / 2

    def build_links (self) :
        if (self.plink) :
            #print " {0} <-- {1}".format(self.plink.name, self.name)
            self.plink.nlink = self
            self.plink.build_links()

    def print_hierarcy (self) :
        if (self.nlink) :
           print("{0}".format(self.name))
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
        #print(rad2deg(self.joint.angle))
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
            self.nlink.update_forward(self.pos)

    # backward dynamics
    def update_inertia (self) :
        # TODO
        if (self.plink == None) :
            return self.nlink.update_inertia()

        # Inertia by link own mass/inertia
        # G = m { (r*r)I - (rt*r) }
        #         ^^^^^    ^^^^^^
        #       G_l(1x1)  G_r(3x3)
        # ------------------------
        #
        # Ii = Ri * Ii+1 * Ri^t  + G
        # 
        ##l_veloc = self.joint.axis * self.joint.veloc
        G_l = dot(self.mass_offset, self.mass_offset) * I3
        G_r = dot(self.mass_offset.T, self.mass_offset)
        G = self.mass * (G_l + G_r)
        # 0.2[msec]
        self.inertia = dot(dot(self.joint.get_rot(), self.mass_inertia), self.joint.get_rot().T) + G
 
        if (self.nlink == None) :
            #print("===self.inertia", self.inertia)
            return

        self.nlink.update_inertia()

        # Inertia by node links mass/inertia
        G_l = dot(self.tip_offset, self.tip_offset) * I3
        G_r = dot(self.tip_offset.T, self.tip_offset)
        G = self.nlink.total_mass * (G_l + G_r)
        # 0.2[msec] = 90 + 90 + 20
        self.inertia = dot(dot(self.joint.get_rot(), self.nlink.inertia), self.joint.get_rot().T) + G + self.inertia

        return

    # backward dynamics
    def update_gforce (self) :
        if (self.nlink == None) :
            self.total_mass = self.mass
            self.total_mass_pos = self.mass_offset
        else :
            self.nlink.update_gforce()

            # Update center of gravity by all of childs' mass

            nodes_mass = self.nlink.total_mass
            nodes_mass_pos = self.nlink.total_mass_pos
            self.total_mass = self.mass + self.nlink.total_mass

            #  (Mi*Ri + Ma*Ri+1*Ra)
            #  --------------------
            #        Mi +  Ma
            self.total_mass_pos = self.tip_offset + dot(self.nlink.joint.get_rot(), nodes_mass_pos)
            self.total_mass_pos = (self.mass * self.mass_offset + nodes_mass * self.total_mass_pos) / (self.mass + nodes_mass)

        # TODO: should be global
        g = 9.80665
        g_axis_at_world = array([0,0,-1])

        # G: gravity vector at world coords
        # Gl: gravity vector at localcoords of link_i
        # Gl = Rt0-i * G
        g_axis = dot(self.local2world.T, g_axis_at_world)
        # Ma * 9.8 * (Ri x Gl)
        self.total_mass_moment = self.total_mass * g * cross(self.total_mass_pos, g_axis)

        self.joint.torque_e = dot(self.total_mass_moment, self.joint.axis)

        return

    # forward dynamics
    def update_movement (self) :
        if (self.nlink == None) :
            return

        # I   : Inertia
        # ddq : rotational acceleration
        # Tact: actuational torque
        # Text: external torque
        # u   : viscosity
        # udq : rotational veloc
        # I * ddq = Tact + Text - udq
        #
        #           Tact + Text - udq
        #     ddq = -----------------
        #                   I
        # 
        #print("link {0} ta:{1} te:{2}".format(self.name, self.joint.torque_a, self.joint.torque_e))
        #print("link {0} I^-1:{1} I^-1{2}".format(self.name, linalg.inv(self.inertia), dot(linalg.inv(self.inertia), self.joint.axis)))
        self.joint.accel = self.joint.torque_a + self.joint.torque_e - self.joint.visco * self.joint.veloc
        self.joint.accel = dot(self.joint.accel * self.joint.axis, dot(linalg.inv(self.inertia), self.joint.axis))

        # 1.0[msec] --> 0.001[sec]
        #tick = 0.1 # 0.001
        #tick = 0.001
        tick = 0.002
        self.joint.angle = self.joint.angle + self.joint.veloc * tick
        self.joint.veloc = self.joint.veloc + self.joint.accel * tick
        #print("link {0} ang:{1} veloc:{2} accel:{3}".format(self.name, self.joint.angle, self.joint.veloc, self.joint.accel))

        self.nlink.update_movement()

        return

    def update_tick (self, base_pos) :
        # forward kinematics
        # 0.2[msec]
        self.update_forward(base_pos)

        # backward dynamics
        # 0.4[msec]
        self.update_gforce()
        # 0.8[msec] - 1.0[msec]
        self.update_inertia()
        # 0.2[msec]
        self.update_movement()

    def update (self, base_pos) :
        # forward kinematics
        self.update_forward(base_pos)

        #TODO
        self.update_gforce()
        self.update_inertia()
        self.update_movement()

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

    def get_angles (self) :
        if self.nlink == None :
            return [self.joint.angle]

        angles = self.nlink.get_angles()
        angles.insert (0, self.joint.angle)
        return angles

    def set_angles (self, rads) :
        rad, rads = rads[0], rads[1:]
        #if self.nlink == None :
        if len(rads) == 0 :
            self.joint.angle = rad
            return rads

        ret = self.nlink.set_angles(rads)
        if len(ret) == 0 :
            self.joint.angle = rad

        return ret

    def print_moment (self) :
        print(" Joint{0} : {1:9.3f} by {2}".format(self.name, dot(self.total_mass_moment, self.joint.axis), self.total_mass_moment))
        if (self.nlink == None) :
            return
        return self.nlink.print_moment()

def calc_local_jacobian_vec (base_link, end_link) :
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
    num_of_links = 0

    def add_joint (self, link) :
        self.link.append(link)

    def construct_joints (self) :
        if len(self.link) == 0 :
            print("error robo does not have any link")
            return 

        # TODO:
        self.num_of_links  = len(self.link)
        self.link[self.num_of_links - 1].build_links()
        self.link[0].print_hierarcy()

    def set_base_pos (self, base_pos) :
        self.base_pos = base_pos

    def set_joint_angle (self, linkno, rad) :
        self.link[linkno].joint.angle = rad

    def add_joint_angle (self, linkno, rad) :
        self.link[linkno].joint.angle = self.link[linkno].joint.angle + rad

    def set_joint_angles (self, rads, slinkno = 0) :
        self.link[slinkno].set_angle(rads)

    def update (self) :
        self.link[0].update(self.base_pos)

    def get_link (self, link_name) :
        return self.link[0].get_link(link_name)

    def get_pos (self, linkno) :
        return self.link[linkno].pos

    def get_angles (self, sidx = 1) :
        return self.link[1].get_angles()

    def set_angles (self, rads, sidx = 1) :
        return self.link[sidx].set_angles(rads)

    def print_moment (self, sidx = 1) :
        return self.link[sidx].print_moment()

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

        for idx, link in enumerate(base_link.get_link_orders()[0:num_of_links]) :
            jacobian_vec = calc_local_jacobian_vec(link, end_link)
            #print "==={0} {1} {2}".format(idx, link.name, jacobian_vec)
            jacobi_local[idx] = jacobian_vec

        return jacobi_local.T

    # calculation jacobian with processing virtual move (not virtual work)
    def calc_jacobian_vd (self, blink, tlink, delta_q = 0.00001) :
        #num = self.get_num_of_links()
        #base_angles = self.get_angles()
        num = blink.get_num_of_links() - tlink.get_num_of_links() + 1
        base_angles = array(blink.get_angles())

        jacobi_from_plus = zeros(6*num).reshape(num, 6)
        jacobi_from_minus = zeros(6*num).reshape(num, 6)
        jacobi_from_plus_rot = zeros(3*3*num).reshape(num, 3, 3)
        jacobi_from_minus_rot = zeros(3*3*num).reshape(num, 3, 3)
        jacobi_from_diff = zeros(6*num).reshape(6, num)

        for idx in range(0,num) :
            angles = base_angles.copy()
            angles[idx] = angles[idx] + delta_q
            for jdx in range(0,num) :
                self.set_joint_angle(jdx + 1, angles[jdx])
            self.update()
            jacobi_from_plus[idx][0:3] = tlink.pos
            jacobi_from_plus_rot[idx] = tlink.local2world

        print(jacobi_from_plus[0])

        for idx in range(0,num) :
            angles = base_angles.copy()
            angles[idx] = angles[idx] - delta_q
            for jdx in range(0,num) :
                self.set_joint_angle(jdx + 1, angles[jdx])
            self.update()
            jacobi_from_minus[idx][0:3] = tlink.pos
            jacobi_from_minus_rot[idx] = tlink.local2world

        print(jacobi_from_minus[0])

        jacobi_from_diff = ((jacobi_from_plus.T - jacobi_from_minus.T) / (delta_q * 2))
        for idx in range(0,num) :
            rot = dot(jacobi_from_plus_rot[idx], jacobi_from_minus_rot[idx].T)
            angle = array(rpy2rad(rot)) / (delta_q * 2)
            jacobi_from_diff[3:6, idx] = angle

        for idx in range(0,num) :
            for jdx in range(0,num) :
                self.set_joint_angle(jdx + 1, base_angles[jdx])
            self.update()

        return jacobi_from_diff
        
if __name__ == "__main__": 
    # TODO: blink should be set with axis=0,0,0 automatically
    base  = link(name='base', axis = array([0,0,0]))
    link0 = link(name='0', parent_link = base, axis = array([1,0,0]))
    link1 = link(name='1', parent_link = link0, axis = array([0,1,0]))
    link2 = link(name='2', parent_link = link1, axis = array([0,0,1]))
    link3 = link(name='3', parent_link = link2, axis = array([1,0,0]))
    link4 = link(name='4', parent_link = link3, axis = array([0,1,0]))
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
    elif 1 :
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
        num = blink.get_num_of_links() - tlink.get_num_of_links() + 1
        print("{0} --> {1} : {2}".format(blink.name, tlink.name, num))

        #base_angles = deg2rad(array([0, 0, 0, 0, 0, 0])) # OK.
        #base_angles = deg2rad(array([90, 0, 0, 0, 0, 0])) # OK.
        #base_angles = deg2rad(array([20, 20, 20, 0, 0, 0])) # OK.
        #base_angles = deg2rad(array([20, 20, 20, 20, 0, 0])) # OK.
        #base_angles = deg2rad(array([20, 20, 20, 20, 20, 20])) # OK.
        base_angles = deg2rad(array([-20, -20, 20, -20, -20, 20])) # OK.
        robo.set_angles(base_angles[:num])
        robo.update()

        robo.print_moment()

        jacobian = robo.calc_jacobian(blink, tlink)
        jacobian_vd = robo.calc_jacobian_vd(blink, tlink)
        print( "**************** jacobian check ********************")
        print( "** jacobian_pd by pd **")
        print( around(jacobian, 4))
        print( "** jacobian_vd by vd **")
        print( around(jacobian_vd, 4))
        print( "** norm(J_vd - J_pd) **")
        print( linalg.norm(jacobian_vd - jacobian))
    else :
        import sys
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
        num = blink.get_num_of_links() - tlink.get_num_of_links() + 1

        print("")
        diff_max = 0
        for idx in range(10) :
            base_angles = random.rand(num) * 2 * pi
            #base_angles = array([20, 30, 40, 60, 80, -20]) # OK.
            #base_angles = deg2rad(base_angles)
            robo.set_angles(base_angles[:num])
            robo.update()

            jacobian = robo.calc_jacobian(blink, tlink)
            jacobian_vm = robo.calc_jacobian_vd(blink, tlink)
            diff = linalg.norm(jacobian - jacobian_vm)

            if diff > diff_max :
                diff_max = diff

            sys.stdout.write("\r {0} norm : {1}, {2} @ {3}".format(idx, diff, diff_max, base_angles))
            if diff > 2.0E-4 :
                print ("ERROR : norm = {0}".format(diff))
                print (" --- jacobian ------")
                print (jacobian)
                print (" --- jacobian vm ---")
                print (jacobian_vm)

        print("")
       
        invJ = linalg.pinv(jacobian)
        print (invJ)
        print (around(dot(jacobian, invJ), 3))
        print (around(dot(invJ, jacobian), 3))
        #print dot(invJ.T, jacobian)

        #base_angles = array([20, 30, 40, 60, 80, -20]) # OK.
        #base_angles = deg2rad(base_angles)

        #robo.set_angles(base_angles)
        #robo.update()

        #jacobian = robo.calc_jacobian(blink, tlink)
        #jacobi_from_diff = robo.calc_jacobian_vd(blink, tlink)
        #    
        #print "----- angles -------"
        #print rad2deg(base_angles)
        #print "----- jacobian from kinematics -------"
        #print jacobian
        #print "----- jacobian from diff -------"
        #print jacobi_from_diff
        #print "-------- norm --------"
        #print linalg.norm(jacobi_from_diff - jacobian)

