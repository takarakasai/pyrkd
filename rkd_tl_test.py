#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from numpy import *
#from numpypy import *

import FTGL

#from dp_roki import *
from rkd import *

from datetime import datetime

window_width = 640
window_height = 480

tz = 1.0
view_dis = 15.5
round = 0.0
view_z = 2.0
#count = 10.0

quadric = None

def robo_func (link, base_pos, pos) :
    #print(" pos --> {0}".format(pos))
    glVertex3d(base_pos[0], base_pos[1], base_pos[2])
    glVertex3d(pos[0], pos[1], pos[2])

    return

def robo_joint_func (link, base_pos, pos) :
    glPushMatrix()
    glTranslatef(base_pos[0], base_pos[1], base_pos[2])

    ltrans = mat3to4(link.local2world.T)
    glMultMatrixd(ltrans)

    glPushMatrix()
    glScalef(0.4,0.4,0.4)
    glMaterialfv(GL_FRONT , GL_SPECULAR, [1.0,1.0,1.0, 1.0])
    glMaterialfv(GL_FRONT , GL_DIFFUSE, [0.0,0.0,0.0, 1.0])
    glMaterialfv(GL_FRONT , GL_AMBIENT, [0.05,0.05,0.05, 1.0])
    glMaterialf(GL_FRONT , GL_SHININESS, 0.8)
    moment = dot(link.total_mass_moment, link.joint.axis)
    glPushMatrix()
    glTranslatef(1.0,0,0)
    glRotatef(90,1.0,0.0,0.0)
    font.Render("Tg {0:.2f} [Nm]".format(moment))
    glPopMatrix()
    glTranslatef(7.0 + 1.0,0,0) # TODO:7.0
    glRotatef(90,1.0,0.0,0.0)
    glRotatef(180,0.0,1.0,0.0)
    font.Render("Tg {0:.2f} [Nm]".format(moment))
    glPopMatrix()

    glPushMatrix()

    ##########################
    ############## draw link
    #glColor3d(1.0,1.0,1.0)
    glMaterialfv(GL_FRONT , GL_SPECULAR, [1.0,1.0,1.0, 1.0])
    glMaterialfv(GL_FRONT , GL_DIFFUSE, [0.7,0.7,0.7, 1.0])
    glMaterialfv(GL_FRONT , GL_AMBIENT, [0.35,0.35,0.35, 1.0])
    glMaterialf(GL_FRONT , GL_SHININESS, 0.8)
    r = link.tip_offset[2] / 15
    gluCylinder( quadric, r, r, link.tip_offset[2], 20, 6)
    
    glTranslatef(0, 0, (link.tip_offset[2]))
    gluDisk( quadric, 0.0, r, 10, 2)

    glPopMatrix()
    glPushMatrix()

    #print link.local2world
    #print ltrans
    ltrans = mat3to4(link.joint.get_rot_z2axis())
    glMultMatrixd(ltrans)

    ############## draw joint
    glMaterialfv(GL_FRONT , GL_SPECULAR, [1.0,1.0,1.0, 1])
    glMaterialfv(GL_FRONT , GL_AMBIENT, [0.4,0.4,0.4, 1])
    glMaterialfv(GL_FRONT , GL_DIFFUSE, [0.2,0.2,0.2, 1])
    glMaterialf(GL_FRONT , GL_SHININESS, 0.8)

    r = link.tip_offset[2] / 10
    cylinder_height = link.tip_offset[2] * 0.3
    glTranslatef(0, 0, -(cylinder_height / 2.0))

    gluCylinder( quadric, r, r, cylinder_height, 10, 2)

    gluQuadricOrientation(quadric, GLU_INSIDE);
    gluDisk( quadric, 0.0, r, 10, 2)
    gluQuadricOrientation(quadric, GLU_OUTSIDE);

    glTranslatef(0, 0, (cylinder_height))
    gluDisk( quadric, 0.0, r, 10, 2)
    ##########################

    glPopMatrix()

    glPopMatrix()
    return

def init():
    global quadric
    quadric = gluNewQuadric()
    gluQuadricDrawStyle(quadric, GLU_FILL);
    gluQuadricNormals(quadric, GLU_SMOOTH);


    glLightfv(GL_LIGHT0, GL_POSITION, [-2.0, -2.0, 4.0, 1.0]);
    glLightfv(GL_LIGHT0, GL_AMBIENT, [1.0, 1.0, 1.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.7, 0.7, 0.7, 1.0]);
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.2, 0.2, 0.2, 1.0]);
    glEnable(GL_LIGHT0)

    #glLightfv(GL_LIGHT2, GL_POSITION, [1.0, 1.0, 4.0, 1.0]);
    #glLightfv(GL_LIGHT2, GL_AMBIENT, [1,0, 1.0, 1.0, 1.0])
    #glLightfv(GL_LIGHT2, GL_DIFFUSE, [0.7, 0.7, 0.7, 1.0]);
    #glLightfv(GL_LIGHT2, GL_SPECULAR, [0.2, 0.2, 0.2, 1.0]);
    #glEnable(GL_LIGHT2)

    glClearColor(1.0, 1.0, 1.0, 0.0)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_CULL_FACE)
    #glCullFace(GL_FRONT)
    glCullFace(GL_BACK)
    glEnable(GL_LIGHTING)

def draw_floor(cx, cy, cz, width, height) :
    global quadric

    glMaterialfv(GL_FRONT , GL_SPECULAR, [1.0,1.0,1.0, 1.0])
    glMaterialfv(GL_FRONT , GL_DIFFUSE, [0.7,0.7,0.7, 1.0])
    glMaterialfv(GL_FRONT , GL_AMBIENT, [0.35,0.35,0.35, 1.0])
    glMaterialf(GL_FRONT , GL_SHININESS, 0.8)
 
    dx = width / 2.0
    dy = height / 2.0

    glBegin(GL_TRIANGLES)
    glNormal3d(1.0,1.0,0.0)
    glVertex3d(cx - dx, cy + dy, cz)
    glVertex3d(cx - dx, cy - dy, cz)
    glVertex3d(cx + dx, cy + dy, cz)
    glEnd()

    glBegin(GL_TRIANGLES)
    glNormal3d(1.0,1.0,0.0)
    glVertex3d(cx + dx, cy + dy, cz)
    glVertex3d(cx - dx, cy - dy, cz)
    glVertex3d(cx + dx, cy - dy, cz)
    glEnd()

def _draw():
    return 

def draw_robot():
    global robo_link

    # model view matrix reset
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    draw_floor(0.0, 0.0, 0.0, 100, 100)

    #glBegin(GL_LINES)
    #robo_link.func_links(array([0.0,0.0,0.0]), robo_func)
    #glEnd()

    # TODO:
    robo_link.func_links(array([0.0,0.0,0.0]), robo_joint_func)

def draw_world():
    draw_robot()

count = 0

def draw_osd():
    global diff_time
    global count
    global font2

    if diff_time == None :
      return

    if diff_time_dyn == None :
      return

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glColor3f(1.0, 1.0, 1.0)

    #glPushMatrix()

    # usec --> msec
    glWindowPos2d(0,3*font2.line_height)
    msec = diff_time / 1000.0            # [msec]
    font2.Render(" FrameRate GL {0:5.0f} at {1:1.4f}".format(1000 / msec, msec))
    #print(" FrameRate GL {0:5.0f} at {1:1.4f}".format(1000 / msec, msec))

    glWindowPos2d(0,2*font2.line_height)
    msec = diff_time_dyn / 1000.0        # [msec]
    font2.Render(" FrameRate DY {0:5.0f} at {1:1.4f}".format(1000 / msec, msec))
    #print(" FrameRate DY {0:5.0f} at {1:1.4f}".format(1000 / msec, msec))

    glWindowPos2d(0,1*font2.line_height)
    msec_c = diff_time_dyn_calc / 1000.0 # [msec]
    font2.Render("    calc cost {1:+1.4f} {1:+1.4f}".format(msec_c, msec - msec_c))
    #print("    calc cost {1:+1.4f} {1:+1.4f}".format(msec_c, msec - msec_c))

    #glPopMatrix()

def draw():
    global view_dis
    global tz
    #glClear(GL_COLOR_BUFFER_BIT)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    gluPerspective(30.0, 1.0, 0.5, 100.0); 
    vx = -view_dis * sin(deg2rad(round))
    vy = -view_dis * cos(deg2rad(round))
    gluLookAt( vx, vy, view_z, 0, 0, view_z, 0, 0, 1)

    draw_world()
    draw_osd()

    glFlush()
    glutSwapBuffers()


def resize(w, h):
    #glShadeModel(GL_SMOOTH) # TODO:
    glViewport(0, 0, w, h)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    # TODO: use glFrustum
    gluPerspective(30.0, -view_dis, 1.0, 100.0)

def mouse(button, state, x, y):
    if button == GLUT_LEFT_BUTTON:
        print("left button")
    elif button == GLUT_MIDDLE_BUTTON:
        print("middle button")
    elif button == GLUT_RIGHT_BUTTON:
        print("right button")
    else:
        print("unknown button {0}".format(button))

    if state == GLUT_DOWN:
        print("down mouse button")
    elif state == GLUT_UP:
        print("up mouse button")
    else:
        print("unknown state: {0}".format(state))

    print(x, y)


def motion(x, y):
    print("drag:{0} {1}".format(x, y))


def keyboard(key, x, y):
    global view_dis
    global tz
    global round
    global view_z
    if key == b'\033':
        sys.exit()
    elif key == b'q':
        sys.exit()
    elif key == b'j':
        view_dis = view_dis + 0.5
        print(view_dis)
        #gluPerspective(30.0, 1.0, -3.0, 10.0); 
        #gluLookAt( 0, 0, -view_dis, 0, 0, 0, 0, 1, 0)
    elif key == b'k':
        view_dis = view_dis - 0.5
        print(view_dis)
        #gluPerspective(30.0, 1.0, -3.0, 10.0); 
        #gluLookAt( 0, 0, -view_dis, 0, 0, 0, 0, 1, 0)
    elif key == b'u':
        round = round + 10.0
    elif key == b'i':
        round = round - 10.0
    elif key == b'n':
        view_z = view_z + 0.5
    elif key == b'm':
        view_z = view_z - 0.5
    #elif key == b'1' or key == b'2' :
    elif key in [b'1',b'2',b'3',b'4',b'5',b'6',b'7',b'8',b'9',b'0'] :
        key = key.decode()
        print("==>{0}".format(str(key)))
        global robo_link
        link = robo_link.get_link(str(key))
        if (link != None) :
            #link.joint.angle = link.joint.angle + deg2rad(10)
            link.joint.torque_a += 1
            return

    elif key in [b'!',b'@',b'#',b'$',b'%',b'^',b'&',b'*',b'(',b')'] :
        index = [b'!',b'@',b'#',b'$',b'%',b'^',b'&',b'*',b'(',b')'].index(key)
        value = ['1','2','3','4','5','6','7','8','9','0'][index]
        print(key)
        global robo_link
        link = robo_link.get_link(str(value))
        if (link != None) :
            #link.joint.angle = link.joint.angle - deg2rad(10)
            link.joint.torque_a -= 1
            return

    elif key == b'c' :
        link = robo_link.get_link('base')
        if (link != None) :
            link.clear_nodes_acuational_torque()
        
    else:
        #gluLookAt( eyeX , eyeY , eyeZ , centerX , centerY , centerZ , upX , upY , upZ ) 
        print(key)

def init_robo() :
    blink = link(name='blink')
    blink.tip_offset = array([0,0,0])
    link0 = link(name='0', parent_link = blink, axis = array([1,0,0]))
    link1 = link(name='1', parent_link = link0, axis = array([0,1,0]))
    link2 = link(name='2', parent_link = link1, axis = array([0,0,1]))
    link3 = link(name='3', parent_link = link2, axis = array([1,0,0]))
    link4 = link(name='4', parent_link = link3)
    link5 = link(name='5', parent_link = link4)
    
    link5.build_links()
    blink.print_hierarcy()
    
    link0.joint.angle = deg2rad(00)
    link2.joint.angle = deg2rad(0)
    link3.joint.angle = deg2rad(0)
    link4.joint.angle = deg2rad(0)
    link5.joint.angle = deg2rad(0)
    #link5.joint.angle = deg2rad(90)
    
    blink.update (array([0,0,0]))

    return blink

font = None
font2 = None
diff_time = None
bf_time = None
count = 0
count_dyn = 0
diff_time_dyn = None
bf_time_dyn = None
diff_time_dyn_calc = 0

def cb_dynamics(value) :
    global robo_link
    global bf_time_dyn
    global diff_time_dyn
    global count_dyn
    global diff_time_dyn_calc

    max_count = 100.0
    count_dyn = count_dyn + 1
    if count_dyn >= max_count :
      count_dyn = 0
      af_time = datetime.now()
      if (bf_time_dyn != None):
          diff_time_dyn = (af_time - bf_time_dyn).microseconds / max_count
      bf_time_dyn = af_time

    bf_time = datetime.now()
    robo_link.update_tick(array([0,0,0]))
    af_time = datetime.now()
    diff_time_dyn_calc = (af_time - bf_time).microseconds

    glutTimerFunc(0, cb_dynamics, 0)
    return

def cb_draw(value) :
    global bf_time
    global diff_time
    global count

    max_count = 10
    count = count + 1
    if count >= max_count :
      count = 0
      af_time = datetime.now()
      if (bf_time != None):
          print (af_time - bf_time)
          diff_time = (af_time - bf_time).microseconds / max_count
      bf_time = af_time

    draw()

    glutTimerFunc(32, cb_draw, 0)

def init_font() :
    import os

    global font
    global font2

    if os.name == 'posix' :
      fontfile = "/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf"
    elif os.name == 'mac' :
      fontfile = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
    else :
      return

    #font = FTGL.OutlineFont(fontfile)
    font = FTGL.PolygonFont(fontfile)
    #font = FTGL.TextureFont(fontfile)
    #font = FTGL.BitmapFont(fontfile)
    #font = FTGL.PixmapFont(fontfile)
    font.FaceSize(0, 1)

    font2 = FTGL.PixmapFont(fontfile)
    font2.FaceSize(24, 72)


glutInit(sys.argv)
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
glutInitWindowSize(window_width, window_height)
glutCreateWindow("PyOpenGL 5")
glutReshapeFunc(resize)
glutDisplayFunc(_draw)
glutMouseFunc(mouse)
glutMotionFunc(motion)
glutKeyboardFunc(keyboard)
glutTimerFunc(0, cb_dynamics, 0)
glutTimerFunc(32, cb_draw, 0)

init()
init_font()
robo_link = init_robo()
glutMainLoop()
