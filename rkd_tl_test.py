#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from numpy import *

#from dp_roki import *
from rkd import *

tz = 1.0
view_dis = 15.5
round = 0.0
view_z = 2.0
#count = 10.0

quadric = None

def robo_func (link, base_pos, pos) :
    print(" pos --> {0}".format(pos))
    glVertex3d(base_pos[0], base_pos[1], base_pos[2])
    glVertex3d(pos[0], pos[1], pos[2])

    return

def robo_joint_func (link, base_pos, pos) :
    glPushMatrix()
    glTranslated(base_pos[0], base_pos[1], base_pos[2])

    ltrans = mat3to4(link.local2world.T)
    glMultMatrixd(ltrans)

    # draw link
    glColor3d(0.8,0.8,0.8)
    glutSolidCylinder( 0.1, link.tip_offset[2], 20, 6)

    #print link.local2world
    #print ltrans
    ltrans = mat3to4(link.joint.get_rot_z2axis())
    glMultMatrixd(ltrans)

    cylinder_height = 0.6
    glTranslated(0, 0, -(cylinder_height / 2.0))

    # draw joint
    glColor3d(0.1,0.5,0.1)
    glutSolidCylinder( 0.2, cylinder_height, 20, 6)
    glColor3d(0.0,0.0,0.0)
    glutWireCylinder( 0.2, cylinder_height, 20, 6)

    #glutSolidSphere( 0.4, 20, 6)
    #gluCylinder(quadric, 0, 10, 10, 20, 6)

    glPopMatrix()
    return

def init():
    global quadric
    glClearColor(0.0, 0.5, 0.5, 0.0)
    #glEnable(GL_DEPTH_TEST)
    #glEnable(GL_CULL_FACE)
    #glEnable(GL_LIGHTING)
    #glEnable(GL_LIGHT0)
    quadric = gluNewQuadric()

def draw_floor(cx, cy, cz, width, height) :
    global quadric
    #gl_rectangle(int left, int top, int right, int bottom, int lwidth, int style, unsigned long prgb, unsigned long hrgb)
    dx = width / 2.0
    dy = height / 2.0
    glColor3d(0.3,0.3,0.3)
    glBegin(GL_TRIANGLES)
    glVertex3d(cx - dx, cy + dy, cz)
    glVertex3d(cx - dx, cy - dy, cz)
    glVertex3d(cx + dx, cy + dy, cz)
    glEnd()
    glColor3d(0.5,0.5,0.5)
    glBegin(GL_TRIANGLES)
    glVertex3d(cx + dx, cy + dy, cz)
    glVertex3d(cx - dx, cy - dy, cz)
    glVertex3d(cx + dx, cy - dy, cz)
    glEnd()

def draw():
    global view_dis
    global tz
    global robo_link
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # model view matrix reset
    glLoadIdentity()
    gluPerspective(30.0, 1.0, 0.5, 100.0); 
    #gluLookAt( 0, -view_dis, 0, 0, 0, 0, 0, 0, 1)
    gluLookAt( -view_dis * sin(deg2rad(round)), -view_dis * cos(deg2rad(round)), view_z, 0, 0, view_z, 0, 0, 1)

    draw_floor(0.0, 0.0, 0.0, 100, 100)

    glBegin(GL_LINES)
    robo_link.func_links(array([0.0,0.0,0.0]), robo_func)
    glEnd()

    # TODO:
    #robo_link.func_links(array([0.0,0.0,0.0]), robo_joint_func)

    #glColor3d(0.0,1.0,0.0)
    #glBegin(GL_TRIANGLES)
    #glVertex(-1, 0, 1)
    #glVertex(1, 0, 1)
    #glVertex(0, 0, 2)
    #glEnd()

    glFlush()
    glutSwapBuffers()

def resize(w, h):
    glViewport(0, 0, w, h)

    #glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(30.0, -view_dis, 1.0, 100.0)
    #glMatrixMode(GL_MODELVIEW)


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
    if key == '\033':
        sys.exit()
    elif key == 'q':
        sys.exit()
    elif key == 'j':
        view_dis = view_dis + 0.5
        print(view_dis)
        #gluPerspective(30.0, 1.0, -3.0, 10.0); 
        #gluLookAt( 0, 0, -view_dis, 0, 0, 0, 0, 1, 0)
        draw()
    elif key == 'k':
        view_dis = view_dis - 0.5
        print(view_dis)
        #gluPerspective(30.0, 1.0, -3.0, 10.0); 
        #gluLookAt( 0, 0, -view_dis, 0, 0, 0, 0, 1, 0)
        draw()
    elif key == 'u':
        round = round + 10.0
        draw()
    elif key == 'i':
        round = round - 10.0
        draw()
    elif key == 'n':
        view_z = view_z + 0.5
        draw()
    elif key == 'm':
        view_z = view_z - 0.5
        draw()
    #elif key == '1' or key == '2' :
    elif key in ['1','2','3','4','5','6','7','8','9','0'] :
        global robo_link
        link = robo_link.get_link(key)
        if (link != None) :
            link.joint.angle = link.joint.angle + deg2rad(10)
            robo_link.update(array([0.0,0.0,0.0]))
            draw()
            #print link.joint.angle
            return

    elif key in ['!','@','#','$','%','^','&','*','(',')'] :
        index = ['!','@','#','$','%','^','&','*','(',')'].index(key)
        value = ['1','2','3','4','5','6','7','8','9','0'][index]
        global robo_link
        link = robo_link.get_link(str(value))
        if (link != None) :
            link.joint.angle = link.joint.angle - deg2rad(10)
            robo_link.update(array([0.0,0.0,0.0]))
            draw()
            #print link.joint.angle
            return
        
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

glutInit(sys.argv)
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
glutInitWindowSize(640, 480)
glutCreateWindow("PyOpenGL 5")
glutReshapeFunc(resize)
glutDisplayFunc(draw)
glutMouseFunc(mouse)
glutMotionFunc(motion)
glutKeyboardFunc(keyboard)

init()
robo_link = init_robo()
glutMainLoop()
