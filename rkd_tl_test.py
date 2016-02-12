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

    glPushMatrix()

    ##########################
    ############## draw link
    #glColor3d(1.0,1.0,1.0)
    glMaterialfv(GL_FRONT , GL_SPECULAR, [1.0,1.0,1.0, 1.0])
    glMaterialfv(GL_FRONT , GL_DIFFUSE, [0.7,0.7,0.7, 1.0])
    glMaterialfv(GL_FRONT , GL_AMBIENT, [0.35,0.35,0.35, 1.0])
    glMaterialf(GL_FRONT , GL_SHININESS, 0.8)
    gluCylinder( quadric, 0.1, 0.1, link.tip_offset[2], 20, 6)
    # TODO: gluDisk

    #print link.local2world
    #print ltrans
    ltrans = mat3to4(link.joint.get_rot_z2axis())
    glMultMatrixd(ltrans)

    ############## draw joint
    glMaterialfv(GL_FRONT , GL_SPECULAR, [1.0,1.0,1.0, 1])
    glMaterialfv(GL_FRONT , GL_AMBIENT, [0.0,1.0,0.0, 1])
    glMaterialfv(GL_FRONT , GL_DIFFUSE, [0.0,0.8,0.0, 1])
    glMaterialf(GL_FRONT , GL_SHININESS, 0.8)

    cylinder_height = 0.6
    glTranslated(0, 0, -(cylinder_height / 2.0))

    gluCylinder( quadric, 0.2, 0.2, cylinder_height, 10, 2)

    #gluQuadricOrientation(quadric, GLU_INSIDE);
    gluDisk( quadric, 0.0, 0.2, 10, 2)
    #gluQuadricOrientation(quadric, GLU_OUTSIDE);

    glTranslated(0, 0, (cylinder_height))
    gluDisk( quadric, 0.0, 0.2, 10, 2)
    ##########################

    glPopMatrix()

    glPopMatrix()
    return

def init():
    global quadric
    quadric = gluNewQuadric()
    #/* 面の塗り潰しを指定する（線画ではなく陰影をつけた円柱を描く）*/
    gluQuadricDrawStyle(quadric, GLU_FILL);
    #/* スムースシェーディングを行うよう設定する */
    gluQuadricNormals(quadric, GLU_SMOOTH);


    #glLightfv(GL_LIGHT0, GL_AMBIENT, [1,0,1.0,1.0])
    glLightfv(GL_LIGHT0, GL_POSITION, [-1.0, -1.0, 4.0, 1.0]);
    glLightfv(GL_LIGHT0, GL_AMBIENT, [1,0, 1.0, 1.0, 1.0])
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
    #glEnable(GL_CULL_FACE)
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

def draw():
    global view_dis
    global tz
    global robo_link
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(30.0, 1.0, 0.5, 100.0); 
    vx = -view_dis * sin(deg2rad(round))
    vy = -view_dis * cos(deg2rad(round))
    gluLookAt( vx, vy, view_z, 0, 0, view_z, 0, 0, 1)

    # model view matrix reset
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    draw_floor(0.0, 0.0, -1.0, 100, 100)

    #glBegin(GL_LINES)
    #robo_link.func_links(array([0.0,0.0,0.0]), robo_func)
    #glEnd()

    # TODO:
    robo_link.func_links(array([0.0,0.0,0.0]), robo_joint_func)

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
        draw()
    elif key == b'k':
        view_dis = view_dis - 0.5
        print(view_dis)
        #gluPerspective(30.0, 1.0, -3.0, 10.0); 
        #gluLookAt( 0, 0, -view_dis, 0, 0, 0, 0, 1, 0)
        draw()
    elif key == b'u':
        round = round + 10.0
        draw()
    elif key == b'i':
        round = round - 10.0
        draw()
    elif key == b'n':
        view_z = view_z + 0.5
        draw()
    elif key == b'm':
        view_z = view_z - 0.5
        draw()
    #elif key == b'1' or key == b'2' :
    elif key in [b'1',b'2',b'3',b'4',b'5',b'6',b'7',b'8',b'9',b'0'] :
        key = key.decode()
        print("==>{0}".format(str(key)))
        global robo_link
        link = robo_link.get_link(str(key))
        if (link != None) :
            link.joint.angle = link.joint.angle + deg2rad(10)
            robo_link.update(array([0.0,0.0,0.0]))
            draw()
            #print (link.joint.angle)
            return

    elif key in [b'!',b'@',b'#',b'$',b'%',b'^',b'&',b'*',b'(',b')'] :
        index = [b'!',b'@',b'#',b'$',b'%',b'^',b'&',b'*',b'(',b')'].index(key)
        value = ['1','2','3','4','5','6','7','8','9','0'][index]
        print(key)
        global robo_link
        link = robo_link.get_link(str(value))
        if (link != None) :
            link.joint.angle = link.joint.angle - deg2rad(10)
            robo_link.update(array([0.0,0.0,0.0]))
            draw()
            #print (link.joint.angle)
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
