import glfw
import OpenGL
OpenGL.ERROR_LOGGING = False

from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

from ForwardKinematics import calculateJointPos, processBVH

zoom = 45.
scr_width = 800.
scr_height = 600.

xtranslation = 0.
ytranslation = 0.

azimuth = -300.
elevation = 0.

right_button_flag = False
left_button_flag = False
oldX = 0.
oldY = 0.

skeleton_data = None
frame_no = 0
n_pressed = False

def scroll_callback(window, xoffset, yoffset):
    global zoom
    
    zoom -= yoffset

def button_callback(window, button, action, mod):
    global right_button_flag, left_button_flag

    if button==glfw.MOUSE_BUTTON_LEFT:
        if action==glfw.PRESS:
            right_button_flag = True
        elif action==glfw.RELEASE:
            right_button_flag = False
    if button==glfw.MOUSE_BUTTON_RIGHT:
        if action==glfw.PRESS:
            left_button_flag = True
        elif action==glfw.RELEASE:
            left_button_flag = False

def key_callback(window, key, scancode, action, mods):
    global n_pressed

    if key==glfw.KEY_N:
        if action==glfw.PRESS:
            n_pressed = True
        elif action==glfw.RELEASE:
            n_pressed = False
        elif action==glfw.REPEAT:
            n_pressed = True

def cursor_callback(window, xpos, ypos):
    global azimuth, elevation, right_button_flag, oldX, oldY, xtranslation, ytranslation
    
    if right_button_flag == True:
        azimuth += xpos - oldX
        elevation += ypos - oldY
    elif left_button_flag == True:
        xtranslation += (xpos - oldX)/50
        ytranslation += (ypos - oldY)/50
        
    oldX = xpos
    oldY = ypos

def drawCube(p1, p2):
    # get the x,y,z coordinated from the points
    p1 = p1[:3]
    p2 = p2[:3]

    if (p1[0] == p2[0] and p1[1] == p2[1] and p1[2] == p2[2]):
        return

    # render the lines
    glColor3f(1, 1, 1)

    glBegin(GL_LINES)
    glVertex3f(p1[0], p1[1], p1[2])
    glVertex3f(p2[0], p2[1], p2[2])
    glEnd()

def draw_figure(bones):
    # iterate over the bones to render each of them
    for bone in bones:
        drawCube(bone[0], bone[1])

def draw_floor():
    glBegin(GL_LINES)
    glColor3ub(0, 0, 255)
    
    for i in range(-120, 121):
        glVertex3fv(np.array([i, 0, 120]))
        glVertex3fv(np.array([i, 0, -120]))
        glVertex3fv(np.array([120, 0, i]))
        glVertex3fv(np.array([-120, 0, i]))
    
    glEnd()

def render(bones):
    global zoom, xtranslation, ytranslation, azimuth, elevation, scr_height, scr_width

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL )
    
    # reshape display window function, locating camera
    glLoadIdentity()
    gluLookAt(195.0 * np.cos(10.0), 110.0, 195.0 * np.sin(10.0), 0.0, 50.0, 0.0, 0.0, 1.0, 0.0)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(zoom, scr_width / scr_height, 1.0, 1000.0)
    glTranslatef(0., 0., -70)
    glMatrixMode(GL_MODELVIEW)

    # Use global parameters to rotate camera
    glTranslatef(-xtranslation, -ytranslation, 0)
    glRotatef(elevation/2., 1., 0., 0.)
    glRotatef(azimuth/2., 0., 1., 0.)

    # drawing the floor
    glColor3ub(255, 255, 255)
    draw_floor()
    
    # draw the bones
    glPushMatrix()
    glColor3ub(0, 255, 255)
    glPopMatrix()

    draw_figure(bones)

# calculate joint positions and render the skeleton and the background
def render_scene(no_of_frames):
    global frame_no
    # Get updated positions
    bones = calculateJointPos(skeleton_data, frame_no)
    # Render bones and the background
    render(bones)
    # Update current frame
    frame_no += 1
    frame_no = frame_no % no_of_frames

def main():
    global skeleton_data, frame_no, n_pressed
    # Initialize the library
    if not glfw.init():
        return
    
    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(int(scr_width), int(scr_height), "CMPT466 Programming Assignment II", None, None)
    if not window:
        glfw.terminate()
        return

    # Define all the callbacks
    glfw.set_key_callback(window, key_callback)
    glfw.set_cursor_pos_callback(window, cursor_callback)
    glfw.set_mouse_button_callback(window, button_callback)
    glfw.set_scroll_callback(window, scroll_callback)

    # Make the window's context current
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    # get total number of frames
    no_of_frames = skeleton_data[2].shape[0]
    
    # render first frame 
    render_scene(no_of_frames)
    glfw.swap_buffers(window)

    # Loop until the user closes the window
    while not glfw.window_should_close(window):
        # Poll for and process events
        glfw.poll_events()

        if n_pressed:
            # render the entire scene
            render_scene(no_of_frames)
            # Swap front and back buffers
            glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    # filename for the bvh file
    bvh_filename = 'running.bvh'
    # parse the bvh file to get info on the skeleton and frames
    skeleton_data = processBVH(bvh_filename)
    
    main()