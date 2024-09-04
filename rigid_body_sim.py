import pygame
import numpy as np
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

from render_dice import Cube, draw_floor, loadTexture
from rigid_body import RigidBody

zoom = 45.
scr_width = 1080
scr_height = 600

xtranslation = 0.
ytranslation = 0.

azimuth = -300.
elevation = 0.

right_button_flag = False
left_button_flag = False
oldX = 0.
oldY = 0.

def render_cube(textures, vertices, position, quaternion):
    Cube(textures, vertices, position, quaternion)
    pygame.display.flip()

def simulate_step(cube: RigidBody, textures, ispaused: bool):
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
    draw_floor(cube.floor_y)
    
    # draw the cube
    glPushMatrix()
    glColor3ub(255, 255, 255)
    glPopMatrix()

    if ispaused == False:    
        # calculate updated state variables for the cube
        for _ in range(cube.nsubsteps):
            cube.simulate_one_substep()
                
    # render cube based on the state variables
    render_cube(textures, cube.vertices, cube.position, cube.quaternion)
    pygame.time.wait(int(1000 * cube.timestep))

if __name__=="__main__":
    # Initialize the pygame library
    pygame.init()
    # Get our instance of rigidbody. In this case its a cube
    cube = RigidBody()
    # Set the size of the display and create a window using pygame
    display = (scr_width, scr_height)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
    
    # Load the textures for the 6 faces for the dice
    textures = []
    for i in range(1, 7):
        textures.append(loadTexture("textures/face{0}.jpg".format(i)))
    
    # pause is control the playback for the simulation 
    pause = True
    
    # Render the first frame
    simulate_step(cube, textures, pause)
    
    while True:
        for event in pygame.event.get():
            # callback for exiting the game
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            
            # callback for various keyboard inputs 
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    pause = ~pause
                if event.key == pygame.K_r:
                    cube = RigidBody()
                    pause = False
                if event.key == pygame.K_s:
                    pause = False
                if event.key == pygame.K_n and pause:
                    for _ in range(cube.nsubsteps):
                        cube.simulate_one_substep()
            
            # callback for mousewheel
            if event.type == MOUSEWHEEL:
                zoom -= event.y
            
            # callbacks for mouse buttons
            if pygame.mouse.get_pressed() == (1, 0, 0):
                right_button_flag = True
                left_button_flag = False

            if pygame.mouse.get_pressed() == (0, 0, 1):
                right_button_flag = False
                left_button_flag = True
            
            if pygame.mouse.get_pressed() == (0, 0, 0):
                right_button_flag = False
                left_button_flag = False
            
            # callback for mouse movements
            xpos, ypos = pygame.mouse.get_pos()
            if right_button_flag == True:
                azimuth += xpos - oldX
                elevation += ypos - oldY
            elif left_button_flag == True:
                xtranslation += (xpos - oldX)/50
                ytranslation += (ypos - oldY)/50
                
            oldX = xpos
            oldY = ypos
        
        # simulate the next step for the simulation
        simulate_step(cube, textures, ispaused=pause)
