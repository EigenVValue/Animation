import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

from quaternion_computation import quaternion_to_rotation

# loading textures from img files
def loadTexture(filename):
    # load the image
    textureSurface = pygame.image.load(filename)

    # get texture information
    textureData = pygame.image.tostring(textureSurface, "RGBA", 1)
    width = textureSurface.get_width()
    height = textureSurface.get_height()

    # get a unique texture id to reference it again 
    glEnable(GL_TEXTURE_2D)
    texid = glGenTextures(1)

    # define texture parameters
    glBindTexture(GL_TEXTURE_2D, texid)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, textureData)

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

    return texid

# render the cube with appropriate textures
def Cube(textures, vertices, position, quaternion):
    # get updated positions for the cube
    rotation = quaternion_to_rotation(quaternion)
    new_vertices = [rotation @ vertex for vertex in vertices]
    new_vertices = np.array(new_vertices)
    new_vertices += position

    glEnable(GL_TEXTURE_2D)

    # Render the cube one face at a time
    #front
    glBindTexture(GL_TEXTURE_2D, textures[0])
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0); glVertex3f(*new_vertices[0])
    glTexCoord2f(1.0, 1.0); glVertex3f(*new_vertices[3])
    glTexCoord2f(1.0, 0.0); glVertex3f(*new_vertices[2])
    glTexCoord2f(0.0, 0.0); glVertex3f(*new_vertices[1])
    glEnd()
    
    #back
    glBindTexture(GL_TEXTURE_2D, textures[5])
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0); glVertex3f(*new_vertices[4])
    glTexCoord2f(1.0, 1.0); glVertex3f(*new_vertices[5])
    glTexCoord2f(1.0, 0.0); glVertex3f(*new_vertices[6])
    glTexCoord2f(0.0, 0.0); glVertex3f(*new_vertices[7])
    glEnd()
    
    #left
    glBindTexture(GL_TEXTURE_2D, textures[1])
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0); glVertex3f(*new_vertices[0])
    glTexCoord2f(1.0, 1.0); glVertex3f(*new_vertices[1])
    glTexCoord2f(1.0, 0.0); glVertex3f(*new_vertices[5])
    glTexCoord2f(0.0, 0.0); glVertex3f(*new_vertices[4])
    glEnd()
    
    #right
    glBindTexture(GL_TEXTURE_2D, textures[4])
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0); glVertex3f(*new_vertices[7])
    glTexCoord2f(1.0, 1.0); glVertex3f(*new_vertices[6])
    glTexCoord2f(1.0, 0.0); glVertex3f(*new_vertices[2])
    glTexCoord2f(0.0, 0.0); glVertex3f(*new_vertices[3])
    glEnd()
    
    #upper
    glBindTexture(GL_TEXTURE_2D, textures[2])
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0); glVertex3f(*new_vertices[1])
    glTexCoord2f(1.0, 1.0); glVertex3f(*new_vertices[2])
    glTexCoord2f(1.0, 0.0); glVertex3f(*new_vertices[6])
    glTexCoord2f(0.0, 0.0); glVertex3f(*new_vertices[5])
    glEnd()
    
    #below
    glBindTexture(GL_TEXTURE_2D, textures[3])
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0); glVertex3f(*new_vertices[0])
    glTexCoord2f(1.0, 1.0); glVertex3f(*new_vertices[4])
    glTexCoord2f(1.0, 0.0); glVertex3f(*new_vertices[7])
    glTexCoord2f(0.0, 0.0); glVertex3f(*new_vertices[3])
    glEnd()

# draw the floor for the simulation
def draw_floor(floor_y):
    glBegin(GL_LINES)
    glColor3ub(0, 0, 255)
    
    for i in range(-120, 121, 5):
        glVertex3fv(np.array([i, floor_y, 120]))
        glVertex3fv(np.array([i, floor_y, -120]))
        glVertex3fv(np.array([120, floor_y, i]))
        glVertex3fv(np.array([-120, floor_y, i]))
    
    glEnd()