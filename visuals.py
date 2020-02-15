import pygame
import helpers as hp
from environment import env as en
import numpy as np

""" Screen parameters """
width = 800
height = 800
center = np.array([width/2, height/2])
screen = pygame.display.set_mode((width, height))
fpsClock = pygame.time.Clock()
fps = 400

""" Colors """
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
white = (255, 255, 255)
yellow = (255,255, 0)
dark_red = (170,0,0)
gray = (100,100,100)

""" Convert coordinates form cartesian to screen  (used to draw in pygame screen) """
def cartesian_to_screen(car_pos):
   factor = 0.02
   screen_pos = np.array([center[0] * factor + car_pos[0], center[1] * factor - car_pos[1]]) / factor
   screen_pos = screen_pos.astype(int)
   return screen_pos


""" Convert coordinates form screen to cartesian  (used to draw in pygame screen) """
def screen_to_cartesian(screen_pos):
   factor = 0.02
   car_pos = np.array([screen_pos[0] - center[0], center[1] - screen_pos[1]]) * factor
   car_pos = car_pos.astype(float)
   return car_pos



""" Draw all elements of scene """
def draw_scene(t, dr):

    pygame.event.get()
    screen.fill((0, 0, 0))                                                                                          # Erase screen

    """ Draw grid of obstacles"""
    for i in range(en.grid.n):
        for j in range(en.grid.n):
            if len(en.grid.grid[t][i][j]) > 0:


                if True not in en.grid.grid[t][i][j]:
                    pygame.draw.circle(screen, blue, cartesian_to_screen(hp.coorToCartesian([i, j], en.grid.n)), 6)
                else:
                    pygame.draw.circle(screen, dark_red, cartesian_to_screen(hp.coorToCartesian([i, j], en.grid.n)), 6)

                if "collision" in en.grid.grid[t][i][j]:
                    pygame.draw.circle(screen, white, cartesian_to_screen(hp.coorToCartesian([i, j], en.grid.n)), 6)
    for hub in en.hubs:
        pygame.draw.circle(screen, yellow, cartesian_to_screen(hub.node.pos), 10)                                   # Draw hub

        for node in hub.graph.nodes:                                                                                # Draw nodes
            pygame.draw.circle(screen, gray, cartesian_to_screen(node.pos), 2)

        for edge in hub.graph.edges:
            pygame.draw.line(screen, gray, cartesian_to_screen(edge.st.pos), cartesian_to_screen(edge.en.pos),1)    # Draw edges

    for drone in dr:                                                                                            # Draw drones
        pygame.draw.circle(screen, white, cartesian_to_screen(drone.node.pos), 10)

        for node in drone.graph.nodes:                                                                                # Draw nodes
            pygame.draw.circle(screen, gray, cartesian_to_screen(node.pos), 2)

        for edge in drone.graph.edges:
            pygame.draw.line(screen, white, cartesian_to_screen(edge.st.pos), cartesian_to_screen(edge.en.pos),1)    # Draw edges
        if drone.path != None:  # Draw path
            for edge in drone.path.edges:
                pygame.draw.line(screen, yellow, cartesian_to_screen(edge.st.pos), cartesian_to_screen(edge.en.pos), 4)
    for drone in en.drones:
        pygame.draw.circle(screen, green, cartesian_to_screen(drone.node.pos), 5)

    pygame.display.flip()


