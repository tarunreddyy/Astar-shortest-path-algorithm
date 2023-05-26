import pygame
import numpy as np
import math
import time
import tkinter as tk
from tkinter import *
from PIL import Image, ImageTk
from tkinter import messagebox
import heapq

pygame.init()

WINDOW_WIDTH, WINDOW_HEIGHT = 1200, 500

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Astar Algorithm Simulation")

MAP = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH), dtype=np.int8)

# Define the colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 191, 255)
BLACK = (0, 0, 0)
GRAY = (50, 50, 50)
LIGHT_GRAY = (100, 100, 100)
YELLOW = (255, 255, 0)

#################################################
# AStar Algorithm
#################################################
class AStar:
    def __init__(self, screen, start, end, start_orientation, end_orientation, L, MAP):
        self.screen = screen
        self.start_node = start
        self.end_node = end
        self.start_orientation = start_orientation
        self.end_orientation = end_orientation
        self.euclidean_threshold = 5
        self.theta_threshold = 30
        self.L = L
        self.MAP = MAP
        self.path = []
    
    def ActionMoveLeft(self, curr_node, orient, angle):
        x1, y1 = curr_node
        angle = orient - angle
        x = y1 + self.L * math.cos(math.radians(angle))
        y = x1 + self.L * math.sin(math.radians(angle))
        return ((int(y), int(x)), angle)

    def ActionMoveRight(self, curr_node, orient, angle):
        x1, y1 = curr_node
        angle = orient + angle
        x = y1 + self.L * math.cos(math.radians(angle))
        y = x1 + self.L * math.sin(math.radians(angle))
        return ((int(y), int(x)), angle)

    def ActionMoveStraight(self, curr_node, orient, angle):
        x1, y1 = curr_node
        angle = orient
        x = y1 + self.L * math.cos(math.radians(angle))
        y = x1 + self.L * math.sin(math.radians(angle))
        return ((int(y), int(x)), angle)

    def generate_successors(self, curr_node, curr_orient, visited):
        actions = [(self.ActionMoveLeft, 60), (self.ActionMoveLeft, 30), (self.ActionMoveStraight, 0), (self.ActionMoveRight, 30), (self.ActionMoveRight, 60)]

        all_neighbors = []

        for action, angle in actions:
            n = action(curr_node, curr_orient, angle)
            if n is not None and n[0][0] >= 0 and n[0][0] < self.MAP.shape[1] and n[0][1] >= 0 and n[0][1] < self.MAP.shape[0] and self.MAP[n[0][1]][n[0][0]] != -1:
                too_close = False
                for v in visited:
                    if self.distance(v, n[0]) < self.euclidean_threshold:
                        too_close = True
                        break
                if not too_close:
                    all_neighbors.append(n)
        return all_neighbors

    def plot_vectors(self, parent_node, child_nodes):
        for end, oriet in child_nodes:
            dx = end[0] - parent_node[0]
            dy = end[1] - parent_node[1]
            angle = math.atan2(dy, dx)

            pygame.draw.line(screen, RED, parent_node, end, 2)

            arrow_length = 4
            arrow_width = 3
            arrow_points = [
                (end[0] - arrow_length * math.cos(angle) + arrow_width * math.sin(angle), end[1] - arrow_length * math.sin(angle) - arrow_width * math.cos(angle)),
                (end[0], end[1]),
                (end[0] - arrow_length * math.cos(angle) - arrow_width * math.sin(angle), end[1] - arrow_length * math.sin(angle) + arrow_width * math.cos(angle))
            ]
            pygame.draw.polygon(self.screen, RED, arrow_points)
        pygame.display.flip()

    def distance(self, node1, node2):
        return math.sqrt((node2[0] - node1[0])**2 + (node2[1] - node1[1])**2)

    def angle_difference(self, angle1, angle2):
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = 360 - diff
        return diff

    def is_goal_reached(self, current_node, curr_orient):
        if self.distance(current_node, self.end_node) <= self.euclidean_threshold:
            return True
        return False

    def generate_path(self, parents, final_node):
        print("Generating Path")
        path = []
        state = final_node
        while state is not None:
            path.append(state)
            state = parents.get(state)
        path.reverse()
        return path

    def heuristic_cost(self, node1, node2):
        return math.sqrt((node2[0] - node1[0])**2 + (node2[1] - node1[1])**2)

    def explore(self):
        start_heuristic = self.heuristic_cost(self.start_node, self.end_node)
        open_set = [(start_heuristic, (self.start_orientation, self.start_node))]
        parent_nodes = {}
        closed_set = {}
        explored = {}
        visited = set()

        while open_set:
            curr_cost, current_node = heapq.heappop(open_set)
            current_cost = curr_cost - self.heuristic_cost(current_node[1], self.end_node)

            if self.is_goal_reached(current_node[1], current_node[0]):
                return parent_nodes, self.generate_path(explored, current_node[1])

            neighbors = self.generate_successors(current_node[1], current_node[0], visited)

            for neighbor in neighbors:
                tentative_g_cost = current_cost + self.heuristic_cost(current_node[1], neighbor[0])
                f_cost = tentative_g_cost + self.heuristic_cost(neighbor[0], self.end_node)

                if neighbor in closed_set and f_cost >= closed_set[neighbor]:
                    continue

                if neighbor not in open_set or f_cost < closed_set[neighbor]:
                    closed_set[neighbor] = f_cost
                    heapq.heappush(open_set, (f_cost, (neighbor[1], neighbor[0])))
                    parent_nodes[neighbor] = current_node[1]
                    explored[neighbor[0]] = current_node[1]
                    visited.add(neighbor[0])
                    self.plot_vectors(current_node[1], neighbors)

        return parent_nodes, None
    

#################################################
# Visualization
#################################################
def draw_shapes(clearance, cal_map = True):
    screen.fill(LIGHT_GRAY)

    # Draw the shapes
    rect1 = pygame.draw.rect(screen, RED, (200, 0, 100, 200))
    rect2 = pygame.draw.rect(screen, RED, (200, 300, 100, 200))

    center = (600, 250)
    side_length = 150
    vertices = []
    for i in range(6):
        angle_deg = 60 * i - 30
        angle_rad = math.pi / 180 * angle_deg
        x = center[0] + side_length * math.cos(angle_rad)
        y = center[1] + side_length * math.sin(angle_rad)
        vertices.append((x, y))

    hexagon = pygame.draw.polygon(screen, RED, vertices, 0)
    triangle = pygame.draw.polygon(screen, RED, [(920, 450), (1020, 250), (920, 50)])
    
    font = pygame.font.Font('freesansbold.ttf', 8)
    direc_center = (60, 430)
    arrow_end = [((direc_center[0]-25,direc_center[1]),"270"),((direc_center[0]+25,direc_center[1]),"90"),((direc_center[0],direc_center[1]-25),"180"),((direc_center[0],direc_center[1]+25),"0")]
    for arrow, text in arrow_end:
        text_surface = font.render(text, True, WHITE, LIGHT_GRAY)
        text_rect = text_surface.get_rect()
        
        dx = arrow[0] - direc_center[0]
        dy = arrow[1] - direc_center[1]
        angle = math.atan2(dy, dx)

        pygame.draw.line(screen, BLACK, direc_center, arrow, 2)

        arrow_length = 4
        arrow_width = 3
        arrow_points = [
            (arrow[0] - arrow_length * math.cos(angle) + arrow_width * math.sin(angle), arrow[1] - arrow_length * math.sin(angle) - arrow_width * math.cos(angle)),
            arrow,
            (arrow[0] - arrow_length * math.cos(angle) - arrow_width * math.sin(angle), arrow[1] - arrow_length * math.sin(angle) + arrow_width * math.cos(angle))
        ]
        pygame.draw.polygon(screen, BLACK, arrow_points)
        
        text_rect.centerx = arrow_points[0][0] + arrow_width * math.sin(angle)
        text_rect.centery = arrow_points[0][1] - arrow_width * math.cos(angle)
        
        screen.blit(text_surface, text_rect)

    
    pygame.display.update()
    if cal_map:
        print("Calculating Robot Clearance")
        MAP = create_map(screen, clearance)

        for i in range(MAP.shape[0]):
            for j in range(MAP.shape[1]):
                if MAP[i][j] == -1:
                    screen.set_at((j, i), WHITE)

    rect1 = pygame.draw.rect(screen, RED, (200, 0, 100, 200))
    rect2 = pygame.draw.rect(screen, RED, (200, 300, 100, 200))

    center = (600, 250)
    side_length = 150
    vertices = []
    for i in range(6):
        angle_deg = 60 * i - 30
        angle_rad = math.pi / 180 * angle_deg
        x = center[0] + side_length * math.cos(angle_rad)
        y = center[1] + side_length * math.sin(angle_rad)
        vertices.append((x, y))

    hexagon = pygame.draw.polygon(screen, RED, vertices, 0)
    triangle = pygame.draw.polygon(screen, RED, [(920, 450), (1020, 250), (920, 50)])

    pygame.display.update()
    return screen

def create_map(screen, clearance):
    for i in range(0, WINDOW_HEIGHT):
        for j in range(0, WINDOW_WIDTH):
            if screen.get_at((j, i)) == RED:
                for ii in range(i - clearance, i + clearance):
                    for jj in range(j - clearance, j + clearance):
                        if ii >= 0 and ii < WINDOW_HEIGHT and jj >= 0 and jj < WINDOW_WIDTH:
                            MAP[ii][jj] = -1
    return MAP


def select_box(screen):
    selected_box = None
    while selected_box is None:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                box_col = mouse_x
                box_row = mouse_y
                selected_box = (box_col, box_row)
                pygame.draw.circle(screen, BLUE, (box_col, box_row), 10)
                pygame.display.update()
                return selected_box
            elif event.type == pygame.QUIT:
                pygame.quit()

def draw_points(point_list):
    radius = 5
    
    for i, point in enumerate(point_list):
        pygame.draw.circle(screen, BLUE, point, radius)
        if i > 0:
            pygame.draw.line(screen, YELLOW, point_list[i-1], point)
            
    pygame.display.update()
    

def get_param():
    robot_clearance = 0
    start_orientation = 0
    end_orientation = 0
    L = 0

    # Create window
    window = Tk()
    window.title("Astar Parameters")
    window.geometry("1500x600")
    window.configure(bg="#f2f2f2")

    title_label = Label(window, text="Astar Parameters", font=("Arial", 24), bg="#f2f2f2")
    title_label.grid(row=0, column=0, columnspan=2, padx=20, pady=20)

    clearance_label = Label(window, text="Robot clearance (0 < Clearance <= 5):", font=("Arial", 16), bg="#f2f2f2")
    clearance_label.grid(row=1, column=0, padx=20, pady=10, sticky=W)
    clearance_entry = Entry(window, font=("Arial", 16))
    clearance_entry.grid(row=1, column=1, padx=20, pady=10)

    st_orientation_label = Label(window, text="Robot Start orientation (Try to Orient the Robot towards the exit, see directions on the right):", font=("Arial", 16), bg="#f2f2f2")
    st_orientation_label.grid(row=2, column=0, padx=20, pady=10, sticky=W)
    st_orientation_entry = Entry(window, font=("Arial", 16))
    st_orientation_entry.grid(row=2, column=1, padx=20, pady=10)

    end_orientation_label = Label(window, text="Robot End orientation:", font=("Arial", 16), bg="#f2f2f2")
    end_orientation_label.grid(row=3, column=0, padx=20, pady=10, sticky=W)
    end_orientation_entry = Entry(window, font=("Arial", 16))
    end_orientation_entry.grid(row=3, column=1, padx=20, pady=10)

    L_label = Label(window, text="Stride Length (0 < Stride <= 10)", font=("Arial", 16), bg="#f2f2f2")
    L_label.grid(row=4, column=0, padx=20, pady=10, sticky=W)
    L_entry = Entry(window, font=("Arial", 16))
    L_entry.grid(row=4, column=1, padx=20, pady=10)

    text1 = Label(window, text="Select the Start and End Nodes after submitting the Parameters", font=("Arial", 14), bg="#f2f2f2")
    text1.grid(row=5, column=0, columnspan=2, padx=20, pady=10)

    text2 = Label(window, text="Try selecting Start and End Nodes either corners on the Map", font=("Arial", 14), bg="#f2f2f2")
    text2.grid(row=6, column=0, columnspan=2, padx=20, pady=10)

    def submit():
        nonlocal robot_clearance, start_orientation, end_orientation, L
        robot_clearance = int(clearance_entry.get())
        start_orientation = int(st_orientation_entry.get())
        end_orientation = int(end_orientation_entry.get())
        L = int(L_entry.get())
        window.destroy()

    submit_button = Button(window, text="Submit", font=("Arial", 16), bg="#4CAF50", fg="white", command=submit)
    submit_button.grid(row=7, column=1, padx=20, pady=20, sticky=E)

    img = Image.open("directions.png")
    img = img.resize((150, 150))
    photo = ImageTk.PhotoImage(img)

    img_label = Label(window, image=photo)
    img_label.image = photo
    img_label.grid(row=1, column=2, rowspan=6, padx=20, pady=10)

    window.mainloop()

    return robot_clearance, start_orientation, end_orientation, L


def main():
    while True:
        robot_clearance, st_orientation, end_orientation, L  = get_param()
        print("Got Parameters")
        print(f"Robot Clearance: {robot_clearance}, Start Orientation: {st_orientation}, End Orientation: {end_orientation}, Stride: {L}")
        screen = draw_shapes(robot_clearance*2, True)

        start_node = select_box(screen)
        end_node = select_box(screen)
        if MAP[start_node[1],start_node[0]] == -1 or MAP[end_node[1],end_node[0]] == -1 or start_node == end_node:
            tk.Tk().wm_withdraw()
            messagebox.showinfo("Try Again! Start node and End node are same")
            break
        else:
            start_time = (time.process_time())
            print(f"Start node: {start_node} and End node: {end_node}")
            astar = AStar(screen, start_node, end_node, st_orientation, end_orientation, L*2, MAP)
            parents, path = astar.explore()
            if path is not None:
                print(f"Path: {path}")
                screen = draw_shapes(robot_clearance*2, False)
                draw_points(path)
            else:
                tk.Tk().wm_withdraw()
                messagebox.showinfo("Try Again! Could not Find Path")
            print(f"Time taken to process: {(time.process_time()-start_time)}")
            break
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()


if __name__ == "__main__":
    main()