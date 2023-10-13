#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math

from tkinter import *
import random
import itertools



class FacePlayerCars():

    def __init__(self, root):


        self.width=1024
        self.height=600
        self.nomwidth = 800
        self.nomheight = 480
        self.eye_pos = 0

        self.eye_width = self.nom_x(100)
        self.eye_height = self.nom_y(280)
        self.blink_factor = 0.002
        self.x_centre = self.nom_x(400)
        self.y_centre = self.nom_y(100)
        self.iris_size = self.nom_x(150)
        self.pupil_size = self.nom_x(76)
        self.highlight_size = self.nom_x(20)
        self.highlight_offset = self.nom_x(25)
        self.eye_spacing = self.nom_x(310)



        self.left_centre = self.x_centre - self.eye_spacing/2
        self.right_centre = self.x_centre + self.eye_spacing/2
        self.eye_top = self.y_centre - self.eye_height/2
        self.eye_bottom = self.y_centre + self.eye_height/2
        self.leye_left = self.left_centre - self.eye_width/2
        self.leye_right = self.left_centre + self.eye_width/2
        self.reye_left = self.right_centre - self.eye_width/2
        self.reye_right = self.right_centre + self.eye_width/2


        self.tk = Frame(root)
        self.tk.pack(expand=True, fill="both")
        
        self.canvas = Canvas(self.tk, width=self.width, height=self.height)
        self.canvas.pack(anchor='nw')
        self.canvas.configure(bg='black')

        self.col_a = "#ff7b00"
        self.col_b = "#a65000"

        # Create eyes
        self.iris_colour = "teal"

        self.iris_l = self.canvas.create_rectangle(self.left_centre - self.iris_size/2, 
                                                self.y_centre - self.iris_size/2, 
                                                self.left_centre + self.iris_size/2, 
                                                self.y_centre + self.iris_size/2,
                                                fill=self.iris_colour)

        self.iris_r = self.canvas.create_rectangle(self.right_centre - self.iris_size/2, 
                self.y_centre - self.iris_size/2, 
                self.right_centre + self.iris_size/2, 
                self.y_centre + self.iris_size/2,
                fill=self.iris_colour)
                                      
        self.counter = 0

        self.mouth_width = self.nom_x(300)
        self.mouth_height = self.nom_y(100)
        self.mouth_bottom = self.nom_y(400) + self.nom_y(50)
        self.mouth_left = self.x_centre - self.mouth_width / 2
        self.mouth_right = self.x_centre + self.mouth_width / 2

        # Create mouth
        self.mouth = self.canvas.create_rectangle(self.mouth_left, self.mouth_bottom - self.mouth_height, 
                                            self.mouth_right, self.mouth_bottom, 
                                            fill='teal', outline='teal')
    
    
    def update_image(self):

        self.tk.update()
        self.counter += 1
        
        self.set_pupil_centre('l', 140*self.eye_pos)
        self.set_pupil_centre('r', 140*self.eye_pos)
        
   
    def end_fullscreen(self, event=None):
        self.tk.attributes("-fullscreen", False)
        return "break"

    def set_pupil_centre(self, pupil, offset):
        if pupil == 'l':
            self.canvas.moveto(self.iris_l, self.left_centre - self.iris_size/2 + offset, self.y_centre-self.iris_size/2)
        else:
            self.canvas.moveto(self.iris_r, self.right_centre - self.iris_size/2 + offset, self.y_centre-self.iris_size/2)

   
    def px_x(self,val):
        return math.floor(val*self.width)

    def px_y(self,val):
        return math.floor(val*self.height)

    def pc_x(self,val,nomwidth):
        return math.floor(val/nomwidth)

    def pc_y(self,val,nomheight):
        return math.floor(val/nomheight)

    def nom_x(self, val):
        return math.floor(val * self.width/self.nomwidth)

    def nom_y(self, val):
        return math.floor(val * self.height/self.nomheight)

    def set_left_eye_pts(self, ptA, ptB, ptC, ptD):
        self.pts[8] = ptD
        self.pts[9] = ptD
        self.pts[10] = ptC
        self.pts[11] = ptB
        self.pts[12] = ptA
        self.pts[13] = ptA

    def set_right_eye_pts(self, ptA, ptB, ptC, ptD):
        self.pts[2] = ptD
        self.pts[3] = ptD
        self.pts[4] = ptC
        self.pts[5] = ptB
        self.pts[6] = ptA
        self.pts[7] = ptA

    def update_values(self, eye_pos, squint_amount):
        self.eye_pos = eye_pos
        if (abs(self.eye_pos) < 0.2):
            self.eye_pos = 0

    def destroy(self):
        self.tk.destroy()

