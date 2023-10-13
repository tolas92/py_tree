#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math

from tkinter import *
import random
import itertools

class ButtonPage():
  

    def __init__(self, root):
        self.tk = Frame(root)
        self.tk.pack(expand=True, fill="both")

        self.tk.configure(bg="white")
        self.tk.rowconfigure(tuple(range(1)), weight=1)
        self.tk.columnconfigure(tuple(range(1)), weight=1)
        self.col_a = "orange"
        self.col_b = "red"
        button_width = 10  # Set the width of the buttons
        button_height = 5 
        self.idli=None
        self.dosa=None
        self.order_picked=False
        # Create buttons for Idli
        self.idli_button = Button(
            self.tk, text="Idli",
              command=self.idli_selected,
                bg=self.col_a,
                  fg="white", 
                    width=button_width,
                     height=button_height,
                       font=("Arial", 30))
        self.idli_button.pack(
            side="top",
              anchor="center",
                pady=50)
        
        #Create button for dosa
        self.dosa_button = Button(
            self.tk, text="Dosa",
              command=self.dosa_selected,
                activebackground="red",
                  activeforeground="white",
                    bg=self.col_a,
                      fg="white",
                        width=button_width,
                          height=button_height,
                            font=("Arial", 30))
        self.dosa_button.pack(side="top", anchor="center",pady=50)

        self.order_picked_button = Button(
            self.tk, text="Order Picked",
              command=self.order_picked_callback,
                bg=self.col_a, 
                fg="white",
                  width=button_width,
                    height=button_height,
                     font=("Arial", 30))
        self.order_picked_button.pack(side="top", anchor="center", pady=50)
        
        # Label to display selection message
        self.label = Label(root, text="")
        self.label.pack(pady=10)

    def order_picked_callback(self):
               # self.order_picked_button.config(state=DISABLED)
            self.order_picked=True  # Disable the button after the order is picked
        
    def idli_selected(self):
        self.label.config(text="You selected: Idli")
        self.idli=1
        print({self.idli})

    def dosa_selected(self):
        self.label.config(text="You selected: Dosa")
        self.dosa=1

    def update_image(self):
        self.tk.update()


    def button_down(self, widget):
        widget.config(relief = "sunken")
        widget["bg"] = self.col_b


    def button_up(self, widget):
        widget.config(relief = "raised")
        widget["bg"] = self.col_a
        widget.invoke()

    def destroy(self):
        self.tk.destroy()


