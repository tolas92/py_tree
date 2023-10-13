#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from tkinter import *

class KitchenTable():
    def __init__(self, root):
        self.tk=Frame(root) 
        self.tk.pack(expand=True,fill="both")
        self.tk.configure(bg="white")
        self.order_ready=False
        self.food_placed=False
        # Set custom colors
        heading_bg = "#FF5733"  # Orange
        button_bg = "#4CAF50"    # Green
        button_fg = "white"
        label_fg = "black"
        heading_label = Label(self.tk, text="Kitchen Table", bg=heading_bg, fg="white", font=("Arial", 24))
        heading_label.grid(row=0, column=0, columnspan=4, pady=(10, 20), sticky="ew")

        self.table_data = [
            ["Table", "Food", "Qty", "Call"],
            [1, "", 0, ""],
            [2, "", 0, ""],
            [3, "", 0, ""]
        ]
        self.create_table()
        update_button = Button(
            self.tk, text="Update Table",
            command=self.update_table,
              bg=button_bg,
                fg=button_fg,
                  relief=FLAT)
        update_button.grid(
            row=len(self.table_data),
              column=0,
                columnspan=4,
                  pady=10,
                    sticky="ew")

    def create_table(self):
        
        self.widgets = []  # Store widgets to be able to update them later
        for i, row in enumerate(self.table_data):
            widgets_row = []
            for j, cell_value in enumerate(row):
                if j == 3 and i != 0:  # For the 4th column and skip the header row
                    button = Button(
                        self.tk, text=cell_value,
                         command=lambda i=i: self.button_clicked(i),
                          bg="#FF5733", fg="white", relief=RAISED)  # Orange button with raised relief
                    button.grid(row=i + 1, column=j, sticky="ew")
                    widgets_row.append(button)
                else:
                    label = Label(self.tk, text=cell_value, relief=RAISED, width=10, fg="black")  # Raised relief
                    label.grid(row=i + 1, column=j, sticky="ew")
                    widgets_row.append(label)
            self.widgets.append(widgets_row)
            
        # Add the "Food Placed" button at the bottom
        food_placed_button = Button(self.tk, text="Food Placed", command=self.food_placed_button_clicked,
                                    bg="#FF5733", fg="white", relief=RAISED)
        food_placed_button.grid(row=len(self.table_data) + 1, column=3, sticky="ew")

    def food_placed_button_clicked(self):
        print("Food Placed Button clicked.")
        self.food_placed=True

    def update_table(self,):
    
        # Update the widgets in the table with the new data 
        for i, row in enumerate(self.table_data):
            for j, cell_value in enumerate(row):
                self.widgets[i][j].config(text=cell_value)

    def button_clicked(self, row_index):
        print(f"Button in row {row_index} clicked.")
        button = self.widgets[row_index][3]  # Get the button widget in the clicked row
        button.config(bg="#4CAF50") 
        self.order_ready=row_index
        print(self.order_ready)

    
    def update_image(self):
        self.tk.update()


    def destroy(self):
            self.tk.destroy()