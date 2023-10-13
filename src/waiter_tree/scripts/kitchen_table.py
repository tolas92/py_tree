#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the necessary ROS2 message type
from tkinter import *
from threading import Thread
from geometry_msgs.msg import Twist
from waiter_tree.kitchen_face import *
from waiter_tree.table_page import *
from rclpy.action import ActionServer
from prototype.action import FoodMenu
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.timer import Timer
import time
from prototype.msg import Food

class UiNode(Node):
    def __init__(self):
        super().__init__('UI_node')
        
        self.use_cmd_vel_for_face = True
        self.disable_cursor = False
        self.fullscreen = False

        
        # ROS2 publisher
        self.pub_=self.create_publisher(String,'chef_calling',10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_=self.create_subscription(Food,'final_order',self.update_order,10)
        #self.subscription=self.create_subscription(String,'/table_number',self.string_callback,10)
        
        self.action_=ActionServer(self,FoodMenu,'Kitchen',execute_callback=self.kitchen_callback,
                                  callback_group=ReentrantCallbackGroup(),
                                  goal_callback=self.goal_callback,
                                  cancel_callback=self.cancel_callback)
        
        self.ttk = Tk()
        self.ttk.title("kitchen")
        self.ttk.geometry("1024x600+0+0")
       #   self.update_timer = self.create_timer(1.0, self.update_image_callback)

        if self.fullscreen:
            self.ttk.bind("<Escape>", self.end_fullscreen)
            self.ttk.attributes("-fullscreen",True)

        if self.disable_cursor:
            self.ttk.config(cursor="none")

        self.ttk.rowconfigure(0, weight=1)
        self.ttk.columnconfigure(0, weight=1)

        self.face_page = None
        self.button_page = None

        
        self.build_button_page()
        
    
    def build_face_page(self):
        self.face_page = FacePlayerCars(self.ttk)
    
    def destroy_face_page(self):
        self.face_page.destroy()
        self.face_page = None

    def update_image(self):
        
        if self.face_page:
            self.face_page.update_image()

        if self.button_page:
            self.button_page.update_image()
        
        order_status_mapping = {
                            1: "one",
                            2: "two"
                            }

        if self.button_page.order_ready in order_status_mapping:
            msg = String()
            msg.data = order_status_mapping[self.button_page.order_ready]
            self.pub_.publish(msg)
            self.button_page.order_ready = None



        return
        
    def cmd_vel_callback(self, cmd_vel):
        
        if self.face_page and self.use_cmd_vel_for_face:
            self.face_page.update_values(cmd_vel.angular.z/1.0, abs(cmd_vel.linear.x/1.0))
            

        return
    
    def build_button_page(self):
        self.button_page = KitchenTable(self.ttk)

    def destroy_button_page(self):
        self.button_page.destroy()
        self.button_page = None

    async def kitchen_callback(self,goal_handle):
        feedback_msg=FoodMenu.Feedback()
        end_time=time.time()+5
        while self.button_page.food_placed is False:
              feedback_msg.waiting="waiting_for_food"
              goal_handle.publish_feedback(feedback_msg)
              
        # If either idli or dosa is not None, succeed the goal
        goal_handle.succeed()
        result=FoodMenu.Result()
        result.done=True
        self.button_page.food_placed=False
                   
        return result   
    
        
    
    def goal_callback(self,goal_request):
        self.get_logger().info("goal request")
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
   # def update_image_callback(self):
        # Callback function for the timer to update the image
    #    self.update_image()

    def update_order(self,msg):
         if msg.table_number==100:
             self.button_page.table_data[1][1]=msg.food_type
             self.button_page.table_data[1][2]=msg.qty
         

         elif msg.table_number==2:
             self.button_page.table_data[2][1]=msg.food_type
             self.button_page.table_data[2][2]=msg.qty

             
         
                
    
def main(args=None):
    
    rclpy.init(args=args)
    ui_node = UiNode()
    """
    def run_gui():
        rclpy.spin(ui_node)
        ui_node.destroy_node()
        rclpy.shutdown()
    gui_thread=Thread(target=run_gui)
    gui_thread.start()
    """
    """
    while rclpy.ok():
        rclpy.spin_once(ui_node)
        #ui_node.check_for_finished_calls()
        ui_node.update_image()
        time.sleep(0.01)
    ui_node.destroy_node()
    rclpy.shutdown()
    """
    executor=MultiThreadedExecutor()
    while rclpy.ok():

        rclpy.spin_once(ui_node,executor=executor,timeout_sec=1.0)
        time.sleep(0.01)
        ui_node.update_image()

    
    ui_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()