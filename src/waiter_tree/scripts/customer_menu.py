#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the necessary ROS2 message type
from tkinter import *
from threading import Thread
from geometry_msgs.msg import Twist
from waiter_tree.face import *
from waiter_tree.menu import *
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
        
        # ROS2 publisher and subscribers
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher=self.create_publisher(Food,'final_order',10)

        #ROS2 Action servers
        self.action_=ActionServer(
            self,FoodMenu,'FoodMenu', 
             execute_callback=self.foodmenu_callback,
                callback_group=ReentrantCallbackGroup(),
                    goal_callback=self.goal_callback,
                            cancel_callback=self.cancel_callback)
        self.action_=ActionServer(
            self,FoodMenu,'Table',
              execute_callback=self.table_callback,
                callback_group=ReentrantCallbackGroup(),
                    goal_callback=self.goal_callback,
                        cancel_callback=self.cancel_callback)

    
        self.ttk = Tk()
        self.ttk.title("Bot UI")
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
            print(self.button_page.idli)

        return
    
    def cmd_vel_callback(self, cmd_vel):
        
        if self.face_page and self.use_cmd_vel_for_face:
            self.face_page.update_values(cmd_vel.angular.z/1.0, abs(cmd_vel.linear.x/1.0))
            

        return
    
    def build_button_page(self):
        self.button_page = ButtonPage(self.ttk)

    def destroy_button_page(self):
        self.button_page.destroy()
        self.button_page = None

    
    async def foodmenu_callback(self,goal_handle):
        feedback_msg=FoodMenu.Feedback()
        end_time=time.time()+5
        while self.button_page.idli is None:
              
              feedback_msg.waiting="going"
              goal_handle.publish_feedback(feedback_msg)
              print(feedback_msg.waiting)
        msg=Food()
        msg.table_number=100
        msg.food_type="idli"
        msg.qty=self.button_page.idli
        self.publisher.publish(msg)
        print(msg.food_type)
        # If either idli or dosa is not None, succeed the goal
        goal_handle.succeed()
        result=FoodMenu.Result()
        result.done=True
        self.button_page.idli=None
                      
        return result      

    async def table_callback(self,goal_handle):
        feedback_msg=FoodMenu.Feedback()
        end_time=time.time()+5
        while self.button_page.order_picked is False:
              
              feedback_msg.waiting="going"
              goal_handle.publish_feedback(feedback_msg)
              print(feedback_msg.waiting)

        goal_handle.succeed()
        result=FoodMenu.Result()
        result.done=True
        self.button_page.order_picked=False
                      
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