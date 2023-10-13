#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import std_msgs
from std_msgs.msg import String

from py_trees_ros_tutorials import behaviours
import operator
import py_trees_ros_interfaces.action as py_trees_actions
import prototype.action as prototype_actions
import prototype.msg as prototype_msgs



def create_root() -> py_trees.behaviour.Behaviour:
     #creating the root that will have a parallel composite.
     root=py_trees.composites.Parallel(name="waiter",policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))

     #assigning the blackboard variables values.
     blackboard=py_trees.blackboard.Blackboard()
     blackboard.set("ai_detected_table_number"," ")
     blackboard.set("Table1_Order","clear")
     blackboard.set("Table1_Delivery","clear")
     blackboard.set("Table2_Order","clear")
     blackboard.set("Table2_Delivery","clear")

     #assigning namespaces     
     sequence=py_trees.composites.Sequence
     sub2BB=py_trees_ros.subscribers.ToBlackboard
     pyactions=py_trees_ros.actions.ActionClient
     qos=py_trees_ros.utilities.qos_profile_unlatched()
     compare=py_trees.common.ComparisonExpression
     setBBVariable=py_trees.behaviours.SetBlackboardVariable
     idiom=py_trees.idioms.either_or

       #locking in the coordinates for table1.
     table1_location=prototype_actions.GoTo.Goal()
     table1_location.x=1.0
     table1_location.y=0.0
     table1_location.z=0.0 
     feedback_1=prototype_actions.GoTo.Feedback()  

     home=prototype_actions.GoTo.Goal()
     home.x=0.0
     home.y=0.0
     home.z=0.0


     #getting the message from the ai and storing it in the ai_detected_table_number variable.  
     ai_detected_table_number_2BB=sub2BB(
         name='which table the ai detected?',
         topic_name="table_number",
         topic_type=String,
         blackboard_variables={'ai_detected_table_number':'data'},
         qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
     )

     #we check if the chef is calling us.
     chef_calling=sub2BB(
         name='chef_calling',
         topic_name='chef_calling',
         topic_type=String,
         blackboard_variables={'chef_calling':'data'},
         qos_profile=qos)


     #changing the status of the table from clear to waiting depending on the ai_detected_table_number variable value.

     customer1_allocation_status_sequence= sequence(
         name="assign 'waiting' status to Table1_Order & clear ai variable",memory=False)
     
     customer_one_waiting= setBBVariable(
         name="changing the Table1_Order status to waiting",
         variable_name="Table1_Order",variable_value="waiting",overwrite=True)
         
     clear_ai_detected_table_number_1= setBBVariable(
         name="clearing the ai variable for the next detection",
         variable_name="ai_detected_table_number",variable_value=" ",overwrite=True)

     customer2_allocation_status_sequence= sequence(
         name="assign 'waiting' status to Table2_Order & clear ai variable",memory=False)
     
     customer_two_waiting= setBBVariable(
         name="changingthe Table2_Order status to waiting" ,
         variable_name="Table2_Order",variable_value="waiting",overwrite=True)
     
     clear_ai_detected_table_number_2= setBBVariable(
         name="clearing the ai variable for the next detection",
         variable_name="ai_detected_table_number",variable_value=" ",overwrite=True)

     idiom_for_table_status= idiom(
         name="here we are checking which table number and assign it waiting status",
        conditions=[        
                    compare(variable="ai_detected_table_number",value="one",operator=operator.eq),
                    compare(variable="ai_detected_table_number",value="two",operator=operator.eq)
                    ],
                    subtrees=
                    [customer1_allocation_status_sequence,customer2_allocation_status_sequence])
     
     table_status = py_trees.decorators.FailureIsRunning(
         name="we will keep this idiom running to monitor the table number value",
         child=idiom_for_table_status)      

     #is it table 1 ?if so then go to table 1 and then change the status of table 1 to clear from waiting.
     check_if_table_1_sequence=sequence(
         name="initiate table 1 navigation sequence",
            memory=True)
     
  
      
    #calling the action client to send the coordinates for table1.
     go_to_table_1_action=py_trees_ros.actions.ActionClient(
     name="go_to_table_1",
     action_type=prototype_actions.GoTo,
     action_name="table_nav",
     action_goal=table1_location,
     generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.distance_left)
        )
     
    
     action_=prototype_actions.FoodMenu.Goal()   
     action_.order="table_1"                                    
     take_order_action=pyactions(name="open menu for table 1",
                                                   action_name="FoodMenu",
                                                   action_type=prototype_actions.FoodMenu,
                                                   action_goal=action_,
                                                   generate_feedback_message=lambda msg: msg.feedback.waiting)
     #after we reach the table 1 , we will change the status from waiting to clear.  
     clear_table_1=setBBVariable(
         name="change the status of Table1_Order to clear",
          variable_name="Table1_Order",
           variable_value="clear",
            overwrite=True)  
       

    #is it table 2 ? if so then go to table 2 and then change the status of table 2 to clear from waiting.
     check_if_table_2_sequence=sequence(
         name="initiate the table 2 navigation",
         memory=True)

     
    #locking in the coordinates for table1.
     table2_location=prototype_actions.GoTo.Goal()
     table2_location.x=3.5
     table2_location.y=0.0
     table2_location.z=0.0 
     feedback_2=prototype_actions.GoTo.Feedback()       
     
     #calling the action client to send the co_ordinates for table 2.
     go_to_table_2_action=py_trees_ros.actions.ActionClient(
       name="go_to_table_2",
         action_type=prototype_actions.GoTo,
           action_name="table_nav",
             action_goal=table2_location,
              generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_2.distance_left)
          )
       
 
    
    #we will attach this variable below the go to table action so that the kitchen table can be updated with the correct table number.  
     action_=prototype_actions.FoodMenu.Goal()   
     action_.order="table 2"                                    
     take_order_action_2=pyactions(name="open menu for table 2",
                                                   action_name="FoodMenu",
                                                   action_type=prototype_actions.FoodMenu,
                                                   action_goal=action_,
                                                   generate_feedback_message=lambda msg: msg.feedback.waiting)
     clear_table_2=setBBVariable(
      name="change the status of Table2_Order to clear",
       variable_name="Table2_Order",
        variable_value="clear",
         overwrite=True)    



    #this is the idiom that decides which table to go to.
     go_to_which_table=py_trees.idioms.either_or(name="here we are checking which table number and then start the navigation actions ",
                                              conditions=[
                                                  compare(
                                                      variable="Table1_Order",
                                                      value="waiting",
                                                      operator=operator.eq),
                                                  compare(
                                                      variable="Table2_Order",
                                                      value="waiting",
                                                      operator=operator.eq)
                                              ],subtrees=
                                              [check_if_table_1_sequence,check_if_table_2_sequence])


    #we will keep the above idiom running.
     waiting_for_order=py_trees.decorators.FailureIsRunning(
         name="keep this running till a table calls",
         child=go_to_which_table)

    #depending on the idiom one of the below sequence for order delivery is selected.   
     delivery_status_for_table_1_sequence=sequence(
         name="delivery_status_table: 1",memory=False)
     
     table_1_waiting_for_delivery=setBBVariable(
         name="delivery table 1:waiting",
         variable_name="Table1_Delivery",
         variable_value="waiting",
         overwrite=True)
     
     clear_table_1_order=setBBVariable(
         name="clear the chef calling variable",
         variable_name="chef_calling",
         variable_value=" ",
         overwrite=True)
     
     delivery_status_for_table_2_sequence= sequence(
         name="delivery_status_table: 2",
         memory=False)
     
     table_2_waiting_for_delivery= setBBVariable(
         name="delivery table 2:waiting",
         variable_name="Table2_Delivery",
         variable_value="waiting",
         overwrite=True)
     
     clear_table_2_order=setBBVariable(
         name="clear the chef calling variable",
         variable_name="chef_calling",
         variable_value=" ",
         overwrite=True)
        


    #we check which table is waiting for the delivery.
     delivery_for_which_table_=py_trees.idioms.either_or(
         name="chef_calling_for_which_table?",
         conditions=[
                    compare(variable="chef_calling",value="one",operator=operator.eq),
                    compare(variable="chef_calling",value="two",operator=operator.eq)
                    ],subtrees=[
                        delivery_status_for_table_1_sequence,
                        delivery_status_for_table_2_sequence])
     
     waiting_for_delivery = py_trees.decorators.FailureIsRunning(
         name="we will keep this idiom to track the chefs calls",
         child=delivery_for_which_table_)

    #we will begin the deliver action for table1 and then clear the status. 
     deliver_to_table1=sequence(
         name="deliver to table 1",
         memory=True)
    
     go_to_kitchen_order_1_action=pyactions(
     name="go_to_kitchen_to get_order_1",
     action_type=prototype_actions.GoTo,
     action_name="table_nav",
     action_goal=table2_location,
     generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_1.distance_left)
     )

     wait_for_food=pyactions(
     name="wait for food to be placed on the tray",
     action_type=prototype_actions.FoodMenu,
     action_name="Kitchen",
     action_goal=action_,
     generate_feedback_message=lambda msg: msg.feedback.waiting
)
     

     
     deliver_to_table1_action=pyactions(
        name="deliver_to_table_1_action",
        action_type=prototype_actions.GoTo,
        action_name="table_nav",
        action_goal=table1_location,
        generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_1.distance_left)
     )
      
     wait_for_food_to_be_picked_by_customer=pyactions(
     name="wait for food to be picked up by customer",
     action_type=prototype_actions.FoodMenu,
     action_name="Table",
     action_goal=action_,
     generate_feedback_message=lambda msg: msg.feedback.waiting
     )
     


     clear_table_1_delivery=setBBVariable(
         name="clear_delivery_for_table_ 1",
         variable_name="Table1_Delivery",
         variable_value="clear",
         overwrite=True
         )

     go_home_after_delivery_1=py_trees_ros.actions.ActionClient(
        name="go home after delivery for table 1",
        action_name='table_nav',
        action_type=prototype_actions.GoTo,
        action_goal=home,
        generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_1.distance_left)
          )
          

     #we will begin the deliver action for table 2 and then clear the status. 
     deliver_to_table2=sequence(
         name="deliver to table 2",
         memory=False
         )
     
     deliver_to_table2_action=pyactions(
        name="deliver_to_table_1_action",
        action_type=prototype_actions.GoTo,
        action_name="table_nav",
        action_goal=table1_location,
        generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_1.distance_left)
     )

     clear_table_2_delivery=setBBVariable(
         name="clear_delivery_for_table_2",
         variable_name="Table2_Delivery",
         variable_value="clear",
         overwrite=True
         )
     go_home_after_delivery_2=py_trees_ros.actions.ActionClient(
        name="go home after delivery for table 2",
        action_name='table_nav',
        action_type=prototype_actions.GoTo,
        action_goal=home,
        generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_1.distance_left)
          )
     
     
     #this idiom will select a table for delivery based on the delivery_table value
     go_to_which_table_for_delivery=py_trees.idioms.either_or(name="which table to deliver the order? ",
                                              conditions=[
                                                  compare(
                                                      variable="Table1_Delivery",
                                                      value="waiting",
                                                      operator=operator.eq),
                                                  compare(
                                                      variable="Table2_Delivery",
                                                      value="waiting",
                                                      operator=operator.eq)
                                              ],subtrees=
                                              [deliver_to_table1,
                                               deliver_to_table2])    
     
     #keep the above idiom running
     delivery_running = py_trees.decorators.FailureIsRunning(
         name="we will keep this idiom running to monitor the table number value",
         child=go_to_which_table_for_delivery) 
     
  


    

#we add to the root the sub to ai detected,kitchen table
# we add the ability to change table order and delivery status     
     root.add_children(
         [ai_detected_table_number_2BB,
          chef_calling,table_status,
          waiting_for_order,
          waiting_for_delivery,delivery_running])
#this sequence changes the table 1 status to waiting
#and clears the ai detected variable
     customer1_allocation_status_sequence.add_children(
         [customer_one_waiting,
          clear_ai_detected_table_number_1])
     
#this sequence changes the table 2 status to waiting
#and clears the ai detected variable
     
     customer2_allocation_status_sequence.add_children(
         [customer_two_waiting,
          clear_ai_detected_table_number_2])
     
#this sequence starts the navigation to table 1
     check_if_table_1_sequence.add_children(
         [go_to_table_1_action,
          clear_table_1,take_order_action])
     
#this sequence starts the navigation to table 2     
     check_if_table_2_sequence.add_children(
         [go_to_table_2_action,
          clear_table_2,take_order_action_2])
     
#we will change the delivery status of table 1
     delivery_status_for_table_1_sequence.add_children(
         [table_1_waiting_for_delivery,
          clear_table_1_order])
     
#we will change the delivery status of table 2   
     delivery_status_for_table_2_sequence.add_children(
         [table_2_waiting_for_delivery,
          clear_table_2_order])
     
#then we will start the process of delivery from kitchen to table     
     deliver_to_table1.add_children(
         [go_to_kitchen_order_1_action,
          wait_for_food,
          deliver_to_table1_action,
          wait_for_food_to_be_picked_by_customer,
          clear_table_1_delivery,
          go_home_after_delivery_1])
     
#then we will start the process of delivery from kitchen to table          
     deliver_to_table2.add_children(
         [deliver_to_table2_action,
          clear_table_2_delivery]
     )




     
     return root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="waiter", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

