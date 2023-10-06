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
from py_trees_ros_tutorials import behaviours
import operator
import py_trees_ros_interfaces.action as py_trees_actions
import prototype.action as prototype_actions


def create_root() -> py_trees.behaviour.Behaviour:
#creating the root that will have a parallel composite.
     root=py_trees.composites.Parallel(name="waiter",policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))

#assigning the blackboard variables values.
     blackboard=py_trees.blackboard.Blackboard()
     blackboard.set("ai_detected_table_number"," ")
     blackboard.set("customer_status_of_table_one","clear")
     blackboard.set("customer_status_of_table_two","clear")
     blackboard.set("ordered_food_status_of_table_one","notyet")
     blackboard.set("ordered_food_status_of_table_two","notyet")

#getting the message from the ai and storing it in the ai_detected_table_number variable.  
     ai_detected_table_number_2BB=py_trees_ros.subscribers.ToBlackboard(
         topic_name="table_number",
         topic_type=std_msgs.msg.String,
         blackboard_variables={'ai_detected_table_number':'data'},
         name='ai_detected_table_number_2BB',
         qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
     )

#changing the status of the table from clear to waiting depending on the ai_detected_table_number variable value.
     customer1_allocation_status_sequence=py_trees.composites.Sequence(name="customer1_allocation_status_sequence",memory=False)
     customer_one_waiting=py_trees.behaviours.SetBlackboardVariable(variable_name="customer_status_of_table_one",variable_value="waiting",overwrite=True,name="allocating_table_one")
     clear_ai_detected_table_number_1=py_trees.behaviours.SetBlackboardVariable(name="clear_ai_detected_table_number_1",variable_name="ai_detected_table_number",variable_value=" ",overwrite=True)

     customer2_allocation_status_sequence=py_trees.composites.Sequence(name="customer2_allocation_status_sequence",memory=False)
     customer_two_waiting=py_trees.behaviours.SetBlackboardVariable(variable_name="customer_status_of_table_two",variable_value="waiting",overwrite=True,name="allocating_table_two")
     clear_ai_detected_table_number_2=py_trees.behaviours.SetBlackboardVariable(name="clear_ai_detected_table_number_2",variable_name="ai_detected_table_number",variable_value=" ",overwrite=True)


     allocate_table=py_trees.idioms.either_or(name="allocate_ai_value_to_right_table",
                                              conditions=[
                                                  py_trees.common.ComparisonExpression(variable="ai_detected_table_number",value="one",operator=operator.eq),
                                                  py_trees.common.ComparisonExpression(variable="ai_detected_table_number",value="two",operator=operator.eq)
                                              ],
                                              subtrees=[customer1_allocation_status_sequence,customer2_allocation_status_sequence])
     table_status = py_trees.decorators.FailureIsRunning(name="table_status_appointment",child=allocate_table)


     
#after the table status has been changed ,we try to figure out which table needs waiting and try to go there.
     table_waiting_sequence=py_trees.composites.Sequence(name="table_waiting",memory=False)
     
#so as to keep the tree running we are keeping a failure is running decorator here.
     waiting_to_be_called=py_trees.decorators.FailureIsRunning(name="table_waiting_running",child=table_waiting_sequence)

     which_table_to_wait_selector=py_trees.composites.Selector(name="check_table_status",memory=False)
     
#is it table 1 ?if so then go to table 1 and then change the status of table 1 to clear from waiting.
     check_if_table_1_sequence=py_trees.composites.Sequence(name="check_if_table_one_sequence",memory=False)
     check_if_table_1_condition = py_trees.behaviours.CheckBlackboardVariableValue(name="Check_table_1?",
       check=py_trees.common.ComparisonExpression(variable="customer_status_of_table_one", value="waiting", operator=operator.eq),
     )

#locking in the coordinates for table1.
     table1_location=prototype_actions.GoTo.Goal()
     table1_location.x=1.0
     table1_location.y=0.0
     table1_location.z=0.0 
     feedback_1=prototype_actions.GoTo.Feedback()  
       
#calling the action client to send the coordinates for table1.
     go_to_table_1_action=py_trees_ros.actions.ActionClient(
        name="go_to_table_1",
        action_type=prototype_actions.GoTo,
        action_name="table_nav",
        action_goal=table1_location,
        generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_1.distance_left)
     )

     clear_table_1=py_trees.behaviours.SetBlackboardVariable(name="clear_table_1",variable_name="customer_status_of_table_one",variable_value="clear",overwrite=True)

     

#is it table 2 ? if so then go to table 2 and then change the status of table 2 to clear from waiting.
     check_if_table_2_sequence=py_trees.composites.Sequence(name="check_if_table_two_sequence",memory=False)

     check_if_table_2_condition= py_trees.behaviours.CheckBlackboardVariableValue(name="Check_table_2?",
       check=py_trees.common.ComparisonExpression(variable="customer_status_of_table_two", value="waiting", operator=operator.eq),
     )
#locking in the coordinates for table1.
     table2_location=prototype_actions.GoTo.Goal()
     table2_location.x=1.0
     table2_location.y=1.0
     table2_location.z=1.0 
     feedback_2=prototype_actions.GoTo.Feedback()       
#calling the action client to send the co_ordinates for table 2.
     go_to_table_2_action=py_trees_ros.actions.ActionClient(
         name="go_to_table_2",
        action_type=prototype_actions.GoTo,
        action_name="table_nav",
        action_goal=table2_location,
        generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_2.distance_left)
     )

     clear_table_2=py_trees.behaviours.SetBlackboardVariable(name="clear_table_2",variable_name="customer_status_of_table_two",variable_value="clear",overwrite=True)

#once we reached the corresponding table.we try to take the order from the customer.
     take_order_sequence=py_trees.composites.Sequence(name="take_order",memory=False)
     take_order_action=py_trees_ros.actions.ActionClient(
         name="take_order",
         action_type=py_trees_actions.Dock,
         action_name="dock",
         action_goal=py_trees_actions.Dock.Goal(),
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
     )
#after taking the order we send it to the kitchen.
     kitchen_location=prototype_actions.GoTo.Goal()
     kitchen_location.x=1.0
     kitchen_location.y=1.0
     kitchen_location.z=1.0 
     feedback_k=prototype_actions.GoTo.Feedback()  


     send_order_kitchen_action=py_trees_ros.actions.ActionClient(
         name="send_order_kitchen",
         action_type=prototype_actions.GoTo,
         action_name="table_nav",
         action_goal=kitchen_location,
         generate_feedback_message=lambda msg: "{:.2f}%%".format(feedback_k.distance_left)
     )
#exit the table waiting sequence.

#this is the order fetching idiom from the kitchen.
#we use the idiom to check if any order is ready and then go to kitchen and fetch that order.

     order_fetch_delivery_task=py_trees.composites.Selector(name="order_fetch_delivery_task",memory=False)

     order_fetch_delivery_idiom_running = py_trees.behaviours.Running(name="table_status_running")

     table_1_order_ready_sequence=py_trees.composites.Sequence(name="table_1_order_sequence",memory=False)

     table_2_order_ready_sequence=py_trees.composites.Sequence(name="table_2_order_sequence",memory=False)

     order_fetch_delivery_idiom=py_trees.idioms.either_or(name="order_fetch_deliver_idiom",
                                                          conditions=[
                                                              py_trees.common.ComparisonExpression(variable="table_one_order",value="ready",operator=operator.eq),
                                                              py_trees.common.ComparisonExpression(variable="table_two_order",value="ready",operator=operator.eq)
                                                          ],
                                                          subtrees=[table_1_order_ready_sequence,table_2_order_ready_sequence])
#this sub tree is for table 1.
     fetch_table_one_order=py_trees_ros.actions.ActionClient(
         name="fetch_table_one_order",
         action_type=py_trees_actions.Dock,
         action_name="dock",
         action_goal=py_trees_actions.Dock.Goal(),
         generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
     )

     reset_table_1_order_status=py_trees.behaviours.SetBlackboardVariable(
         name="reset_table_1_order_status",
         variable_name="table_one_order",
         variable_value="notyet",
         overwrite=True
     )

     deliver_order_4_table_1=py_trees_ros.actions.ActionClient(
         name="deliver_order_4_table_1",
         action_type=py_trees_actions.Dock,
         action_name="dock",
         action_goal=py_trees_actions.Dock.Goal(),
         generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)         
     )

      
# this sub tree is for table 2.
     fetch_table_two_order=py_trees_ros.actions.ActionClient(
         name="fetch_table_two_order",
         action_type=py_trees_actions.Dock,
         action_name="dock",
         action_goal=py_trees_actions.Dock.Goal(),
         generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
     )


     reset_table_2_order_status=py_trees.behaviours.SetBlackboardVariable(
         name="reset_table_two_order_status",
         variable_name="table_two_order",
         variable_value="notyet",
         overwrite=True
     )



     deliver_order_4_table_2_=py_trees_ros.actions.ActionClient(
         name="deliver_order_4_table_2",
         action_type=py_trees_actions.Dock,
         action_name="dock",
         action_goal=py_trees_actions.Dock.Goal(),
         generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)


     )


#make rosie check for any pending works and if not then send her home.

     anything_pending=py_trees.behaviours.CheckBlackboardVariableValues(name="pending?",checks=[
        py_trees.common.ComparisonExpression("table_one","waiting",operator.eq),
        py_trees.common.ComparisonExpression("table_two","waiting",operator.eq),
        py_trees.common.ComparisonExpression("table_one_order","ready",operator.eq),
        py_trees.common.ComparisonExpression("table_two_order","ready",operator.eq)],operator=operator.eq),
     

     
     
     go_home=py_trees_ros.actions.ActionClient(
         name="go_home",
         action_name="dock",
         action_type=py_trees_actions.Dock,
         action_goal=py_trees_actions.Dock.Goal(),
         generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
     )
    
    

    
     root.add_child(ai_detected_table_number_2BB)
     root.add_child(table_status)
     root.add_child(waiting_to_be_called)

     customer1_allocation_status_sequence.add_children([customer_one_waiting,clear_ai_detected_table_number_1])
     customer2_allocation_status_sequence.add_children([customer_two_waiting,clear_ai_detected_table_number_2])

    
     table_waiting_sequence.add_child(which_table_to_wait_selector)

     which_table_to_wait_selector.add_child(check_if_table_1_sequence)
     which_table_to_wait_selector.add_child(check_if_table_2_sequence)

     check_if_table_1_sequence.add_child(check_if_table_1_condition)
    # check_if_table_1_sequence.add_child(clear_ai_detected_table_number)
    # check_if_table_1_sequence.add_child(clear_table_1)
     check_if_table_1_sequence.add_child(go_to_table_1_action)
     check_if_table_1_sequence.add_child(clear_table_1)

     
     check_if_table_2_sequence.add_child(check_if_table_2_condition)
    # check_if_table_2_sequence.add_child(clear_table_2)
     check_if_table_2_sequence.add_child(go_to_table_2_action)
    # table_waiting_sequence.add_child(clear_table_1)
     table_waiting_sequence.add_child(clear_table_2)
     #table_waiting_sequence.add_child(take_order_sequence)

     take_order_sequence.add_child(take_order_action)
     take_order_sequence.add_child(send_order_kitchen_action)

     #root.add_child(order_fetch_delivery_task)
     #order_fetch_delivery_task.add_child(order_fetch_delivery_idiom)
     #order_fetch_delivery_task.add_child(order_fetch_delivery_idiom_running)


     
     #table_1_order_ready_sequence.add_child(fetch_table_one_order)
     #table_1_order_ready_sequence.add_child(reset_table_1_order_status)
     #table_1_order_ready_sequence.add_child(deliver_order_4_table_1)

     #table_2_order_ready_sequence.add_child(fetch_table_two_order)
     #table_2_order_ready_sequence.add_child(reset_table_2_order_status)
     #table_2_order_ready_sequence.add_child(deliver_order_4_table_2_)



     #root.add_child(check_blackboard_variable)
     #root.add_child(go_to_table_action)
     
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

