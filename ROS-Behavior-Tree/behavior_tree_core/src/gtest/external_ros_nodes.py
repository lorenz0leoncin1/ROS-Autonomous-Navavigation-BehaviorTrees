#!/usr/bin/env python3

import rospy
from py_trees import BehaviourTree, Sequence, Selector, Blackboard
from py_trees.common import Status
from py_trees.composites import Sequence, Selector
from py_trees.decorators import Inverter
from py_trees.trees import BehaviourTree

class ROSAction:
    def __init__(self, name):
        self.name = name

    def tick(self):
        # Sostituisci con la logica dell'azione ROS
        rospy.loginfo(f"Executing ROS action: {self.name}")
        return Status.SUCCESS

class ROSCondition:
    def __init__(self, name):
        self.name = name

    def tick(self):
        # Sostituisci con la logica della condizione ROS
        rospy.loginfo(f"Evaluating ROS condition: {self.name}")
        return Status.SUCCESS

if __name__ == '__main__':
    rospy.init_node('BehaviorTree')

    try:
        tick_period_milliseconds = 1000

        # Creazione dei nodi
        action = ROSAction("action")
        condition = ROSCondition("condition")

        # Costruzione del Behavior Tree
        root = Selector("Root")
        sequence1 = Sequence("seq1")
        decorator = Inverter("decorator")

        # Aggiunta dei nodi al Behavior Tree
        root.add_children([sequence1, decorator])
        sequence1.add_children([condition, action])
        decorator.add_child(condition)

        # Creazione dell'oggetto Behavior Tree
        tree = BehaviourTree(root)

        # Esecuzione del Behavior Tree
        tree.setup(Blackboard(), tick_period_milliseconds)
        while not rospy.is_shutdown():
            tree.tick_root()
            rospy.sleep(tick_period_milliseconds / 1000.0)

    except Exception as e:
        rospy.logerr(f"Exception: {str(e)}")

    rospy.loginfo("Behavior Tree finished")
