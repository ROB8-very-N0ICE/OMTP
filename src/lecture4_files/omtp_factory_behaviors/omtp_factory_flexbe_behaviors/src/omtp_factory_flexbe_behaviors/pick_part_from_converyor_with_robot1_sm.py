#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.moveit_to_joints_dyn_state import MoveitToJointsDynState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from omtp_factory_flexbe_states.compute_grasp_state import ComputeGraspState
from omtp_factory_flexbe_states.detect_part_camera_state import DetectPartCameraState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Apr 27 2022
@author: Reshad Zadran
'''
class Pickpartfromconveryorwithrobot1SM(Behavior):
	'''
	This behavior will make robot1 pick a part from the conveyor in our factory
	'''


	def __init__(self):
		super(Pickpartfromconveryorwithrobot1SM, self).__init__()
		self.name = 'Pick part from converyor with robot1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		panda1 = panda_arm_1
		hand1 = panda_hand_1
		panda1_home = [1.57, -1.57, 1.24, -1.57, -1.57, 0]
		panda1_joints = ['panda_1_joint1', 'panda_1_joint2', 'panda_1_joint3', 'panda_1_joint4', 'panda_1_joint5', 'panda_1_joint6', 'panda_1_joint7', 'panda_1_joint8']
		hand1_joints = ['panda_1_finger_joint1']
		hand_open = [0.04]
		hand_close = [0]
		# x:7 y:607, x:0 y:267
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = [ ]
		_state_machine.userdata.pick_config = panda1_home
		_state_machine.userdata.joint_names = [ ]
		_state_machine.userdata.tool_open = hand_open
		_state_machine.userdata.tool_close = hand_close
		_state_machine.userdata.home1 = panda1_home

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:335 y:29
			OperatableStateMachine.add('home',
										MoveitToJointsState(move_group=panda1, joint_names=panda1_joints, action_topic='/move_group'),
										transitions={'reached': 'tool_open', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'home1'})

			# x:337 y:264
			OperatableStateMachine.add('Detect part',
										DetectPartCameraState(ref_frame='panda_1_link0', camera_topic='/omtp/my_logical_camera1', camera_frame='logical_camera1_frame'),
										transitions={'continue': 'Compute grasp configuration', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'object_pose'})

			# x:339 y:494
			OperatableStateMachine.add('panda1_to_pick',
										MoveitToJointsDynState(move_group=panda1, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_config', 'joint_names': 'joint_names'})

			# x:336 y:144
			OperatableStateMachine.add('tool_open',
										MoveitToJointsState(move_group=hand1, joint_names=hand1_joints, action_topic='/move_group'),
										transitions={'reached': 'Detect part', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'tool_open'})

			# x:330 y:375
			OperatableStateMachine.add('Compute grasp configuration',
										ComputeGraspState(group=panda1, offset=0.25, joint_names=['panda_1_joint1', 'panda_1_joint2', 'panda_1_joint3', 'panda_1_joint4', 'panda_1_joint5', 'panda_1_joint6', 'panda_1_joint7', 'panda_1_joint8'], tool_link=panda_1_finger_joint1, rotation=3.1415),
										transitions={'continue': 'panda1_to_pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'object_pose', 'joint_values': 'pick_config', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
