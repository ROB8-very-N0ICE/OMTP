#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from omtp_factory_flexbe_states.compute_grasp_state import ComputeGraspState
from omtp_factory_flexbe_states.detect_part_camera_state import DetectPartCameraState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 28 2022
@author: fk me
'''
class fkmeSM(Behavior):
	'''
	fk me
	'''


	def __init__(self):
		super(fkmeSM, self).__init__()
		self.name = 'fk me'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		panda1 = 'panda_arm_1'
		home1 = [0, 0, 0, -1.57, 0, 1.56, 0]
		panda1_joints = ['panda_1_joint1', 'panda_1_joint2', 'panda_1_joint3', 'panda_1_joint4', 'panda_1_joint5', 'panda_1_joint6', 'panda_1_joint7']
		hand1 = 'panda_hand_1'
		hand1_joints = ['panda_1_finger_joint1', 'panda_1_finger_joint2']
		hand_close = [0, 0]
		hand_open = [0.4, 0.4]
		home2 = [0, 0.29, 0, -2.57, 0, 3.16, -0.75]
		# x:701 y:347, x:0 y:421
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goRdy1 = home1
		_state_machine.userdata.part_pose = [ ]
		_state_machine.userdata.joint_names = [ ]
		_state_machine.userdata.tool_open = hand_open
		_state_machine.userdata.tool_close = hand_close
		_state_machine.userdata.pick_confi = home2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:93 y:28
			OperatableStateMachine.add('rdy',
										MoveitToJointsState(move_group=panda1, joint_names=panda1_joints, action_topic='/move_group'),
										transitions={'reached': 'tool_rdy', 'planning_failed': 'failed', 'control_failed': 'tool_rdy'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'goRdy1'})

			# x:180 y:284
			OperatableStateMachine.add('grasp_rdy',
										MoveitToJointsState(move_group=panda1, joint_names=panda1_joints, action_topic='/move_group'),
										transitions={'reached': 'tool_close', 'planning_failed': 'failed', 'control_failed': 'tool_close'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'pick_confi'})

			# x:342 y:537
			OperatableStateMachine.add('home',
										MoveitToJointsState(move_group=panda1, joint_names=panda1_joints, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'finished'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'goRdy1'})

			# x:93 y:670
			OperatableStateMachine.add('rdy_grasp',
										ComputeGraspState(group=panda1, offset=0.25, joint_names=panda1_joints, tool_link='panda_1_hand', rotation=3.1415),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'pick_confi', 'joint_names': 'joint_names'})

			# x:226 y:417
			OperatableStateMachine.add('tool_close',
										MoveitToJointsState(move_group=hand1, joint_names=hand1_joints, action_topic='/move_group'),
										transitions={'reached': 'home', 'planning_failed': 'failed', 'control_failed': 'home'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'tool_close'})

			# x:135 y:150
			OperatableStateMachine.add('tool_rdy',
										MoveitToJointsState(move_group=hand1, joint_names=hand1_joints, action_topic='/move_group'),
										transitions={'reached': 'grasp_rdy', 'planning_failed': 'failed', 'control_failed': 'grasp_rdy'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'tool_open'})

			# x:161 y:585
			OperatableStateMachine.add('detect_object',
										DetectPartCameraState(ref_frame='panda_1_link0', camera_topic='/omtp/my_logical_camera1', camera_frame='logical_camera1_frame'),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
