import nengo
import numpy as np
import motionPrimitives

leg_0_AD_joints = [['/robot_leg0_alpha_joint_pos_cntr/command'],['/robot_leg0_delta_joint_pos_cntr/command']]
leg_4_AD_joints = [['/robot_leg4_alpha_joint_pos_cntr/command'],['/robot_leg4_delta_joint_pos_cntr/command']]

leg_0_BG_joints = [['/robot_leg0_beta_joint_pos_cntr/command'],['/robot_leg0_gamma_joint_pos_cntr/command']]
leg_4_BG_joints = [['/robot_leg4_beta_joint_pos_cntr/command'],['/robot_leg4_gamma_joint_pos_cntr/command']]



def sinus(t):
    return 0.5+(np.sin(t * np.pi/2))/2

swing = motionPrimitives.MotionPrimitives(0, sinus, leg_0_AD_joints, leg_4_AD_joints)
liftleg = motionPrimitives.MotionPrimitives(1, sinus, leg_0_BG_joints,leg_4_BG_joints)


model = nengo.Network()
with model:
    net_swing = swing.get_network()
    net_liftleg = liftleg.get_network()