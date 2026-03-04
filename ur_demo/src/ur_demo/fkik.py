import subprocess
from trac_ik_python.trac_ik import IK
from trac_ik_python.trac_ik_wrap import TRAC_IK

import numpy as np
from scipy.spatial.transform import Rotation as R


'''
Made using trac_ik_lib and trac_ik_python

'''




class FKIK:
      def __init__(self):

            xacro_path = "/home/rcnl/Documents/ROS_Workspace/graspdetect_ws/src/universal_robot/ur_description/urdf/ur10.urdf"
            urdf_xml = subprocess.check_output(['xacro', xacro_path])
            urdf = urdf_xml.decode('utf-8')
            self.solver = TRAC_IK(
                  'iiwa_link_0',
                  'end_effector',
                  urdf,
                  0.005,
                  1e-5, 
                  "Speed"
            )
            self.num_jts = self.solver.getNrOfJointsInChain()
            self.joint_names = self.solver.getJointNamesInChain(urdf)
            self.link_names = self.solver.getLinkNamesInChain()

      def get_ik(self, 
                  qinit,
                  pos,
                  quat,
                  bx=1e-5, by=1e-5, bz=1e-5,
                  brx=1e-3, bry=1e-3, brz=1e-3):
            if len(qinit) > self.num_jts:
                  n = len(qinit)- self.num_jts
                  for i in range(n):
                        qinit.pop()
            assert len(qinit) == self.num_jts, f"qinit has {len(qinit)} jts but it should have {self.num_jts}"
            solution = self.solver.CartToJnt(
                  qinit, pos[0], pos[1], pos[2],
                  quat[0], quat[1], quat[2], quat[3],
                  bx, by, bz, 
                  brx, bry, brz
            )
            if solution:
                  return solution
            else:
                  print("solution not found!")
                  return qinit
            
      def get_jt_limits(self):
            lb = self.solver.getLowerBoundLimits()
            ub = self.solver.getUpperBoundLimits()
            return lb, ub
      
      def get_fk(self, joints):
            return self.solver.JntToCart(joint_positions=joints)

if __name__ == '__main__':
      fkik = FKIK()