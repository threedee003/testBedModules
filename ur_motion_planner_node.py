import rospy
from atls_msgs_srvs.srv import ExecuteURJointPos, ExecuteURJointPosResponse
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import subprocess
import shlex

import socket

ROBOT_HOST = "192.168.8.20"
ROBOT_PORT = 30003

 

# # Subscribe to a joint_states
curr_jt=JointState()
def jt_callback(msg):
    curr_jt.position=msg.position
    
js_sub = rospy.Subscriber('/joint_states', JointState, jt_callback)
print("Subscriber and publisher created\n")
rospy.loginfo("Subscriber and publisher created\n")
def rad2deg(rad):
    return (180/np.pi)*rad

def deg2rad(deg):
    return (np.pi/180)*deg


def exec_jt_pos(req):
    print("Service call has been accepted")
    move_type=req.movement_type.data
    final_jts= np.array([req.target_jt.data[0], req.target_jt.data[1], req.target_jt.data[2], req.target_jt.data[3], req.target_jt.data[4], 
                        req.target_jt.data[5] ])
    exec_state = Bool
    exec_state.data=False
    vel=deg2rad(req.vel_in_deg.data)
    acc=deg2rad(req.acc_in_deg.data)
    print("Commanded Jt: ", rad2deg(final_jts))
    cmd= move_type + "([" + str(final_jts[0]) + "," + str(final_jts[1]) + "," + str(final_jts[2]) \
    + "," + str(final_jts[3]) + "," + str(final_jts[4])  +  "," + str(final_jts[5]) + "]" +  ","  \
    + "a=" + str(acc) +  "," + "v=" + str(vel) + ")" + "\n"
    
    print(cmd)
    s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ROBOT_HOST,ROBOT_PORT) )
    
    s.send(cmd.encode("utf-8"))
    data=s.recv(1024)
    

    
    r=rospy.Rate(125)
    while not rospy.is_shutdown():        
        # rospy.spin()
    
        curr_jp_=curr_jt.position
    
        curr_jp_np=np.array([curr_jp_[2],curr_jp_[1],curr_jp_[0],curr_jp_[3],curr_jp_[4],curr_jp_[5]])
    
        # rospy.loginfo("curr jp: ", curr_jp_np)
        #print("HI::::::       ", np.linalg.norm(curr_jp_np-final_jts))
        if(np.linalg.norm(curr_jp_np-final_jts)<0.005):
            exec_state.data=True
            break
        r.sleep()
    # print("closing socket")
    s.close()
    print("socket closed")
    return ExecuteURJointPosResponse(exec_state)

def execute_ur_joint_pos_server():

    rospy.init_node('execute_ur_joint_pos_server')
    s = rospy.Service('ur_motion_planner/execute_ur_joint_pos_service', ExecuteURJointPos, exec_jt_pos)
    print("Ready to Execute desired Joint Position...")
    rospy.spin()

if __name__ == "__main__":
    try: 
        execute_ur_joint_pos_server()
    except rospy.ROSInterruptException: 
        pass

