import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from demo_interfaces.srv import ExecuteJob

from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat
from std_msgs.msg import String

import yaml
import enum
import os

# Using enum class create enumerations
class States(enum.Enum):
   IDLE = 0
   BUSY = 1
   FINISHED = 2
   ERROR = 3
   EMERGENCY =4

class ClientManager(Node):
    def __init__(self):
        super().__init__('client_manager')
        self.workcell_data = None
        self.machines = self.parse_machines()
        self.machine_states = self.create_states()
        self.steps = self.workcell_data['actions']

        self.create_subs()
        # self.create_clients()
        #### HEAD (CURRENT CHANGE)
        # self.executeJob_client = self.create_client(ExecuteJob, '/ot2_1/execute_job')
        # self.executeJob_client = self.create_client(ExecuteJob, 'ot2_1/execute_job') ## TODO 'ot2_1/execute_job
        self.executeJob_client = None
        self.emergency = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)

    def parse_machines(self):
        user_path = "/root/example_ws.yml"
        self.workcell_data = yaml.safe_load(open(user_path))
        machines= self.workcell_data['modules']
        names = []
        for m in machines:
            names.append([m['name'],m['type']])
        return names

    def create_states(self):
        dicts = {}
        for machine,type in self.machines:
            self.get_logger().info(machine)
            dicts[machine]="ERROR"
        return dicts

    def common_callback(self,msg):
        # self.get_logger().info('I heard: "%s"'%msg.data)
        machine = msg.header.src.split("/")[1]
        state = States(msg.state).name
        
        self.machine_states[machine] = state
        info = msg.message
        if self.machine_states[machine]=="ERROR" and state=="IDLE":
            self.get_logger().info(machine+" is now IDLE!")
            self.machine_states[machine]="IDLE"
        elif self.machine_states[machine]=="ERROR":
            self.get_logger().info(machine + info)
            self.machine_states[machine]="ERROR"

    def create_subs(self):
        for name,type in self.machines:
            
            #### HEAD (CURRENT CHANGE)
            # setattr(self,"sub"+name, self.create_subscription(Heartbeat,"/"+name+"/state",lambda msg:self.common_callback(msg),10))
            
            setattr(self,"sub"+name, self.create_subscription(Heartbeat, "/{}/action_client/heartbeat".format(name), lambda msg:self.common_callback(msg),10))

    # def create_clients(self):
    #     for name,type in self.machines:
    #         setattr(self,"executeJob_"+name, self.create_client(ExecuteJob, "/{}/execute_job".format(name))) 

    # def send_request(self,rc_path,pc_path, machine):
    #     self.get_logger().info("Sending request")
    #     req = ExecuteJob.Request()
    #     req.rc_path = rc_path
    #     req.pc_path = pc_path
        
    #     #### HEAD (CURRENT CHANGE)
    #     # req.simulate = sim
    #     # req.robot_ip = os.getenv('robot_ip')

    #     req.simulate = False
    #     if os.getenv('simulate') and os.getenv('simulate').lower()=='true':
    #         req.simulate = True
    #     # req.robot_ip = robot_ip
    #     if os.getenv('robot_ip'):
    #         req.robot_ip = os.getenv('robot_ip')

    #     self.future = self.executeJob_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()

    def send_request(self,module,command):
        req = ExecuteJob.Request()

        #how to change dictionary variable to string
        req.string = str(command)

        #how to call execute client based on needs
        self.executeJob_client = self.create_client(ExecuteJob, "/{}/execute_job".format(module))
        self.future = self.executeJob_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
    

    def emergency_callback(self,msg):
        if msg.is_emergency==True:
            self.get_logger().info("client_manager received an emergency alert: " + msg.message)

def main(args=None):
    rclpy.init(args=args)
    client_manager = ClientManager()
    steps_lens = len(client_manager.steps)
    current_step = 0
    working_flag = 0
    while True:
        if current_step>=steps_lens:
            break
        module = client_manager.steps[current_step]['module']
        command = client_manager.steps[current_step]['command']
        if (not working_flag) and client_manager.machine_states[module]=="IDLE":
            working_flag=1
            client_manager.send_request(module,command)
            if client_manager.machine_states[module]=="FINISHED":
                current_step+=1
                working_flag=0

        # if start_exec==1:
        #     if client_manager.machine_states[module]=="IDLE":
        #         client_manager.get_logger().info("Result from if")

        #         pc_path = client_manager.steps[0]['command']['args']['pc_path']
        #         rc_path = client_manager.steps[0]['command']['args']['rc_path']
        #         response = client_manager.send_request(rc_path=rc_path,pc_path=pc_path, machine = module)
        #         client_manager.get_logger().info("send request success: {}".format(response.success))
        #         client_manager.get_logger().info("----message: {}".format(response.error_msg))
        #         start_exec = 2
        # elif 
        rclpy.spin_once(client_manager,timeout_sec=0)
