"""

"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from demo_interfaces.srv import ExecuteJob
from demo_interfaces.srv import StartJob

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
        """
        Subscribe to emergency alerts, repond to service call from visual_tool, and send out requests to corresponding
        action_client.
        """
        super().__init__('client_manager')
        self.workcell_data = None
        self.machines = self.parse_machines()
        self.machine_states = self.create_states()
        self.steps = self.workcell_data['actions']

        self.create_subs()

        self.execute_job_service = self.create_service(StartJob,'/execute_job',self.exectute_job_callback)
        self.startJob_client = None
        self.emergency = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)

        self.start_flag = False

    def parse_machines(self):
        """
        Get machine name and type information based on input workflow file
        """
        user_path = "/root/example_ws.yml"
        self.workcell_data = yaml.safe_load(open(user_path))
        machines= self.workcell_data['modules']
        names = []
        for m in machines:
            names.append([m['name'],m['type']])
        return names

    def create_states(self):
        """
        Create dictionary to keep track of the states of all machines
        """
        dicts = {}
        for machine,type in self.machines:
            self.get_logger().info(machine)
            dicts[machine]="ERROR"
        return dicts

    def common_callback(self,msg):
        """
        Update machine states based on the information from the action client heartbeat
        """

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
        """
        Create subscriptions for all the hearbeat message using a common callback function
        """
        for name,type in self.machines:
                       
            setattr(self,"sub"+name, self.create_subscription(Heartbeat, "/{}/action_client/heartbeat".format(name), lambda msg:self.common_callback(msg),10))


    def send_request(self,module,command):
        """
        Call the execute_job service to send out different commands to different module(machines) 
        """
        req = StartJob.Request()

        #how to change dictionary variable to string
        req.command = yaml.dump(command)

        #how to call execute client based on needs
        self.startJob_client = self.create_client(StartJob, "/{}/execute_job2".format(module))
        self.future = self.startJob_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        if not self.future.result().success:
            self.get_logger().info("%s"%self.future.result().error_msg)
            return False
        return True

    def exectute_job_callback(self,request,response):
        """
        Responding to the service call from gui, and set start flag to trigger the start of execution
        """
        self.get_logger().info("receive request from user:"+request.command)
        if request.command=="start":

            if self.start_flag==True:
                response.error_msg = "workflow is running!"
                response.success=False
            else:
                self.start_flag=True
                response.success=True
            return response

    def emergency_callback(self,msg):
        """
        Deal with the emergency situation, now only showing up info on terminals 
        """
        if msg.is_emergency==True:
            self.get_logger().info("client_manager received an emergency alert: " + msg.message)

def main(args=None):
    rclpy.init(args=args)
    client_manager = ClientManager()
    steps_lens = len(client_manager.steps)
    current_step = 0
    working_flag = 1
    try:
        while True:
            #only execute once 
            if current_step>=steps_lens:
                break

            module = client_manager.steps[current_step]['module']
            command = client_manager.steps[current_step]['command']

            if client_manager.start_flag and working_flag and client_manager.machine_states[module]=="IDLE":
                working_flag=0
                client_manager.send_request(module,command)
            if client_manager.machine_states[module]=="FINISHED":
                current_step+=1
                working_flag=1

            rclpy.spin_once(client_manager,timeout_sec=0)
    
    except KeyboardInterrupt:
        pass
