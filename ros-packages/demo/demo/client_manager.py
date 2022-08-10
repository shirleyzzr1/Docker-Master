import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from demo_interfaces.srv import ExecuteJob
from std_msgs.msg import String

import yaml

class ClientManager(Node):
    def __init__(self):
        super().__init__('client_manager')
        self.workcell_data = None
        self.machines = self.parse_machines()
        self.machine_states = self.create_states()

        self.create_subs()

        self.steps = self.workcell_data['actions']
        self.executeJob_client = self.create_client(ExecuteJob, 'ot2_1/execute_job')

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
        data = msg.data.split("/")
        machine = data[0]
        states = data[1]
        info = data[2]
        if self.machine_states[machine]=="ERROR" and states=="IDLE":
            self.get_logger().info(machine+" is now IDLE!")
            self.machine_states[machine]="IDLE"
        elif self.machine_states[machine]=="ERROR":
            self.get_logger().info(machine + info)
            self.machine_states[machine]="ERROR"

    def create_subs(self):
        for name,type in self.machines:
            setattr(self,"sub"+name, self.create_subscription(String,"/"+name+"/status",lambda msg:self.common_callback(msg),10))

    def send_request(self,rc_path,pc_path,machine):
        self.get_logger().info("Sending request")
        req = ExecuteJob.Request()
        req.rc_path = rc_path
        req.pc_path = pc_path
        self.future = self.executeJob_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    client_manager = ClientManager()
    module = client_manager.steps[0]['module']
    start_exec = 1
    while True:
        if start_exec==1:
            if client_manager.machine_states[module]=="IDLE":
                client_manager.get_logger().info("Result from if")
                pc_path = client_manager.steps[0]['command']['args']['pc_path']
                rc_path = client_manager.steps[0]['command']['args']['rc_path']
                response = client_manager.send_request(rc_path=rc_path,pc_path=pc_path,machine = module)
                start_exec = 2
        rclpy.spin_once(client_manager,timeout_sec=0)