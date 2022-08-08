import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml

from demo_interfaces.action import OT2Job
from demo_interfaces.msg import OT2Job as OT2Jobmsg
from demo_interfaces.msg import JobHeader
from demo_interfaces.srv import ExecuteJob
from std_msgs.msg import String


class DemoActionClient(Node):
    """

    """
    def __init__(self):
        """
        
        """
        super().__init__('demo_action_client')
        self._action_client = ActionClient(self, OT2Job, 'OT2')
        # self.heartbeat_publisher = self.create_publisher()

        self.machines = self.parse_machines()
        self.machine_states = self.create_states()
        self.create_subs()

        self.srv = self.create_service(ExecuteJob,'execute_job',self.exectute_job_callback)

        self.get_logger().info("OT2 Action Client running!")
        self.get_logger().info("Send rc_path and pc_path with /execute_job service call")
        
    def parse_machines(self):
        user_path = "/root/example_ws.yml"
        load_data = yaml.safe_load(open(user_path))
        machines= load_data['modules']
        names = []
        for m in machines:
            names.append(m['name'])
        return names

    def create_states(self):
        dicts = {}
        for machine in self.machines:
            self.get_logger().info(machine)
            dicts[machine]="ERROR"
        return dicts

    def create_subs(self):
        for name in self.machines:
            setattr(self,"sub"+name, self.create_subscription(String,"/"+name+"/status",lambda msg:self.common_callback(msg),10))
        
    def common_callback(self,msg):
        # self.get_logger().info('I heard: "%s"'%msg.data)
        machine,states = msg.data.split(" ")
        if self.machine_states[machine]=="ERROR" and states=="IDLE":
            self.get_logger().info(machine+" is now IDLE!")
            self.machine_states[machine]="IDLE"
    
    def exectute_job_callback(self,request,response):
        """

        """
        rc_config = None
        pc_config = None
        try:
            rc_config = yaml.dump(yaml.safe_load(open(request.rc_path)))
        except IOError:
            response.error_msg = "No such file or directory:" + request.rc_path
            response.success = False
            return response
            
        try:
            pc_config = yaml.dump(yaml.safe_load(open(request.pc_path)))
        except IOError:
            response.error_msg = "No such file or directory:" + request.pc_path
            response.success = False
            return response

        response.success = True
        self.send_goal(protocol_config=pc_config,robot_config=rc_config)

        return response


    def send_goal(self, pc_path=None, rc_path=None, protocol_config="", robot_config=""):
        """

        """
        goal_msg = OT2Job.Goal()
        if rc_path is not None:
            goal_msg.job.rc_path = rc_path
        if pc_path is not None:
            goal_msg.job.pc_path = pc_path

        goal_msg.job.robot_config = robot_config
        goal_msg.job.protocol_config = protocol_config
        goal_msg.job.simulate = True
        
        self._action_client.wait_for_server()

        print("Sending goal to action server ...")

        self.goal_future = self._action_client.send_goal_async(goal_msg) #, feedback_callback=self.goal_feedback_callback)
        self.goal_future.add_done_callback(self.goal_respond_callback)
        


    def goal_feedback_callback(self,feed):
        pass
        # feedback_msg = feed.feedback
        # self.get_logger().info("Progress: {:.2f}".format(feedback_msg.total_percentage_progress))


    def goal_respond_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result from action server:")
        self.get_logger().info("--success: {}".format(result.success))
        self.get_logger().info("--message: {}".format(result.error_msg))
        # if not result.success:
        #     self.get_logger().info("Error Message: " + result.error_msg)
        # rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    demo_action_client = DemoActionClient()
    #future = demo_action_client.send_goal()
    rclpy.spin(demo_action_client)
    

if __name__ == '__main__':
    main()