import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import yaml
import enum
import os
import time

from demo_interfaces.action import OT2Job

from demo_interfaces.msg import OT2Job as OT2Jobmsg
from demo_interfaces.msg import JobHeader
from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat

from demo_interfaces.srv import ExecuteJob
from demo_interfaces.srv import StartJob


from std_msgs.msg import String

# Using enum class create enumerations
class States(enum.Enum):
   IDLE = 0
   BUSY = 1
   FINISHED = 2
   ERROR = 3
   EMERGENCY =4

state = States.IDLE

class DemoActionClient(Node):
    """

    """
    def __init__(self):
        """
        Subscribe to emergency alerts
        Create client to report emergencies
        Publish Heartbeat on a timer
        Create OT2 action clinet to send OT2 goals and recieve progress feedback and results
        TODO Not sending goals during emergency
        """

        #### HEAD (CURRENT CHANGE)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        super().__init__('demo_action_client')

        ## Action client to be namespaced with action server
        self._action_client = ActionClient(self, OT2Job, 'OT2')

        ## TODO replace self.name where references are made and delete
        # self.name = self.get_namespace()[1:]
        self.action_server_name = "demo_action_server"
        
        ## Set up Emergency tracking and service proxy (client)
        self.emergency_sub = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)
        self.emergency_flag = False ## TODO Maybe should default to True
        # self.emergency_client = self.create_client(RaiseEmergency, "/raise_emergency")

        ## Set up Heartbeat publishing on timer
        heartbeat_timer_period = 0.5  # seconds
        self.heartbeat_timer = self.create_timer(heartbeat_timer_period, self.heartbeat_timer_callback)
        # self.heartbeat_publisher = self.create_publisher(Heartbeat, 'state', 10) ## TODO make "namespace/self.get_name()/heartbeat" the standard standard
        self.heartbeat_publisher = self.create_publisher(Heartbeat, '{}/heartbeat'.format(self.get_name()), 10) 
        self.heartbeat_msg = Heartbeat()
        self.heartbeat_msg.header.src = self.get_fully_qualified_name()
        self._heartbeat_state = Heartbeat.IDLE ## TODO maybe should default to BUSY or ERROR
        self._heartbeat_info = ""

        ## TODO: Should subscribe to ActionServer node's state, track the heartbeat state of AS
        ##          And only send goal when AS is IDLE
        self.server_heartbeat_sub = self.create_subscription(Heartbeat, 'demo_action_server/heartbeat', self.server_heartbeat_callback, 10)
        self.server_heartbeat_flag = Heartbeat.IDLE ## TODO INnitiated state?
        
        ## Job service to trigger actions
        self.execute_job_service = self.create_service(StartJob,'execute_job',self.exectute_job_callback)
        
        ## Alert that the Action Server has been created
        self.get_logger().info("OT2 Action Client running!")
        self.get_logger().info("Send rc_path and pc_path with /execute_job service call")

    def get_fully_qualified_name(self) -> str:
        return "{}/{}".format(self.get_namespace(), self.get_name())
    
    def heartbeat_timer_callback(self):
        """
        Update the heartbeat's header timestamp and heartbeat state
        and publish
        """

        # self.update_state()
        # self.get_logger().info('Publishing: "%s"' % msg.data
        # self.heartbeat_msg.state = state.value ## TODO Revise bcos Heartbeat state enumeration already encoded in the the Heartbeat Msg
        self.heartbeat_msg.state = self._heartbeat_state 
        self.heartbeat_msg.message = self._heartbeat_info
        self.heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        self.heartbeat_publisher.publish(self.heartbeat_msg)
    
    def server_heartbeat_callback(self, msg):
        """
        Update privately tracked state of ActionServer 
        TODO: Should probably pay attention to the heartbeat timestamps
        """
        if self.server_heartbeat_flag != msg.state:
            self.server_heartbeat_flag = msg.state


    def emergency_callback(self,msg):
        """
        Update private emergency status 
        """
        # if msg.message!="":
            # self.get_logger().info(self.name + " client received an emergency alert: " + msg.message)
        
        if msg.is_emergency and not self.emergency_flag:
            self.emergency_flag = True
            self.get_logger().warn("{} action received an emergency alert: {}".format(self.get_fully_qualified_name(),msg.message)) 
            ## TODO: Should include the source of error in the warn

        if not msg.is_emergency and self.emergency_flag:
            self.emergency_flag = False
            self.get_logger().info("Emergency alert(s) cleared.")


    def exectute_job_callback(self,request,response):
        """
        Forwards the robot_ip, protocol config, robot_config and simulate option
        through the ActionClient's goal interface to the ActionServer
        """

        command = yaml.safe_load(request.command)
        try:
            pc_path = command['args']['pc_path']
        except KeyError:
            response.error_msg = "pc_path not specified in the workflow file" 
            response.success = False
            return response
        pc_config = None
            
        try:
            pc_config = yaml.dump(yaml.safe_load(open(pc_path)))
        except IOError:
            response.error_msg = "No such file for protocol config :" + pc_path
            response.success = False
            return response

        simulate = False
        robot_ip = "192.168.10.1"
        if os.getenv('simulate') and os.getenv('simulate').lower()=='true':
            simulate = True
        # req.robot_ip = robot_ip
        if os.getenv('robot_ip'):
            robot_ip = os.getenv('robot_ip')

        response.success = True
        self.send_goal(robot_ip, protocol_config=pc_config, simulate=simulate)

        ## TODO Should probably validate the arguments passed on. Extra Redundancy

        return response


    # def send_goal(self, pc_path=None, rc_path=None, protocol_config="", robot_config=""):
    def send_goal(self, robot_ip, protocol_config="", robot_config="", simulate=False ):
        """
        Send stamped header, robot_ip, protocol_config, robot_config and simulate flag to ActionServer
        If self.emergency_flag, do not send. Redundancy checks on both server and action side

        """
        ## TODO Should probably validate the arguments

        wait_period = 0.5

        ## Construct OT2Job goal message
        goal_msg = OT2Job.Goal()

        goal_msg.job.header.src = self.get_fully_qualified_name()
        goal_msg.job.header.dest = "{}/{}".format(self.get_namespace(), self.action_server_name)
        goal_msg.job.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.job.robot_config = robot_config
        goal_msg.job.protocol_config = protocol_config
        #### HEAD (CURRENT CHANGE)
        # sim = False
        # if os.getenv('simulate').lower()=='true':
        #     sim = True
        # goal_msg.job.simulate = sim
        goal_msg.job.robot_ip = robot_ip
        goal_msg.job.simulate = simulate
        
        
        ## Wait for server, then for IDLE state in server's heartbeat
        ## TODO Emergency check: currently looping but should this terminate after one (?)
        self._action_client.wait_for_server()
        
        # while self.emergency_flag: ## TODO: determine if this is blocking to the whole node ie publishing and subscriptions
        #     self.get_logger().warn("Cannot send goal in an emergency event. Waiting...")
        #     time.sleep(wait_period) 

        # while self.server_heartbeat_flag != Heartbeat.IDLE: ## TODO: determine if this is blocking to the whole node
        #     self.get_logger().warn("Cannot send goal unless {}'s Heartbeat.state is IDLE. Waiting...".format(goal_msg.job.header.dest))
        #     time.sleep(wait_period)

        ## Send the goal message
        self.get_logger().info("Sending goal to {} ...".format(goal_msg.job.header.dest))
        self.goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_callback)
        self.goal_future.add_done_callback(self.goal_response_callback)
        


    def goal_feedback_callback(self,feed):
        """
        Currently reporting the string feedback from the ActionServer
        """
        feedback_msg = feed.feedback
        self.get_logger().info("From {}: {}".format(feedback_msg.progress.header.src, feedback_msg.progress.progress_msg))
        


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by ActionClient, {}/{}'.format(self.get_namespace(), self.action_server_name))
            ## TODO: State reporting
            return
        self.get_logger().info('Goal accepted by ActionClient, {}/{}'.format(self.get_namespace(), self.action_server_name))

        
        self._heartbeat_state = Heartbeat.BUSY
        self._heartbeat_info = "Processing Job"
        self.get_logger().info('{}: IDLE --> BUSY'.format(self.get_fully_qualified_name()))
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.goal_result_callback)
        

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result from ActionServer:")
        self.get_logger().info("--success: {}".format(result.success))
        self.get_logger().info("--message: {}".format(result.error_msg))
        if result.success:
            # state = States.IDLE  ## TODO: Termination State needed from ClientManager
            self._heartbeat_state = Heartbeat.IDLE ## TODO: Termination State needed from ClientManager
            self._heartbeat_info = ""
        else:
            ## TODO Propogating Error Alerts Upstream
            self._heartbeat_state = Heartbeat.ERROR
            self._heartbeat_info = result.error_msg  


def main(args=None):
    rclpy.init(args=args)
    demo_action_client = DemoActionClient()
    while True:
        
        rclpy.spin_once(demo_action_client,timeout_sec=0)
    

if __name__ == '__main__':
    main()
