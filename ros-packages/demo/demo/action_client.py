import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import yaml

from demo_interfaces.action import OT2Job
from demo_interfaces.msg import OT2Job 
from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat
from demo_interfaces.srv import ExecuteJob


class DemoActionClient(Node):
    """
    A ROS2 node to plug into the ROS2 node that interfaces directly with the 
    OT2.
    This node provides an action client and upstream-facing service to pass 
    OT2 instruction sets/ protocols throught to the OT2 robot.

    ActionClient:
     - ~/OT2 [demo_interfaces/action/OT2Job] 
            -- for instructing OT2 runs defined by protocol configuration 
                files

    Publishers:
     - ~/action_client/heartbeat [demo_interfaces/msg/Heartbeat] 
            -- to alert subscribers to node state

    Subscribers:
     - /emergency [demo_interfaces/msg/EmergencyAlert]  
            -- for global emergency tracking
    - ~/action_server/heartbeat [demo_interfaces/msg/Heartbeat] 
            -- to track OT2/action_server node's state

    Service:
     - ~/action_client/execute_job [demo_interfaces/EmergencyAlert]
            -- upstream plug to send goal content to OT2 action server

    Service Clients:
     - /raise_emergency [demo_interfaces/srv/RaiseEmergency] 
            -- As part of global emergency system to alert system to OT2 
                emergencies [currently deactivated]
     - /clear_emergency [demo_interfaces/srv/RaiseEmergency] 
            -- As part of global emergency system to alert system to resolved 
                OT2 emergency [currently deactivated]
    """

    def __init__(self):
        """
        Subscribe and respond to Emergency Alerts system
            Respond by not forwarding OT2 instruction set during emergency 
            event
            Respond by forwarding OT2 instruction set when emergency event is 
            resolved
        Create service client to report emergencies to Emergency Alert system
        Publish Heartbeat on a timer
        Create action client to send OT2 goals and recieve progress feedback 
        and results
        """

        super().__init__('action_client')

        ## Action client to be namespaced with action server
        self._action_client = ActionClient(self, OT2Job, 'OT2')


        ## TODO !!!!!!ERROR This has to be fetch and enforced by a check on the 
        ## parameter server since the action_server node's name is 
        ## overriden by the launch file
        self.action_server_name = "action_server"  ## was "demo_action_server"
        

        ## Set up global Emergency tracking and service proxy (client)
        ## Initialized as True for safety; waiting for clearance from 
        ## emergency system
        ## No use case here for raising/clearing emergency so client is commented out
        self.emergency_sub = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)
        self.emergency_flag = True
        # self.emergency_flagging_client = self.create_client(RaiseEmergency, "/raise_emergency") 
        # self.emergency_deflaggin_client = self.create_client(RaiseEmergency, "/clear_emergency") 


        ## Set up Heartbeat publishing on timer
        heartbeat_timer_period = 0.5  # seconds
        self.heartbeat_timer = self.create_timer(heartbeat_timer_period, self.heartbeat_timer_callback)
        self.heartbeat_publisher = self.create_publisher(Heartbeat, '{}/heartbeat'.format(self.get_name()), 10) 
        self.heartbeat_msg = Heartbeat()
        self.heartbeat_msg.header.src = self.get_fully_qualified_name()
        self._heartbeat_state = Heartbeat.IDLE 
        self._heartbeat_info = ""


        ## Subscribe to and track the heartbeat of the counterpart action 
        ## server node
        ## And only send goal when action server's heartbeat state is IDLE 
        ## (ie primed)
        self.server_heartbeat_sub = self.create_subscription(Heartbeat, '{}/heartbeat'.format(self.action_server_name), self.server_heartbeat_callback, 10)
        self.server_heartbeat_flag = 100 ## TODO Initialize to Unknown for safety; waiting for known state from heartbeat
        

        ## Job service to trigger actions
        self.execute_job_service = self.create_service(ExecuteJob,'execute_job',self.exectute_job_callback)
        

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
        self.heartbeat_msg.state = self._heartbeat_state 
        self.heartbeat_msg.message = self._heartbeat_info
        self.heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        self.heartbeat_publisher.publish(self.heartbeat_msg)
    


    def server_heartbeat_callback(self, msg):
        """
        Update privately tracked state of ActionServer 
        TODO: Validation check on heartbeat timestamps. Received-at time 
                and timestamp should be appropriately close
        """
        if self.server_heartbeat_flag != msg.state:
            self.server_heartbeat_flag = msg.state



    def emergency_callback(self,msg):
        """
        Update emergency_flag status
        """
        
        if msg.is_emergency and not self.emergency_flag:
            self.emergency_flag = True
            self.get_logger().warn("Received an emergency alert: {}".format(msg.message)) 
            self.get_logger().warn("Emergency reported by {} at {}".format(msg.header.src, msg.header.stamp))  ## TODO Verify correcting printing

        elif not msg.is_emergency and self.emergency_flag:
            self.emergency_flag = False
            self.get_logger().info("Emergency alert(s) cleared.")



    def exectute_job_callback(self,request,response):
        """
        Forwards the robot_ip, protocol config, robot_config and simulate option
        through the ActionClient's goal interface to the ActionServer
        TODO Should validate the arguments passed in the request. Extra Redundancy
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

        robot_ip = request.robot_ip
        simulate = request.simulate

        response.success = True
        self.send_goal(robot_ip, protocol_config=pc_config,robot_config=rc_config, simulate=simulate)

        return response



    def send_goal(self, robot_ip, protocol_config, robot_config, simulate=False ):
        """
        Prep stamped header, robot_ip, protocol_config, robot_config and 
        simulate flag for sending to ActionServer through a timer that waits 
        for out any blocking event ie emergency event or non-readiness at 
        the action server.
        TODO Should probably validate the arguments passed in. Extra redundancy
        """

        ## Construct OT2Job goal message
        self.goal_msg = OT2Job.Goal()
        self.goal_msg.job.header.src = self.get_fully_qualified_name()
        self.goal_msg.job.header.dest = "{}/{}".format(self.get_namespace(), self.action_server_name)
        self.goal_msg.job.header.stamp = self.get_clock().now().to_msg()

        self.goal_msg.job.robot_config = robot_config
        self.goal_msg.job.protocol_config = protocol_config
        self.goal_msg.job.robot_ip = robot_ip
        self.goal_msg.job.simulate = simulate
        
        
        ## Wait for server, then start timer polls until all clear
        self._action_client.wait_for_server()
        self.send_goal_timer = self.create_timer(1, self.send_goal_timer_callback)



    def send_goal_timer_callback(self):
        """
        A timer callback that runs continually while pre-checks fail.
        Implements a waiting behavior.
        If self.emergency_flag or action server node is not ready, do not send. 
        Redundant emergency checks on both server and action side.
        Thereafter, the timer is canceled and the goal is sent
        """

        response_message = ""
        
        ## Pre-Checks
        if self.emergency_flag:
            response_message = "Cannot send goal in an emergency event. Waiting for emergency to be cleared..."

        elif self.server_heartbeat_flag != Heartbeat.IDLE: 
            response_message = "Cannot send goal unless {}'s Heartbeat.state is IDLE. Waiting...".format(self.goal_msg.job.header.dest)

        ## Send the goal
        else:
            response_message = "Sending goal to {} ...".format(self.goal_msg.job.header.dest)
            self.goal_future = self._action_client.send_goal_async(self.goal_msg, feedback_callback=self.goal_feedback_callback)
            self.goal_future.add_done_callback(self.goal_response_callback)
            self.send_goal_timer.cancel()

        ## Log response message 
        if self._heartbeat_info != response_message:
            self._heartbeat_info = response_message
            self.get_logger().info(self._heartbeat_info)

                

    def cancel_goal(self):
        """Trigger cancellation of recent goal requested"""

        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)
        self.get_logger().info("Requesting to cancel goal ...")



    def cancel_done_callback(self, future):
        """ Verify and alert about goal cancellation """

        cancel_goal_result = future.result()

        if len(cancel_goal_result.goals_canceling) > 0:
            self.get_logger().info("Goal successfully cancelled")
        else:
            self.get_logger().info("Goal failed to cancel")



    def goal_feedback_callback(self,feed):
        """
        Currently echoing the string feedback from the ActionServer
        """
        feedback_msg = feed.feedback
        self.get_logger().info("From {}: {}".format(feedback_msg.progress.header.src, feedback_msg.progress.progress_msg))



    def goal_response_callback(self, future):
        """
        Checks if the goal was accepted or reject by the action server
        Update action client heartbeat if goal was accepted
        Sets up callback for when action server returns result
        TODO Update action client heartbeat if goal was rejected
        """
        goal_handle = future.result()
        self._goal_handle = goal_handle

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by ActionClient, {}/{}'.format(self.get_namespace(), self.action_server_name))
            ## TODO: Heartbeat reporting when goal rejected
            return

        self.get_logger().info('Goal accepted by ActionClient, {}/{}'.format(self.get_namespace(), self.action_server_name))

        self._heartbeat_state = Heartbeat.BUSY
        self._heartbeat_info = "Processing Job"
        self.get_logger().info(self._heartbeat_info)
        self.get_logger().info('{}: IDLE --> BUSY'.format(self.get_fully_qualified_name()))
        
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.goal_result_callback)
        


    def goal_result_callback(self, future):
        """
        Logs result from action server
        
        TODO:   Termination State needed from ClientManager, currently resets 
                to IDLE. Will determine whether the node's resource will be 
                freed up or used again.
                IDLE will indicate reuse. FINISHED will indicate no reuse
        """
        result = future.result().result
        self.get_logger().info("Result from ActionServer:")
        self.get_logger().info("--success: {}".format(result.success))
        self.get_logger().info("--message: {}".format(result.error_msg))
        
        if result.success: 
            self._heartbeat_state = Heartbeat.IDLE 
            self._heartbeat_info = ""
            self.get_logger().info("Job processed sucessfully.")
            self.get_logger().info('{}: BUSY --> IDLE'.format(self.get_fully_qualified_name()))

        else:
            ## Propogate error alerts upstream through heartbeat
            self._heartbeat_state = Heartbeat.ERROR
            self._heartbeat_info = result.error_msg
            self.get_logger().info("Job not processed sucessfully.")
            self.get_logger().info('{}: BUSY --> ERROR'.format(self.get_fully_qualified_name()))

             


def main(args=None):
    """
    """
    
    rclpy.init(args=args)
    
    demo_action_client = DemoActionClient()

    try:
        rclpy.spin(demo_action_client)
    except KeyboardInterrupt:
        pass
    
if __name__ == '__main__':
    main()
