import rclpy
from rclpy.node import Node

from demo_interfaces.srv import RaiseEmergency
from demo_interfaces.msg import EmergencyAlert


class ErrorHandler(Node):
    """
    A ROS2 node serving as the central server for the global emergency system.
    Emergency alerts are registered with the service, as well as when they are
    resolved.
    In turn, the global emergency system broadcasts an emergency status to all 
    other emergency-sensitive nodes.

    TODO The emergency status is maintain for as long as there are emergency 
    events that have not been cleared.
    TODO Emergency should be switched from timer based to latched publishing

    Publishers:
     - /emergency [demo_interfaces/msg/EmergencyAlert] 
            -- to broadcast systemwide the existence of an emergency situation

    Services:
     - /raise_emergency [demo_interfaces/srv/RaiseEmergency] 
            -- As part of global emergency system to alert system to OT2 
                emergencies [currently deactivated]
     - /clear_emergency [demo_interfaces/srv/RaiseEmergency] 
            -- As part of global emergency system to alert system to resolved 
                OT2 emergency [currently deactivated]
    """    


    def __init__(self):
        """
        Provides:
         - services for registering and deregistering emergency events
         - a timer-based broadcast to all emergency-sensitive nodes
        """
        super().__init__('error_handler')

        ## Services
        self.raise_emergency_srv = self.create_service(RaiseEmergency,'/raise_emergency',lambda req,resp:self.emergency_callback(req,resp))
        self.clear_emergency_srv = self.create_service(RaiseEmergency,'/clear_emergency',lambda req,resp:self.emergency_callback(req,resp))
        
        ## Publisher(s)
        self.emergency_publisher = self.create_publisher(EmergencyAlert, '/emergency', 10)
        
        ## Publishng timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.emergency_msg = ""
        self.emergency_flag = False

    def timer_callback(self):
        """Creates and publishes a new emergencyAlert with current emergency state"""
        msg = EmergencyAlert()
        msg.message = self.emergency_msg
        msg.is_emergency = self.emergency_flag
        self.emergency_publisher.publish(msg)

    def emergency_callback(self,request,response):
        """
        Updates emergency state according to reported emergency activity
        
        TODO Track reportees
        TODO Keep log of id-able emergency events.
        """
        self.emergency_msg = request.alert.message
        self.emergency_flag = request.alert.is_emergency
        response.success= True
        response.error_msg = "None"
        
        return response



def main(args=None):
    rclpy.init(args=args)
    error_handler = ErrorHandler()
    try:
        rclpy.spin(error_handler)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

