import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from demo_interfaces.srv import ExecuteJob
from demo_interfaces.srv import StartJob


from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat
from demo_interfaces.srv import RaiseEmergency
from std_msgs.msg import String

import yaml
import enum
import os
import curses
from curses.textpad import Textbox, rectangle
from curses import wrapper

# Using enum class create enumerations
class States(enum.Enum):
   IDLE = 0
   BUSY = 1
   FINISHED = 2
   ERROR = 3
   EMERGENCY =4

class VisualTool(Node):
    def __init__(self):
        super().__init__('visual_tool')
        self.workcell_data = None
        self.machines = self.parse_machines()
        self.machine_states = self.create_states()
        self.steps = self.workcell_data['actions']

        self.emergency_flag = False

        self.create_subs()
        self.emergency = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)
        self.executeJob_client = self.create_client(StartJob, '/execute_job') ## TODO 'ot2_1/execute_job
        self.raiseEmergency_client = self.create_client(RaiseEmergency, '/raise_emergency')
        self.clearEmergency_client = self.create_client(RaiseEmergency, '/clear_emergency')


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
            dicts[machine]="ERROR"
        return dicts

    def common_callback(self,msg):
        """
        Update machine states based on heartbeat
        """
        machine = msg.header.src.split("/")[1]
        state = States(msg.state).name
        self.machine_states[machine]=state
        info = msg.message
        if self.machine_states[machine]=="ERROR" and state=="IDLE":
            self.machine_states[machine]="IDLE"
        elif self.machine_states[machine]=="ERROR":
            self.machine_states[machine]="ERROR"

    def create_subs(self):
        """
        Create subscription for all the heartbeat
        """
        for name,type in self.machines:
            setattr(self,"sub"+name, self.create_subscription(Heartbeat,"/{}/action_client/heartbeat".format(name),lambda msg:self.common_callback(msg),10))
    
    def send_exec_job_request(self):
        """
        """
        req = StartJob.Request()
        req.command = "start"
        self.future = self.executeJob_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def send_raise_emer_request(self):
        """
        call raise_emergency service
        """
        req = RaiseEmergency.Request()
        req.alert.is_emergency = True
        self.future = self.raiseEmergency_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_clear_emer_request(self):
        """
        call clear_emergency service
        """
        req = RaiseEmergency.Request()
        req.alert.is_emergency = False
        self.future = self.clearEmergency_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def emergency_callback(self,msg):
        """
        Deal with the emergency callback
        """
        if msg.is_emergency==True:
            self.emergency_flag=True
        else:
            self.emergency_flag = False
            # self.get_logger().info("client_manager received an emergency alert: " + msg.message)

class DrawCurses(object):
    def __init__(self,machines) -> None:
        self.stdscr = curses.initscr()
        self.curses_init()
        self.machines = machines
        self.wins = []
        self.draw_init()


    def curses_init(self):
        curses.curs_set(0)
        self.stdscr.keypad(True)
        self.stdscr.nodelay(True)
        self.stdscr.clear()

        curses.init_pair(1,curses.COLOR_GREEN,curses.COLOR_BLACK)
        curses.init_pair(2,curses.COLOR_RED,curses.COLOR_BLACK)
        self.GREEN_AND_BLACK = curses.color_pair(1)
        self.RED_AND_BLACK = curses.color_pair(2)

    def draw_init(self):
        for i in range(len(self.machines)):
            rectangle(self.stdscr,2,10+20*i,10,25+20*i)
            self.stdscr.addstr(2,15+20*i,self.machines[i][0])
            type = "type: {}".format(self.machines[i][1])
            self.stdscr.addstr(4,11+20*i,type)
            self.stdscr.addstr(5,11+20*i,"State:ERROR",self.RED_AND_BLACK)
            wins = curses.newwin(2,14,5,11+20*i)
            self.wins.append(wins)


    def update_state(self,states,emergency_flag):
        for i in range(len(self.wins)):
            state = "State: {} ".format(states[self.machines[i][0]])
            if states[self.machines[i][0]]=="ERROR":
                self.wins[i].addstr(0,0,state,self.RED_AND_BLACK)
            else:
                self.wins[i].addstr(0,0,state,self.GREEN_AND_BLACK)
            if emergency_flag:
                self.wins[i].addstr(1,0,"EMERGENCY!",curses.A_BLINK | self.RED_AND_BLACK)
            else:
                self.wins[i].addstr(1,0,"          ",curses.A_BLINK | self.RED_AND_BLACK)
            self.wins[i].refresh() 

class Menu(object):
    def __init__(self,items,stdscr):
        self.items = items
        self.window = stdscr.subwin(10,30,13,10)
        self.infowindow = stdscr.subwin(10,30,13,40)
        self.position = 0
        self.items.append("exit")
        self.flag = -1
    
    def navigate(self,n):
        """
        Make sure the cursor is not out of boundry
        """
        self.position+=n
        if self.position<0:
            self.position = 0
        elif self.position >= len(self.items):
            self.position = len(self.items) - 1

    def display(self,key):
        """
        Update menu display
        """
        for index, item in enumerate(self.items):
            if index == self.position:
                mode = curses.A_REVERSE
            else:
                mode = curses.A_NORMAL

            msg = "%d. %s" % (index, item)
            self.window.addstr(1 + index, 1, msg, mode)

        if key in [curses.KEY_ENTER, ord("\n")]:
            if self.position == len(self.items) - 1:
                return True
            else:
                self.flag = self.position

        elif key == curses.KEY_UP:
            self.navigate(-1)

        elif key == curses.KEY_DOWN:
            self.navigate(1)
        self.window.refresh()
    
    def show_info(self,msg):
        self.infowindow.clear()
        self.infowindow.addstr(2, 0, msg)
        self.infowindow.refresh()


def main(stdscr):
    rclpy.init()
    vis = VisualTool()

    drawcurses = DrawCurses(vis.machines)
    items = ["Start Execution","Raise Emergency","Clear Emergency"]
    menu = Menu(items,drawcurses.stdscr)
    module = vis.steps[0]['module']
    while True:
        try:
            k = drawcurses.stdscr.getch()
        except:
            k = None
        if menu.display(k):
            break
        if menu.flag==0:
            if vis.emergency_flag:
                menu.show_info("Emergency, send failed!")
            elif vis.machine_states[module]=="IDLE":
                response = vis.send_exec_job_request()
                if response.success:
                    menu.show_info("{} success!".format(menu.items[0]))
                else:
                    menu.show_info(response.error_msg)
            else:
                menu.show_info("{} is not IDLE!".format(module))
        elif menu.flag==1:
            response = vis.send_raise_emer_request()
            if response.success:
                menu.show_info("{} success!".format(menu.items[1]))

        elif menu.flag==2:
            response = vis.send_clear_emer_request()
            if response.success:
                menu.show_info("{} success!".format(menu.items[2]))
        menu.flag = -1           

        drawcurses.update_state(vis.machine_states,vis.emergency_flag)
        rclpy.spin_once(vis,timeout_sec=0)

    rclpy.shutdown()

if __name__ == '__main__':
    wrapper(main)
