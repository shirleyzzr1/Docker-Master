from launch import LaunchDescription
from launch_ros.actions import Node
import yaml 

def generate_launch_description():
    """
    Parse workflow and launch action_clients per listed machines
    Also launch 
     - error_handler node(emergency system core)
     - client_manager (goal distribution command center)
    """
    Nodelist = []

    ## Parsing workcell for machines
    ## Running action_client per

    user_path = "/root/example_ws.yml"
    workcell_data = yaml.safe_load(open(user_path))
    machines= workcell_data['modules']
    for i in range(len(machines)):
        machine = machines[i]['name']
        node = Node(
            package = 'demo',
            namespace = machine,
            executable = 'action_client',
            name = 'action_client',
            emulate_tty = True
        )
        Nodelist.append(node)

    ## Command distribution center and Emergency hub
    Manager_node = Node(
                package='demo',
                namespace='',
                executable='client_manager',
                name='client_manager',
                emulate_tty = True
    )
    ErrorHandler_node = Node(
                package='demo',
                namespace='',
                executable='error_handler',
                name='error_handler',
                emulate_tty = True
    )

    Nodelist.append(Manager_node)
    Nodelist.append(ErrorHandler_node)


    return LaunchDescription(Nodelist)
