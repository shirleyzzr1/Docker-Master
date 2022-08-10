from launch import LaunchDescription
from launch_ros.actions import Node
import yaml 

def generate_launch_description():
    Nodelist = []
    user_path = "/root/example_ws.yml"
    workcell_data = yaml.safe_load(open(user_path))
    machines= workcell_data['modules']
    for i in range(len(machines)):
        machine = machines[i]['name']
        node = Node(
            package = 'demo',
            namespace = machine,
            executable = 'action_client',
            name = 'action_client'
        )
        Nodelist.append(node)
    Manager_node = Node(
                package='demo',
                namespace='',
                executable='client_manager',
                name='client_manager'
    )
    Nodelist.append(Manager_node)

    return LaunchDescription(Nodelist)
