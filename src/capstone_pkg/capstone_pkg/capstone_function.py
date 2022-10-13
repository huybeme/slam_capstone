import rclpy
from rclpy.node import Node
import std_msgs.msg, geometry_msgs.msg, std_srvs.srv
from functools import partial


def send_service_request(node, name, node_name):

    try:
        client = node.create_client(std_srvs.srv.SetBool, name)

        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("waiting for server to recieve request for node " + node_name)
        
        req = std_srvs.srv.SetBool.Request()
        req.data = True

        future = client.call_async(req)

        # future.add_done_callback(
        #     # response not used as variable, only for troubleshooting as messages
        #     partial(call_send_service)
        # )
        node.get_logger().info(f"node {node_name} recieved service request")
        
        return True
    except:
        return False

def call_send_service(future):
    node = Node("temp_service_node")
    try:
        response = future.result()

        # node.get_logger().info(str(response.success))
        node.get_logger().info(str(response.message))
        node.get_logger().info("service call completed")
    except:
        node.get_logger().info("service call failed")