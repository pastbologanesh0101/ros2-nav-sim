import unittest

from ros2_nav_sim.core import Node, TopicRegistry


class TestPubSub(unittest.TestCase):
    def setUp(self):
        self.registry = TopicRegistry()

    def test_published_message_delivered_to_all_subscribers(self):
        node = Node("test_node", self.registry)
        received_a = []
        received_b = []
        node.create_subscription(str, "/chatter", received_a.append)
        node.create_subscription(str, "/chatter", received_b.append)
        pub = node.create_publisher(str, "/chatter")

        pub.publish("hello")

        self.assertEqual(received_a, ["hello"])
        self.assertEqual(received_b, ["hello"])

    def test_subscriber_receives_multiple_messages_in_order(self):
        node = Node("test_node", self.registry)
        received = []
        node.create_subscription(int, "/numbers", received.append)
        pub = node.create_publisher(int, "/numbers")

        pub.publish(1)
        pub.publish(2)
        pub.publish(3)

        self.assertEqual(received, [1, 2, 3])

    def test_subscriber_on_different_topic_does_not_receive_message(self):
        node = Node("test_node", self.registry)
        chatter_received = []
        other_received = []
        node.create_subscription(str, "/chatter", chatter_received.append)
        node.create_subscription(str, "/other_topic", other_received.append)
        pub = node.create_publisher(str, "/chatter")

        pub.publish("hello")

        self.assertEqual(chatter_received, ["hello"])
        self.assertEqual(other_received, [])

    def test_two_nodes_share_topic_by_name(self):
        node_a = Node("node_a", self.registry)
        node_b = Node("node_b", self.registry)
        received = []
        node_b.create_subscription(str, "/shared", received.append)
        pub = node_a.create_publisher(str, "/shared")

        pub.publish("cross-node message")

        self.assertEqual(received, ["cross-node message"])

    def test_late_subscriber_receives_last_message_transient_local(self):
        # Mirrors ROS2's TRANSIENT_LOCAL ("latched") QoS behavior.
        node = Node("test_node", self.registry)
        pub = node.create_publisher(str, "/latched")
        pub.publish("already published before subscribing")

        received = []
        node.create_subscription(str, "/latched", received.append)

        self.assertEqual(received, ["already published before subscribing"])


if __name__ == "__main__":
    unittest.main()
