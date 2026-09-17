"""A minimal, in-process pub/sub framework mirroring the *shape* of the
ROS2 node graph API: Node, Topic, create_publisher(), create_subscription(),
and an Executor with a spin() loop.

This is a simulation. There is no DDS, no serialization, no processes, no
QoS negotiation -- publishing a message simply calls each subscriber's
callback synchronously, in-process, in the order it was subscribed. That is
enough to demonstrate the architecture (nodes communicate only through named
topics, never by holding direct references to each other) and to build a
working navigation demo on top of it.

See README.md "Porting to real ROS2 / rclpy" for what would need to change
to run this on an actual ROS2 install.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List


class Topic:
    """A named channel. Publishers publish messages onto it; every
    subscriber callback registered on it is invoked, in order, with the
    message. Mirrors a ROS2 topic (minus DDS transport and QoS)."""

    def __init__(self, name: str, msg_type: Any = None):
        self.name = name
        self.msg_type = msg_type
        self._subscribers: List[Callable[[Any], None]] = []
        self._last_msg: Any = None
        self._has_msg: bool = False

    def subscribe(self, callback: Callable[[Any], None]) -> None:
        self._subscribers.append(callback)
        # Mirrors ROS2's TRANSIENT_LOCAL ("latched") QoS: a subscriber that
        # joins after a message was already published still gets the most
        # recent one immediately, instead of only future messages. This
        # keeps demo wiring order (which node subscribes/publishes first)
        # from silently dropping the first message.
        if self._has_msg:
            callback(self._last_msg)

    def unsubscribe(self, callback: Callable[[Any], None]) -> None:
        if callback in self._subscribers:
            self._subscribers.remove(callback)

    def publish(self, msg: Any) -> None:
        # Synchronous fan-out to every subscriber of this topic only.
        self._last_msg = msg
        self._has_msg = True
        for callback in list(self._subscribers):
            callback(msg)

    @property
    def subscriber_count(self) -> int:
        return len(self._subscribers)


class TopicRegistry:
    """Shared "middleware" that maps topic name -> Topic. Every Node that
    talks to the same registry can discover each other's topics purely by
    name, exactly like real ROS2 nodes discover each other via DDS discovery
    without holding direct references."""

    def __init__(self):
        self._topics: Dict[str, Topic] = {}

    def get_topic(self, name: str, msg_type: Any = None) -> Topic:
        if name not in self._topics:
            self._topics[name] = Topic(name, msg_type)
        return self._topics[name]

    def topic_names(self) -> List[str]:
        return list(self._topics.keys())


class Publisher:
    """Handle returned by Node.create_publisher(). Mirrors rclpy's
    Publisher.publish(msg)."""

    def __init__(self, topic: Topic):
        self._topic = topic

    @property
    def topic_name(self) -> str:
        return self._topic.name

    def publish(self, msg: Any) -> None:
        self._topic.publish(msg)


class Subscription:
    """Handle returned by Node.create_subscription(). Holding a reference
    keeps the underlying callback registered; there is nothing else the
    caller needs to do with it (mirrors rclpy's Subscription)."""

    def __init__(self, topic: Topic, callback: Callable[[Any], None]):
        self._topic = topic
        self._callback = callback
        topic.subscribe(callback)

    @property
    def topic_name(self) -> str:
        return self._topic.name

    def destroy(self) -> None:
        self._topic.unsubscribe(self._callback)


class Node:
    """Base class mirroring rclpy.node.Node's public shape closely enough to
    be recognizable: create_publisher(), create_subscription(),
    create_timer(). A "timer" here just means "a callback the Executor calls
    once per spin iteration" -- a simplification of ROS2 timers (which fire
    on wall-clock periods), appropriate for a discrete-step simulation."""

    def __init__(self, name: str, registry: TopicRegistry):
        self.name = name
        self._registry = registry
        self._timers: List[Callable[[], None]] = []
        self._publishers: List[Publisher] = []
        self._subscriptions: List[Subscription] = []

    def create_publisher(self, msg_type: Any, topic_name: str) -> Publisher:
        topic = self._registry.get_topic(topic_name, msg_type)
        pub = Publisher(topic)
        self._publishers.append(pub)
        return pub

    def create_subscription(
        self, msg_type: Any, topic_name: str, callback: Callable[[Any], None]
    ) -> Subscription:
        topic = self._registry.get_topic(topic_name, msg_type)
        sub = Subscription(topic, callback)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, period_s: float, callback: Callable[[], None]) -> None:
        # period_s is accepted for API-shape compatibility with rclpy, but
        # this simulation calls every registered timer callback exactly
        # once per spin iteration (see Executor.spin_once).
        self._timers.append(callback)


class Executor:
    """Mirrors rclpy's SingleThreadedExecutor: nodes are added to it, and
    spinning it drives every added node's timer callbacks."""

    def __init__(self):
        self.nodes: List[Node] = []

    def add_node(self, node: Node) -> None:
        self.nodes.append(node)

    def remove_node(self, node: Node) -> None:
        if node in self.nodes:
            self.nodes.remove(node)

    def spin_once(self) -> None:
        for node in self.nodes:
            for callback in list(node._timers):
                callback()

    def spin(self, max_iterations: int) -> int:
        """Run max_iterations spin_once() steps. Returns the number of
        iterations actually executed (always max_iterations here; the
        parameter exists because this simulation has no wall clock / no
        rclpy.shutdown() signal to stop on)."""
        for _ in range(max_iterations):
            self.spin_once()
        return max_iterations


def spin(node: Node, max_iterations: int = 100) -> int:
    """Convenience function mirroring rclpy.spin(node), extended with a
    max_iterations bound (a real, unbounded rclpy.spin() runs until
    interrupted or shut down; this simulation needs a deterministic stop
    condition instead)."""
    executor = Executor()
    executor.add_node(node)
    return executor.spin(max_iterations)
