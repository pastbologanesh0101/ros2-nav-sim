import unittest

from ros2_nav_sim.core import Executor, Node, TopicRegistry, spin


class TestExecutor(unittest.TestCase):
    def setUp(self):
        self.registry = TopicRegistry()

    def test_spin_calls_timer_callback_exact_number_of_times(self):
        node = Node("counter_node", self.registry)
        call_count = {"n": 0}
        node.create_timer(0.1, lambda: call_count.__setitem__("n", call_count["n"] + 1))

        iterations = spin(node, max_iterations=7)

        self.assertEqual(iterations, 7)
        self.assertEqual(call_count["n"], 7)

    def test_spin_zero_iterations_calls_nothing(self):
        node = Node("counter_node", self.registry)
        call_count = {"n": 0}
        node.create_timer(0.1, lambda: call_count.__setitem__("n", call_count["n"] + 1))

        spin(node, max_iterations=0)

        self.assertEqual(call_count["n"], 0)

    def test_executor_spins_multiple_nodes_each_iteration(self):
        node_a = Node("a", self.registry)
        node_b = Node("b", self.registry)
        counts = {"a": 0, "b": 0}
        node_a.create_timer(0.1, lambda: counts.__setitem__("a", counts["a"] + 1))
        node_b.create_timer(0.1, lambda: counts.__setitem__("b", counts["b"] + 1))

        executor = Executor()
        executor.add_node(node_a)
        executor.add_node(node_b)
        executor.spin(max_iterations=4)

        self.assertEqual(counts["a"], 4)
        self.assertEqual(counts["b"], 4)

    def test_node_with_multiple_timers_all_get_called(self):
        node = Node("multi_timer_node", self.registry)
        counts = {"first": 0, "second": 0}
        node.create_timer(0.1, lambda: counts.__setitem__("first", counts["first"] + 1))
        node.create_timer(0.2, lambda: counts.__setitem__("second", counts["second"] + 1))

        spin(node, max_iterations=3)

        self.assertEqual(counts["first"], 3)
        self.assertEqual(counts["second"], 3)


if __name__ == "__main__":
    unittest.main()
