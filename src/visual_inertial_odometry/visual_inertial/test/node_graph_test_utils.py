import time

import rclpy
from ament_index_python.packages import get_package_share_directory


def visual_inertial_params_file():
    return (
        get_package_share_directory("visual_inertial")
        + "/config/visual_inertial_params_realsense_splitter.yaml"
    )


def wait_for(predicate, timeout_sec=10.0, period_sec=0.1):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(period_sec)
    return False


def fully_qualified_topic_infos(node, topic_name):
    fq_topic = topic_name if topic_name.startswith("/") else f"/{topic_name}"
    return (
        node.get_publishers_info_by_topic(fq_topic),
        node.get_subscriptions_info_by_topic(fq_topic),
    )


class GraphAssertsMixin:
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._node = rclpy.create_node("visual_inertial_smoke_test")

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()
        rclpy.shutdown()

    def wait_for_nodes(self, required_names, forbidden_names=(), timeout_sec=10.0):
        def predicate():
            names = set(self._node.get_node_names())
            return required_names.issubset(names) and names.isdisjoint(forbidden_names)

        if not wait_for(predicate, timeout_sec=timeout_sec):
            self.fail(
                f"Timed out waiting for node graph. required={sorted(required_names)} "
                f"forbidden={sorted(forbidden_names)} "
                f"seen={sorted(self._node.get_node_names())}"
            )

    def wait_for_topic_wiring(
        self,
        topic_name,
        *,
        min_publishers=0,
        min_subscribers=0,
        timeout_sec=10.0,
    ):
        fq_topic = topic_name if topic_name.startswith("/") else f"/{topic_name}"

        def predicate():
            pubs, subs = fully_qualified_topic_infos(self._node, fq_topic)
            return len(pubs) >= min_publishers and len(subs) >= min_subscribers

        if not wait_for(predicate, timeout_sec=timeout_sec):
            pubs, subs = fully_qualified_topic_infos(self._node, fq_topic)
            self.fail(
                f"Timed out waiting for topic wiring on {fq_topic}: "
                f"publishers={len(pubs)} subscribers={len(subs)} "
                f"expected publishers>={min_publishers} subscribers>={min_subscribers}"
            )

    def wait_for_topic_endpoints(
        self,
        topic_name,
        *,
        publisher_nodes=(),
        subscriber_nodes=(),
        timeout_sec=10.0,
    ):
        fq_topic = topic_name if topic_name.startswith("/") else f"/{topic_name}"
        publisher_nodes = set(publisher_nodes)
        subscriber_nodes = set(subscriber_nodes)

        def predicate():
            pubs, subs = fully_qualified_topic_infos(self._node, fq_topic)
            pub_names = {info.node_name for info in pubs}
            sub_names = {info.node_name for info in subs}
            return publisher_nodes.issubset(pub_names) and subscriber_nodes.issubset(sub_names)

        if not wait_for(predicate, timeout_sec=timeout_sec):
            pubs, subs = fully_qualified_topic_infos(self._node, fq_topic)
            self.fail(
                f"Timed out waiting for endpoint ownership on {fq_topic}: "
                f"publishers={[info.node_name for info in pubs]} "
                f"subscribers={[info.node_name for info in subs]} "
                f"expected publishers={sorted(publisher_nodes)} "
                f"expected subscribers={sorted(subscriber_nodes)}"
            )
