import unittest
from pathlib import Path
import sys

import launch
import launch_ros.actions
import launch_testing.actions

sys.path.append(str(Path(__file__).resolve().parent))

from node_graph_test_utils import GraphAssertsMixin, visual_inertial_params_file


def generate_test_description():
    params_file = visual_inertial_params_file()

    optimization_node = launch_ros.actions.Node(
        package="visual_inertial",
        executable="optimization_node",
        name="visual_inertial_optimization",
        output="screen",
        parameters=[
            params_file,
            {
                "operation_mode": "mapping",
            },
        ],
    )

    return (
        launch.LaunchDescription(
            [
                optimization_node,
                launch_testing.actions.ReadyToTest(),
                launch.actions.TimerAction(
                    period=5.0,
                    actions=[launch.actions.Shutdown(reason="mapping node-graph test complete")],
                ),
            ]
        ),
        {},
    )


class TestMappingModeGraph(GraphAssertsMixin, unittest.TestCase):
    def test_mapping_mode_nodes_and_topics_exist(self):
        self.wait_for_nodes(
            {"visual_inertial_optimization"},
            forbidden_names={"visual_inertial_localization"},
        )
        self.wait_for_topic_endpoints(
            "keyframes",
            subscriber_nodes={"visual_inertial_optimization"},
        )
        self.wait_for_topic_endpoints(
            "/imu_bias",
            publisher_nodes={"visual_inertial_optimization"},
        )
        self.wait_for_topic_endpoints(
            "optimization_result",
            publisher_nodes={"visual_inertial_optimization"},
        )
