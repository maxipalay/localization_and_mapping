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

    localization_node = launch_ros.actions.Node(
        package="visual_inertial",
        executable="localization_node",
        name="visual_inertial_localization",
        output="screen",
        parameters=[
            params_file,
            {
                "localization_tag_map_path": "",
            },
        ],
    )

    optimization_node = launch_ros.actions.Node(
        package="visual_inertial",
        executable="optimization_node",
        name="visual_inertial_optimization",
        output="screen",
        parameters=[
            params_file,
            {
                "operation_mode": "localization",
            },
        ],
    )

    return (
        launch.LaunchDescription(
            [
                localization_node,
                optimization_node,
                launch_testing.actions.ReadyToTest(),
                launch.actions.TimerAction(
                    period=5.0,
                    actions=[launch.actions.Shutdown(reason="localization node-graph test complete")],
                ),
            ]
        ),
        {},
    )


class TestLocalizationModeGraph(GraphAssertsMixin, unittest.TestCase):
    def test_localization_mode_nodes_and_topics_exist(self):
        self.wait_for_nodes(
            {
                "visual_inertial_localization",
                "visual_inertial_optimization",
            }
        )
        self.wait_for_topic_endpoints(
            "keyframes",
            subscriber_nodes={
                "visual_inertial_localization",
                "visual_inertial_optimization",
            },
        )
        self.wait_for_topic_endpoints(
            "/imu_bias",
            publisher_nodes={"visual_inertial_optimization"},
        )
        self.wait_for_topic_endpoints(
            "localization_command",
            publisher_nodes={"visual_inertial_localization"},
            subscriber_nodes={"visual_inertial_optimization"},
        )
        self.wait_for_topic_endpoints(
            "localization_feedback",
            publisher_nodes={"visual_inertial_optimization"},
            subscriber_nodes={"visual_inertial_localization"},
        )
        self.wait_for_topic_endpoints(
            "optimization_result",
            publisher_nodes={"visual_inertial_optimization"},
        )
