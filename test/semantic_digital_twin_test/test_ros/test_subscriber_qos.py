# %% imports

from __future__ import annotations

from dataclasses import dataclass
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from krrood.ormatic.utils import classproperty
from semantic_digital_twin.adapters.ros.latest_message_subscriber import (
    LatestMessageSubscriber,
)

# %% state subscriptions


@dataclass
class RetainedStateSubscriber(LatestMessageSubscriber[Bool]):
    """
    Request the last retained state as well as future changes.
    """

    @classproperty
    def qos_profile(cls) -> QoSProfile:
        return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@dataclass
class FutureStateSubscriber(LatestMessageSubscriber[Bool]):
    """
    Retain the ordinary subscription policy for future changes.
    """


def test_retained_state_profile_reaches_subscription() -> None:
    """
    A subclass can request retained state from the actual subscription factory.
    """
    node = Mock(spec=Node)

    RetainedStateSubscriber(node=node, topic_name="test_retained_state")

    assert (
        node.create_subscription.call_args.args[3].durability
        == DurabilityPolicy.TRANSIENT_LOCAL
    )


def test_default_subscription_remains_volatile() -> None:
    """
    Existing subscribers keep their volatile durability.
    """
    node = Mock(spec=Node)

    FutureStateSubscriber(node=node, topic_name="test_future_state")

    assert (
        node.create_subscription.call_args.args[3].durability
        == DurabilityPolicy.VOLATILE
    )
