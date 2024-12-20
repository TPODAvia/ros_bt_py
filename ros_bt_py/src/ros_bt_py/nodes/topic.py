# Copyright 2018-2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from threading import Lock
import rospy
from roslib.message import get_message_class

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="1.0.0",
        options={"topic_type": type, "topic_name": str},
        inputs={},
        outputs={"message": OptionRef("topic_type")},
        max_children=0,
    )
)
class TopicSubscriber(Leaf):
    """Subscribe to the specified topic and output the received messages

    This node will return RUNNING until a message is received on the topic.
    When a message is received, it outputs the message and returns SUCCEEDED.
    The message is then cleared for the next run.

    This node never returns FAILED.
    """

    def _do_setup(self):
        self._lock = Lock()
        self._msg = None
        self._subscriber = rospy.Subscriber(
            self.options["topic_name"], self.options["topic_type"], self._callback
        )
        return NodeMsg.IDLE

    def _callback(self, msg):
        with self._lock:
            self._msg = msg

    def _do_tick(self):
        with self._lock:
            if self._msg is None:
                return NodeMsg.RUNNING
            self.outputs["message"] = self._msg
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        # Unsubscribe from the topic so we don't receive further updates
        try:
            self._subscriber.unregister()
        except AttributeError:
            self.logwarn("Can not unregister as no subscriber is available.")

    def _do_reset(self):
        # discard the last received message
        self._msg = None
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        resolved_topic = rospy.resolve_name(self.options["topic_name"])

        for topic, topic_type_name in rospy.get_published_topics():
            topic_type = get_message_class(topic_type_name)
            if topic == resolved_topic and topic_type == self.options["topic_type"]:
                # if the topic we want exists, we can do our job, so
                # set all the bounds and leave their values at 0
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="1.0.0",
        options={"topic_type": type, "topic_name": str, "memory_delay": float},
        inputs={},
        outputs={"message": OptionRef("topic_type")},
        max_children=0,
    )
)
class TopicMemorySubscriber(Leaf):
    """Subscribe to the specified topic and returns FAILED if no message was recently received.

    This node will return FAILED if no message has been received since
    the last memory_delay seconds.
    When a message is received, it outputs the message and returns SUCCEEDED.
    The message is not cleared for the next runs.

    This node never returns RUNNING.
    """

    def _do_setup(self):
        self._lock = Lock()
        self._msg = None
        self._last_time = rospy.Time(0)
        self._subscriber = rospy.Subscriber(
            self.options["topic_name"], self.options["topic_type"], self._callback
        )
        return NodeMsg.IDLE

    def _callback(self, msg):
        with self._lock:
            self._msg = msg
            self._last_time = rospy.Time.now()

    def _do_tick(self):
        with self._lock:
            if (
                self._msg is None
                or (rospy.Time.now() - self._last_time).to_sec()
                > self.options["memory_delay"]
            ):
                return NodeMsg.FAILED
            self.outputs["message"] = self._msg
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        self._msg = None
        self._last_time = rospy.Time(0)
        # Unsubscribe from the topic so we don't receive further updates
        try:
            self._subscriber.unregister()
        except AttributeError:
            self.logwarn("Can not unregister as no subscriber is available.")

    def _do_reset(self):
        # discard the last received message and re-subscribe to the
        # topic, so we receive any latched messages again
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        resolved_topic = rospy.resolve_name(self.options["topic_name"])

        for topic, topic_type_name in rospy.get_published_topics():
            topic_type = get_message_class(topic_type_name)
            if topic == resolved_topic and topic_type == self.options["topic_type"]:
                # if the topic we want exists, we can do our job, so
                # set all the bounds and leave their values at 0
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="1.0.0",
        options={"topic_type": type, "topic_name": str},
        inputs={"message": OptionRef("topic_type")},
        outputs={},
        max_children=0,
    )
)
class TopicPublisher(Leaf):
    def _do_setup(self):
        self._publisher = rospy.Publisher(
            self.options["topic_name"],
            self.options["topic_type"],
            latch=True,
            queue_size=1,
        )
        return NodeMsg.IDLE

    def _do_tick(self):
        # Only publish a new message if our input data has been updated - the
        # old one is latched anyway.
        if self.inputs.is_updated("message"):
            self._publisher.publish(self.inputs["message"])
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        # Unregister the publisher
        try:
            if self._publisher is not None:
                self._publisher.unregister()
        except AttributeError:
            self.logwarn("Can not unregister as no publisher is available.")
        self._publisher = None

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="1.0.0",
        options={
            "topic_type": type,        # Type of the ROS topic (e.g., std_msgs.String)
            "topic_name": str,        # Name of the topic to publish to
            "default_message": dict,  # Default message content if no input is provided
            "rate_hz": float          # Publishing rate in Hz (optional)
        },
        inputs={"message": OptionRef("topic_type")},  # Input for message to publish
        outputs={"message": OptionRef("topic_type")},  # No outputs for the publisher
        max_children=0,
    )
)
class CustomTopicPublisher(Leaf):
    """Custom topic publisher node for ROS behavior trees.

    This node publishes a message to a specified topic. If an input message is
    provided, it is published. Otherwise, a default message is published.

    - Supports a publishing rate controlled by the 'rate_hz' option.
    """

    def _do_setup(self):
        # Initialize the publisher
        self._publisher = rospy.Publisher(
            self.options["topic_name"],
            self.options["topic_type"],
            latch=True,
            queue_size=1,
        )
        if self.options["rate_hz"] > 0:
            self._rate = rospy.Rate(self.options["rate_hz"])  # Default rate is 1 Hz
        self._default_msg = self.options["default_message"]
        return NodeMsg.IDLE

    def _do_tick(self):
        """Called periodically to publish messages."""
        # Check if the input message has been updated
        if self.inputs.is_updated("message"):
            msg = self.inputs["message"]
        else:
            # Use default message if no input is provided
            msg = self.options["topic_type"](**self._default_msg)

        # Publish the message
        self._publisher.publish(msg)
        rospy.loginfo(f"Published message to {self.options['topic_name']}: {msg}")

        # Sleep for the specified rate
        self._rate.sleep()
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        """Shutdown the publisher and clean up resources."""
        try:
            if self._publisher is not None:
                self._publisher.unregister()
                rospy.loginfo(f"Unregistered publisher on {self.options['topic_name']}")
        except AttributeError:
            self.logwarn("Cannot unregister as no publisher is available.")
        self._publisher = None

    def _do_reset(self):
        """Reset the node to its initial state."""
        return NodeMsg.IDLE

    def _do_untick(self):
        """Called when the node is unticked."""
        return NodeMsg.IDLE
