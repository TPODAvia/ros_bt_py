import math

import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'needed_successes': int},
    inputs={},
    outputs={},
    max_children=None))
class Parallel(FlowControl):
    """The Parallel ticks all of its children every tick

    Its success is determined depending on `needed_successes` - if at
    least `needed_success` of its children have returned SUCCEEDED,
    the Parallel too returns SUCCEEDED.

    On the other hand, if more than N - `needed_successes` of its
    children have returned FAILED, so will the Parallel.

    Different values of `needed_successes` can produce behavior
    similar to the "barrier" (wait until all actions are finished) and
    "heureka" (first success aborts all other actions) parallel
    programming primitives, and more.

    To prevent thrashing when multiple children return RUNNING for
    different lengths of time, a child that has returned SUCCEEDED or
    FAILED once will not be ticked again untill **all** children have
    reached a result.

    """
    def _do_setup(self):
        if len(self.children) < self.options['needed_successes']:
            raise BehaviorTreeException(
                ('Option value needed_successes (%d) cannot be larger than '
                 'the number of children (%d)') % (
                     self.options['needed_successes'],
                     len(self.children)))
        for child in self.children:
            child.setup()

    def _do_tick(self):
        # Just like Sequence and Fallback, reset after having returned
        # SUCCEEDED or FAILED once
        if self.state in [NodeMsg.SUCCEEDED,
                          NodeMsg.FAILED]:
            for child in self.children:
                child.reset()

        successes = 0
        failures = 0
        for child in self.children:
            if child.state not in [NodeMsg.SUCCEEDED,
                                   NodeMsg.FAILED]:
                child.tick()
            if child.state == NodeMsg.SUCCEEDED:
                successes += 1
            if child.state == NodeMsg.FAILED:
                failures += 1
        if successes >= self.options['needed_successes']:
            return NodeMsg.SUCCEEDED
        elif failures > len(self.children) - self.options['needed_successes']:
            return NodeMsg.FAILED
        return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        for child in self.children:
            child.reset()()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        if len(self.children) < self.options['needed_successes']:
            raise BehaviorTreeException(
                ('Option value needed_successes (%d) cannot be larger than '
                 'the number of children (%d)') % (
                     self.options['needed_successes'],
                     len(self.children)))

        # This calculation is fairly straightforward:
        #
        # The lower bound for success is the lowest sum of
        # "needed_successes" children's lower success bounds.
        #
        # The upper bound for success is the highest sum of
        # "needed_successes" children's upper success bounds.
        #
        # We get both by first sorting the array of bounds by the
        # respective bound (ascending for the lower bounds, descending
        # for the upper bounds) and then summing the first
        # "needed_successes" values from the sorted array.

        child_bounds = [child.calculate_utility() for child in self.children]
        bounds = UtilityBounds()
        bounds.can_execute = all((b.can_execute for b in child_bounds))

        # Early return if any child cannot execute
        if not bounds.can_execute:
            return bounds

        success_threshold = self.options['needed_successes']

        best_success_group = sorted(
            child_bounds, key=lambda b: b.lower_bound_success)[:success_threshold]
        bounds.has_lower_bound_success = all(
            (b.has_lower_bound_success for b in best_success_group))
        if bounds.has_lower_bound_success:
            bounds.lower_bound_success = math.fsum(
                (b.lower_bound_success for b in best_success_group))

        worst_success_group = sorted(
            child_bounds, reverse=True, key=lambda b: b.upper_bound_success)[:success_threshold]
        bounds.has_upper_bound_success = all(
            (b.has_upper_bound_success for b in worst_success_group))
        if bounds.has_upper_bound_success:
            bounds.upper_bound_success = math.fsum(
                (b.upper_bound_success for b in worst_success_group))

        # Now we do the same for the failures - the only difference
        # here is that failure_threshold is
        #
        # 1 + (num_children - success_threshold)
        #
        # because of the semantics of Parallel explained in the class
        # docstring

        # The minimum number of child failures for an overall Parallel failure
        failure_threshold = 1 + (len(self.children) - success_threshold)

        best_failure_group = sorted(
            child_bounds, key=lambda b: b.lower_bound_failure)[:failure_threshold]
        bounds.has_lower_bound_failure = all(
            (b.has_lower_bound_failure for b in best_failure_group))
        if bounds.has_lower_bound_failure:
            bounds.lower_bound_failure = math.fsum(
                (b.lower_bound_failure for b in best_failure_group))

        worst_failure_group = sorted(
            child_bounds, reverse=True, key=lambda b: b.upper_bound_failure)[:failure_threshold]
        bounds.has_upper_bound_failure = all(
            (b.has_upper_bound_failure for b in worst_failure_group))
        if bounds.has_upper_bound_failure:
            bounds.upper_bound_failure = math.fsum(
                (b.upper_bound_failure for b in worst_failure_group))

        return bounds