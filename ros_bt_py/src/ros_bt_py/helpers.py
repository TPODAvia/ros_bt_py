#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
#  -------- END LICENSE BLOCK --------
import sys
import jsonpickle
import logging
import rospy
import functools
from collections import OrderedDict

from ros_bt_py_msgs.msg import CapabilityInterface

from ros_bt_py.ros_helpers import EnumValue, LoggerLevel

from ros_bt_py_msgs.srv import FixYamlResponse, FixYamlRequest

try:  # pragma: no cover
    basestring
except NameError:  # pragma: no cover
    basestring = str

try:  # pragma: no cover
    unicode
except NameError:  # pragma: no cover
    unicode = str

try:  # pragma: no cover
    long
except NameError:  # pragma: no cover
    long = int


def loglevel_is(level):
    """Determine the current logging level of the default ROS logger

    Useful to guard log statements that would incur a performance
    penalty if they ran when the log isn't published.

    For easier use, this translates the `rospy` logger levels into
    `logging` levels.

    :param int level:

    The logger level to compare against. Since lower levels are more
    verbose, any level *below* `level` also returns `True`.

    """
    logging_level = logging.getLogger("rosout").getEffectiveLevel()
    return logging_level <= rospy_log_level_to_logging_log_level(level)


def rospy_log_level_to_logging_log_level(rospy_level):
    if rospy_level == rospy.DEBUG:
        return logging.DEBUG
    if rospy_level == rospy.INFO:
        return logging.INFO
    if rospy_level == rospy.WARN:
        return logging.WARN
    if rospy_level == rospy.ERROR:
        return logging.ERROR
    if rospy_level == rospy.FATAL:
        return logging.FATAL


def fix_yaml(request: FixYamlRequest) -> FixYamlResponse:
    """
    Fixes a yaml file and ensures it conforms to the expected format for ros msg de-/serializing.
    :param request: The ros service request containing the yaml file.
    :return: Always returns successfully.
    """
    response = FixYamlResponse()

    tree_yaml = request.broken_yaml

    last_index = 0

    index = 0
    search_string = "child_names: - "
    replace_string = "child_names:"
    search_len = len(search_string)
    replace_len = len(replace_string)
    while index < len(tree_yaml):
        index = tree_yaml.find(search_string, index)
        if index == -1:
            break

        # find the last linebreak and count number of spaces until child_names:
        linebreak_index = tree_yaml.rfind("\n", last_index, index)

        indent = index - linebreak_index - 1 + 2

        # now replace the search_string with the proper linebreak
        tree_yaml = (
            f"{tree_yaml[:index + replace_len]}\n{tree_yaml[index + replace_len + 1:]}"
        )

        # now check all newlines until they are not "\n- " any more
        newline_index = index + replace_len

        # update for next check
        index = index + search_len
        last_index = index

        # rospy.logerr("newline: %s" % tree_yaml[newline_index:])
        while newline_index < len(tree_yaml):
            # skip "\n"
            newline_index = newline_index + 1
            # check if the line starts with "- "
            if tree_yaml[newline_index : newline_index + 2] == "- ":
                # fix it by adding the correct indent:
                tree_yaml = (
                    tree_yaml[:newline_index] + " " * indent + tree_yaml[newline_index:]
                )
            else:
                # found no compatible newline
                break
            # find next "\n"
            newline_index = tree_yaml.find("\n", newline_index + indent + 2)

    response.success = True
    response.fixed_yaml = tree_yaml

    return response


def remove_input_output_values(tree):
    """Removes all input and output values from the tree nodes.
    This is achieved by replacing every nodes input/output serialized_value with "null"
    """
    for node in tree.nodes:
        for node_input in node.inputs:
            node_input.serialized_value = "null"
        for node_output in node.outputs:
            node_output.serialized_value = "null"
    return tree


# handling nested objects,
# see https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects
def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)

    return functools.reduce(_getattr, [obj] + attr.split("."))


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition(".")
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


def get_default_value(data_type, ros=False):
    if data_type is type:
        return int
    elif data_type is int or data_type is long:
        return 0
    elif data_type is str or data_type is basestring or data_type is unicode:
        return "foo"
    elif data_type is float:
        return 1.2
    elif data_type is bool:
        return False
    elif data_type is list:
        return []
    elif data_type is dict:
        return {}
    elif data_type is OrderedDict:
        return OrderedDict()
    elif data_type is LoggerLevel:
        return LoggerLevel()
    elif data_type is EnumValue:
        return EnumValue()
    elif ros:
        return data_type()
    else:
        return {}


def json_encode(data):
    """Wrapper for jsonpickle.encode
    Makes sure that python2 __builtin__ gets encoded as python3 builtins
    """
    return jsonpickle.encode(data).replace("builtins.", "__builtin__.")


def json_decode(data):
    """Wrapper for jsonpickle.decode
    Makes sure that python3 builtins get treated as __builtin__ in python2
    """
    if sys.version_info.major == 2:
        data = data.replace("builtins.", "__builtin__.")
    elif sys.version_info.major >= 2:
        data = data.replace("__builtin__.", "builtins.")
    return jsonpickle.decode(data)


class MathUnaryOperator(object):
    def __init__(self, operator):
        self.operator = operator


class MathBinaryOperator(object):
    def __init__(self, operator):
        self.operator = operator


class MathOperandType(object):
    def __init__(self, operand_type):
        self.operand_type = operand_type


class MathUnaryOperandType(object):
    def __init__(self, operand_type):
        self.operand_type = operand_type


class HashableCapabilityInterface:
    """
    Wrapper class to allow for the hashing of capability interfaces.
    """

    def __init__(self, interface: CapabilityInterface):
        self.interface: CapabilityInterface = interface

    def __eq__(self, other: object) -> bool:
        def compare_node_data_lists(list1: list, list2: list) -> bool:
            l1_node_data = {(x.key, json_decode(x.serialized_type)) for x in list1}
            l2_node_data = {(x.key, json_decode(x.serialized_type)) for x in list2}

            return l1_node_data == l2_node_data

        if not isinstance(other, HashableCapabilityInterface):
            return False

        return (
            (self.interface.name == other.interface.name)
            and compare_node_data_lists(self.interface.inputs, other.interface.inputs)
            and compare_node_data_lists(self.interface.outputs, other.interface.outputs)
            and compare_node_data_lists(self.interface.options, other.interface.options)
        )

    def __ne__(self, other: object) -> bool:
        return not self.__eq__(other)

    def __hash__(self) -> int:
        return hash(
            (
                self.interface.name,
                frozenset(
                    {
                        (x.key, json_decode(x.serialized_type))
                        for x in self.interface.inputs
                    }
                ),
                frozenset(
                    {
                        (x.key, json_decode(x.serialized_type))
                        for x in self.interface.outputs
                    }
                ),
                frozenset(
                    {
                        (x.key, json_decode(x.serialized_type))
                        for x in self.interface.options
                    }
                ),
            )
        )
