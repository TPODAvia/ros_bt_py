from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import IO, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from rosbridge_library.internal.message_conversion import populate_instance, extract_values
from rosbridge_library.internal.message_conversion import InvalidMessageException
# FIXME: input/output are exact copies of each other, move this code to a common implementation


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'io_type': type,
             'default': OptionRef('io_type')},
    inputs={'in': OptionRef('io_type')},
    outputs={'out': OptionRef('io_type')},
    max_children=0,
    option_wirings=[{'source': 'io_type', 'target': 'default'}]))
class IOInputOption(IO):
    """Explicitly marks the input of a subtree.
    If no input is connected to `in`, the value provided via the `default` option is used.
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _handle_inputs(self):
        """ This overwrites the Nodes _handle_input() function, ignoring an unset 'in' input
        Execute the callbacks registered by :meth:`_wire_input`

        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == 'in' and self.inputs[input_name] is None:
                self.logwarn('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logwarn('Running tick() with stale data!')
        self.inputs.handle_subscriptions()

    def _do_tick(self):
        if self.inputs['in'] is not None:
            self.outputs['out'] = self.inputs['in']
        else:
            self.outputs['out'] = self.options['default']
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['out'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'io_type': type},
    inputs={'in': OptionRef('io_type'),
            'default': OptionRef('io_type')},
    outputs={'out': OptionRef('io_type')},
    max_children=0))
class IOInput(IO):
    """Explicitly marks the input of a subtree.
    If no input is connected to `in`, the value provided via the `default` input is used.
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _handle_inputs(self):
        """ This overwrites the Nodes _handle_input() function, ignoring an unset 'in' input
        Execute the callbacks registered by :meth:`_wire_input`

        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == 'in' and self.inputs[input_name] is None:
                self.logwarn('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logwarn('Running tick() with stale data!')
                if self.inputs[input_name] is None:
                    raise ValueError('Trying to tick a node with an unset input (%s)!' %
                                     input_name)
        self.inputs.handle_subscriptions()

    def _do_tick(self):
        if self.inputs['in'] is not None:
            self.outputs['out'] = self.inputs['in']
        else:
            self.outputs['out'] = self.inputs['default']
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['out'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'io_type': type,
             'default': OptionRef('io_type')},
    inputs={'in': OptionRef('io_type')},
    outputs={'out': OptionRef('io_type')},
    max_children=0,
    option_wirings=[{'source': 'io_type', 'target': 'default'}]))
class IOOutputOption(IO):
    """Explicitly marks the output of a subtree.
    If no input is connected to `in`, the value provided via the `default` option is used.
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _handle_inputs(self):
        """ This overwrites the Nodes _handle_input() function, ignoring an unset 'in' input
        Execute the callbacks registered by :meth:`_wire_input`

        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == 'in' and self.inputs[input_name] is None:
                self.logwarn('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logwarn('Running tick() with stale data!')
        self.inputs.handle_subscriptions()

    def _do_tick(self):
        if self.inputs['in'] is not None:
            self.outputs['out'] = self.inputs['in']
        else:
            self.outputs['out'] = self.options['default']
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['out'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'io_type': type},
    inputs={'in': OptionRef('io_type'),
            'default': OptionRef('io_type')},
    outputs={'out': OptionRef('io_type')},
    max_children=0))
class IOOutput(IO):
    """Explicitly marks the output of a subtree.
    If no input is connected to `in`, the value provided via the `default` input is used.
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _handle_inputs(self):
        """ This overwrites the Nodes _handle_input() function, ignoring an unset 'in' input
        Execute the callbacks registered by :meth:`_wire_input`

        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == 'in' and self.inputs[input_name] is None:
                self.logwarn('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logwarn('Running tick() with stale data!')
                if self.inputs[input_name] is None:
                    raise ValueError('Trying to tick a node with an unset input (%s)!' %
                                     input_name)
        self.inputs.handle_subscriptions()

    def _do_tick(self):
        if self.inputs['in'] is not None:
            self.outputs['out'] = self.inputs['in']
        else:
            self.outputs['out'] = self.inputs['default']
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['out'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE
