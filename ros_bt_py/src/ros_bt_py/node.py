from contextlib import contextmanager
from copy import deepcopy

import importlib
import jsonpickle
import re

import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData as NodeDataMsg
from ros_bt_py_msgs.msg import NodeDataLocation, NodeDataWiring
from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException, NodeStateError, NodeConfigError
from ros_bt_py.node_data import NodeData, NodeDataMap
from ros_bt_py.node_config import NodeConfig, OptionRef


def _required(meth):
    """Mark a method as required.

    Not using :module:`abc` here because a subclass with missing
    methods could still be instantiated and used, just not as part of
    a BT.
    """
    meth._required = True
    return meth


def define_bt_node(node_config):
    """Provide information about this Node's interface

    Every class that derives, directly or indirectly, from :class:`Node`,
    must be decorated with this!

    :param NodeConfig node_config:

    This describes your Node's interface. All inputs, outputs and
    options defined here are automatically registered with your
    class. You should not need to register anything manually!
    """
    def inner_dec(node_class):
        if not issubclass(node_class, Node):
            rospy.logerr(
                'Class %s is not a subclass of Node, cannot apply define_bt_node decorator!',
                node_class.__name__)
            raise TypeError()

        # Merge supplied node config with those of base classes
        for base in node_class.__bases__:
            if hasattr(base, '_node_config') and base._node_config:
                node_config.extend(base._node_config)
        node_class._node_config = node_config

        # Find unimplemented required methods
        missing_methods = set()
        for member in dir(node_class):
            if getattr(getattr(node_class, member, None), '_required', False):
                missing_methods.add(member)

        if missing_methods:
            # Don't register the class if it doesn't implement all required
            # methods
            rospy.logdebug('Assigned NodeData to class %s, but did not register '
                           'the class because it does not implement all required methods. '
                           'Missing methods: %s',
                           node_class.__name__, str(missing_methods))
            return node_class

        if node_class.__module__ not in Node.node_classes:
            Node.node_classes[node_class.__module__] = {
                node_class.__name__: node_class
                }
        else:
            Node.node_classes[node_class.__module__][node_class.__name__] = node_class
        return node_class
    return inner_dec


class NodeMeta(type):
    """Override the __doc__ property to add a list of BT params

    (inputs, outputs and options) to every node class.
    """
    def __new__(cls, name, bases, attrs):
        attrs['_doc'] = attrs.get('__doc__', '')
        return super(NodeMeta, cls).__new__(cls, name, bases, attrs)

    @property
    def __doc__(self):
        if hasattr(self, '_node_config') and self._node_config is not None and \
           (self._node_config.inputs or self._node_config.outputs or self._node_config.options):
            # Build table of inputs, outputs and options
            # Start with two newlines to separate from the original docstring
            param_table = ['\n\n'
                           '**Behavior Tree I/O keys**\n\n']
            if self._node_config.options:
                param_table.append('*Options*\n\n')
                for option_key in self._node_config.options:
                    param_table.append(
                        '* %s: :class:`%s`\n' %
                        (option_key,
                         self._node_config.options[option_key].__name__))
                param_table.append('\n')
            if self._node_config.inputs:
                param_table.append('*Inputs*\n\n')
                for input_key in self._node_config.inputs:
                    if isinstance(self._node_config.inputs[input_key], OptionRef):
                        param_table.append(
                            '* %s: ``%s``\n' %
                            (input_key,
                             str(self._node_config.inputs[input_key])))
                    else:
                        param_table.append(
                            '* %s: :class:`%s`\n' %
                            (input_key,
                             self._node_config.inputs[input_key].__name__))
                param_table.append('\n')
            if self._node_config.outputs:
                param_table.append('*Outputs*\n\n')
                for output_key in self._node_config.outputs:
                    if isinstance(self._node_config.outputs[output_key], OptionRef):
                        param_table.append(
                            '* %s: ``%s``\n' %
                            (output_key,
                             str(self._node_config.outputs[output_key])))
                    else:
                        param_table.append(
                            '* %s: :class:`%s`\n' %
                            (output_key,
                             self._node_config.outputs[output_key].__name__))
                param_table.append('\n')

            return self._doc + ''.join(param_table)
        else:
            return self._doc


class Node(object):
    """Base class for Behavior Tree nodes

    Each node has a set of inputs, outputs and options. At every tick
    (usually somewhere between 10 and 30 times a second),
    :meth:`tick` is called with the appropriate data.

    Nodes in a behavior Tree can be roughly divided into two classes,
    with two sub-classes each:

    Leaf Nodes
      These do not have any children and can take one of two forms:
      *Predicates* and *Behaviors*. *Predicates* check a condition and instantly
      return `SUCCEEDED` or `FAILED`. *Behaviors* are more involved and may
      return `RUNNING`, but should be interruptible (see :meth:`untick`).

    Inner Nodes
      These too come in two flavors: *Combiners* and *Decorators*. *Combiners*
      have multiple children and decide which of those children to run (and in
      what fashion) based on some criteria. *Decorators* however have only a
      single child and work with that child's result - for instance, a *Decorator*
      could invert `FAILED` into `SUCCEEDED`.
    """
    __metaclass__ = NodeMeta

    @contextmanager
    def _dummy_report_tick(self):
        self.loginfo('Ticking without debug manager')
        yield

    node_classes = {}
    _node_config = None

    def __init__(self, options=None, debug_manager=None, name=None):
        """Prepare class members

        After this finishes, the Node is *not* ready to run. You still
        need to do your own initialization in :meth:`_do_setup`.

        :param dict options:

        Map from option names to option values. Use these for
        configuring your node, do not provide a custom `__init()__`
        method!

        :param ros_bt_py.debug_manager.DebugManager debug_manager:

        :param str name:

        Name of the node - defaults to None, in which case the node name
        is set to the name of its class.

        :raises: NodeConfigError

        If anything is wrong with the node configuration defined via
        :function:`ros_bt_py.node.define_bt_node`
        """
        if name is not None:
            self.name = name
        else:
            self.name = type(self).__name__
        # Only used to make finding the root of the tree easier
        self.parent = None
        self.state = NodeMsg.UNINITIALIZED
        self.children = []

        self.subscriptions = []
        self.subscribers = []

        self.debug_manager = debug_manager

        if not self._node_config:
            raise NodeConfigError('Missing node_config, cannot initialize!')

        # Copy the class NodeConfig so we can mess with it (but we
        # only should in very rare cases!)
        self.node_config = deepcopy(self._node_config)

        self.options = NodeDataMap(name='options')
        self._register_node_data(source_map=self.node_config.options,
                                 target_map=self.options,
                                 allow_ref=False)

        # Set options from constructor parameter
        unset_option_keys = []
        for key in self.options:
            if options is None or key not in options:
                unset_option_keys.append(key)
            else:
                self.options[key] = options[key]
        if unset_option_keys:
            raise ValueError('Missing options: %s'
                             % str(unset_option_keys))

        self.inputs = NodeDataMap(name='inputs')
        self._register_node_data(source_map=self.node_config.inputs,
                                 target_map=self.inputs,
                                 allow_ref=True)

        self.outputs = NodeDataMap(name='outputs')
        self._register_node_data(source_map=self.node_config.outputs,
                                 target_map=self.outputs,
                                 allow_ref=True)

        # Don't setup automatically - nodes should be available as pure data
        # containers before the user decides to call setup() themselves!

    def setup(self):
        """Prepare the node to be ticked for the first time.

        This is called after all the input, output and option values
        have been registered (and in the case of options, populated), so
        you can use those values in your implementation of
        :meth:`_do_setup`

        Sets the state of the node to IDLE.

        :raises: BehaviorTreeException if called when the node is not UNINITIALIZED or SHUTDOWN
        """
        if self.state != NodeMsg.UNINITIALIZED and self.state != NodeMsg.SHUTDOWN:
            raise BehaviorTreeException(
                'Calling setup() is only allowed in states %s and %s, but node %s is in state %s'
                % (NodeMsg.UNINITIALIZED, NodeMsg.SHUTDOWN, self.name, self.state))
        self._do_setup()
        self.state = NodeMsg.IDLE
        self._setup_called = True

    @_required
    def _do_setup(self):
        """Use this to do custom node setup.

        Note that this will be called once, when the tree is first
        started, before the first call of :meth:`tick`.

        :rtype: basestring
        :returns:
          Nothing. If this method doesn't raise an exception, the node state will be
          IDLE afterwards.
        """

        msg = ('Trying to setup a node of type %s without _do_setup function!'
               % self.__class__.__name__)
        self.logerr(msg)
        raise NotImplementedError(msg)

    def _handle_inputs(self):
        """Execute the callbacks registered by :meth:`_wire_input`

        But only if an input has been updated since the last tick.

        :raises: ValueError
          If any input is unset
        """
        for input_name in self.inputs:
            if not self.inputs.is_updated(input_name):
                self.logwarn('Running tick() with stale data!')
            if self.inputs[input_name] is None:
                raise ValueError('Trying to tick a node with an unset input (%s)!' %
                                 input_name)
        self.inputs.handle_subscriptions()

    def _handle_outputs(self):
        """Execute the callbacks registered by :meth:`NodeDataMap.subscribe`:

        But only if the output has changed during this tick (see where
        the :meth:`NodeDataMap.reset_updated` is called in
        :meth:`tick`)
        """
        self.outputs.handle_subscriptions()

    def tick(self):
        """This is called every tick (ticks happen at ~10-20Hz, usually.

        You should not need to override this method, but instead
        implement :meth:`_do_tick` in your own class.

        :returns:
          The state of the node after ticking - should be `SUCCEEDED`, `FAILED` or `RUNNING`.

        :raises: BehaviorTreeException if a tick is impossible / not allowed
        """
        report_tick = self._dummy_report_tick()
        if self.debug_manager:
            report_tick = self.debug_manager.report_tick(self)

        with report_tick:
            if self.state is NodeMsg.UNINITIALIZED:
                raise BehaviorTreeException('Trying to tick uninitialized node!')

            unset_options = []
            for option_name in self.options:
                if not self.options.is_updated(option_name):
                    unset_options.append(option_name)
            if unset_options:
                msg = 'Trying to tick node with unset options: %s' % str(unset_options)
                self.logerr(msg)
                raise BehaviorTreeException(msg)
            self.options.handle_subscriptions()

            # Outputs are updated in the tick. To catch that, we need to reset here.
            self.outputs.reset_updated()

            # Inputs can override options!
            self._handle_inputs()

            self.state = self._do_tick()

            # Inputs are updated by other nodes' outputs, i.e. some time after
            # we use them here. In some cases, inputs might be connected to
            # child outputs (or even our own). If they are, update information
            # is lost, unless it is processed after all child ticks in the same
            # cycle!
            self.inputs.reset_updated()

            self.raise_if_in_invalid_state(allowed_states=[NodeMsg.RUNNING,
                                                           NodeMsg.SUCCEEDED,
                                                           NodeMsg.FAILED],
                                           action_name='tick()')
            self._handle_outputs()

            return self.state

    def raise_if_in_invalid_state(self, allowed_states, action_name):
        """Raise an error if `self.state` is not in `allowed_states`"""
        if self.state not in allowed_states:
            raise NodeStateError('Node %s (%s) was in invalid state %s after action %s. '
                                 'Allowed states: %s' % (
                                     self.name,
                                     type(self).__name__,
                                     self.state,
                                     action_name,
                                     str(allowed_states)))

    @_required
    def _do_tick(self):
        """
        Every Node class must override this.

        This method should **NOT** block, ever, and return one of the
        constants from `Node.Status`.

        :returns:
          One of the constants in :class:`ros_bt_py_msgs.msg.Node`
        """
        msg = ('Ticking a node of type %s without _do_tick function!'
               % self.__class__.__name__)
        self.logerr(msg)
        raise NotImplementedError(msg)

    def untick(self):
        """Calling this method signals to a node that it should stop any background tasks.

        A new tick has started and this node has **not** been ticked.
        The node's state should be `IDLE` after calling this.

        The node's outputs' `updated` flags are also reset!

        A class inheriting from :class:`Node` should override :meth:`_do_untick` instead of this!

        :raises: BehaviorTreeException
          When trying to untick a node that has not been initialized yet.
        """
        if self.state is NodeMsg.UNINITIALIZED:
            raise BehaviorTreeException('Trying to untick uninitialized node!')
        self.state = self._do_untick()
        self.raise_if_in_invalid_state(allowed_states=[NodeMsg.IDLE,
                                                       NodeMsg.PAUSED],
                                       action_name='untick()')

        self.outputs.reset_updated()

    @_required
    def _do_untick(self):
        """This is called by :meth:`untick` - override it!

        After executing this method, your node should:

        1. Be in the IDLE or PAUSED state, unless an error happened
        2. Not execute any of its behavior in the background
        3. Be ready to resume on the next call of :meth:`tick`
        """
        msg = ('Unticking a node of type %s without _do_untick function!'
               % self.__class__.__name__)
        self.logerr(msg)
        raise NotImplementedError(msg)

    def reset(self):
        """Use this to reset a node completely

        Whereas :meth:`untick` / :meth:`_do_untick` only pauses
        execution, ready to be resumed, :meth:`reset` means returning
        to the same state the node was in right after calling :meth:`setup`

        :raises: BehaviorTreeException
          When trying to reset a node that hasn't been initialized yet
        """
        if self.state is NodeMsg.UNINITIALIZED:
            raise BehaviorTreeException('Trying to reset uninitialized node!')
        self.state = self._do_reset()
        self.raise_if_in_invalid_state(allowed_states=[NodeMsg.IDLE],
                                       action_name='reset()')

    @_required
    def _do_reset(self):
        """
        This is called to reset the node to its initial state.

        After executing this method, your node should:

        1. Be in the IDLE state
        2. Not be doing anything in the background
        3. On the next tick, behave as if it has just been created

        :returns:
          The new state of the node (should be IDLE unless an error happened)
        """
        msg = ('Resetting a node of type %s without _do_reset function!'
               % self.__class__.__name__)
        self.logerr(msg)
        raise NotImplementedError(msg)

    def shutdown(self):
        """Should be called before deleting a node.

        This method just calls :meth:`_do_shutdown`, which any
        subclass must override.

        This gives the node a chance to clean up any resources it might
        be holding before getting deleted.

        :meth:`_do_shutdown` will not be called if the node has not been initialized yet.
        """
        if self.state != NodeMsg.SHUTDOWN:
            self._do_shutdown()
            self.state = NodeMsg.SHUTDOWN
        else:
            self.logwarn('Shutdown called twice')

        unshutdown_children = ['%s (%s), state: %s' % (child.name,
                                                       type(child).__name__,
                                                       child.state)
                               for child in self.children
                               if child.state != NodeMsg.SHUTDOWN]
        if unshutdown_children:
            self.logwarn('Not all children are shut down after calling shutdown(). '
                         'List of not-shutdown children and states:\n' +
                         '\n'.join(unshutdown_children))

    @_required
    def _do_shutdown(self):
        """This is called before destroying the node.

        Implement this in your node class and release any resources you
        might be holding (file pointers, ROS topic subscriptions etc.)
        """
        msg = ('Shutting down a node of type %s without _do_shutdown function!'
               % self.__class__.__name__)
        self.logerr(msg)
        raise NotImplementedError(msg)

    def calculate_utility(self):
        """Calculate the utility bounds for this node.

        Unlike the other node functions, there is a default
        implementation for the corresponding method,
        :meth:`Node._do_calculate_utility()`.

        However, in order to get meaningful results, one should take
        care to use as many nodes as possible that provide their own
        implementation, since the default only reports that there are no
        bounds.
        """
        return self._do_calculate_utility()

    def _do_calculate_utility(self):
        return UtilityBounds(has_lower_bound_success=False,
                             has_upper_bound_success=False,
                             has_lower_bound_failure=False,
                             has_upper_bound_failure=False)

    def validate(self):
        """You must also override this.

        Called after constructing and wiring up a node to check that
        everything is in proper working order.

        Errors to check for:

        1. if any child input hasn't been wired, that's a
           **fatal** error
        2. if a child input has been wired to an output of a child that
           may execute after it (or never), that's a **warning**
        3. any unconnected outputs are logged for possible **debugging**
        """
        pass

    def get_child_index(self, child_name):
        """Get the index in the `children` array of the child with the given
        name.

        This is useful if you want to replace a child with another node.

        :returns:

        An integer index if a child with the given name exists, `None`
        if there's no such child

        """
        try:
            return [child.name for child in self.children].index(child_name)
        except ValueError:
            return None

    def add_child(self, child, at_index=None):
        """Add a child to this node at the given index

        :raises: BehaviorTreeException, KeyError

        `BehaviorTreeException` if the number of children after the add operation
        would exceed the maximum number of children, `KeyError` if a
        child of the same name already exists
        """
        if (self.node_config.max_children is not None and
                len(self.children) == self.node_config.max_children):
            error_msg = ('Trying to add child when maximum number of '
                         'children (%d) is already present'
                         % self.node_config.max_children)
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        if child.name in (child.name for child in self.children):
            raise KeyError('Already have a child with name "%s"' % child.name)
        if at_index is None:
            at_index = len(self.children)

        if at_index < 0:
            at_index += len(self.children) + 1
        # Use array slicing to efficiently insert child at the correct position
        # (the value we assign needs to be a list for this to work)
        self.children[at_index:at_index] = [child]
        child.parent = self

        # return self to allow chaining of addChild calls
        return self

    def move_child(self, child_name, new_index):
        """Move the child with the name `child_name` to `new_index`

        :raises:

        IndexError if `new_index` is invalid, KeyError if no child
        with the name `child_name` exists

        """
        old_index = self.get_child_index(child_name)
        if old_index is None:
            raise KeyError('Node %s has no child named "%s"' % (self.name, child_name))
        self.children[old_index], self.children[new_index] = \
            self.children[new_index], self.children[old_index]

    def remove_child(self, child_name):
        """Remove the child with the given name and return it.

        :param basestring child_name: The name of the child to remove

        :rtype: Node
        :returns: The child that was just removed
        :raises: KeyError if no child with that name exists
        """

        child_index = self.get_child_index(child_name)
        if child_index is None:
            raise KeyError('Node %s has no child named "%s"' % (self.name, child_name))

        tmp = self.children[child_index]
        del self.children[child_index]
        tmp.parent = None
        return tmp

    def _register_node_data(self, source_map, target_map, allow_ref):
        """Register a number of typed :class:`NodeData` in the given map

        :param dict(str, type) source_map: a dictionary mapping from data keys to data types,
        i.e. ``{ 'a_string' : str, 'an_int' : int }``

        :param NodeDataMap target_map:
        The :class:`NodeDataMap` to add :class:`NodeData` values to

        :param bool allow_ref:
        Decides whether :class:`OptionRef` is accepted as a type. If True, the
        option keys referenced by any :class:`OptionRef` objects must
        exist and be populated!

        :raises: NodeConfigError in any of the following cases:
          * If any of the keys in `source_map` already exist in `target_map`
          * If an OptionRef value is passed, but `allow_ref` is `False`
          * If an OptionRef references an option value that has not been set or
            does not exist
          * If an OptionRef references an option value that does not hold a `type`
        """
        for key, data_type in source_map.iteritems():
            if key in target_map:
                raise NodeConfigError('Duplicate output name: %s' % key)

            if isinstance(data_type, OptionRef):
                if not allow_ref:
                    raise NodeConfigError('OptionRef not allowed while adding to %s' %
                                          target_map.name)
                if data_type.option_key not in self.options:
                    raise NodeConfigError('OptionRef for %s key "%s" references invalid '
                                          'option key "%s"' %
                                          (target_map.name, key, data_type.option_key))
                if not self.options.is_updated(data_type.option_key):
                    raise NodeConfigError('OptionRef for %s key "%s" references unwritten '
                                          'option key "%s"' %
                                          (target_map.name, key, data_type.option_key))
                if not isinstance(self.options[data_type.option_key], type):
                    raise NodeConfigError('OptionRef for %s key "%s" references option key '
                                          '"%s" that does not contain a type!' %
                                          (target_map.name, key, data_type.option_key))
                target_map.add(key, NodeData(data_type=self.options[data_type.option_key]))
            else:
                target_map.add(key, NodeData(data_type=data_type))

    def __repr__(self):
        return '%s(options=%r, name=%r), parent_name:%r, state:%r, inputs:%r, outputs:%r, children:%r' % (
            type(self).__name__,
            {key: self.options[key] for key in self.options},
            self.name,
            self.parent.name if self.parent else '',
            self.state,
            self.inputs,
            self.outputs,
            self.children)

    def __eq__(self, other):
        return (self.name == other.name and
                self.parent == other.parent and
                self.state == other.state and
                type(self).__module__ == type(other).__module__ and
                type(self).__name__ == type(other).__name__ and
                self.options == other.options and
                self.inputs == other.inputs and
                self.outputs == other.outputs and
                self.children == other.children)

    def __ne__(self, other):
        return not self == other

    def get_data_map(self, data_kind):
        """Return one of our NodeDataMaps by string name

        :param basestring data_kind:
          One of the constants in :class:`ros_bt_py_msgs.msg.NodeDataLocation`

        :rtype: NodeDataMap
        """
        if data_kind == NodeDataLocation.INPUT_DATA:
            return self.inputs
        if data_kind == NodeDataLocation.OUTPUT_DATA:
            return self.outputs
        if data_kind == NodeDataLocation.OPTION_DATA:
            return self.options

        raise KeyError('%s is not a valid value to pass to Node.get_data_map()!' % data_kind)

    # Logging methods - these just use the ROS logging framework, but add the
    # name and type of the node so it's easier to trace errors.

    def logdebug(self, message):
        """Wrapper for :func:rospy.logdebug

        Adds this node's name and type to the given message"""
        rospy.logdebug('%s (%s): %s', self.name, type(self).__name__, message)

    def loginfo(self, message):
        """Wrapper for :func:rospy.loginfo

        Adds this node's name and type to the given message"""
        rospy.loginfo('%s (%s): %s', self.name, type(self).__name__, message)

    def logwarn(self, message):
        """Wrapper for :func:rospy.logwarn

        Adds this node's name and type to the given message"""
        rospy.logwarn('%s (%s): %s', self.name, type(self).__name__, message)

    def logerr(self, message):
        """Wrapper for :func:rospy.logerr

        Adds this node's name and type to the given message"""
        rospy.logerr('%s (%s): %s', self.name, type(self).__name__, message)

    def logfatal(self, message):
        """Wrapper for :func:rospy.logfatal

        Adds this node's name and type to the given message"""
        rospy.logfatal('%s (%s): %s', self.name, type(self).__name__, message)

    @classmethod
    def from_msg(cls, msg):
        """Construct a Node from the given ROS message.

        This will try to import the requested node class, instantiate it
        and populate its `name`, `options`, `input` and `output` members
        from the ROS message.

        :param ros_bt_py_msgs.msg.Node msg:

        A ROS message describing a node class. The node class must be
        available in the current environment (but does not need to be
        imported before calling this).

        :returns:

        An instance of the class named by `msg`, populated with the
        values from `msg`.

        Note that this does *not* include the node's state. Any node
        created by this will be in state UNININITIALIZED.

        :raises:

        BehaviorTreeException if
        node cannot be instantiated.
        """
        if (msg.module not in cls.node_classes or
                msg.node_class not in cls.node_classes[msg.module]):
            # If the node class was not available, try to load it
            load_node_module(msg.module)

        # If loading didn't work, abort
        if (msg.module not in cls.node_classes or
                msg.node_class not in cls.node_classes[msg.module]):
            raise BehaviorTreeException(
                'Failed to instantiate node from message - node class '
                'not available. Original message:\n%s' % str(msg))

        node_class = cls.node_classes[msg.module][msg.node_class]

        # Populate options dict
        options_dict = {}
        try:
            for option in msg.options:
                options_dict[option.key] = jsonpickle.decode(option.serialized_value)
        except ValueError, e:
            raise BehaviorTreeException('Failed to instantiate node from message: %s' %
                                        str(e))

        # Instantiate node - this shouldn't do anything yet, since we don't
        # call setup()
        node_instance = node_class(options=options_dict)

        # Set name from ROS message
        if msg.name:
            node_instance.name = msg.name

        # Set inputs
        try:
            for input_msg in msg.inputs:
                node_instance.inputs[input_msg.key] = jsonpickle.decode(
                    input_msg.serialized_value)
        except ValueError, e:
            raise BehaviorTreeException('Failed to instantiate node from message: %s' %
                                        str(e))

        # Set outputs
        try:
            for output_msg in msg.outputs:
                node_instance.outputs[output_msg.key] = jsonpickle.decode(
                    output_msg.serialized_value)
        except ValueError, e:
            raise BehaviorTreeException('Failed to instantiate node from message: %s' %
                                        str(e))

        return node_instance

    def get_children_recursive(self):
        yield self
        for child in self.children:
            for x in child.get_children_recursive():
                yield x

    def get_subtree_msg(self):
        """Populate a TreeMsg with the subtree rooted at this node

        This can be used to "shove" a subtree to a different host, by
        using that host's load_tree service.

        The subtree message will have public node data for every piece
        of node data that is wired to a node outside the subtree.

        :returns:

        A tuple consisting of a :class:`ros_bt_py_msgs.msg.Tree` message and a
        list of :class:`ros_bt_py_msgs.msg.NodeDataWiring` messages
        (external_connections). The latter can be used to determine what
        parameters need to be forwarded to / from the remote executor if
        the subtree is to be executed remotely.

        Crucially, the resulting subtree will not be tick-able until all
        the incoming wirings from external_connections have been
        connected.
        """
        subtree_name = "%s_subtree" % self.name
        subtree = Tree(name=subtree_name,
                       root_name=self.name,
                       nodes=[node.to_msg() for node in self.get_children_recursive()],
                       state=Tree.IDLE)

        node_map = {node.name: node for node in subtree.nodes}
        external_connections = []
        for node in self.get_children_recursive():
            for sub in node.subscriptions:
                source_node = node_map.get(sub.source.node_name)
                target_node = node_map.get(sub.target.node_name)

                # For subscriptions where source and target are in the subtree,
                # add a wiring.
                if source_node and target_node:
                    subtree.data_wirings.append(NodeDataWiring(
                        source=sub.source,
                        target=sub.target))
                # In the other cases, add that datum to public_node_data
                elif source_node:
                    subtree.public_node_data.append(sub.source)
                    external_connections.append(sub)
                elif target_node:
                    subtree.public_node_data.append(sub.target)
                    external_connections.append(sub)
                else:
                    raise BehaviorTreeException(
                        "Subscription in subtree has source *AND* target "
                        "outside of subtree!")

            for wiring, _, _ in node.subscribers:
                if wiring.target.node_name not in node_map:
                    subtree.public_node_data.append(wiring.source)
                    external_connections.append(wiring)

        # TODO(berg): Maybe also make the currently unconnected inputs of all
        # nodes publicly available by default?

        return subtree, external_connections

    def find_node(self, other_name):
        """Try to find the node with the given name in the tree

        This is not a particularly cheap operation, since it ascends
        the tree up to the root and then recursively descends back
        until it finds the node.  Probably best not to use it in a
        tick function.
        """
        root = self
        while root.parent is not None:
            root = root.parent

        for node in root.get_children_recursive():
            if node.name == other_name:
                return node

        return None

    def _subscribe(self, wiring, cb, expected_type):
        """Subscribe to a piece of Nodedata this node has.

        Call this on a node to *subscribe to NodeData **from** that
        node*!

        :param `ros_bt_py_msgs.msg.NodeDataWiring` wiring:

        Defines the source and target of the subscribe operation.
        `wiring.source` must point to a valid data key in this node
        (`self`).  `wiring.target` is not checked here (because it's
        only used to label subscriptions), but it should also point to a
        valid data key of a node in the tree.

        :param cb:

        A callback function that will be called whenever there's an updated
        value for the key (and this node receives a tick)

        :param `type` expected_type:

        The type that the subscriber expects our piece of data to
        have. If it doesn't match the type of the data at the requested
        key, raise a `BehaviorTreeException`.

        :raises:

        KeyError if the requested data location is not in this node, or
        the requested key does not exist in this node.

        BehaviorTreeException if `expected_type` and the actual type of
        the data are incompatible.
        """
        if wiring.source.node_name != self.name:
            raise KeyError('%s: Trying to subscribe to another node (%s)' %
                           (self.name, source.node_name))

        for sub, cb, cb_type in self.subscribers:
            if sub.target == wiring.target:
                if sub.source == wiring.source:
                    raise BehaviorTreeException('Duplicate subscription!')
                self.logwarn(
                    'Subscriber %s is subscribing to multiple sources with the same target %s[%s]'
                    % (wiring.target.node_name,
                       wiring.target.data_kind,
                       wiring.target.data_key))

        source_map = self.get_data_map(wiring.source.data_kind)

        if wiring.source.data_key not in source_map:
            raise KeyError('Source key %s.%s[%s] does not exist!' % (
                self.name,
                wiring.source.data_kind,
                wiring.source.data_key))

        if not issubclass(source_map.get_type(wiring.source.data_key),
                          expected_type):
            raise BehaviorTreeException((
                'Type of %s.%s[%s] (%s) is not compatible with '
                'Type of %s.%s[%s] (%s)!' % (
                    self.name,
                    wiring.source.data_kind,
                    wiring.source.data_key,
                    source_map.get_type(wiring.source.data_key).__name__,
                    wiring.target.node_name,
                    wiring.target.data_kind,
                    wiring.target.data_key,
                    expected_type)))

        source_map.subscribe(wiring.source.data_key, cb, '%s.%s[%s]' %
                             (wiring.target.node_name,
                              wiring.target.data_kind,
                              wiring.target.data_key))
        self.subscribers.append((deepcopy(wiring), cb, expected_type))

    def wire_data(self, wiring):
        """Wire a piece of Nodedata from another node to this node.

        Call this on a node to *connect it to NodeData from
        **another** node*!

        :param `ros_bt_py_msgs.msg.NodeDataWiring` wiring:

        Indicates both the piece of NodeData we want to subscribe to
        (`wiring.source`) and the data key inside this node we want to
        connect it to (`wiring.target`).

        If we cannot find a node named `wiring.source.node_name`, or the location is
        otherwise invalid, this will raise a KeyError.

        :param `ros_bt_py_msgs.msg.NodeDataLocation` target:
           The position *inside this node* we want to wire *to*.

        :raises:

        KeyError if either the source or target location is invalid.

        BehaviorTreeException if a subscription with the same source and
        target exists already, or if the types of source and target data
        are incompatible.
        """
        if wiring.target.node_name != self.name:
            raise BehaviorTreeException('Target of wiring (%s) is not this node (%s)' % (
                wiring.target.node_name,
                self.name))

        for sub in self.subscriptions:
            if sub.target == wiring.target:
                if sub.source == wiring.source:
                    raise BehaviorTreeException('Duplicate subscription!')
                self.logwarn('Subscribing to two different sources for key %s[%s]'
                             % (wiring.target.data_kind, wiring.target.data_key))
        source_node = self.find_node(wiring.source.node_name)
        if not source_node:
            raise BehaviorTreeException(
                'Source node %s does not exist or is not connected to target node %s' % (
                    wiring.source.node_name,
                    self.name))
        try:
            source_map = source_node.get_data_map(wiring.source.data_kind)
        except KeyError as exception:
            raise BehaviorTreeException(str(exception))

        if wiring.source.data_key not in source_map:
            raise KeyError('Source key %s.%s[%s] does not exist!' % (
                source_node.name,
                wiring.source.data_kind,
                wiring.source.data_key))

        try:
            target_map = self.get_data_map(wiring.target.data_kind)
        except KeyError as exception:
            raise BehaviorTreeException(str(exception))

        if wiring.target.data_key not in target_map:
            raise KeyError('Target key %s.%s[%s] does not exist!' % (
                self.name,
                wiring.target.data_kind,
                wiring.target.data_key))

        source_node._subscribe(wiring,
                               target_map.get_callback(wiring.target.data_key),
                               target_map.get_type(wiring.target.data_key))

        self.subscriptions.append(deepcopy(wiring))

    def _unsubscribe(self, wiring):
        """Unsubscribe from a piece of NodeData this node has.

        Call this to undo a call to `Node._subscribe()'

        :raises:

        KeyError if the requested data location is not in this node, or
        the requested key does not exist in this node.

        """
        if wiring.source.node_name != self.name:
            raise KeyError('%s: Trying to unsubscribe from another node (%s)' %
                           (self.name, source.node_name))
        source_map = self.get_data_map(wiring.source.data_kind)

        if wiring.source.data_key not in source_map:
            raise KeyError('Source key %s.%s[%s] does not exist!' % (
                self.name,
                wiring.source.data_kind,
                wiring.source.data_key))

        for sub_wiring, cb, _ in self.subscribers:
            if wiring.target == sub_wiring.target:
                source_map.unsubscribe(wiring.source.data_key, cb)
        # remove subscriber data from list
        self.subscribers = [sub for sub in self.subscribers
                            if sub[0].target != wiring.target]

    def unwire_data(self, wiring):
        """Unwire the given wiring.

        This entails finding the source of the wiring, calling its
        :meth:`Node._unsubscribe` method and removing the wiring from
        this node's list of subscriptions.

        :raises: BehaviorTreeException

        If the given wiring's source node cannot be found from this
        node.
        """
        if wiring.target.node_name != self.name:
            raise KeyError('Target of wiring (%s) is not this node (%s)' % (
                wiring.target.node_name,
                self.name))
        source_node = self.find_node(wiring.source.node_name)
        if not source_node:
            raise BehaviorTreeException(
                'Source node %s does not exist or is not connected to target node %s' % (
                    wiring.source.node_name,
                    self.name))

        if wiring not in self.subscriptions:
            # Nothing to do
            return
        source_node._unsubscribe(wiring)
        self.subscriptions.remove(wiring)

    def to_msg(self):
        """Populate a ROS message with the information from this Node

        Round-tripping the result through :meth:`Node.from_msg` should
        yield a working node object, with the caveat that state will not
        be preserved.

        :rtype: ros_bt_py_msgs.msg.Node
        :returns:

        A ROS message that describes the node.
        """
        node_type = type(self)
        return NodeMsg(is_subtree=False,
                       module=node_type.__module__,
                       node_class=node_type.__name__,
                       name=self.name,
                       child_names=[child.name for child in self.children],
                       options=[NodeDataMsg(key=key,
                                            serialized_value=jsonpickle.encode(
                                                self.options[key]),
                                            serialized_type=jsonpickle.encode(
                                                self.options.get_type(key)))
                                for key in self.options],
                       inputs=[NodeDataMsg(key=key,
                                           serialized_value=jsonpickle.encode(
                                               self.inputs[key]),
                                           serialized_type=jsonpickle.encode(
                                               self.inputs.get_type(key)))
                               for key in self.inputs],
                       outputs=[NodeDataMsg(key=key,
                                            serialized_value=jsonpickle.encode(
                                                self.outputs[key]),
                                            serialized_type=jsonpickle.encode(
                                                self.outputs.get_type(key)))
                                for key in self.outputs],
                       max_children=(self.node_config.max_children
                                     if self.node_config.max_children is not None
                                     else -1),
                       state=self.state)


def load_node_module(package_name):
    """Import the named module at run-time.

    If the module contains any (properly decorated) node classes,
    they will be registered and available to load via the other
    commands in this class.
    """
    try:
        return importlib.import_module(package_name)
    except (ImportError, ValueError):
        return None


def increment_name(name):
    """If `name` does not already end in a number, add "_2" to it.

    Otherwise, increase the number after the underscore.
    """
    match = re.search('_([0-9]+)$', name)
    prev_number = 1
    if match:
        prev_number = int(match.group(1))
        # remove the entire _$number part from the name
        name = name[:len(name) - len(match.group(0))]

    name += '_%d' % (prev_number + 1)
    return name


def is_valid_node_message(msg, node_dict=None):
    """Check whether a message contains a valid message definition.

    Tries to instantiate the node using :meth:`Node.from_msg` and
    returns a tuple `(success, error_message)`.

    :param ros_bt_py_msgs.msg.Node node: The message to check

    :param dict node_dict:

    If given, check missing parent errors against this dictionary. If
    not given, those errors are ignored since of course there's no
    parent.
    """
    try:
        Node.from_msg(msg, node_dict)
        return (True, '')
    except KeyError, e:
        if node_dict is None:
            return (True, '')
        else:
            return (False, str(e))
    except BehaviorTreeException, e:
        return (False, str(e))


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=1))
class Decorator(Node):
    """Base class for Decorator nodes.

    Decorators have exactly one child and somehow modify that child's
    output. Subclasses can add inputs, outputs and options, but never
    change `max_children`.
    """
    def _do_calculate_utility(self):
        """
        Pass on the utility value of the (only allowed) child.
        """
        if self.children:
            return self.children[0].calculate_utlity()
        else:
            return UtilityBounds(has_lower_bound_success=False,
                                 has_upper_bound_success=False,
                                 has_lower_bound_failure=False,
                                 has_upper_bound_failure=False)


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=0))
class Leaf(Node):
    """Base class for leaf nodes in the tree.

    Leaf nodes have no children. Subclasses can define inputs, outputs
    and options, but never change `max_children`.
    """
    pass


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=None))
class FlowControl(Node):
    """Base class for flow control nodes

    Flow control nodes (mostly Sequence, Fallback and their derivatives)
    can have an unlimited number of children and each have a unique set
    of rules for when to tick which of their children.
    """
    pass
