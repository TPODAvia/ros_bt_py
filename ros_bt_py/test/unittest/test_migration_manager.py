import unittest

import jsonpickle
import sys
import time

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.srv import MigrateTreeRequest, GetAvailableNodesRequest

from ros_bt_py.testing_nodes import migrations_test_nodes

from ros_bt_py.migration import MigrationManager
from ros_bt_py.tree_manager import TreeManager


class TestMigrationManager(unittest.TestCase):
    def testSimpleMigration(self):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(
            node_modules=['ros_bt_py.testing_nodes.migrations_test_nodes',
                          'ros_bt_py.testing_nodes.migrations_test_nodes_without_migrations'])

        response = tree_manager.get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree()
        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_nodewithworkingmigrations.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_nodewithworkingmigrations2.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migration_nodewithbrokenmigrationpath.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # package, but file does not exist
        tree.path = 'package://ros_bt_py/etc/trees/notareal.file'
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = '/notareal.file'
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = 'file://'
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/etc/trees/two_trees.yaml'
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/etc/trees/empty.yaml'
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # test get/add exceptions
        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_nodewithgetoptionexception.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_nodewithaddoptionexception.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_nodewithaddinputexception.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_nodewithaddoutputexception.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_nodewithoptionrefexception.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

    def testNodesMigrations(self):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(
            node_modules=['ros_bt_py.nodes.action',
                          'ros_bt_py.nodes.constant',
                          'ros_bt_py.nodes.sequence'])

        response = tree_manager.get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree()
        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_action.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_service.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_waitforservice.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_compare.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_decorators.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_fallback.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_file.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_format.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_getters.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_io.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_log.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_log_error.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_log_warn.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_log_err.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_mock_nodes.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_parallel_if_remote.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_parallel.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_passthrough_node.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_random_number.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_remote_slot.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_ros_param.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_sequence.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_setters.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_shovable.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_subtree.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_topic.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_wait.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        # ros_nodes
        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_fields_to_message.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_message_to_fields.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_message_from_const_dict.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_message_from_dict.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

    def testCheckNodeVersions(self):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(
            node_modules=['ros_bt_py.nodes.action',
                          'ros_bt_py.nodes.constant',
                          'ros_bt_py.nodes.sequence'])

        response = tree_manager.get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree()
        # package, but file does not exist
        tree.path = 'package://ros_bt_py/etc/trees/notareal.file'
        migrate_request = MigrateTreeRequest(tree=tree)
        migrate_reply = migration_manager.check_node_versions(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = '/notareal.file'
        migrate_request = MigrateTreeRequest(tree=tree)
        migrate_reply = migration_manager.check_node_versions(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = 'file://'
        migrate_request = MigrateTreeRequest(tree=tree)
        migrate_reply = migration_manager.check_node_versions(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = 'package://ros_bt_py/test/testdata/trees/' \
                    'migrations_action.yaml'
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.check_node_versions(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)