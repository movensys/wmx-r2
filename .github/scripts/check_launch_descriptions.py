#!/usr/bin/env python3
"""Evaluate the repository launch descriptions without launching anything."""

import importlib.util
import logging
import os
import sys
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

import launch.logging
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import Command, LocalSubstitution
from launch.utilities import perform_substitutions

from launch_ros.actions import LifecycleNode, Node

sys.dont_write_bytecode = True

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LAUNCH_PACKAGES = ('wmx_r2_package', 'wmx_r2_control')
SHARED_SUBDIRS = ('config', 'launch', 'urdf', 'rviz', 'meshes')

# launch loggers do not propagate to the root logger, so the collector has to
# be attached to each one that can report a problem with a description.
WATCHED_LOGGERS = (
    'launch',
    'launch_ros',
    'launch.actions',
    'launch.launch_description',
    'launch_ros.actions.node',
    'launch_ros.actions.lifecycle_node',
)

EXPECTED_LIFECYCLE_NODES = {
    'wmx_r2_general_nodes.launch.py': 3,
    'wmx_r2_cr3a_manipulator.launch.py': 4,
    'wmx_r2_cr5a_manipulator.launch.py': 3,
    'wmx_r2_diffbot_navigation.launch.py': 2,
    'wmx_r2_control_cr3a_manipulator.launch.py': 3,
    'wmx_r2_control_cr5a_manipulator.launch.py': 2,
    'wmx_r2_control_diffbot_navigation.launch.py': 0,
}


class LaunchWarningCollector(logging.Handler):
    """Record every warning or error the launch machinery emits."""

    def __init__(self):
        """Start with an empty record list."""
        super().__init__(level=logging.WARNING)
        self.records = []

    def emit(self, record):
        """Keep the message of one warning or error record."""
        self.records.append(f'{record.levelname.lower()}: {record.getMessage()}')

    def drain(self):
        """Return the collected messages and start over."""
        drained = list(self.records)
        self.records.clear()
        return drained


def stub_command_substitutions():
    """Resolve what a Command substitution wraps without running the command.

    Command runs an external process (xacro), which needs robot description
    packages that CI does not install. The substitutions inside it are still
    resolved, so a bad LaunchConfiguration in the command line is still caught.
    """
    def perform(self, context):
        perform_substitutions(context, list(self.command))
        return 'command-not-run'

    Command.perform = perform


def ensure_package_share(package):
    """Make get_package_share_directory(package) resolve, stubbing it if unbuilt.

    The launch files ask for their own share directory to build paths to
    configs, descriptions and to the general-nodes launch file. The stub links
    the in-repo directories so those paths point at the real files and can be
    checked for existence.
    """
    try:
        get_package_share_directory(package)
        return
    except PackageNotFoundError:
        pass

    prefix = tempfile.mkdtemp(prefix='launch_check_')
    share = os.path.join(prefix, 'share', package)
    os.makedirs(share)

    for subdir in SHARED_SUBDIRS:
        source = os.path.join(REPO_ROOT, package, subdir)
        if os.path.isdir(source):
            os.symlink(source, os.path.join(share, subdir))

    marker_dir = os.path.join(prefix, 'share', 'ament_index', 'resource_index', 'packages')
    os.makedirs(marker_dir)
    open(os.path.join(marker_dir, package), 'w').close()

    os.environ['AMENT_PREFIX_PATH'] = os.pathsep.join(
        [prefix, os.environ.get('AMENT_PREFIX_PATH', '')]).rstrip(os.pathsep)
    print(f'stubbed share directory for {package} at {prefix}')


def load(path):
    """Import a launch file and return its LaunchDescription."""
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def entities_of(description, context):
    """Top-level entities, with OpaqueFunctions and groups expanded."""
    entities = []

    def walk(items):
        for entity in items:
            if isinstance(entity, DeclareLaunchArgument):
                entity.visit(context)
            elif isinstance(entity, OpaqueFunction):
                walk(entity.execute(context) or [])
            elif isinstance(entity, GroupAction):
                walk(entity.execute(context) or [])
            else:
                entities.append(entity)

    walk(description.entities)
    return entities


def check_node(entity, context, name, failures):
    """Resolve every substitution the node carries and check what it points at."""
    label = f'{entity.__class__.__name__} in {name}'

    try:
        entity._perform_substitutions(context)
    except Exception as exc:
        failures.append(f'{name}: {label} failed to resolve: {exc}')
        return

    label = f'node {entity.node_name} in {name}'

    if isinstance(entity, LifecycleNode) and not entity.is_node_name_fully_specified():
        failures.append(f'{name}: lifecycle node {entity.node_name} has no namespace')

    for part in entity.cmd[1:]:
        if any(isinstance(sub, LocalSubstitution) for sub in part):
            continue

        try:
            resolved = perform_substitutions(context, part)
        except Exception as exc:
            failures.append(f'{name}: {label} has an unresolvable argument: {exc}')
            continue

        if os.path.sep in resolved and resolved.endswith(('.yaml', '.yml', '.xml')):
            if not os.path.isfile(resolved):
                failures.append(f'{name}: {label} points at a missing file: {resolved}')


def check_launch_file(name, path, collector, failures):
    """Build one launch description and check everything it points at."""
    try:
        description = load(path)
    except Exception as exc:
        failures.append(f'{name}: failed to build LaunchDescription: {exc}')
        return

    context = LaunchContext()

    try:
        entities = entities_of(description, context)
    except Exception as exc:
        failures.append(f'{name}: failed to expand entities: {exc}')
        return

    lifecycle_nodes = [entity for entity in entities if isinstance(entity, LifecycleNode)]

    for entity in entities:
        if isinstance(entity, Node):
            check_node(entity, context, name, failures)

    for warning in collector.drain():
        failures.append(f'{name}: {warning}')

    expected = EXPECTED_LIFECYCLE_NODES.get(name)
    if expected is None:
        failures.append(
            f'{name}: not listed in EXPECTED_LIFECYCLE_NODES, add it with its node count')
    elif len(lifecycle_nodes) != expected:
        failures.append(
            f'{name}: expected {expected} lifecycle nodes, found {len(lifecycle_nodes)}')

    print(f'{name}: checked ({len(lifecycle_nodes)} lifecycle nodes)')


def main():
    """Check every launch file in the repository and report what is broken."""
    failures = []

    stub_command_substitutions()

    collector = LaunchWarningCollector()
    for name in WATCHED_LOGGERS:
        launch.logging.get_logger(name).addHandler(collector)

    for package in LAUNCH_PACKAGES:
        ensure_package_share(package)

    launch_files = []
    for package in LAUNCH_PACKAGES:
        launch_dir = os.path.join(REPO_ROOT, package, 'launch')
        if not os.path.isdir(launch_dir):
            failures.append(f'{package}: no launch directory at {launch_dir}')
            continue
        for entry in sorted(os.listdir(launch_dir)):
            if entry.endswith('.launch.py'):
                launch_files.append((entry, os.path.join(launch_dir, entry)))

    found = {name for name, _ in launch_files}
    for name in sorted(set(EXPECTED_LIFECYCLE_NODES) - found):
        failures.append(f'{name}: listed in EXPECTED_LIFECYCLE_NODES but no such launch file')

    for name, path in launch_files:
        check_launch_file(name, path, collector, failures)

    for failure in failures:
        print(f'ERROR: {failure}', file=sys.stderr)

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
