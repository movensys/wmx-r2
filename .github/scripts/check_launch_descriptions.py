#!/usr/bin/env python3
"""Evaluate the repository launch descriptions without launching anything."""

import collections
import importlib.util
import logging
import os
import sys
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

import launch.logging
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import Command, LocalSubstitution
from launch.utilities import perform_substitutions

from launch_ros.actions import LifecycleNode, Node

sys.dont_write_bytecode = True

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LAUNCH_PACKAGES = ('wmx_r2_package', 'wmx_r2_control')
SHARED_SUBDIRS = ('config', 'example', 'launch', 'urdf', 'rviz', 'meshes')

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

Case = collections.namedtuple('Case', 'launch_file label arguments lifecycle_nodes')


def cases():
    """Every launch file with the arguments it is documented to take.

    The launch files take all of their paths as arguments, so a file is checked
    once per robot it is meant to serve. Every argument value that names a file
    is checked for existence too, which keeps this table honest.
    """
    wmx = get_package_share_directory('wmx_r2_package')
    ctrl = get_package_share_directory('wmx_r2_control')

    def example(name):
        return os.path.join(wmx, 'example', name)

    def general(name):
        return os.path.join(wmx, 'config', name)

    def urdf(name):
        return os.path.join(ctrl, 'urdf', name)

    def controllers(name):
        return os.path.join(ctrl, 'config', name)

    def manipulator(robot, gripper):
        return {
            'use_sim_time': 'false',
            'config_file': example(f'{robot}_manipulator_config.yaml'),
            'wmx_param_file': example(f'{robot}_wmx_parameters.xml'),
            'use_gripper': 'true' if gripper else 'false',
        }

    def differential():
        return {
            'use_sim_time': 'false',
            'config_file': example('diffbot_differential_config.yaml'),
            'wmx_param_file': example('diffbot_wmx_parameters.xml'),
        }

    def with_control(arguments, robot, controllers_name):
        return dict(arguments,
                    urdf_file=urdf(f'{robot}.wmx.urdf.xacro'),
                    controllers_file=controllers(controllers_name))

    return [
        Case('wmx_r2_general_nodes.launch.py', 'defaults', {
            'use_sim_time': 'false',
            'config_file': general('wmx_r2_general_nodes_config.yaml'),
            'wmx_param_file': general('wmx_parameters.xml'),
        }, 3),

        Case('wmx_r2_manipulator.launch.py', 'cr3a',
             manipulator('cr3a', gripper=True), 7),
        Case('wmx_r2_manipulator.launch.py', 'cr5a',
             manipulator('cr5a', gripper=False), 6),
        Case('wmx_r2_differential.launch.py', 'diffbot',
             differential(), 5),

        Case('wmx_r2_control_manipulator.launch.py', 'cr3a',
             with_control(manipulator('cr3a', gripper=True),
                          'cr3a', 'cr3a_controllers.yaml'), 6),
        Case('wmx_r2_control_manipulator.launch.py', 'cr5a',
             with_control(manipulator('cr5a', gripper=False),
                          'cr5a', 'cr5a_controllers.yaml'), 5),
        Case('wmx_r2_control_differential.launch.py', 'diffbot',
             with_control(differential(), 'diffbot', 'diffbot_controllers.yaml'), 3),
    ]


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
    """Top-level entities, with OpaqueFunctions and groups expanded.

    An entity whose condition evaluates false is dropped, the way launch itself
    would drop it, so a flag such as use_gripper changes the node count here.
    """
    entities = []

    def skipped(entity):
        condition = getattr(entity, 'condition', None)
        return condition is not None and not condition.evaluate(context)

    def walk(items):
        for entity in items:
            if isinstance(entity, DeclareLaunchArgument):
                entity.visit(context)
            elif skipped(entity):
                continue
            elif isinstance(entity, OpaqueFunction):
                walk(entity.execute(context) or [])
            elif isinstance(entity, GroupAction):
                walk(entity.execute(context) or [])
            elif isinstance(entity, IncludeLaunchDescription):
                entity.execute(context)
                walk(entity.launch_description_source.get_launch_description(
                    context).entities)
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


def check_case(case, path, collector, failures):
    """Build one launch description with its arguments and check what it points at."""
    name = f'{case.launch_file} [{case.label}]'

    for argument, value in sorted(case.arguments.items()):
        if os.path.sep in value and not os.path.isfile(value):
            failures.append(f'{name}: {argument} points at a missing file: {value}')

    try:
        description = load(path)
    except Exception as exc:
        failures.append(f'{name}: failed to build LaunchDescription: {exc}')
        return

    context = LaunchContext()
    context.launch_configurations.update(case.arguments)

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

    if len(lifecycle_nodes) != case.lifecycle_nodes:
        failures.append(
            f'{name}: expected {case.lifecycle_nodes} lifecycle nodes, '
            f'found {len(lifecycle_nodes)}')

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

    paths = dict(launch_files)
    check_cases = cases()
    covered = {case.launch_file for case in check_cases}

    for name in sorted(covered - set(paths)):
        failures.append(f'{name}: has a case in cases() but no such launch file')
    for name in sorted(set(paths) - covered):
        failures.append(f'{name}: no case in cases(), add it with its node count')

    for case in check_cases:
        if case.launch_file in paths:
            check_case(case, paths[case.launch_file], collector, failures)

    for failure in failures:
        print(f'ERROR: {failure}', file=sys.stderr)

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
