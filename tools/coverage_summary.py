#!/usr/bin/env python3
"""
Summarise per-package coverage and enforce a floor on the pure-logic modules.

Reads the ``coverage.xml`` that ``colcon test --pytest-with-coverage`` writes into each
package's build directory and prints a Markdown report for the CI job summary.

Only the logic modules are gated (test plan 7). A node file always reads as 0% here even
though the L2 suites exercise it, because launch_testing starts the node in a separate
process that this coverage run does not instrument -- so node numbers are reported for
information and never gated.
"""

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

SKIP = {'setup.py', '__init__.py'}


def is_logic(path):
    """Return True for a pure-logic module: package source that is not a node."""
    name = Path(path).name
    return (not path.startswith('test/')
            and name not in SKIP
            and not name.endswith('_node.py')
            and name.endswith('.py'))


def is_node(path):
    """Return True for a node module: the rclpy wiring around the logic."""
    return not path.startswith('test/') and Path(path).name.endswith('_node.py')


def collect(build_dir):
    """Return (logic, nodes) as lists of (package, file, percent)."""
    logic, nodes = [], []
    for report in sorted(Path(build_dir).glob('*/coverage.xml')):
        package = report.parent.name
        for cls in ET.parse(report).iter('class'):
            path = cls.get('filename') or ''
            percent = float(cls.get('line-rate') or 0.0) * 100.0
            if is_logic(path):
                logic.append((package, path, percent))
            elif is_node(path):
                nodes.append((package, path, percent))
    return logic, nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--build', default='build', help='colcon build directory')
    parser.add_argument('--min-logic', type=float, default=90.0,
                        help='minimum line coverage for each logic module')
    args = parser.parse_args()

    logic, nodes = collect(args.build)
    if not logic:
        print('## Coverage\n\nNo coverage.xml found; was --pytest-with-coverage used?')
        return 0

    failing = [row for row in logic if row[2] < args.min_logic]

    print('## Coverage\n')
    print(f'Logic modules must reach {args.min_logic:.0f}% line coverage.\n')
    print('| Module | Package | Line coverage |')
    print('|---|---|---|')
    for package, path, percent in sorted(logic, key=lambda r: r[2]):
        mark = '' if percent >= args.min_logic else ' :x:'
        print(f'| `{Path(path).name}` | {package} | {percent:.0f}%{mark} |')
    print()

    if nodes:
        print('Node files are exercised by the L2 launch tests in a separate process, so '
              'they are not instrumented here and are listed for information only:\n')
        print('| Node | Package | Line coverage |')
        print('|---|---|---|')
        for package, path, percent in sorted(nodes):
            print(f'| `{Path(path).name}` | {package} | {percent:.0f}% |')
        print()

    if failing:
        names = ', '.join(f'{Path(p).name} ({v:.0f}%)' for _, p, v in failing)
        print(f'**Below the {args.min_logic:.0f}% floor: {names}**')
        return 1
    print(f'All {len(logic)} logic modules meet the floor.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
