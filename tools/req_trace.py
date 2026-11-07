#!/usr/bin/env python3
"""Report which requirements are covered by tests tagged with @pytest.mark.req('<id>')."""

import argparse
from collections import defaultdict
from pathlib import Path
import re
import sys

REQ_LINE = re.compile(r'^\*\*(\d+\.\d+\.\d+)\*\*\s+(.*?)(?:\s+—\s+(P\d)\s+·\s+([TDIA]).*)?$')
TRAILER = re.compile(r'—\s+(P\d)\s+·\s+([TDIA])\b')
REQ_TAG = re.compile(r"mark\.req\(\s*((?:['\"][\d.]+['\"]\s*,?\s*)+)\)")
TAG_ID = re.compile(r"['\"]([\d.]+)['\"]")


def load_requirements(req_dir):
    """Return {id: (priority, method, text, file)} for every top-level requirement."""
    reqs = {}
    for path in sorted(req_dir.glob('[0-9]*.md')):
        pending = None  # requirement whose priority comes after its sub-requirements
        for line in path.read_text().splitlines():
            m = REQ_LINE.match(line)
            if m:
                rid, text, prio, method = m.groups()
                if prio:
                    reqs[rid] = (prio, method, text, path.name)
                    pending = None
                else:
                    pending = (rid, text)
                continue
            t = TRAILER.search(line)
            if t and pending:
                reqs[pending[0]] = (t.group(1), t.group(2), pending[1], path.name)
                pending = None
    return reqs


def load_tags(root):
    """Return {id: [test files]} for every @pytest.mark.req tag under root."""
    tags = defaultdict(list)
    for path in root.glob('*/test/**/*.py'):
        for m in REQ_TAG.finditer(path.read_text()):
            for rid in TAG_ID.findall(m.group(1)):
                tags[rid].append(str(path.relative_to(root)))
    return tags


def covers(tag_id, req_id):
    """Match a tag to a requirement; a sub-requirement tag (5.3.1.2) counts for its parent."""
    return tag_id == req_id or tag_id.startswith(req_id + '.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default=Path(__file__).resolve().parents[1], type=Path,
                        help='repository root (the workspace src/ folder)')
    parser.add_argument('--fail-on-missing-p1', action='store_true',
                        help='exit 1 if any P1 requirement verified by Test has no tagged test')
    args = parser.parse_args()

    reqs = load_requirements(args.root / 'docs' / 'requirements')
    tags = load_tags(args.root)

    rows = []
    missing_p1 = []
    for rid, (prio, method, _text, fname) in reqs.items():
        tests = sorted({t for tid, files in tags.items() if covers(tid, rid) for t in files})
        rows.append((rid, prio, method, fname, tests))
        if prio == 'P1' and method == 'T' and not tests:
            missing_p1.append(rid)

    automatable = [r for r in rows if r[2] == 'T']
    covered = [r for r in automatable if r[4]]
    unknown = sorted(set(tags) - {t for t in tags for r in reqs if covers(t, r)})

    print('## Requirement trace\n')
    print(f'- Requirements: {len(rows)}')
    print(f'- Verified by Test (T): {len(automatable)}, with automated tests: {len(covered)}')
    print(f'- P1 Test requirements without a test: {len(missing_p1)}\n')
    if covered:
        print('| ID | Priority | Tests |')
        print('|---|---|---|')
        for rid, prio, _m, _f, tests in covered:
            print('| {} | {} | {} |'.format(rid, prio, ', '.join(tests)))
        print()
    if unknown:
        print('**Tags that match no requirement:** {}\n'.format(', '.join(unknown)))
    if missing_p1:
        print('<details><summary>P1 Test requirements without a test ({})</summary>\n\n{}\n\n'
              '</details>'.format(len(missing_p1), ', '.join(missing_p1)))

    if args.fail_on_missing_p1 and missing_p1:
        return 1
    return 1 if unknown else 0


if __name__ == '__main__':
    sys.exit(main())
