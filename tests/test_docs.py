#!/usr/bin/env python3
import pathlib
import re


ROOT = pathlib.Path(__file__).resolve().parents[1]
DOCUMENTS = (ROOT / "README.md", ROOT / "README.zh-CN.md")
LINK = re.compile(r"\]\(([^)]+)\)")


def local_targets(document):
    in_fence = False
    for line_number, line in enumerate(document.read_text(encoding="utf-8").splitlines(), 1):
        if line.lstrip().startswith("```"):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for match in LINK.finditer(line):
            target = match.group(1).split("#", 1)[0]
            if target and not target.startswith(("http://", "https://", "mailto:")):
                yield line_number, target


for document in DOCUMENTS:
    assert document.is_file() and document.stat().st_size > 0, document
    for line_number, target in local_targets(document):
        resolved = (document.parent / target).resolve()
        assert resolved.exists(), f"{document.name}:{line_number}: missing {target}"

english = DOCUMENTS[0].read_text(encoding="utf-8")
chinese = DOCUMENTS[1].read_text(encoding="utf-8")
assert "(README.zh-CN.md)" in english
assert "(README.md)" in chinese
for required in (
    "apriltag_docking_demo.mp4",
    "ros-jazzy-apriltag-msgs",
    "ros2 launch",
    "colcon test",
    "guard_required",
):
    assert required in chinese, f"Chinese README is missing {required}"

print("PASS: bilingual README links and contracts")
