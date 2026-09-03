# Comment style

Comments, log messages and user-facing strings in this repository are English, following the
`aidin_hand2` C++ SDK. The reference files inside this repository are
`aidin_hand2_hardware/{include/aidin_hand2_hardware,src}/aidin_hand2_system_interface.*`.

## Form

- `//` line comments only. No `/* */`, no Doxygen (`@param`, `@brief`, `///`), no `TODO`/`FIXME`
- Sentence case. An identifier keeps its own casing: `command_lock is claim-only, ...`
- **No terminal period** on a one-sentence comment
- Wrap at 100 columns, continuing on a second `//` line
- **No `—`, `→` or `·` inside a comment.** Use a comma, `->`, `,`.
  `—` belongs to log and exception strings only
- Member comments go **above** the declaration, one per group. Trailing comments are for aligned
  tables of constants only, two spaces before `//`
- Thread ownership is a bracket tag: `[CM thread]`, `[service thread]`, `[bridge thread]`

## Content

Write only what the code cannot say:

- interface contracts
- units, ranges, sentinels (`-1 = unset`, `0 = no limit`, `1000 = 100%`)
- blocking or non-blocking
- thread ownership
- invariants
- data flow (`target -> clamp -> IK -> encoder count -> FK -> joint position`)
- non-obvious call order
- hardware manual references

Do **not** write:

- **reasons and history** — design rationale, alternatives that were rejected, past designs,
  measurement dates. State the constraint as a fact, not why it exists.
  `// Homing is triggered from write() with start_homing(), never by the SDK auto-home`,
  not `// The SDK blocking auto-home would hold the CM executor inside run()`
- anything the declaration name already says
  (`// Declare the parameter default` above a body that only calls `auto_declare`)
- a unit already carried by the name (`_rad`, `_cnt`, `_pct`, `_ms`), the initializer or the type
- "nothing to do here" on an empty function
- step narration, "returns X", example values
- a comment on the second and later overloads, on special members (ctor/dtor/copy/move), or on a
  `.cpp` definition whose header declaration already carries one

## Section banners

Exactly **79 columns** including the indent, `-` filled, Sentence case noun-phrase label, blank
line above and below.

```
  // -------------------- Single thread: plain [CM thread] --------------------
```

`=` is the second tier, used only to split a large `private:` block into `Functions` and
`Variables`.

Generate them, never count by hand:

```python
def banner(label, indent=""):
    avail = 79 - len(indent) - 3 - len(label) - 2
    left = (avail + 1) // 2
    return f"{indent}// {'-' * left} {label} {'-' * (avail - left)}"
```

Rules:

- **A `.cpp` banner label matches its `.hpp` label exactly**, thread tag included, and the
  sections appear in the same order in both files
- Variable sections follow the SDK vocabulary and order:
  `Config` -> `Single thread: plain` -> `Cross thread: buffer` -> `Cross thread: atomic`.
  Inside a section, subgroups are one-line comments, not more banners
- Constructors and destructors live under `Construction`
- Do not use box-drawing dividers (`── … ──`)

## Log and exception strings

- Log: lowercase start, no terminal period, `cause — remedy`, ALL-CAPS on the pivotal word,
  `topic: ` prefix for a subsystem
- Exception: `Cannot <verb phrase>: <cause> — <remedy naming the call>`
- Public vocabulary only, no internal protocol terms

## Terminology

- `joint` and `actuator` names as they appear in the URDF: `thumb_joint0..4`,
  `index_joint1..4`, `thumb_actuator0..3`, `index_actuator1..3`. Never `q1`, `q2`, `q3`
- Spell array order out in full so it cannot be read the wrong way:
  `order = thumb_actuator0..3, then index_actuator1..3, middle_actuator1..3, ring_actuator1..3,
  baby_actuator1..3`
- `bridge` belongs to the Isaac interface only, which bridges the simulator over ROS 2 topics.
  The real interface uses `service node`, the mock has no node

## Checks

```bash
# Korean left in code and config
grep -rnP '[가-힣]' --exclude-dir=.git --exclude-dir=ko --exclude=README.ko.md .

# Banner width, prints every line that is not 79 columns
grep -rnE '^\s*// [-=]{3,}' --include=*.hpp --include=*.cpp . \
  | while IFS= read -r l; do s="${l#*:}"; s="${s#*:}"; \
      [ ${#s} -ne 79 ] && echo "WIDTH ${#s}: $s"; done

# Forbidden glyphs and terminal periods in comments
grep -rnE '^\s*//.*[—→·]'            --include=*.hpp --include=*.cpp .
grep -rnE '^\s*//.*[a-z0-9)]\.\s*$'  --include=*.hpp --include=*.cpp .

# Rationale clauses
grep -rnE '^\s*//.*\b(so|since|because|would)\b' --include=*.hpp --include=*.cpp .
```

## Migration status

Korean is being removed package by package. Within a package the file that first fixes a shared
wording comes first, then the files that reuse it, with manifests and build files last.

Done:

- `aidin_hand2_msgs` — 7 files
- `aidin_hand2_hardware` — 7 files, plus the two reference files brought in line
- `aidin_hand2_controllers/src/joint_position_controller.cpp`

Remaining, in order:

1. `aidin_hand2_controllers` — `joint_position_controller.hpp`, then joint_impedance,
   actuator_position, actuator_effort (`.cpp` then `.hpp` each), `hand_state_broadcaster`,
   `diagnostics_broadcaster`, `plugin/*.xml`, `CMakeLists.txt`, `package.xml`
2. `aidin_hand2_description` — `ros2_control/aidin_hand2.ros2_control.xacro`,
   `urdf/aidin_hand2.urdf.xacro`, `_left`, `_right`, `launch/description.launch.py`, `package.xml`
3. `aidin_hand2_bringup` — `config/hand_bringup.yaml` and `launch/aidin_hand2.launch.py` first,
   then the controllers, mock and isaac config/launch pairs, `gui_bridge.launch.py`, `package.xml`
4. `aidin_hand2_examples` — glove_teleop `.hpp`, `.cpp`, `.yaml`, `glove_calibrate.py`, the four
   upper controllers, `plugin/*.xml`, `CMakeLists.txt`, `package.xml`
5. Markdown — `docs/en` mirroring `docs/ko`, the three package READMEs and `EXAMPLE.md`,
   and the `docs/ko` links in `README.md` repointed to `docs/en`. `docs/ko` and `README.ko.md`
   stay Korean

Stale claims found and corrected so far, worth watching for elsewhere:

- "98 command interface resources" in the mock and isaac interfaces. The contract is
  `command_lock` 1 + 16 x 4 = **65**
- `actuator_position_controller.cpp` and `actuator_effort_controller.cpp` still claim that
  activation seeds the references from state. Both fill NaN and claim no state interface at all
