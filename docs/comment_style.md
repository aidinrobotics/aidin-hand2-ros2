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
- Thread ownership is a bracket tag: `[CM thread]`, `[service thread]`, `[bridge thread]`.
  These belong to `aidin_hand2_hardware` alone, it being the package that meets the SDK.
  Everywhere else state the invariant in words and leave the threading unlabelled, even where
  the code does cross threads

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

A contract line carries a **count**, which is what catches the claim going stale: "the claim is
`command_lock` x1 and `target_position_rad` x16, 17 resources", "39 resources", "313 state
interfaces".

## Section banners

Exactly **79 columns** including the indent, `-` filled, Sentence case noun-phrase label, blank
line above and below.

```
  // -------------------- Single thread: plain [CM thread] --------------------
```

`=` is the second tier, used only to split a large `private:` block into `Functions` and
`Variables`.

YAML and CMake banners are the same shape with `#`. **XML and xacro use `=` as the fill**, since
an XML comment may not contain `--`:

```
  <!-- ============================= Placement ============================ -->
```

Generate them, never count by hand:

```python
def banner(label, indent="", comment="//"):
    close = " -->" if comment == "<!--" else ""
    fill = "=" if comment == "<!--" else "-"
    avail = 79 - len(indent) - len(comment) - 1 - len(label) - 2 - len(close)
    left = (avail + 1) // 2
    return f"{indent}{comment} {fill * left} {label} {fill * (avail - left)}{close}"
```

Rules:

- **A `.cpp` banner label matches its `.hpp` label exactly**, thread tag included, and the
  sections appear in the same order in both files
- Variable sections in `aidin_hand2_hardware` follow the SDK vocabulary and order:
  `Config` -> `Single thread: plain` -> `Cross thread: buffer` -> `Cross thread: atomic`.
  Inside a section, subgroups are one-line comments, not more banners. Other packages group
  their members with one-line comments and no banner
- Constructors and destructors live under `Construction`
- A label names what the section holds, and an instantiation section is named after the thing
  it instantiates, not after the file it came from
- Do not use box-drawing dividers (`── … ──`)

## Argument and parameter lists

An argument is described **once, where it is defined**, as a name-keyed aligned list. The macro,
the launch file and the URDF that only forward it carry one line pointing at that definition,
never a second copy of the table.

```
    can_interface              one CAN bus per hand, tactile included
    disabled_actuators         comma separated index list such as "0,1,2,3", empty = all on
    isaac_state_timeout        seconds without a new state before the link counts as lost
```

- The list order matches the declaration order, and both stay in step with the call sites
- No comment between the declarations themselves, the list above them carries everything
- Arguments that apply to one backend are separated by a light comment, not a banner:
  `<!-- Isaac only -->`
- Every argument reaches the hardware the same way. The URDF holds the default and the macro
  always emits the `<param>`, so a hardware member initialiser is only the fallback for a URDF
  written without this macro. Do not add a `<xacro:if>` that emits a parameter only when it is
  non-empty

## Constants

A comment that repeats a number the code could have named is a comment that will go stale, so the
constants come first.

- **Structural counts come from `aidin_hand2/types/description.hpp`**, never a local `= 16` or a
  bare `21`, `17`, `20`, `18`. Alias the namespace once per file and use it:

  ```cpp
  namespace ah2 = aidin_hand2;
  ...
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
  ```

- **Interface-name constants are `constexpr char kXxxInterface[] = "..."`**, one shape across the
  whole repository. Not `const char *`, not an `InterfaceName` suffix
- **Sibling files share the constant names, values differ.** The four command controllers all
  declare `kCommandLockInterface`, `kTargetInterface`, `kReferenceInterface` and
  `kCommandLockCount`, so the four files diff against each other line for line. Resist a
  per-controller name like `kTargetPositionInterfaceName`
- **Name an offset after what makes it what it is.** `kCommandLockCount = 1` says why the target
  block starts at 1; `kHardwareTargetOffset` only restates the number. Drop an offset that is
  always `0`
- **`kXxxBaseNames` holds the unprefixed name**, and the side prefix is applied at the use site.
  Never store the prefixed name and cut it back off with `substr`. Where both live in one file the
  `Base` in the name is the only thing keeping them apart

### Name array layout

A naming array is written **one entry per line**, trailing comma, `};` on its own line, with the
index contract above it. Never packed several to a line, which is how a wrong order hides.

```cpp
// Interface names without the prefix
// The position in the list is the actuator index
constexpr std::array<const char *, ah2::kActuatorCount> kActuatorBaseNames = {
  "thumb_actuator0",
  "thumb_actuator1",
  ...
  "baby_actuator3",
};
```

## Log and exception strings

- Log: lowercase start, no terminal period, `cause — remedy`, ALL-CAPS on the pivotal word,
  `topic: ` prefix for a subsystem
- Exception: `Cannot <verb phrase>: <cause> — <remedy naming the call>`
- Public vocabulary only, no internal protocol terms
- One wording per check across the repository, and it carries the offending value. Parameter
  validation is `"hand_side must be 'left' or 'right', got '%s'"`, never a second phrasing such as
  `"hand_side parameter is invalid"`

## Terminology

- `joint` and `actuator` names as they appear in the URDF: `thumb_joint0..4`,
  `index_joint1..4`, `thumb_actuator0..3`, `index_actuator1..3`. Never `q1`, `q2`, `q3`
- Spell array order out in full so it cannot be read the wrong way:
  `order = thumb_actuator0..3, then index_actuator1..3, middle_actuator1..3, ring_actuator1..3,
  baby_actuator1..3`
- `bridge` belongs to the Isaac interface only, which bridges the simulator over ROS 2 topics.
  The real interface uses `service node`, the mock has no node
- **`SDK` belongs to `aidin_hand2_hardware`**, the package that meets it. Elsewhere name the
  concrete thing instead: "tuned on the hardware node", not "belongs to the SDK ControllerConfig".
  An internal type name such as `ControllerConfig` goes with it
- Only the hardware interface talks about threads at all, see the bracket-tag rule under
  [Form](#form)

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

# Structural count redeclared locally instead of taken from description.hpp
grep -rnE 'constexpr std::size_t k(Actuator|ActiveJoint|Joint|Finger)Count *= *[0-9]' \
  --include=*.hpp --include=*.cpp .

# Name array packed several entries to a line
grep -rnE '^\s*"[^"]*", *"[^"]*"' --include=*.hpp --include=*.cpp .

# Old interface-name constant shape, the plural kXxxInterfaceNames array being fine
grep -rnE 'InterfaceName\b' --include=*.hpp --include=*.cpp .

# SDK named outside the hardware package
grep -rn 'SDK' --include=*.hpp --include=*.cpp . | grep -v aidin_hand2_hardware
```

## Migration status

Korean is being removed package by package. Within a package the file that first fixes a shared
wording comes first, then the files that reuse it, with manifests and build files last.

Done:

- `aidin_hand2_msgs` — 7 files
- `aidin_hand2_hardware` — 7 files, plus the two reference files brought in line
- `aidin_hand2_controllers` — 12 files, plus `plugin/*.xml`, `CMakeLists.txt`, `package.xml`

Remaining, in order:

1. `aidin_hand2_description` — `ros2_control/aidin_hand2.ros2_control.xacro`,
   `urdf/aidin_hand2.urdf.xacro`, `_left`, `_right`, `launch/description.launch.py`, `package.xml`
2. `aidin_hand2_bringup` — `config/hand_bringup.yaml` and `launch/aidin_hand2.launch.py` first,
   then the controllers, mock and isaac config/launch pairs, `gui_bridge.launch.py`, `package.xml`
3. `aidin_hand2_examples` — glove_teleop `.hpp`, `.cpp`, `.yaml`, `glove_calibrate.py`, the four
   upper controllers, `plugin/*.xml`, `CMakeLists.txt`, `package.xml`
4. Markdown — `docs/en` mirroring `docs/ko`, the three package READMEs and `EXAMPLE.md`,
   and the `docs/ko` links in `README.md` repointed to `docs/en`. `docs/ko` and `README.ko.md`
   stay Korean

Stale claims found and corrected so far, worth watching for elsewhere:

- "98 command interface resources" in the mock and isaac interfaces. The contract is
  `command_lock` 1 + 16 x 4 = **65**
- `actuator_position_controller.cpp` and `actuator_effort_controller.cpp` claimed that
  activation seeds the references from state, and `actuator_position_controller.cpp` listed a
  state interface. Both fill NaN and claim `NONE`
- `joint_impedance_controller.cpp` claimed "hardware command port 48". The claim is
  `command_lock` 1 + 16 = **17**, the same as the other three controllers
- `joint_impedance_controller.cpp`, the plugin description and `package.xml` claimed an impedance
  gain parameter. No controller declares one, the gains are tuned on the hardware node
- `package.xml` claimed the `aidin_hand2` header is taken for gain defaults. It is taken for the
  structural constants and the lifecycle, homing and fault names
- `hand_state_broadcaster.hpp` listed only joint, actuator and tactile. It also carries the
  command echo and the observation timestamp, **313** state interfaces in total
- The `DiagnosticsBroadcaster` description named only `/diagnostics`. That publisher is gone,
  `~/hand_diagnostics` is the only topic
