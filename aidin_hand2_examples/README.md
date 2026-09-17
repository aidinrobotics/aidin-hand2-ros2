# aidin_hand2_examples

| Kind | Source | Config |
|---|---|---|
| Upper controller skeleton, one per command controller | `src/upper_controllers/` | `config/upper_controllers/` |

`src/upper_controllers/` holds a chainable upper controller skeleton for each of the four command
controllers (JointPosition, JointImpedance, ActuatorPosition, ActuatorEffort). They are the same
template, so they share one directory and one plugin library, and only the parameters are split
into `config/upper_controllers/<mode>_upper.yaml`. Each one keeps the whole `HandState` and
generates no value.

[EXAMPLE.md](EXAMPLE.md) is the reference for the interface names and for what to change in each
skeleton.
