# CAN/DBC multiplexing

A C++ excerpt from a telemetry consumer that selects signals from multiplexed
CAN messages. A multiplexer is a signal whose value selects a page of fields
that share the same payload. After a configuration reload, a child signal can
still refer to an older selector. The code resolves selectors by ID so those
references keep working.

`resolve_mux_selectors` resolves each child's selector layout from a prepared
signal plan. `MuxFrame` validates the layout against the payload, reads each
selector at most once per frame, and uses the value to decide which children
are active. A truncated selector or an all-ones value activates no page.
`frame_bounds.hpp` checks bit ranges for Intel and Motorola DBC layouts.

The caller supplies the signal plan and a bit-reading callback. The callback
returns a `uint64_t` from a selector between 1 and 64 bits wide and interprets
the configured byte order. Page IDs use `int64_t`. Use a new `MuxFrame` for
each payload. The tests use a small Intel reader to exercise page selection
and check Motorola ranges separately.

The published excerpt preserves the original selection and bounds logic,
uses synthetic test frames and IDs, and adds a standalone build. The excerpt
also rejects selectors wider than the callback can return and uses
byte-based bounds arithmetic to handle extreme widths without overflow.

## Build and test

With a C++20 compiler, CMake 3.20 or later, and GoogleTest installed:

```sh
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```
