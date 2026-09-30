# Transport reliability and audit

The MCP server sends each command once. If transmission begins and the reply is
lost, malformed, late, or carries a different request ID, it closes that connection
and raises `CommandOutcomeUnknown` (a `BridgeError`). The error says that execution
may already have happened and the command was not retried. A subsequent command
reconnects. Read the simulation state before deciding whether to retry a mutation.

A timeout **does not cancel** a queued or running Webots operation. Reconnection
is not cancellation either. Heartbeats prove that the connection is alive; they
do not extend the requested deadline. One monotonic deadline covers waiting for
the connection lock, connecting, sending, and receiving all response fragments.
Connection attempts may retry before any command transmission. Failures before
transmission retain the `ConnectionError` contract and explicitly say “not sent.”

The bridge applies the same deadline and response-ID rules to robot-agent calls.
An uncertain agent reply closes and unregisters that connection. The operation
is not replayed. A live agent reconnects after noticing the disconnect. While the
simulation is logically paused, agents waiting in `robot.step()` cannot process
new commands; resume or step before using per-robot tools.

Reload can acknowledge just before the controllers restart, or lose its reply
during restart. Both are normal possibilities. The server never resends a reload
to recover from a lost reply. Scene node IDs and handles should be reacquired
after reload, reset, deletion, or regeneration; bridge reference resolution does
not retain a cache across calls. Name-field fallback searches are bounded to four
scene levels. Use an actual DEF or current numeric ID for deeper nodes.

Successful tool names and response formats are unchanged. Intentional error and
termination changes:

- Communication failures after transmission report uncertainty instead of replay.
- Invalid/mismatched response envelopes invalidate the connection.
- `step_simulation` accepts integer counts from 0 through 100000. Negative,
  fractional, boolean, and oversized counts are rejected before stepping, instead
  of silently coercing/capping them. Early termination returns the actual completed
  count in `stepped` alongside `terminated: true`.
- Agent registration detects EOF, validates its acknowledgment, and times out.
  Partial incoming command frames remain buffered without blocking the simulation
  loop. Reply-send failure closes the socket and reconnects on a subsequent step.
- Idle agent disconnects remove stale registrations. A replacement registration
  cannot be removed by the old connection's cleanup.

## Regression evidence

`server/tests/test_transport_regressions.py` uses scripted real TCP peers. Its
lost-reply case executes a mutation, drops the reply, and accepts another
connection: the fixed client raises uncertainty, and an explicit subsequent read
confirms exactly one execution. Other cases cover heartbeats, partial frames,
invalid IDs/JSON, lock deadlines, positive short robot timeouts, and healthy reuse
after a normal error response.

`server/tests/test_controller_protocol.py` imports the actual bridge and agent
against a small Webots API double. It covers incremental framing, registration
EOF, idle disconnect, reply-send loss, agent deadlines, exact and invalid step
counts, termination counts, fresh scene reference resolution, and traversal bounds.

Before the fixes, five of the six initial transport regressions failed; eight of
the ten initial controller regressions failed. Two subsequently added tests also
reproduced idle-registration leakage and an uncaught agent reply-send failure.
`server/tests/test_tool_contracts.py` checks literal tool actions against controller
handlers and forwarding/error behavior. Existing registration, parameter, result,
batch-whitelist, and tool-group tests remain in the complete suite.

The complete standalone suite passed **109 tests** on the audited revision. The
consumer factory passed **689 tests** after controller synchronization. One
pre-existing Pydantic settings warning about the FastMCP `lifespan` annotation
remains; it did not fail tests or the live session.

Live validation on Webots R2025a exercised public MCP tools in a disposable demo:
DEF/name/ID scene reads, missing/deleted-node errors, pause/read/step/resume,
screenshots, robot devices and errors, agent deletion, reload/reconnection, and
simulation reset. Physical acceptance and machine-specific timings are retained
in the consuming factory project, outside this reusable repository.

## Remaining limits

This is an audit of reproduced defects, not proof that every tool or robot model
is defect-free. It does not add cancellation, request deduplication across server
restarts, or background execution of agent commands while paused. Agent connection
and registration attempts are bounded but may temporarily delay a controller step
during recovery. Initial agent startup still requires a reachable bridge. No
physics, motion speeds, timestep, or synchronization settings were changed.
