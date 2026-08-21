"""Links to the two processes this one does not own.

``detections`` subscribes to the vision process (Bthcorn's, MQTT, JSON).
``esp32`` speaks the serial protocol to the firmware (Teemy's).

Both of those specs are proposals, not agreements: docs/protocol.md has not
been reviewed by Teemy and the detection schema has not been implemented by
Bthcorn. Build against both, but keep every field name inside these two
modules so that a change to either is a one-file change.
"""
