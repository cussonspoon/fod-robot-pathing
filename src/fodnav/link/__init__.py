"""Links to the two things this process does not own.

``vision`` adapts Bthcorn's ``fod-vision`` library -- which runs a capture
thread inside this process, and is emphatically not a service. ``esp32`` speaks
the serial protocol to Teemy's firmware.

Both of those interfaces are somebody else's. Keep every field name from them
inside these two modules so that a change on either side is a one-file change
here. The MQTT detection schema this repo was first written against never
existed; that lesson is why the rule matters.
"""
