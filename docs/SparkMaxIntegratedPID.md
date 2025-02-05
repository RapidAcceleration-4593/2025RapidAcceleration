# Integrated PID with Spark Max
Integrated PID is simple when using the motor's built in encoder.
Simply call <code>motorName.getPIDController.setP(kP)</code> to set the proportional gain, and the sister methods to set integral and derivative values.
This can also be done using REV Hardware Client.
Then, to set the PID setpoint, use <code>motorName.getPIDController.setReference(value, controlType)</code>.
(<code>controlType</code> specifies whether the setpoint is a velocity setpoint or a position one or something else.)

To use the REV Through Bore Encoder when the motor controller is in brushed mode, simply wire the outputs of the encoder to the Data Port slot on the front of the Spark Max.

See here for the Data Port info and pinout: https://docs.revrobotics.com/brushless/spark-max/specs/data-port

See here for the Through Bore encoder info and pinout: https://www.revrobotics.com/content/docs/REV-11-1271-DS.pdf

If the motor controller is in brushless mode, connecting the encoder the way specified in the links above will not work.
Instead, the motor controller must be set to Alternate Encoder mode, and the encoder wired to the Data Port accordingly.

See here for more info on Alternate Encoder Mode: https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
