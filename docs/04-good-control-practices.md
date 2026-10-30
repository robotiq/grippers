# Good control practices

The gripper can be controlled in position, speed and force, and it gives
feedback such as object detection. Knowing these control features is not
enough to control the gripper efficiently: you also need to know the good
control practices to get the best gripper performance. These practices are
useful for people or AI agents who want to implement gripper control in
their systems.

This page assumes the register mnemonics introduced in
[How it works](03-how-it-works.md) (`rACT`, `gSTA`, `gOBJ`, ...). The
numeric position thresholds below (3, 230, 5, 225) are measured on a 2F-85
with stock fingers — see [`profiles::k2F85`](03-how-it-works.md#commanding-in-si-units);
a different model or custom fingers shifts them.

## Activation and status

- Activate with a rising edge of `rACT` (0, then 1). Wait for `gSTA` = 3
  (`ActivationState::Complete`) before relying on the position and object
  status.
- Make sure the gripper fingers are free to move during activation. The
  activation moves the fingers through their full stroke to calibrate the
  open and closed positions, so an object or obstacle between the fingers
  can make it fail or give a wrong calibration.
- Redo the activation if the fingers or fingertips are changed. The
  activation adjusts the gripper to the fingertips, so that 0 is always
  fully open and 255 fully closed (manual section 4.3.4).
- Use `rGTO` = 0 to stop a motion, not `rACT` = 0. Clearing `rACT` resets
  the gripper, and it then needs a new activation.
- `gOBJ` is only valid when `gGTO` = 1. After a new position request,
  `gOBJ` goes to 0 (`ObjectDetection::Moving`) before it gives the result.
  Wait until `gPR` echoes your request and `gOBJ` leaves `Moving` before
  reading the result.
- Read `gFLT` with the status. A non-zero fault explains a gripper that
  stops or ignores requests (see the
  [fault table](03-how-it-works.md#status), or the manual).

## Position commands

- Avoid controlling the gripper position at high frequency as much as
  possible. Instead, decide on a target, send it once and let the gripper
  reach it. Do not micromanage the gripper motion.
- Command 0 or 255 when committing to a grip, not the end positions read
  back on `gPO` (3 fully open, about 230 closed on itself, on a 2F-85). The
  firmware clamps requests below 3 to 3, which keeps the fingers clear of
  the open hard stop. Positions 231 to 255 lie past the closed-on-itself
  position, so 255 keeps the fingers pushing until they stop on the
  object.
- Write position, speed and force in the same Modbus request. A position
  change starts the motion at once with the speed and force registers as
  they stand. `GripperCommand`/`setCommand()` already does this for you —
  there's no way to write just one field with this SDK.
- Understand that there is no fixed correspondence between the position
  value (0 to 255) and the finger opening in mm. It depends on the finger
  and fingertip design. The correspondence can be made if you know the
  gripper opening in mm at the open and closed positions with your
  fingers. Since the relationship is quasi-linear (manual section 4.3.4),
  you can interpolate between these two values — this SDK does exactly
  that via
  [`DeviceProfile`/`units::openingToRegister`](03-how-it-works.md#commanding-in-si-units).

## Gripping and releasing an object

Grip an object as follows:

1. Position the gripper so that the fingers are on both sides of the
   object. Keep a gap of about 1 cm between the fingers and the object
   surface.
2. Commit to the grip by commanding position 255 for a full close or 0 for
   a full open. When sending the grip command, also set the force and
   speed of your choice.

- Do not grip an object with small position requests, such as real-time
  finger position feedback from a teleoperation controller. This results
  in a weak grip. The gripper gets its best grip when it receives a clear,
  anticipated grip request. Such a request lets the gripper build kinetic
  energy and use the impact force to secure the object.
- Use a force parameter above 0 when committing to a grip on an object, so
  that the Re-grasp function is enabled.
- Use maximum force and maximum speed to release an object. Significant
  force can be applied to the gripper mechanism during a grip, so high
  force and speed parameters are needed for a proper release. Firmware
  1.7.0 already switches to maximum force when the fingers reverse after a
  grip; setting it explicitly also covers older firmware.

## Teleoperation

Teleoperation typically uses real-time control with constant position
requests sent to the gripper.

- Real-time control with constant position requests can be used to
  preposition the gripper fingers before committing to a grip (about 1 cm
  from the surface). This prepositioning can be done with a trigger. When
  you want to grip, press a grip button rather than pressing the trigger
  to change the finger position. Pressing the grip button should send a
  position request of 255 or 0 with the desired speed and force.
- Make sure that displacement requests with a target close to the actual
  finger position are done at low speed and low force. This avoids finger
  position overshoot and unnecessary use of the gripper impact mechanism
  (click sound).

## Grip force

- Understand that the force parameter is not a force in newtons, and there
  is no single formula to convert it to the actual grip force. The actual
  grip force depends on the speed setting, the object hardness and the
  fingertips.
- Above a speed setting of about 100 (125 to 150 on soft objects), the
  speed parameter has more effect on the actual grip force than the force
  parameter, because the gripper gets its grip force from kinetic impact.
  Below that speed, the force setting dominates. On very soft objects (30
  OO polyurethane) with a force setting of 128 or more, speed has no
  effect.
- The force parameter has more effect on soft objects than on hard
  objects.
- Understand that the actual grip force cannot be decreased by changing
  the force and speed parameters, because the gripper has a self-locking
  mechanism. The only way to decrease the grip force is to request a
  position out of the grip.
- Analyzing `gCU` (`GripperStatus::current`) to estimate the grip force is
  not very relevant. The gripper does not provide actual grip force
  feedback; the grip force is set by choosing the corresponding
  combination of force and speed. The manual provides curves showing the
  approximate actual grip force for different combinations of force and
  speed parameters, for different object stiffnesses and a given type of
  fingertip.
- If you need grip force feedback, use a tactile sensor such as the
  TSF-85.

## Object detection and Re-grasp

- After an object is detected, do not change the position request. It
  should stay at 0 or 255. Changing the position target clears the object
  detection status and forces the gripper to retry a grip. This can happen
  if you grip by moving the gripper position progressively with a trigger
  rather than using a button to commit to a grip. Constantly forcing the
  gripper to retry a grip by clearing the object detection puts
  unnecessary stress on the gripper mechanism.
- Understand that setting `rGTO` to 0 and then back to 1 resets the object
  detection.
- If you want to adjust the force after the gripper has detected an
  object, reset the object detection status by setting `rGTO` to 0 and
  then back to 1 with the new force and speed parameters. A re-grip from
  contact cannot reach the maximum grip force, which comes from the impact
  force — but it can still increase the grip force when the first grip was
  made with low speed and force, and the parameters are increased after
  object detection.
- Understand that Re-grasp is slow: it pushes with a low holding current,
  not with the force setting, so the speed setting is only an upper limit.
  If you want a faster Re-grasp, clear the object detection by changing
  the position target (for example from 254 to 255) with the speed and
  force you want. Any position change starts a new grip, with the impact
  mechanism.
- Speed and force changes are ignored once the fingers have stopped
  (object detected or target reached) if the position request stays the
  same. They only apply while the fingers are still moving towards the
  target. To apply new values after a stop, change the position request or
  toggle `rGTO`.
- Do not rely on `gOBJ` to detect a slip during Re-grasp. While the
  fingers move to re-grasp, `gOBJ` stays at 2 (`DetectedWhileClosing`). It
  only changes to 3 (`AtRequestedPosition`) when the fingers reach the
  requested position or leave the detection zone — this differs from the
  manual (section 4.5.2). To detect a slip, watch for a change of `gPO`
  while `gOBJ` is `DetectedWhileClosing`.
- Object detection only works when the fingers stop between positions 5
  and 225 (2F-85; the exact zone shifts slightly by model/profile). If
  they stop outside this zone, for example on a very thin object, `gOBJ`
  reports `AtRequestedPosition` even though an object is held. Check `gPO`
  in that case.

---

**Next:** [Robust example walkthrough →](05-robust-example-walkthrough.md)
