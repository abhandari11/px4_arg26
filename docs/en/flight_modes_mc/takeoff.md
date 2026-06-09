# Takeoff Mode (Multicopter)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Takeoff_ flight mode causes the vehicle to take off to a specified height and wait for further input.

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires at least a valid local position estimate (does not require a global position).
  - Flying vehicles can't switch to this mode without valid local position.
  - Flying vehicles will failsafe if they lose the position estimate.
  - Disarmed vehicles can switch to mode without valid position estimate but can't arm.
- RC control switches can be used to change flight modes.
- RC stick movement will [by default](#COM_RC_OVERRIDE) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.
- The [Failure Detector](../config/safety.md#failure-detector) will automatically stop the engines if there is a problem on takeoff.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Technical Summary

A multi rotor ascends vertically to the altitude defined in [MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT) and holds position.

RC stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#COM_RC_OVERRIDE)).

### Parameters

Takeoff is affected by the following parameters:

| Parameter                                                                                                | Description                                                                                                                                                                                                                                                  |
| -------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MIS_TAKEOFF_ALT"></a>[MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT) | Target altitude during takeoff (default: 2.5m)                                                                                                                                                                                                               |
| <a id="MPC_TKO_SPEED"></a>[MPC_TKO_SPEED](../advanced_config/parameter_reference.md#MPC_TKO_SPEED)       | Speed of ascent (default: 1.5m/s)                                                                                                                                                                                                                            |
| <a id="MPC_TKO_RAMP_T"></a>[MPC_TKO_RAMP_T](../advanced_config/parameter_reference.md#MPC_TKO_RAMP_T)    | Smooth takeoff ramp time: how long the climb rate ramps up to [MPC_TKO_SPEED](#MPC_TKO_SPEED) (default: 3.0s)                                                                                                                                                |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE) | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md). This can be separately enabled for auto modes and for offboard mode, and is enabled in auto modes by default. |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | The amount of stick movement that causes a transition to [Position mode](../flight_modes_mc/position.md) (if [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) is enabled)                                                                                                 |

## Takeoff From a Moving Platform (Boat Deck) <Badge type="tip" text="PX4 v1.19" />

Automatic takeoff from a moving platform, such as a boat deck, is supported. The multicopter climbs straight up and holds the point where the takeoff was commanded.

::: warning
Given that the vehicle flies to the point where the takeoff was commanded and it does not travel along with the boat, always take off from the **back (stern) of the boat**, with clear open space behind it.
Taking off from the bow or the middle of the boat can leave it behind across the deck and into masts, antennas, superstructure, or crew.
:::

For takeoffs from a fast-moving deck, increase [MPC_TKO_SPEED](#MPC_TKO_SPEED) and [MPC_TKO_RAMP_T](#MPC_TKO_RAMP_T) so the vehicle climbs high enough to clear the deck before it brakes to a stop over the water. The faster the boat, the more height the vehicle needs during lift-off.

::: info
Takeoff from a moving platform is only supported in [Takeoff mode](#technical-summary) (or the equivalent coordinate-less takeoff commanded from a ground station), not as part of a [Mission](../flight_modes_mc/mission.md).
A mission takeoff item is tied to a fixed coordinate and will not follow the deck.
To run a mission from a boat, take off first in Takeoff mode and then start the mission once the vehicle is airborne.
:::

The moving-platform case can be exercised in simulation with the Gazebo [Moving Platform world](../sim_gazebo_gz/worlds.md#moving-platform).

## See Also

- [Throw Launch (MC)](../flight_modes_mc/throw_launch.md)
- [Takeoff Mode (FW)](../flight_modes_fw/takeoff.md)

<!-- this maps to AUTO_TAKEOFF in dev -->
