# Script for control Uavos GSG140D/T gimbals

## Default variables
**radar_dy**: gimbal's yaw in global coords
**radar_dz**: gimbal's pitch in global coords
**radar_dx**: gimbal's roll in global coords
**user1**: gimbal's yaw in local coords
**user2**: gimbal's pitch in local coords

## Configuration
**Control modes**:
- parking (power_payload)
- by position (cam_mode=2, camcmd_yaw, camcmd_pitch)
- by speed (cam_mode=3, camcmd_yaw, camcmd_pitch)
- look here (click on map)
- follow (cam_mode=5)
- course (cam_mode=6)

Line 1:
**PORT_ID**: virtual port id for communication with gimbal
Lines 54-55:
**FIXED_POS_YAW = 0**: yaw pos in cam_mode=5 (follow mode)
**FIXED_POS_PITCH = -45**: pitch pos in cam_mode=5 (follow mode)
Lines 60-65:
**g_yawOffset**: offset for gimbal's yaw
**g_pitchOffset**: offset for gimbal's pitch
**g_yawPosReverse**: multiplier for gimbal's yaw
**g_pitchPosReverse**: multiplier for gimbal's pitch
**g_yawCmdReverse**: multiplier for yaw commands
**g_pitchCmdReverse**: multiplier for pitch commands
