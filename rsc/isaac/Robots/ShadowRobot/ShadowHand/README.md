# ShadowHand USD Asset

This directory contains the Shadow Robot ShadowHand USD files used by the
`shadow_hand_usd_cube` example.

Source:
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/ShadowRobot/ShadowHand/`

Downloaded files:
- `shadow_hand.usd`
- `shadow_hand_instanceable.usd`

NVIDIA's Isaac Sim robot asset documentation lists
`ShadowRobot/ShadowHand/shadow_hand_instanceable.usd` as an Isaac Lab robot asset
with a BSD-3 license. The BSD-3 license text is included in `LICENSE`.

The Isaac Lab cube asset (`Props/Blocks/DexCube/dex_cube_instanceable.usd`) is
not vendored here. The RaiSim example uses a native `world.addBox` cube for
physics and a small procedural OBJ/MTL/PNG visual proxy under
`rsc/isaac/Props/Blocks/DexCube` so the TCP viewer shows a textured cube without
redistributing NVIDIA's separate DexCube prop asset.
