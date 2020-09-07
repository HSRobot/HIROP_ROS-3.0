rosservice call /loadObject "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'base_link'
object_id: ['box1', 'l_box2', 'r_box3', 'f_box4', 'ceil', 'taizi']
primitives:
- type: 1
  dimensions: [0.01, 1.6, 1.2]
- type: 1
  dimensions: [1.5, 0.01, 1.2]
- type: 1
  dimensions: [1.5, 0.01, 1.2]
- type: 1
  dimensions: [1.5, 1.6, 0.01]
- type: 1
  dimensions: [1.5, 1.6, 0.01]
- type: 1
  dimensions: [0.35, 0.35, 0.12]
pose:
- position: {x: -0.9, y: -0.13, z: 0.6}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: -0.15, y: -0.90, z: 0.6}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: -0.15, y: 0.50, z: 0.6}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: -0.15, y: -0.13, z: 0.03}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: -0.15, y: -0.13, z: 1.2}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: 0.4, y: -0.45, z: 0.06}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
rgba:
- {r: 0.0, g: 255, b: 0.0, a: 0.5}
- {r: 0.0, g: 255, b: 0.0, a: 0.5}
- {r: 0.0, g: 255, b: 0.0, a: 0.5}
- {r: 0.0, g: 255, b: 0.0, a: 0.5}
- {r: 0.0, g: 255, b: 0.0, a: 0.1}
- {r: 0.0, g: 255, b: 0.0, a: 1.0}"