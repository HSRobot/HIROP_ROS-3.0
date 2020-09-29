rosservice call /loadMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
file_name: 'shelf'
object_id: 'shelf'
rgba: {r: 255.0, g: 255.0, b: 255.0, a: 1.0}
pose:
  position: {x: -0.2, y: -1, z: 1.0}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}"
rosservice call /loadMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
file_name: 'CQtable'
object_id: 'table'
rgba: {r: 255.0, g: 255.0, b: 255.0, a: 1.0}
pose:
  position: {x: 0.85, y: -0.45, z: 0.86}
  rpy: {R: 0.0, P: 0.0, Y: 1.57}"
rosservice call /loadObject "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
object_id: ['floor', 'ceil', 'hou']
primitives:
- type: 1
  dimensions: [2, 2, 0.01]
- type: 1
  dimensions: [2, 2, 0.01]
- type: 1
  dimensions: [0.01, 2, 1]
pose:
- position: {x: 0.0, y: 0.0, z: 1.02}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: 0.0, y: 0.0, z: 1.95}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: -0.43, y: 0.0, z: 1.5}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
rgba:
- {r: 0.0, g: 255.0, b: 0.0, a: 1.0}
- {r: 0.0, g: 255.0, b: 0.0, a: 0}
- {r: 0.0, g: 255.0, b: 0.0, a: 0.5}"
