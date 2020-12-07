rosservice call /loadMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
file_name: 'shelf'
object_id: 'shelf'
rgba: {r: 255.0, g: 255.0, b: 255.0, a: 1.0}
pose:
  position: {x: -0.1, y: -0.95, z: 1.0}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}"
rosservice call /loadMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
file_name: 'shelf'
object_id: 'shelf2'
rgba: {r: 255.0, g: 255.0, b: 255.0, a: 1.0}
pose:
  position: {x: -0.1, y: 1.75, z: 1.0}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}"
rosservice call /loadMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
file_name: 'CQtable'
object_id: 'table'
rgba: {r: 255.0, g: 255.0, b: 255.0, a: 1.0}
pose:
  position: {x: 0.50, y: -0.50, z: 1.02}
  rpy: {R: 0.0, P: 0.0, Y: 0}"
rosservice call /loadMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
file_name: 'CQtable'
object_id: 'table2'
rgba: {r: 255.0, g: 255.0, b: 255.0, a: 1.0}
pose:
  position: {x: 0.50, y: 1.0, z: 1.02}
  rpy: {R: 0.0, P: 0.0, Y: 0}"
rosservice call /loadObject "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
object_id: ['floor', 'ceil', 'behind', 'column']
primitives:
- type: 1
  dimensions: [4, 4, 0.01]
- type: 1
  dimensions: [4, 4, 0.01]
- type: 1
  dimensions: [0.01, 2, 1]
- type: 1
  dimensions: [0.04, 0.04, 1.5]
pose:
- position: {x: 0.0, y: 0.5, z: 1.02}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: 0.0, y: 0.5, z: 1.95}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: -0.45, y: 0.0, z: 1.5}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
- position: {x: -0.28, y: 0.35, z: 1.75}
  rpy: {R: 0.0, P: 0.0, Y: 0.0}
rgba:
- {r: 0.0, g: 255.0, b: 0.0, a: 1.0}
- {r: 0.0, g: 255.0, b: 0.0, a: 0}
- {r: 0.0, g: 255.0, b: 0.0, a: 0}
- {r: 255.0, g: 255.0, b: 255.0, a: 1}"
