'use strict';

Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJiYmQ0NmZiZC0zZGNlLTQ4ZWMtODJiYi0xYTY2OTk4MDMxNDkiLCJpZCI6ODk1MCwic2NvcGVzIjpbImFzciIsImdjIl0sImlhdCI6MTU1MzExNjM4M30.cUiEWi_tRRvqkGIY7rNG40aVA9rPKaz8vYF2wQuaLyU';

var viewer = new Cesium.Viewer('cesiumContainer',{
  infoBox: true,
  selectionIndicator: true,
  shadows: false,
  shouldAnimate: true,
  terrainProvider : Cesium.createWorldTerrain(),
});

// model inspector menu
// viewer.extend(Cesium.viewerCesiumInspectorMixin);

// web sockets
var ws = new WebSocket('ws://' + document.location.host + '/ws');
ws.onopen = function() {
  console.log('Connected to scrimmage!');
};
ws.onclose = function() {
  $('#message').text('Lost connection.');
  console.log('Connection to scrimmage closed.');
};
ws.onerror = function(error) {
  console.log('onerror ' + error);
  console.log(error);
};

ws.onNewEntity = function(entity) {
};


ws.onmessage = function(message) {
  var json;
  try {
    json = JSON.parse(message.data);
  } catch (e) {
    return;
  }

  // dispatch message based on topic
  if (json.topic === "vehicle") {
    callbackVehicle(json.data);
  } else if (json.topic === "shape") {
    callbackShape(json.data);
  }
};

function callbackVehicle(data) {
  var entity = viewer.entities.getById(data.id)
  if (entity === undefined) {
    entity = addNewEntity(data.id);
  }

  // get initial model from map, then adjust position and orientation
  var key;
  if (data.id in model_map) {
    key = data.id;
  } else {
    key = 'default';
  }
  entity.model = model_map[key];
  entity.model.rotation = model_map[key].rotation;
  if (entity.model.rotation === undefined) {
    entity.model.rotation = new Cesium.Quaternion(0, 0, 0, 1);
  }

  var position = new Cesium.Cartesian3.fromDegrees(
    data.position.lon,
    data.position.lat,
    data.position.alt);
  var orientation = transformYawPitchRoll(
    data.orientation.yaw,
    data.orientation.pitch,
    data.orientation.roll,
    position);

  entity.position = position;

  /// apply model rotation
  var quatFull = new Cesium.Quaternion();
  Cesium.Quaternion.multiply(orientation, entity.model.rotation, quatFull);
  entity.orientation = quatFull;


  if (!data.isAlive) {
    console.log("no longer alive");
    entity.model.color = Cesium.Color.RED.withAlpha(0.5);
    entity.model.runAnimations = false;
  }
}

function update_entity() {
}

function addNewEntity(id) {
  var position = new Cesium.Cartesian3(0, 0, 0);
  var orientation = new Cesium.Quaternion(0, 0, 0, 1);
  var entity = viewer.entities.add({
    name: id,
    id: id,
    position : position,
    orientation : orientation,
  });
  return entity;
}

function transformYawPitchRoll(yaw, pitch, roll, position) {
  // get quat from ENU to body
  // NOTE: Cesium defines yaw and pitch as being about the NEGATIVE
  // z- and y- axes (!)
  var hprEnuToBody = new Cesium.HeadingPitchRoll(-yaw, -pitch, roll);
  var quatEnuToBody = new Cesium.Quaternion.fromHeadingPitchRoll(hprEnuToBody);

  // get quat from ECEF to ENU
  var transformEnuToEcef = new Cesium.Transforms.eastNorthUpToFixedFrame(position);
  const rotEnuToEcef = new Cesium.Matrix3();
  Cesium.Matrix4.getMatrix3(transformEnuToEcef, rotEnuToEcef);
  var quatEnuToEcef = new Cesium.Quaternion.fromRotationMatrix(rotEnuToEcef)

  // multiply to get ECEF to body
  // NOTE: for some reason, we use EnuToEcef here...
  var quatEcefToBody = new Cesium.Quaternion()
  Cesium.Quaternion.multiply(quatEnuToEcef, quatEnuToBody, quatEcefToBody)
  return quatEcefToBody;
}

function callbackShape(data) {
  var shape = viewer.entities.getOrCreateEntity(data.id);
  shape.name = data.name;

  // handle different shapes
  if (data.ellipsoid) {
    shape.position = new Cesium.Cartesian3.fromDegrees(
      data.position[0],
      data.position[1],
      data.position[2],
    );
    shape.ellipsoid = data.ellipsoid;
    shape.ellipsoid.material = new Cesium.Color(
      data.ellipsoid.material[0],
      data.ellipsoid.material[1],
      data.ellipsoid.material[2],
      data.ellipsoid.material[3],
    )
    shape.ellipsoid.radii = new Cesium.Cartesian3(
      data.ellipsoid.radii[0],
      data.ellipsoid.radii[1],
      data.ellipsoid.radii[2],
    )
  } else if (data.polyline) {
    shape.polyline = data.polyline;
    shape.polyline.width = Math.max(1, data.polyline.width);
    shape.polyline.positions = new Cesium.Cartesian3.fromDegreesArrayHeights(
      data.polyline.positions);
    shape.polyline.material = new Cesium.Color(
      data.polyline.material[0],
      data.polyline.material[1],
      data.polyline.material[2],
      data.polyline.material[3],
    )
  } else if (data.box) {
    shape.position = new Cesium.Cartesian3.fromDegrees(
      data.position[0],
      data.position[1],
      data.position[2],
    );
    shape.box = data.box;
    shape.box.material = new Cesium.Color(
      data.box.material[0],
      data.box.material[1],
      data.box.material[2],
      data.box.material[3],
    )
    shape.box.dimensions = new Cesium.Cartesian3(
      data.box.dimensions[0],
      data.box.dimensions[1],
      data.box.dimensions[2],
    );
  }
}

// TEST ENTITIES
// var entity = viewer.entities.add({
//   name: 'Test Entity Plane',
//   position: Cesium.Cartesian3.fromDegrees(-84.395208, 33.785466, 700),
//   orientation: Cesium.Quaternion(0, 0, 0, 1),
//   model: {
//     uri: './models/CesiumAir/Cesium_Air.gltf',
//     minimumPixelSize: 128,
//   },
// });

// viewer.entities.add({
//   name: "Test Entity Shape",
//   position: Cesium.Cartesian3.fromDegrees(-84.395208, 33.785466, 500),
//   ellipsoid: {
//     radii: new Cesium.Cartesian3(200.0, 200.0, 200.0),
//     innerRadii: new Cesium.Cartesian3(100.0, 100.0, 100.0),
//     minimumCone: Cesium.Math.toRadians(60.0),
//     maximumCone: Cesium.Math.toRadians(140.0),
//     material: Cesium.Color.ORCHID.withAlpha(0.3),
//     outline: true,
//   },
// });
