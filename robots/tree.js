//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "tree";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,0], rpy:[-Math.PI/2,Math.PI/2,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "l1";  

robot.size = 255;
        
// specify and create data objects for the links of the robot
robot.links = {};
for (var i=1; i<=robot.size; i++) {
    robot.links["l"+i] = {};
}
/* for you to do
, "shoulder_left": {}  , "upperarm_left": {} , "forearm_left": {} };
*/

function linkLength(id) {
    return Math.pow(0.5, Math.floor(Math.log2(id)));
}


//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

for (var i=2; i<=robot.size; i++) {
    var parentId = Math.floor(i/2);
    robot.joints["j"+i] = {parent:"l"+parentId, child:"l"+i};
    robot.joints["j"+i].origin = {
        xyz: [0.0,0.0,linkLength(parentId)], 
        rpy: [Math.PI/4*((i%2)*2-1),0,0]
    };
    robot.joints["j"+i].axis = [0.0,1.0,0.0]; 
}

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};
joints_geom = {};

for (var i=1; i<=robot.size; i++) {
    var l = linkLength(i);
    links_geom["l"+i] = new THREE.CubeGeometry( l*0.25, l*0.25, l*1.0 );
    links_geom["l"+i].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, l*0.5) );

    joints_geom["j"+i] = l*0.2;

}






