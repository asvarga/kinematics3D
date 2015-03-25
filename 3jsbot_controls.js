//////////////////////////////////////////////////
/////     ROBOT DYNAMICS PLACEHOLDER 
//////////////////////////////////////////////////

// CS148: add kinematics update from controls here
function robot_apply_controls() {
// apply robot controls to robot kinematics transforms and joint angles, then zero controls
// (for now) includes update of camera position based on base movement 

	function apply_control_to_joint(joint) {
		joint.angle += joint.control;
		joint.control = 0;

		var link = joint.childLink;
		if (link) {
			for (var i in link.childJoints) {
				apply_control_to_joint(link.childJoints[i]);
			}
		}
	}

	for (var i in robot.baseLink.childJoints) {
		apply_control_to_joint(robot.baseLink.childJoints[i]);
	}

	// robot.control.xyz = [0.1, 0, 0];
	var T = generate_translation_matrix(robot.control.xyz);
	robot.origin.xyz = multiply_matrix_vector(T, robot.origin.xyz);
	robot.control.xyz = [0, 0, 0];

	// robot.control.rpy = [0, 0.01, 0];
	robot.origin.rpy = vector_add(robot.origin.rpy, robot.control.rpy);
	robot.control.rpy = [0, 0, 0];

    // move camera with robot base 
    // CS148: do not delete this
    // (need to pull this out and into 3jsbot support, at some point)
    camera_controls.object.position.x += robot.control.xyz[0];
    camera_controls.object.position.y += robot.control.xyz[1];
    camera_controls.object.position.z += robot.control.xyz[2];

}


