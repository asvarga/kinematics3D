//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/

function robot_forward_kinematics() {
	var T = generate_translation_matrix(robot.origin.xyz);
	var R = generate_rotation_matrix(robot.origin.rpy);
	var M = matrix_multiply(T, R);

	traverse_forward_kinematics_link(robot.baseLink, M);

	var heading = generate_translation_matrix([0,0,1]);
	heading = matrix_multiply(R, heading);
	heading = matrix_multiply(T, heading);
	var heading3 = matrix_2Darray_to_threejs(heading);
	simpleApplyMatrix(robot.heading_geom, heading3);

	var lateral = generate_translation_matrix([1,0,0]);
	lateral = matrix_multiply(R, lateral);
	lateral = matrix_multiply(T, lateral);
	var lateral3 = matrix_2Darray_to_threejs(lateral);
	simpleApplyMatrix(robot.lateral_geom, lateral3);
}

function traverse_forward_kinematics_link(link, M) {
	link.xform = M;
	var tempmat = matrix_2Darray_to_threejs(link.xform);
	simpleApplyMatrix(link.geom, tempmat);

	// for (var jointName in link.childJoints) {
	// 	traverse_forward_kinematics_joint(link.childJoints[jointName], M);
	// }

	for (var i=0; i<link.childJoints.length; i++) {
		traverse_forward_kinematics_joint(link.childJoints[i], M);
	}
}

function traverse_forward_kinematics_joint(joint, M) {

	if (robot.wavy) {
		var i = parseInt(joint.name.substring(1));
		var depth = Math.floor(Math.log(i*3+4)/Math.log(4)+Math.pow(10, -6));
		joint.origin.rpy[0] = Math.PI/4 + Math.PI/8 * Math.sin(Date.now()/1000);// + depth);
		// joint.origin.rpy[2] = Math.PI/2*(i%4) + Date.now()/1000;
	}

	var C = rotation_matrix_from_axisangle(joint.axis, joint.angle || 0);

	// only translate joint.origin.geom
	var T = generate_translation_matrix(joint.origin.xyz);
	var Mnew = matrix_multiply(M, T);
	var MnewControl = matrix_multiply(Mnew, C);

	joint.origin.xform = MnewControl;
	var tempmat = matrix_2Darray_to_threejs(joint.origin.xform);
	simpleApplyMatrix(joint.origin.geom, tempmat);

	////

	// translate and rotate joint.geom
	var R = generate_rotation_matrix(joint.origin.rpy);
	var Mnewer = matrix_multiply(Mnew, R);
	var MnewerControl = matrix_multiply(Mnewer, C);

	joint.xform = MnewerControl;
	var tempmat2 = matrix_2Darray_to_threejs(joint.xform);
	simpleApplyMatrix(joint.geom, tempmat2);

	traverse_forward_kinematics_link(joint.childLink, MnewerControl);
}

// function compute_and_draw_heading() {  }






