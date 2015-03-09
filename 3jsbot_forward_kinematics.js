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
	var T = generate_translation_matrix(joint.origin.xyz);
	var R = generate_rotation_matrix(joint.origin.rpy);
	var Mnew = matrix_multiply(M, matrix_multiply(T, R));

	joint.xform = Mnew;
	var tempmat = matrix_2Darray_to_threejs(joint.xform);
	simpleApplyMatrix(joint.geom, tempmat);

	traverse_forward_kinematics_link(joint.childLink, Mnew);
}








