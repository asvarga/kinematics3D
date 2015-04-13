//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {

	endeffector_joint = robot.joints[endeffector_joint];

	var end_pos_ax = get_global_pos_axis(endeffector_joint);

	function build_jacobian(joint) {
		var pos_ax = get_global_pos_axis(joint);
    	var zi_1 = pos_ax.axis;
    	var on = end_pos_ax.position;
    	var oi_1 = pos_ax.position;
    	var on_oi_1 = [on[0]-oi_1[0], on[1]-oi_1[1], on[2]-oi_1[2]];
    	var zi_1xon_oi_1 = vector_cross(zi_1, on_oi_1);

    	var Ji = zi_1xon_oi_1.concat(zi_1);

    	var J = [];
    	if (joint.parentLink && joint.parentLink.parentJoint) {
    		J = build_jacobian(joint.parentLink.parentJoint);
    	}
    	J.push(Ji);
    	return J;
    }

    function apply_jacobian(joint, J_1, dx) {
    	var J_1i = J_1.pop();
    	var dq = J_1i[0]*dx[0] + J_1i[1]*dx[1] + J_1i[2]*dx[2];
    	joint.control = -0.04*dq;

    	if (joint.parentLink && joint.parentLink.parentJoint) {
    		apply_jacobian(joint.parentLink.parentJoint, J_1, dx);
    	}
    }

    var M = endeffector_joint.xform;
	var v = endeffector_local_pos.slice(0, 3);
	var xd = multiply_matrix_vector(M, v).slice(0, 3);	// current endpoint
	var x = target_pos.slice(0, 3);			// desired endpoint
    var dx = [xd[0]-x[0], xd[1]-x[1], xd[2]-x[2]];	// error

    dx[1] *= -1;	// I HAVE ZERO EXPLANATION FOR THIS

    // compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {
    	var Jt = build_jacobian(endeffector_joint);
    	var J_1 = Jt;
    	if (j_pseudo_inv) {
    		var J = matrix_transpose(Jt);
    		J_1 = pseudoinverse(J);
    		// console.log([J_1.length, J_1[0].length]);
    	}

    	apply_jacobian(endeffector_joint, J_1, dx);

        //iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
        endeffector_geom.visible = true;
        target_geom.visible = true;
    } else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

    // draw endeffector and target position indicators
    var end_transform = generate_translation_matrix(xd);
    var endeffector_mat = matrix_2Darray_to_threejs(end_transform);
    simpleApplyMatrix(endeffector_geom,endeffector_mat);

    var target_transform = generate_translation_matrix(x);
    var target_mat = matrix_2Darray_to_threejs(target_transform);
    simpleApplyMatrix(target_geom,target_mat);
}

// function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {

// }
