//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/


function quaternion_from_axisangle(axis, angle) {
	var cos = Math.cos(angle/2);
	var sin = Math.sin(angle/2);

	var x = axis[0];
	var y = axis[1];
	var z = axis[2];

	return [cos, x*sin, y*sin, z*sin];
}

function quaternion_normalize(Q) {
	var a = Q[0];
	var b = Q[1];
	var c = Q[2];
	var d = Q[3];

	var norm = Math.sqrt(a*a+b*b+c*c+d*d);

	return [a/norm, b/norm, c/norm, d/norm];
}

function quaternion_multiply(Q1, Q2) {
	var a = Q1[0];
	var b = Q1[1];
	var c = Q1[2];
	var d = Q1[3];
	var e = Q2[0];
	var f = Q2[1];
	var g = Q2[2];
	var h = Q2[3];

	var ret = [];
	ret[0] = a*e-b*f-c*g-d*h;
	ret[1] = a*f+b*e+c*h-d*g;
	ret[2] = a*g-b*h+c*e+d*f;
	ret[3] = a*h+b*g-c*f+d*e;

	return ret;
}

function quaternion_to_rotation_matrix(Q) {
	var ret = [[],[],[],[]];
	var q0 = Q[0];
	var q1 = Q[1];
	var q2 = Q[2];
	var q3 = Q[3];

	ret[0][0] = q0*q0+q1*q1-q2*q2-q3*q3;
	ret[0][1] = 2*(q1*q2-q0*q3);
	ret[0][2] = 2*(q0*q2+q1*q3);
	ret[0][3] = 0;
	ret[1][0] = 2*(q1*q2+q0*q3);
	ret[1][1] = q0*q0-q1*q1+q2*q2-q3*q3;
	ret[1][2] = 2*(q2*q3-q0*q1);
	ret[1][3] = 0;
	ret[2][0] = 2*(q1*q3-q0*q2);;
	ret[2][1] = 2*(q0*q1+q2*q3);;
	ret[2][2] = q0*q0-q1*q1-q2*q2+q3*q3;
	ret[2][3] = 0;

	ret[3] = [0,0,0,1];

	return ret;
}

function quaternion_conjugate(Q) {
	return [Q[0], -Q[1], -Q[2], -Q[3]];
}

function quaternion_inverse(Q) {
	return quaternion_normalize(quaternion_conjugate(Q));
}

function rotate_point_by_quaternion(v, Q) {
	var v0 = [0, v[0], v[1], v[2]];
	var Qinv = quaternion_inverse(Q);

	return quaternion_multiply(quaternion_multiply(Q, v0), Qinv);
}

////////

function rotation_matrix_from_axisangle(axis, angle) {
	return quaternion_to_rotation_matrix(quaternion_from_axisangle(axis, angle));
}




