//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

matrix_multiply
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/

function get_col(M, i) {
	return [M[0][i], M[1][i], M[2][i], M[3][i]];
}

function matrix_multiply(M1, M2) {
	var ret = [[],[],[],[]];
	for (var y=0; y<M1.length; y++) {
		for (var x=0; x<M2.length; x++) {
			ret[y].push(vector_dot(M1[y], get_col(M2, x)));
		}	
	}
	return ret;
}

function matrix_transpose(M) {
	var ret = [];
	for (var x=0; x<M[0].length; x++) {
		ret.push([]);
	}
	for (var y=0; y<M.length; y++) {
		for (var x=0; x<M[y].length; x++) {
			ret[x].push(M[y][x]);
		}
	}
	return ret;
}

function vector_length(v) {
	return Math.sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2))
}

function vector_normalize(v) {
	var l = vector_length(v);
	var ret = [];
	for (var i=0; i<v.length; i++) { ret[i] = v[i]; }
	ret[0] /= l;
	ret[1] /= l;
	ret[2] /= l;
	return ret;
}

function vector_cross(u, v) {
	return [u[1]*v[2]-u[2]*v[1], u[0]*v[2]-u[2]*v[0], u[0]*v[1]-u[1]*v[0]];
}

function vector_dot(u, v) {
	return u[0]*v[0]+u[1]*v[1]+u[2]*v[2]+u[3]*v[3];
}

function generate_identity() {
	return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
}

function generate_translation_matrix(v) {
	return [[1,0,0,v[0]],[0,1,0,v[1]],[0,0,1,v[2]],[0,0,0,1]];
}

function generate_rotation_matrix_X(t) {
	return [
		[1,0,0,0],
		[0,Math.cos(t),-Math.sin(t),0],
		[0,Math.sin(t),Math.cos(t),0],
		[0,0,0,1]
	];
}

function generate_rotation_matrix_Y(t) {
	return [
		[Math.cos(t),0,Math.sin(t),0],
		[0,1,0,0],
		[-Math.sin(t),0,Math.cos(t),0],
		[0,0,0,1]
	];
}

function generate_rotation_matrix_Z(t) {
	return [
		[Math.cos(t),-Math.sin(t),0,0],
		[Math.sin(t),Math.cos(t),0,0],
		[0,0,1,0],
		[0,0,0,1]
	];
}

function generate_rotation_matrix(v) {
	var rx = generate_rotation_matrix_X(v[0]);
	var ry = generate_rotation_matrix_Y(v[1]);
	var rz = generate_rotation_matrix_Z(v[2]);
	return matrix_multiply(rz, matrix_multiply(ry, rx));
}







