//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {

	var date = new Date();
	var t = date.getTime();
	var dt = (t - robot.prevt)/1000.0;
	robot.prevt = t;

	var angle = date.getSeconds()/60*2*Math.PI;

	function pd_control_joint(joint) {

		if (keyboard.pressed("o")) {
			joint.pd.desired = angle;
		}
		var error = (joint.pd.desired - joint.angle + 5*Math.PI) % (2*Math.PI) - Math.PI;
		
		var v = joint.pd.vals;
		var k = joint.pd.k;

		v.d = (error - v.p)/dt;
		v.p = error;

		var wsum = k.p * v.p + k.d * v.d;

		joint.control = wsum*dt;

		var link = joint.childLink;
		if (link) {
			for (var i in link.childJoints) {
				pd_control_joint(link.childJoints[i]);
			}
		}
	}

	for (var i in robot.baseLink.childJoints) {
		pd_control_joint(robot.baseLink.childJoints[i]);
	}
}

