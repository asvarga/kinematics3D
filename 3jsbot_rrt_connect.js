//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

/*
CS148: reference code has functions for:

    tree_add_vertex
    tree_add_edge
    random_config
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/

function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    q_L = q_start_config.length;

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // CS148: add necessary RRT initialization here
    tree_a = tree_init(q_start_config);
    tree_b = tree_init(q_goal_config);

    eps = 0.5;
    K = 3;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    console.log("planner initialized");
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (typeof rrt_iterate !== 'undefined' && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // CS148: implement RRT iteration here
        var q_rand = random_config();
        if (rrt_extend(tree_a, q_rand) !== "trapped" && rrt_connect(tree_b, q_rand) === "reached") {
            return find_path(tree_a, tree_b);
        }
        if (rrt_extend(tree_b, q_rand) !== "trapped" && rrt_connect(tree_a, q_rand) === "reached") {
            return find_path(tree_a, tree_b);
        }
    }

    // return path not currently found
    return false;
}

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}


function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
}

///////////

function tree_add_vertex(T, q, parent) {
    var vertex = {
        vertex: q,
        edges: [],
        parent: parent
    }
    T.vertices.push(vertex);

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(vertex);

    parent.edges.push(vertex);
}

// function tree_add_edge() {

// }

function random_config() {
    var x = Math.random()*(robot_boundary[1][0]-robot_boundary[0][0])+robot_boundary[0][0];
    var z = Math.random()*(robot_boundary[1][2]-robot_boundary[0][2])+robot_boundary[0][2];
    var rot = Math.random()*Math.PI*2;
    var config = [x, 0, z, 0, rot, 0];
    for (var i=6; i<q_L; i++) {
        config.push(Math.random()*Math.PI*2);
    }
    return config;
}

function new_config(q_rand, q_near) {
    var D = diff(q_rand, q_near);
    var len = get_len(D);

    var config = [];
    if (len < eps) {
        config = q_rand;
    } else {
        for (var i=0; i<q_L; i++) {
            config.push(q_near[i] + D[i]/len*eps);
        }
    }

    if (robot_collision_test(config)) {
        return false;
    }
    return config;
}

function nearest_neighbor(q, T) {   // nothing fancy, pretty slow
    var nearest = null;
    var nearest_dist = Infinity;
    for (var i=0; i<T.vertices.length; i++) {
        var D = diff(q, T.vertices[i].vertex);
        var len = get_len(D);
        if (len < nearest_dist) {       // new nearest
            nearest_dist = len;
            nearest = T.vertices[i];
        }
    }
    return nearest;
}

function rrt_extend(T, q_rand) {
    var q_near = nearest_neighbor(q, T);
    var q_new = new_config(q_rand, q_near)
    if (q_new) {
        tree_add_vertex(T, q_new, q_near);
        var D = diff(q_new, q_near);
        var len = get_len(D);
        if (len < eps/1000) {   // basically q_new == q_near
            return "reached";
        } else {
            return "advanced";
        }
    }
    return "trapped";
}

function rrt_connect(T, q_rand) {
    var S = "advanced";
    while (S === "advanced") {
        S = rrt_extend(T, q_rand);
    }
    return S;
}

function find_path() {

}

function path_dfs() {

}

function diff(a, b) {
    var D = [];
    for (var i=0; i<3; i++) {           // positions
        var d = a[i]-b[i];
        D.push(d);
    }
    for (var i=3; i<q_L; i++) {         // angles
        var d = fix_angle(a[i]-b[i]);
        D.push(d);
    }
}

function get_len(x) {
    var len2 = 0;
    for (var i=0; i<x.length; i++) {           // positions
        len2 += x[i]*x[i];
    }
    return Math.sqrt(len2);
}

function fix_angle(x) {     // get into [-PI, PI] range
    return (x + Math.PI*5)%(2*Math.PI)-Math.PI;
}









