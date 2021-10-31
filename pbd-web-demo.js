"use strict";



var canvas = null;//document.getElementById("myCanvas");
var c = null;//canvas.getContext("2d");

var simMinWidth = 2.0;
var cScale = 1;//Math.min(canvas.width, canvas.height) / simMinWidth;
var simWidth = 1;//canvas.width / cScale;
var simHeight = 1;//canvas.height / cScale;


var doc = null;

var printError = function(error, explicit) {
    console.log(`[${explicit ? 'EXPLICIT' : 'INEXPLICIT'}] ${error.name}: ${error.message}`);
}

function setupGlobals(canvas_, win, doc_) {
    canvas = canvas_;
    c = canvas.getContext("2d");
    canvas.width = win.innerWidth - 20;
    canvas.height = win.innerHeight - 100;

    simMinWidth = 2.0;
    cScale = Math.min(canvas.width, canvas.height) / simMinWidth;
    simWidth = canvas.width / cScale;
    simHeight = canvas.height / cScale;

    doc = doc_;
}

function cX(posX) {
    return posX * cScale;
}

function cY(posY) {
    return canvas.height - posY * cScale;
}

// vector math -------------------------------------------------------

function length(x0, y0, x1, y1) {
    return Math.sqrt((x0 - x1)*(x0-x1) + (y0-y1)*(y0-y1));
}

class Vector2 {
    constructor(x = 0.0, y = 0.0) {
        this.x = x; 
        this.y = y;
    }

    set(v) {
        this.x = v.x; this.y = v.y;
    }

    clone() {
        return new Vector2(this.x, this.y);
    }

    add(v, s = 1.0) {
        this.x += v.x * s;
        this.y += v.y * s;
        return this;
    }

    addVectors(a, b) {
        this.x = a.x + b.x;
        this.y = a.y + b.y;
        return this;
    }

    subtract(v, s = 1.0) {
        this.x -= v.x * s;
        this.y -= v.y * s;
        return this;
    }

    subtractVectors(a, b) {
        this.x = a.x - b.x;
        this.y = a.y - b.y;
        return this;			
    }

    length() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    scale(s) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    dot(v) {
        return this.x * v.x + this.y * v.y;
    }

    perp() {
        return new Vector2(-this.y, this.x);
    }

    rotate(m) {
        var x = m.e00 * this.x + m.e01 * this.y;
	    var y = m.e10 * this.x + m.e11 * this.y;
        this.x = x;
        this.y = y;
    }
}

class Mat2 {
    constructor(e00_, e01_, e10_, e11_) {
        this.e00 = e00_;
        this.e01 = e01_;
        this.e10 = e10_;
        this.e11 = e11_;
    }

    static rotation(angle) {
 	    var cosA = Math.cos(angle);
        var sinA = Math.sin(angle);

	    return new Mat2(cosA, -sinA, sinA,  cosA);
    }

}

class ConstraintsArray {
    static IS = 2;
    static DS = 7;

    constructor(size) { 
        this.Idx = new Int32Array(ConstraintsArray.IS*size); 
        this.Data = new Float32Array(ConstraintsArray.DS*size); 
        this.maxSize = size;
        this.size = 0;
    }

    clear() {
        this.size = 0;
    }

    pushBack(A, B, dvx, dvy, nx, ny, d_lambda_n, mu_k, e) {
        if (this.size >= this.maxSize) {
            this.maxSize *= 2;
            var oldIdx = this.Idx;
            var oldData = this.Data;

            this.Idx = new Int32Array(ConstraintsArray.IS*this.maxSize); 
            this.Data = new Float32Array(ConstraintsArray.DS*this.maxSize); 

            for (var i = 0; i < oldIdx.length; ++i)
            {
                this.Idx[i] = oldIdx[i];
            }
            for (i = 0; i < oldData.length; ++i)
            {
                this.Data[i] = oldData[i];
            }
        }
        var is = ConstraintsArray.IS;
        var ds = ConstraintsArray.DS;
        this.Idx[is*this.size + 0] = A; 
        this.Idx[is*this.size + 1] = B; 

        this.Data[ds*this.size + 0] = dvx; 
        this.Data[ds*this.size + 1] = dvy; 
        this.Data[ds*this.size + 2] = nx; 
        this.Data[ds*this.size + 3] = ny; 
        this.Data[ds*this.size + 4] = d_lambda_n; 
        this.Data[ds*this.size + 5] = mu_k; 
        this.Data[ds*this.size + 6] = e; 
        this.size++;
    }

    idxA(i) {
        return this.Idx[ConstraintsArray.IS*i+ 0]; 
    }
    idxB(i) {
        return this.Idx[ConstraintsArray.IS*i + 1]; 
    }
    dvx(i) {
        return this.Data[ConstraintsArray.DS*i + 0]; 
    }
    dvy(i) {
        return this.Data[ConstraintsArray.DS*i + 1]; 
    }
    nx(i) {
        return this.Data[ConstraintsArray.DS*i + 2]; 
    }
    ny(i) {
        return this.Data[ConstraintsArray.DS*i + 3]; 
    }
    d_lambda_n(i) {
        return this.Data[ConstraintsArray.DS*i + 4]; 
    }
    mu_k(i) {
        return this.Data[ConstraintsArray.DS*i + 5]; 
    }
    e(i) {
        return this.Data[ConstraintsArray.DS*i + 6]; 
    }
}


class Vector {
    constructor(size) { 
        this.vals = new Int32Array(size); 
        this.maxSize = size;
        this.size = 0;
    }
    clear() {
        this.size = 0;
    }
    pushBack(val) {
        if (this.size >= this.maxSize) {
            this.maxSize *= 2;
            var old = this.vals;
            this.vals = new Int32Array(this.maxSize);
            for (var i = 0; i < old.length; i++)
                this.vals[i] = old[i];
        }
        this.vals[this.size++] = val;
    }
}

// boundary class -------------------------------------------------------
class CircleBoundary {
    constructor(pos_, radius_, inverted_) { 
        this.pos = new Vector2(pos_.x, pos_.y);
        this.radius = radius_;
        this.inverted = inverted_;
    }
}

// scene -------------------------------------------------------

var physicsScene = 
    {
        gravity : new Vector2(0.0, -9.81),
        dt : 1.0 / 60.0,
        numSteps : 10,
        paused : false,
        numRows:  50,       
        numColumns:  3,
        numParticles: 0,
        boundaryCenter: new Vector2(0.0, 0.0),
        boundaryRadius: 0.0,
        boundaries: null,
        rotation_angle: 0.0
    };

var particleRadius = 0.01;
var maxVel = 0.4 * particleRadius;
var mu_s = 0.2; // static friction
var mu_k = 0.2; // dynamic friction
var e = 0.2; // restitution

// if enabled, then using velocity pass as described in 
// Detailed Rigid Body Sumulatio with Extended PBD
var use_velocity_pass = false;

var maxParticles = 10000;

var particles = {
    pos : new Float32Array(2 * maxParticles),
    prev : new Float32Array(2 * maxParticles),
    vel : new Float32Array(2 * maxParticles)
}

var constraints = new ConstraintsArray(1);

// -------------------------------------------------------

function setupScene() 
{
    // init particle positions
    var particleDiameter = 2*particleRadius;

    if(physicsScene.numColumns*physicsScene.numRows > maxParticles)
    {
        alert("Too many particles, please increase maxPariticles value")
        return;
    }

    physicsScene.radius = simMinWidth * 0.05;
    physicsScene.paused = true;
    physicsScene.boundaryCenter.x = simWidth / 2.0;
    physicsScene.boundaryCenter.y = simHeight / 2.0;
    physicsScene.boundaryRadius = simMinWidth * 0.4;

    var offsetX = physicsScene.boundaryCenter.x - physicsScene.numColumns*particleDiameter*0.5;
    var offsetY = physicsScene.boundaryCenter.y + physicsScene.numRows*particleDiameter*0.5;

    //offsetX += particleRadius * 50;
    //offsetY -= particleRadius * 50;

    var idx = 0;
    for (var y = 0; y < physicsScene.numRows; y++) {
        for (var x = 0; x < physicsScene.numColumns; x++) {
            // x
            particles.pos[idx] = offsetX + x * 1.2*particleDiameter;
            particles.pos[idx] += 0.01 * particleDiameter * (y % 2);
            // y
            particles.pos[idx+ 1] = offsetY - y * particleDiameter;

            // v.x v.y
            particles.vel[idx] = 0.0;
            particles.vel[idx + 1] = 0.0;
            idx += 2;				
        }
    }
    physicsScene.numParticles = physicsScene.numColumns * physicsScene.numRows;

    // add some more boundaries
    physicsScene.boundaries = new Array();
    physicsScene.boundaries.push(new CircleBoundary(physicsScene.boundaryCenter,
        physicsScene.boundaryRadius, true));

    var num_boundaries = 5;
    var delta_angle = 360 / num_boundaries;
    var smallBoundaryRadius = 0.25*physicsScene.boundaryRadius;
    var bpos = new Vector2(0,0);
    for(var i=0;i<num_boundaries;++i)
    {
        var rot_m = Mat2.rotation(delta_angle*i*Math.PI/180.0)
        bpos.x = physicsScene.boundaryRadius;
        bpos.y = 0;
        bpos.rotate(rot_m);
        bpos.x += physicsScene.boundaryCenter.x;
        bpos.y += physicsScene.boundaryCenter.y;
        physicsScene.boundaries.push(new CircleBoundary(bpos, smallBoundaryRadius, false));
    }

    initNeighborsHash();
    constraints.clear();

    doc.getElementById("mu_s_slider").setAttribute("value", 100*mu_s);		
    doc.getElementById("mu_k_slider").setAttribute("value", 100*mu_k);		
    doc.getElementById("e_slider").setAttribute("value", 100*e);		
    doc.getElementById("mu_k").innerHTML = mu_k;
    doc.getElementById("mu_s").innerHTML = mu_s;
    doc.getElementById("e").innerHTML = e;
    doc.getElementById("use_velocity_pass").checked = use_velocity_pass;
}

// draw -------------------------------------------------------

function drawCircle(posX, posY, radius, filled)
{
    c.beginPath();			
    c.arc(
        cX(posX), cY(posY), cScale * radius, 0.0, 2.0 * Math.PI); 
    c.closePath();
    if (filled)
        c.fill();
    else 
        c.stroke();
}

function drawCircleV(pos, radius, filled)
{
    drawCircle(pos.x, pos.y, radius, filled);
}

function draw() 
{
    c.clearRect(0, 0, canvas.width, canvas.height);

    c.fillStyle = "#FF8800";
    c.lineWidth = 2.0;

    for(var i=0;i<physicsScene.numParticles; ++i)
    {
        var x = particles.pos[2*i + 0];
        var y = particles.pos[2*i + 1];
        drawCircle(x, y, particleRadius, true);
    }

    c.fillStyle = "#FF0000";

    drawCircleV(physicsScene.boundaryCenter, physicsScene.boundaryRadius, false);

    c.fillStyle = "#5555FF";

    for(var i=1;i<physicsScene.boundaries.length;++i)
    {
        var x = physicsScene.boundaries[i].pos.x;
        var y = physicsScene.boundaries[i].pos.y;
        var r = physicsScene.boundaries[i].radius;
        drawCircle(x, y, r, true);
    }
}

// ------------------------------------------------

function calcStaticFriction(dp, n, dist, frict)
{
    var dp_n = new Vector2(0,0);
    var dp_t = new Vector2(0,0);

    // perp projection
    var proj = dp.dot(n);
    dp_n.add(n, proj);

    dp_t.subtractVectors(dp, dp_n);
    var dp_t_len = dp_t.length();
    // part of velocity update pass

    if (dp_t_len < mu_s * dist) {
        frict.x = dp_t.x;
        frict.y = dp_t.y;
    }
}

// calculated static & dynamic friction if not using velocity update pass
// the way Unified Particle Physics describes it in 6.1
function calcFriction(dp, n, dist, frict)
{
    if(use_velocity_pass) {
        calcStaticFriction(dp, n, dist, frict);
        return;
    }

    var dp_n = new Vector2(0,0);
    var dp_t = new Vector2(0,0);

    // perp projection
    var proj = dp.dot(n);
    dp_n.add(n, proj);

    dp_t.subtractVectors(dp, dp_n);
    var dp_t_len = dp_t.length();
    // part of velocity update pass

    if (dp_t_len < mu_s * dist) {
        frict.x = dp_t.x;
        frict.y = dp_t.y;
    }
    else {
        // 0/0 = NaN, so avoid it
        var k = mu_k == 0 ? 0 : Math.min(mu_k * dist / dp_t_len, 1);
        frict.x = k * dp_t.x;
        frict.y = k * dp_t.y;
    }
}

// ------------------------------------------------

function solveBoundaryConstraint(is_stab)
{
    //var br = physicsScene.boundaryRadius - particleRadius;
    //var bc = physicsScene.boundaryCenter;

    var n = new Vector2(0,0);
    var pi = new Vector2(0,0);
    var dp = new Vector2(0,0);
    var dp_n = new Vector2(0,0);
    var dp_t = new Vector2(0,0);
    var path = new Vector2(0,0);
    var frict = new Vector2(0,0);
    for(var i=0;i<physicsScene.numParticles; ++i)
    {
        pi.x = particles.pos[2*i + 0];
        pi.y = particles.pos[2*i + 1];

        for(var j=0;j<physicsScene.boundaries.length;++j) {

            var inv = physicsScene.boundaries[j].inverted;
            var br = physicsScene.boundaries[j].radius - (inv ? particleRadius : -particleRadius);
            var bc = physicsScene.boundaries[j].pos;

            n.subtractVectors(pi, bc);
            var d = n.length();
            n.scale(1 / d);
            var C = d - br;
            if(inv) {
                n.scale(-1);
                C = -C;
            }

            if (C < 0) {
                dp.x = 0;
                dp.y = 0;
                dp.add(n, -C);
                var dist = Math.abs(C);

                particles.pos[2 * i + 0] += dp.x;
                particles.pos[2 * i + 1] += dp.y;

                // corrected pos minus old pos
                path.x = (particles.pos[2 * i + 0] - particles.prev[2 * i + 0]);
                path.y = (particles.pos[2 * i + 1] - particles.prev[2 * i + 1]);


                calcFriction(path, n, dist, frict)

                if (use_velocity_pass) {
                    var dvx = particles.vel[2 * i] - 0;
                    var dvy = particles.vel[2 * i + 1] - 0;
                    var d_lambda_n = dist;//path.dot(n);
                    constraints.pushBack(i, -1, dvx, dvy, n.x, n.y, d_lambda_n, mu_k, e);
                }

                particles.pos[2 * i + 0] -= frict.x;
                particles.pos[2 * i + 1] -= frict.y;

                if(is_stab) {
                    particles.prev[2 * i + 0] += dp.x - frict.x;
                    particles.prev[2 * i + 1] += dp.y - frict.y;
                }
            }
        }
    }
}

function solveCollisionConstraints(is_stab)
{
    var frict = new Vector2(0,0);
    var path = new Vector2(0,0);
    var n = new Vector2(0,0);
    for(var i=0;i<physicsScene.numParticles; ++i)
    {
        var px = particles.pos[2 * i];
        var py = particles.pos[2 * i + 1];

        var first = firstNeighbor[i];
        var num = firstNeighbor[i + 1] - first;

        for (var j = 0; j < num; j++) {


            var id = neighbors.vals[first + j];				
            if(id == i)
                continue;

            n.x = px - particles.pos[2 * id];				
            n.y = py - particles.pos[2 * id + 1];
            var d = n.length();

            if (d > 0) {
                n.scale(1/d);
            }

            var C = 2*particleRadius - d;
            if(C > 0) {
                // particles have same mass, so  w1/(w1+w2) = w2/(w1+w2) = 0.5

                var dx = 0.5*C*n.x;
                var dy = 0.5*C*n.y;

                particles.pos[2 * i] += dx;
                particles.pos[2 * i + 1 ]+= dy;

                particles.pos[2 * id] -= dx;				
                particles.pos[2 * id + 1] -= dy;

                // corrected pos minus old pos
                path.x = (particles.pos[2 * i + 0] - particles.prev[2 * i + 0]) -
                (particles.pos[2 * id + 0] - particles.prev[2 * id + 0]);
                path.y = (particles.pos[2 * i + 1] - particles.prev[2 * i + 1]) - 
                (particles.pos[2 * id + 1] - particles.prev[2 * id + 1]);

                var dist = C;
                frict.x = frict.y = 0;
                calcFriction(path, n, dist, frict)
                particles.pos[2 * i + 0] -= 0.5*frict.x;
                particles.pos[2 * i + 1] -= 0.5*frict.y;
                particles.pos[2 * id + 0] += 0.5*frict.x;
                particles.pos[2 * id + 1] += 0.5*frict.y;

                if(is_stab) {
                    particles.prev[2 * i + 0] += dx-0.5*frict.x;
                    particles.prev[2 * i + 1] += dy-0.5 * frict.y;
                    particles.prev[2 * id + 0] += -dx + 0.5 * frict.x;
                    particles.prev[2 * id + 1] += -dy + 0.5 * frict.y;
                }

                if (use_velocity_pass) {
                    // fill constraint for velocity pass
                    var dvx = particles.vel[2 * i] - particles.vel[2 * id];
                    var dvy = particles.vel[2 * i + 1] - particles.vel[2 * id + 1];
                    var d_lambda_n = 0.5 * path.dot(n);
                    constraints.pushBack(i, id, dvx, dvy, n.x, n.y, d_lambda_n, mu_k, e);
                }
            }
        }
    }

}

// -----------------------------------------------------------------------------------

var hashSize = 370111;

var hash = {
    size : hashSize,

    first : new Int32Array(hashSize),
    marks : new Int32Array(hashSize),
    currentMark : 0,

    next : new Int32Array(maxParticles),

    orig : { left : -100.0, bottom : -1.0 }		
}

var gridSpacing = 2*5*particleRadius;
var invGridSpacing = 1/gridSpacing;
var firstNeighbor = new Int32Array(maxParticles + 1);
var neighbors = new Vector(10);// * maxParticles);

function initNeighborsHash() {
    for (var i = 0; i < hashSize; i++) {
        hash.first[i] = -1;
        hash.marks[i] = 0;
    }			
}

function findNeighbors() 
{
    // hash particles
    hash.currentMark++;

    for (var i = 0; i < physicsScene.numParticles; i++) {
        var px = particles.pos[2 * i];
        var py = particles.pos[2 * i + 1];

        var gx = Math.floor((px - hash.orig.left) * invGridSpacing);
        var gy = Math.floor((py - hash.orig.bottom) * invGridSpacing);

        var h = (Math.abs((gx * 92837111) ^ (gy * 689287499))) % hash.size;

        if (hash.marks[h] != hash.currentMark) {				
            hash.marks[h] = hash.currentMark;
            hash.first[h] = -1;
        }

        hash.next[i] = hash.first[h];
        hash.first[h] = i;
    }

    // collect neighbors
    neighbors.clear();

    var h2 = gridSpacing * gridSpacing;

    for (var i = 0; i < physicsScene.numParticles; i++) {
        firstNeighbor[i] = neighbors.size;

        var px = particles.pos[2 * i];
        var py = particles.pos[2 * i + 1];

        var gx = Math.floor((px - hash.orig.left) * invGridSpacing);
        var gy = Math.floor((py - hash.orig.bottom) * invGridSpacing);

        var x, y;

        for (x = gx - 1; x <= gx + 1; x++) {
            for (y = gy - 1; y <= gy + 1; y++) {

                var h = (Math.abs((x * 92837111) ^ (y * 689287499))) % hash.size;

                if (hash.marks[h] != hash.currentMark) 
                    continue;

                var id = hash.first[h];
                while (id >= 0) 
                {
                    var dx = particles.pos[2 * id] - px;
                    var dy = particles.pos[2 * id + 1] - py;

                    if (dx * dx + dy * dy < h2) 
                        neighbors.pushBack(id);

                    id = hash.next[id];						
                }
            }
        }
    }
    firstNeighbor[physicsScene.numParticles] = neighbors.size;
}

// ------------------------------------------------
var very_small_float = 1e-6;
function velocityUpdate(h)
{
    //var mu_k; 
    //var e; 
    var v = new Vector2(0,0);
    var vprev = new Vector2(0,0);
    var n = new Vector2(0,0);
    var dv = new Vector2(0,0);
    var vt = new Vector2(0,0);
    for (var i = 0; i < constraints.size; i++) {
        var idA = constraints.idxA(i);
        var idB = constraints.idxB(i);

        // relative velocity
        v.x = particles.vel[2*idA + 0];
        v.y = particles.vel[2*idA + 1];
        v.x -= idB==-1 ? 0 : particles.vel[2*idB + 0];
        v.y -= idB==-1 ? 0 : particles.vel[2*idB + 1];

        // previous relative velocity
        vprev.x = constraints.dvx(i);
        vprev.y = constraints.dvy(i);

        n.x = constraints.nx(i);
        n.y = constraints.ny(i);
        var d_lambda_n = constraints.d_lambda_n(i);

        var vn = n.dot(v);
        vt.x = v.x - vn * n.x;
        vt.y = v.y - vn * n.y;

		// friction force (dynamic friction)
		var vt_len = vt.length();
		var fn = Math.abs(d_lambda_n / (h * h));
		// Eq. 30
        if(vt_len > very_small_float) {
            dv.x = -(vt.x / vt_len) * Math.min(h * mu_k * fn, vt_len)
            dv.y = -(vt.y / vt_len) * Math.min(h * mu_k * fn, vt_len)
        } else {
           dv.x = dv.y = 0; 
        }

		// restitution, Eq. 34
		var v_prev_n = vprev.dot(n);
        // !NB: original paper has: min() but probably it is an error
		var restitution_x = n.x * (-vn + Math.max(-e * v_prev_n, 0.0));
		var restitution_y = n.y * (-vn + Math.max(-e * v_prev_n, 0.0));

		// now, apply delta velocity
		// NOTE: probably should first apply dv and then calculate restitution_dv and
		// apply it again?
		var w1 = 1;
		var w2 = idB==-1 ? 0.0 : 1;

		particles.vel[2*idA + 0] += (dv.x + restitution_x) * w1 / (w1 + w2) ;
		particles.vel[2*idA + 1] += (dv.y + restitution_y) * w1 / (w1 + w2) ;
		if (idB != -1) {
			restitution_x = n.x * (-vn + Math.max(-e * v_prev_n, 0.0));
			restitution_y = n.y * (-vn + Math.max(-e * v_prev_n, 0.0));
		    particles.vel[2*idB + 0] -= (dv.x + restitution_x) * w2 / (w1 + w2) ;
		    particles.vel[2*idB + 1] -= (dv.y + restitution_y) * w2 / (w1 + w2) ;
		}
    }

}


// ------------------------------------------------

function simulate() 
{
    if (physicsScene.paused)
        return;

    var h = physicsScene.dt / physicsScene.numSteps;
    var force, analyticForce;
    var g = physicsScene.gravity;
    var force = 0;
    var vi = new Vector2(0,0);
    var pi = new Vector2(0,0);
    var pprevi = new Vector2(0,0);

    var delta_angle = 120*physicsScene.dt;
    var smallBoundaryRadius = 0.25*physicsScene.boundaryRadius;
    var bpos = new Vector2(0,0);
    var rot_m = Mat2.rotation(delta_angle*Math.PI/180.0)
    for(var i=1;i<physicsScene.boundaries.length;++i)
    {   bpos = physicsScene.boundaries[i].pos;
        bpos.x -= physicsScene.boundaryCenter.x;
        bpos.y -= physicsScene.boundaryCenter.y;
        bpos.rotate(rot_m);
        bpos.x += physicsScene.boundaryCenter.x;
        bpos.y += physicsScene.boundaryCenter.y;
        physicsScene.boundaries.pos = bpos;
    }

    constraints.clear();
    findNeighbors();

    for (var step = 0; step < physicsScene.numSteps; step++)
    {
        constraints.clear();
        // predict
        for (var i = 0; i < physicsScene.numParticles; i++)
        {
            // use temp var to not overwrite correct velocities
            // we need them when filling constraint structure
            var vx = particles.vel[2 * i + 0] + g.x * h;
            var vy = particles.vel[2 * i + 1] + g.y * h;
            particles.prev[2 * i] = particles.pos[2 * i];
            particles.prev[2 * i + 1] = particles.pos[2 * i + 1];
            particles.pos[2 * i] += vx * h;
            particles.pos[2 * i + 1] += vy * h;
        }

        var is_stab = false;
        if(step<5)
           is_stab = true;

        solveCollisionConstraints(is_stab);
        solveBoundaryConstraint(is_stab);

        //force = Math.abs(lambda / sdt / sdt);

        // update velocities
        for (i = 0; i < physicsScene.numParticles; i++)
        {
            pi.x = particles.pos[2*i + 0];
            pi.y = particles.pos[2*i + 1];

            pprevi.x = particles.prev[2*i + 0];
            pprevi.y = particles.prev[2*i + 1];

            vi.subtractVectors(pi, pprevi);
            vi.scale(1.0 / h);
            if(Number.isNaN(vi.x) || Number.isNaN(vi.y)) {
                //alert("Nan");
                vi.x = 0;
            }

            particles.vel[2 * i + 0] = vi.x;
            particles.vel[2 * i + 1] = vi.y;
        }

        velocityUpdate(h);
    }
}

// --------------------------------------------------------

function run() {
    physicsScene.paused = false;
}

function step() {
    physicsScene.paused = false;
    simulate();
    physicsScene.paused = true;
}

function update() {
    try {
        simulate();
        draw();
        requestAnimationFrame(update);
    } catch(err) {
        if (err instanceof TypeError) {
            printError(err, true);
        } else {
            printError(err, false);
        }
    }
}

//setupScene();
//update();
